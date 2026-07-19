/* init.cpp — setup() and its GNSS PVT callback. */
#include <Arduino.h>
#include <TeensyTimerTool.h>
#include <FlexCAN_T4.h>
#include "pMBB32.h"
#include "defines.h"
#include "Fsm.h"
#include "lin.h"
#include "sdlog.h"
#include "fsm_states.h"
#include "can_handlers.h"
#include "callbacks.h"

using namespace TeensyTimerTool;

#ifdef UBLOX_GNSS
void printPVTdata(UBX_NAV_PVT_data_t *ubxDataStruct)
{
    GPSaltitude = ubxDataStruct->hMSL;
    groundSpeed = ubxDataStruct->gSpeed;
    fixType     = ubxDataStruct->fixType;

    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 5000) {
      lastPrint = millis();
      Serial.printf("GNSS fix=%u SIV=%u\n", fixType, ubxDataStruct->numSV);
    }
}
#endif
/* Setup */
void setup() {
  //timing debug pins
  pinMode(3,OUTPUT);        //loop timing
  digitalWriteFast(3, LOW);
  pinMode(4,OUTPUT);        //CAN1 RX timing
  digitalWriteFast(4, LOW);
  pinMode(5,OUTPUT);        //CAN2 RX timing
  digitalWriteFast(5, LOW);
  pinMode(6,OUTPUT);        //display timing
  digitalWriteFast(6, LOW);

  Serial.begin(115200);
  delay(50);

  linInit();//BMW changeover valve LIN master on Serial3

  pinMode(KL15R_PIN, INPUT_PULLDOWN); // KL15R key position 1 — LOW = key off → sleep

  // Detect CAN-wake boot: sleepMagic is set in enterSleep() based on wake source.
  // SLEEP_MAGIC_CAN_WAKE + KL15R LOW = CAN2 woke the VCU → enter KL30C standby.
  if (sleepMagic == SLEEP_MAGIC_CAN_WAKE && !digitalRead(KL15R_PIN))
    extWakePending = true;
  sleepMagic = 0; // clear so a cold-boot after this point does not misfire

  pinMode(CAN_STBY_PIN, OUTPUT);
  digitalWrite(CAN_STBY_PIN, LOW);    // transceivers active; driven HIGH in enterSleep()

  pinMode(PRECHARGE_EN_PIN, OUTPUT);
  digitalWrite(PRECHARGE_EN_PIN, LOW); // pre-charge disabled until PreCharge state

#ifdef UBLOX_GNSS
  pinMode(GNSS_EXTINT_PIN, OUTPUT);
  digitalWrite(GNSS_EXTINT_PIN, LOW); // idle low; pulsed high in enterSleep() to wake module
#endif

  initCAN(500000, 500000, 1000000);//start CAN1, CAN2, CAN3 at 500kbps, 500kbps, 1Mbps
  delay(100);

  // Power up pMBB32 modules via PDU-8 CH2 as early as possible so they have
  // time to boot (GNSS init below takes ~300-500ms — used as free settling time)
  PDUmsg1.id = 0x0A0620;
  PDUmsg1.len = 8;
  PDUmsg1.buf[0] = 0;     // CH1 — negative contactor (off)
  PDUmsg1.buf[1] = 0x05;  // CH2 — pMBB32 modules 2A
  PDUmsg1.buf[2] = 0;     // CH3 — positive pre-charge (off)
  PDUmsg1.buf[3] = 0;     // CH4 — positive contactor (off)
  PDUmsg1.buf[4] = 0;
  PDUmsg1.buf[5] = 0;
  PDUmsg1.buf[6] = 0;
  PDUmsg1.buf[7] = 0;
  uint32_t ch2OnAt = millis(); // used below to guarantee pMBB32 boot time
  can1.write(PDUmsg1);

  PDUmsg2.id = 0x0A0630;
  PDUmsg2.len = 8;
  PDUmsg2.buf[0] = 0;
  PDUmsg2.buf[1] = 0xFE;  // CH2 — pMBB32
  PDUmsg2.buf[2] = 0;
  PDUmsg2.buf[3] = 0;
  PDUmsg2.buf[4] = 0;
  PDUmsg2.buf[5] = 0;
  PDUmsg2.buf[6] = 0;
  PDUmsg2.buf[7] = 0;
  can1.write(PDUmsg2);

  t0.begin(callback_t0, 10000); // 10ms LDU torque command loop
  t0.priority(0);               // highest priority on ARM Cortex-M7 (0=highest, 255=lowest)
  t1.begin(callback_t1, t1CallbackRate); // start early: keeps CH2=0x05 refreshing every 62.5ms while PDU-8/pMBB32 boot

  pinMode(LED_BUILTIN, OUTPUT);

#ifdef UBLOX_GNSS
  Wire.setClock(400 * 1000);//for U-blox GPS
  Wire.begin();
  Wire.setTimeout(50);             // 50ms — backup-wake I2C responses are slower

  // GNSS was woken via EXTINT rising edge in enterSleep() before AIRCR reset;
  // it hot-starts during Teensy setup().  Just wait for I2C to respond.
  while (myGNSS.begin() == false)
  {
    Serial.println(F("u-blox GNSS not detected — retrying"));
    delay(500);
  }

  Serial.println("GNSS online");

  Serial.printf("setI2COutput:  %s\n", myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA) ? "OK" : "FAIL");
  Serial.printf("saveConfig:    %s\n", myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT) ? "OK" : "FAIL");
  Serial.printf("setNavFreq:    %s\n", myGNSS.setNavigationFrequency(10) ? "OK" : "FAIL");
  Serial.printf("setAutoPVT:    %s\n", myGNSS.setAutoPVTcallbackPtr(&printPVTdata) ? "OK" : "FAIL");
#endif

  // ADS1115 — I2C0 shares Wire with GNSS (already started above).
  if (!ads.begin(ADS1115_ADDR)) {
    Serial.println("ADS1115 not found");
  } else {
    Serial.println("ADS1115 online");
    ads.setGain(GAIN_ONE);                // ±4.096 V — covers 3.3V sensors
    ads.setDataRate(RATE_ADS1115_860SPS); // 860 SPS → ~1.2ms/conversion, ~3.5ms/channel cycle
    ads.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0, false); // start first read
  }

  // Guarantee at least 250 ms since PDU-8 CH2 was enabled before waking pMBB32.
  // The dsPIC33 boots in <50 ms; 250 ms gives comfortable margin for the PL455
  // wake sequence and daisy-chain reset before AutoAddress runs.
  {
    uint32_t elapsed = millis() - ch2OnAt;
    if (elapsed < 250) delay(250 - elapsed);
  }
  wakepMBB32();

  t2.begin(callback_t2, t2CallbackRate);
  t3.begin(callback_t3, t3CallbackRate);

  sdInit(); // must be after timers start (uses millis())

  msg2.id = 0x412;//send IVT command
  msg2.flags.extended = 0;
  msg2.len = 6;
  msg2.buf[0] = 0x34;//set SET_MODE command
  msg2.buf[1] = 0;//set actual mode: 0 = stop, 1 = run
  msg2.buf[2] = 1;//set startup operation mode: 0 = stop, 1 = run
  can2.write(msg2);
  delay(1);
  msg2.buf[0] = 0x24;//configuration of measurement 0x2x: 0 = I, 1 = U1, 2 = U2, 3 = U3, 4 = T, 5 = W, 6 = As, 7 = Wh
  msg2.buf[1] = 2;//low nibble = 0 for disabled, 1 for triggered, 2 for cyclic running
  msg2.buf[3] = 0xC8;//200ms
  can2.write(msg2);
  delay(1);
  msg2.buf[0] = 0x25;//configuration of measurement 0x2x: 0 = I, 1 = U1, 2 = U2, 3 = U3, 4 = T, 5 = W, 6 = As, 7 = Wh
  msg2.buf[3] = 0x64;//100ms
  can2.write(msg2);
  delay(1);
  msg2.buf[0] = 0x26;//configuration of measurement 0x2x: 0 = I, 1 = U1, 2 = U2, 3 = U3, 4 = T, 5 = W, 6 = As, 7 = Wh
  can2.write(msg2);
  delay(1);
  msg2.buf[0] = 0x27;//configuration of measurement 0x2x: 0 = I, 1 = U1, 2 = U2, 3 = U3, 4 = T, 5 = W, 6 = As, 7 = Wh
  msg2.buf[3] = 0x64;//100ms
  can2.write(msg2);
  delay(1);
  msg2.buf[0] = 0x34;//set SET_MODE command
  msg2.buf[1] = 1;//set actual mode: 0 = stop, 1 = run
  msg2.buf[2] = 1;//set startup operation mode: 0 = stop, 1 = run
  can2.write(msg2);
  delay(1);

  msg2.id = 0xA100101;//send SIM100MOD Request Part Name command
  msg2.flags.extended = 1;
  msg2.len = 1;
  msg2.buf[0] = 1;//request part name 0
  can2.write(msg2);
  delay(2);
  msg2.id = 0xA100101;//send SIM100MOD Request Part Name command
  msg2.len = 1;
  msg2.buf[0] = 2;//request part name 1
  can2.write(msg2);
  delay(2);
  msg2.id = 0xA100101;//send SIM100MOD Request Part Name command
  msg2.len = 1;
  msg2.buf[0] = 3;//request part name 2
  can2.write(msg2);
  delay(2);
  msg2.id = 0xA100101;//send SIM100MOD Request Part Name command
  msg2.len = 1;
  msg2.buf[0] = 4;//request part name 3
  can2.write(msg2);
  delay(1);

  /* OpenInverter Tesla LDU V2 init ------------------------------------------
   * Send torque command frame to enable the inverter.
   * TODO: confirm CAN ID (0x19B default), byte map, and enable sequence.
   * buf[0:1] = torque setpoint (Nm, signed 16-bit, big-endian)
   * buf[2]   = enable bit (0x01 = enable, 0x00 = disable)
   */
  //msg2.id = 0x19B;
  //msg2.flags.extended = 0;
  //msg2.len = 3;
  //msg2.buf[0] = 0x00;//torque setpoint high byte (0 Nm)
  //msg2.buf[1] = 0x00;//torque setpoint low byte
  //msg2.buf[2] = 0x01;//enable
  //can2.write(msg2);
  //delay(1);

  /* EMP WP29-12V-CV-A Water Pump init ---------------------------------------
   * Motor Command Message: 0x18EF{pump_addr}{vcu_addr}, extended, 500kbps
   * Send Motor Off command at startup; periodic commands via callback_t2().
   * EMP proprietary protocol — 9980001068 Rev. N
   */
  invPumpSetpoint     = 0;
  msg2.id             = EMP_WP29_CMD_ID;
  msg2.flags.extended = 1;
  msg2.len            = 8;
  msg2.buf[0]         = 0xFC;  // Motor Off + Don't Care power hold
  msg2.buf[1]         = 0xFF;
  msg2.buf[2]         = 0xFF;
  msg2.buf[3]         = 0x00;  // 0 % speed
  msg2.buf[4]         = 0xFF;
  msg2.buf[5]         = 0xFF;
  msg2.buf[6]         = 0xFF;
  msg2.buf[7]         = 0xFF;
  can2.write(msg2);
  delay(1);

  battPumpSetpoint    = 0;
  msg1.id             = EMP_WP29_CMD_ID;
  msg1.flags.extended = 1;
  msg1.len            = 8;
  msg1.buf[0]         = 0xFC;
  msg1.buf[1]         = 0xFF;
  msg1.buf[2]         = 0xFF;
  msg1.buf[3]         = 0x00;
  msg1.buf[4]         = 0xFF;
  msg1.buf[5]         = 0xFF;
  msg1.buf[6]         = 0xFF;
  msg1.buf[7]         = 0xFF;
  can1.write(msg1);
  delay(1);

/*
  // Set all keypad buttons to amber at startup
  msg2.id = 0x18EF2100;
  msg2.flags.extended = 1;
  msg2.len = 8;
  msg2.buf[0] = 0x04;
  msg2.buf[1] = 0x1B;
  msg2.buf[2] = KEYPAD_CMD_LIVE_BACKLIGHT_COLOR;
  msg2.buf[3] = KEYPAD_COLOR_AMBER;// Amber
  msg2.buf[4] = 0xFF;
  msg2.buf[5] = 0xFF;
  msg2.buf[6] = 0xFF;
  msg2.buf[7] = 0xFF;
  can2.write(msg2);
  delay(1);

  // Set brightness of all keypad buttons to 50% at startup
  msg2.id = 0x18EF2100;
  msg2.flags.extended = 1;
  msg2.len = 8;
  msg2.buf[0] = 0x04;
  msg2.buf[1] = 0x1B;
  msg2.buf[2] = KEYPAD_CMD_LIVE_BRIGHTNESS;
  msg2.buf[3] = 0x20;// 50% brightness
  msg2.buf[4] = 0xFF;
  msg2.buf[5] = 0xFF;
  msg2.buf[6] = 0xFF;
  msg2.buf[7] = 0xFF;
  can2.write(msg2);
  delay(1);
*/
  // Off → PreCharge (KL15_ON or EVCC sets chargeMode=true then fires KL15_ON)
  fsm.add_transition(&state_Off, &state_PreCharge, KL15_ON, &on_trans_Off_PreCharge);

  // PreCharge outcomes
  fsm.add_transition(&state_PreCharge, &state_Idle,   PRECHARGE_OK,     &on_trans_PreCharge_Idle);
  fsm.add_transition(&state_PreCharge, &state_Charge, PRECHARGE_CHARGE, &on_trans_PreCharge_Charge);
  fsm.add_transition(&state_PreCharge, &state_Fault,  PRECHARGE_FAIL,   &on_trans_PreCharge_Fault);
  fsm.add_transition(&state_PreCharge, &state_Off,    KL15_OFF,         &on_trans_Idle_Off);

  // Idle ↔ Drive
  fsm.add_transition(&state_Idle,  &state_Drive, DRIVE_ON,  &on_trans_Idle_Drive);
  fsm.add_transition(&state_Drive, &state_Idle,  DRIVE_OFF, &on_trans_Drive_Idle);

  // Any active state → Off on KL15_OFF
  fsm.add_transition(&state_Idle,     &state_Off, KL15_OFF, &on_trans_Idle_Off);
  fsm.add_transition(&state_Drive,    &state_Off, KL15_OFF, &on_trans_Idle_Off);
  fsm.add_transition(&state_Charge,   &state_Off, KL15_OFF, &on_trans_Idle_Off);
  fsm.add_transition(&state_HeatPack, &state_Off, KL15_OFF, &on_trans_Idle_Off);
  fsm.add_transition(&state_CoolPack, &state_Off, KL15_OFF, &on_trans_Idle_Off);

  // Fault paths
  fsm.add_transition(&state_Drive,      &state_Fault, FAULT_EV,    nullptr);
  fsm.add_transition(&state_Charge,     &state_Fault, FAULT_EV,    nullptr);
  fsm.add_transition(&state_PreCharge,  &state_Fault, FAULT_EV,    nullptr);
  fsm.add_transition(&state_Fault,      &state_Off,   FAULT_CLEAR, &on_trans_Fault_Off);

  // Charge stop (EVCC request)
  fsm.add_transition(&state_Charge, &state_Off, CHARGE_OFF, nullptr);

  // Thermal management
  fsm.add_transition(&state_Idle,     &state_HeatPack, TEMP_LOW,  nullptr);
  fsm.add_transition(&state_Drive,    &state_HeatPack, TEMP_LOW,  nullptr);
  fsm.add_transition(&state_Charge,   &state_HeatPack, TEMP_LOW,  nullptr);
  fsm.add_transition(&state_Idle,     &state_CoolPack, TEMP_HIGH, nullptr);
  fsm.add_transition(&state_Drive,    &state_CoolPack, TEMP_HIGH, nullptr);
  fsm.add_transition(&state_Charge,   &state_CoolPack, TEMP_HIGH, nullptr);
  fsm.add_transition(&state_HeatPack, &state_Idle,     TEMP_OK,   nullptr);
  fsm.add_transition(&state_CoolPack, &state_Idle,     TEMP_OK,   nullptr);

  // KL30C — CAN/EVCC external wake standby
  fsm.add_transition(&state_Off,   &state_KL30C,    EXT_WAKE,        nullptr);
  fsm.add_transition(&state_KL30C, &state_Off,      KL15_ON,         nullptr);
  fsm.add_transition(&state_KL30C, &state_PreCharge, AC_CHARGE_START, nullptr);

  // Set keypad LEDs to Off state
  msg2.id = 0x18EF2100;
  msg2.flags.extended = 1;
  msg2.len = 8;
  msg2.buf[0] = 0x04;
  msg2.buf[1] = 0x1B;
  msg2.buf[2] = KEYPAD_CMD_BACKLIGHT_BRIGHTNESS;
  msg2.buf[3] = KEYPAD_COLOR_OFF;
  msg2.buf[4] = 0xFF;
  msg2.buf[5] = 0xFF;
  msg2.buf[6] = 0xFF;
  msg2.buf[7] = 0xFF;
  can2.write(msg2);
  // Set keypad LEDs to Off state
  msg2.id = 0x18EF2100;
  msg2.flags.extended = 1;
  msg2.len = 8;
  msg2.buf[0] = 0x04;
  msg2.buf[1] = 0x1B;
  msg2.buf[2] = KEYPAD_CMD_BACKLIGHT_COLOR;
  msg2.buf[3] = KEYPAD_COLOR_AMBER;
  msg2.buf[4] = 0xFF;
  msg2.buf[5] = 0xFF;
  msg2.buf[6] = 0xFF;
  msg2.buf[7] = 0xFF;
  can2.write(msg2);
  Off_enter();

  // CAN-wake boot detected above — enter KL30C standby immediately.
  if (extWakePending) {
    extWakePending = false;
    fsm.trigger(EXT_WAKE);
  }

  // // Set all keypad buttons to amber at startup
  // msg2.id = 0x18EF2100;//keypad button color command
  // msg2.flags.extended = 1;
  // msg2.len = 8;
  // msg2.buf[0] = 0x04;
  // msg2.buf[1] = 0x1B;
  // msg2.buf[2] = KEYPAD_CMD_LIVE_BACKLIGHT_COLOR;
  // msg2.buf[3] = KEYPAD_COLOR_AMBER;// Amber
  // msg2.buf[4] = 0xFF;
  // msg2.buf[5] = 0xFF;
  // msg2.buf[6] = 0xFF;
  // msg2.buf[7] = 0xFF;
  // can2.write(msg2);
  // // Set brightness of all keypad buttons to 50% at startup
  // msg2.id = 0x18EF2100;
  // msg2.flags.extended = 1;
  // msg2.len = 8;
  // msg2.buf[0] = 0x04;
  // msg2.buf[1] = 0x1B;
  // msg2.buf[2] = KEYPAD_CMD_LIVE_BRIGHTNESS;
  // msg2.buf[3] = 0x20;// 50% brightness
  // msg2.buf[4] = 0xFF;
  // msg2.buf[5] = 0xFF;
  // msg2.buf[6] = 0xFF;
  // msg2.buf[7] = 0xFF;
  // can2.write(msg2);
}//end of setup()
