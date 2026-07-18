/* main.cpp — VCU entry point: object definitions, setup(), loop().
 * All functional modules extracted to separate .cpp files.
 */
#include <Arduino.h>
#include <SdFat.h>
#include <TeensyTimerTool.h>
#include <FlexCAN_T4.h>
#include "vcu.h"
#include "sdlog.h"
#include "can_handlers.h"
#include "callbacks.h"
#include "fsm_states.h"
#include "throttle.h"
#include "sleep.h"
#include "display.h"
#include "lin.h"

using namespace TeensyTimerTool;

// ── Timer objects ─────────────────────────────────────────────────────────────
IntervalTimer  t0;
PeriodicTimer t1(RTC);
PeriodicTimer t2(GPT1);
PeriodicTimer t3(GPT2);

// ── CAN buses ─────────────────────────────────────────────────────────────────
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;

// ── CAN message buffers ───────────────────────────────────────────────────────
CAN_message_t msg1;
CAN_message_t msg2;
CAN_message_t msg3;
CAN_message_t PDUmsg1;
CAN_message_t PDUmsg2;
CAN_message_t LDUmsg;

// ── FSM states and machine ────────────────────────────────────────────────────
State state_Off(&Off_enter, &check_KL15, &Off_exit);
State state_PreCharge(&PreCharge_enter, &check_PreCharge, &PreCharge_exit);
State state_Idle(&Idle_enter, &check_Idle, &Idle_exit);
State state_Drive(&Drive_enter, &check_DriveState, &Drive_exit);
State state_Charge(&Charge_enter, &check_Charge, &Charge_exit);
State state_HeatPack(&HeatPack_enter, &check_HeatPack, &HeatPack_exit);
State state_CoolPack(&CoolPack_enter, &check_CoolPack, &CoolPack_exit);
State state_Fault(&Fault_enter, &check_Fault, &Fault_exit);
State state_KL30C(&KL30C_enter, &check_KL30C, &KL30C_exit);
Fsm fsm(&state_Off);

// ── Main-scope globals ────────────────────────────────────────────────────────
// DMAMEM places sleepMagic in OCRAM (.bss.dma, NOLOAD): survives AIRCR reset.
DMAMEM volatile uint32_t sleepMagic;
bool          extWakePending   = false;
uint32_t      lastExtActivityMs = 0;
bool          kl30cKL15Rstate  = false;
bool          evccIsACSession  = false;
bool          acReadyToDeliver = false;
VCUStateEnum  VCUstate         = VCU_STATE_OFF;

// ── GNSS callback (uses myGNSS from globals.cpp via defines.h) ────────────────
#ifdef UBLOX_GNSS
void printPVTdata(UBX_NAV_PVT_data_t *ubxDataStruct) {
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

// ── Misc utilities (small, rarely called, not worth a separate file) ──────────
void ReadDigitalStatuses() {
  digitalPins = 0;
  int bit = 0;
  for (int i = 1; i < 14; i++) {
    if (digitalRead(i) == HIGH) digitalPins |= (1 << bit);
    bit++;
  }
}

void ReadAnalogStatuses() {
  for (int i = 0; i < 7; i++) analogPins[i] = analogRead(i);
}

/* Setup */
void setup() {
  pinMode(3, OUTPUT); digitalWriteFast(3, LOW); // loop timing
  pinMode(4, OUTPUT); digitalWriteFast(4, LOW); // CAN1 RX
  pinMode(5, OUTPUT); digitalWriteFast(5, LOW); // CAN2 RX
  pinMode(6, OUTPUT); digitalWriteFast(6, LOW); // display

  Serial.begin(115200);
  delay(50);

  linInit();

  pinMode(KL15R_PIN, INPUT_PULLDOWN);

  if (sleepMagic == SLEEP_MAGIC_CAN_WAKE && !digitalRead(KL15R_PIN))
    extWakePending = true;
  sleepMagic = 0;

  pinMode(CAN_STBY_PIN, OUTPUT);
  digitalWrite(CAN_STBY_PIN, LOW);

  pinMode(PRECHARGE_EN_PIN, OUTPUT);
  digitalWrite(PRECHARGE_EN_PIN, LOW);

#ifdef UBLOX_GNSS
  pinMode(GNSS_EXTINT_PIN, OUTPUT);
  digitalWrite(GNSS_EXTINT_PIN, LOW);
#endif

  initCAN(500000, 500000, 1000000);
  delay(100);

  PDUmsg1.id      = 0x0A0620;
  PDUmsg1.len     = 8;
  PDUmsg1.buf[0]  = 0;
  PDUmsg1.buf[1]  = 0x05;  // CH2 — pMBB32 modules 2A
  PDUmsg1.buf[2]  = PDUmsg1.buf[3] = PDUmsg1.buf[4] = 0;
  PDUmsg1.buf[5]  = PDUmsg1.buf[6] = PDUmsg1.buf[7] = 0;
  uint32_t ch2OnAt = millis();
  can1.write(PDUmsg1);

  PDUmsg2.id      = 0x0A0630;
  PDUmsg2.len     = 8;
  PDUmsg2.buf[0]  = 0;
  PDUmsg2.buf[1]  = 0xFE;  // CH2 PWM
  PDUmsg2.buf[2]  = PDUmsg2.buf[3] = PDUmsg2.buf[4] = 0;
  PDUmsg2.buf[5]  = PDUmsg2.buf[6] = PDUmsg2.buf[7] = 0;
  can1.write(PDUmsg2);

  t0.begin(callback_t0, 10000);
  t0.priority(0);
  t1.begin(callback_t1, t1CallbackRate);

  pinMode(LED_BUILTIN, OUTPUT);

#ifdef UBLOX_GNSS
  Wire.setClock(400 * 1000);
  Wire.begin();
  Wire.setTimeout(50);

  while (myGNSS.begin() == false) {
    Serial.println(F("u-blox GNSS not detected — retrying"));
    delay(500);
  }
  Serial.println("GNSS online");
  Serial.printf("setI2COutput:  %s\n", myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA) ? "OK" : "FAIL");
  Serial.printf("saveConfig:    %s\n", myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT) ? "OK" : "FAIL");
  Serial.printf("setNavFreq:    %s\n", myGNSS.setNavigationFrequency(10) ? "OK" : "FAIL");
  Serial.printf("setAutoPVT:    %s\n", myGNSS.setAutoPVTcallbackPtr(&printPVTdata) ? "OK" : "FAIL");
#endif

  if (!ads.begin(ADS1115_ADDR)) {
    Serial.println("ADS1115 not found");
  } else {
    Serial.println("ADS1115 online");
    ads.setGain(GAIN_ONE);
    ads.setDataRate(RATE_ADS1115_860SPS);
    ads.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0, false);
  }

  {
    uint32_t elapsed = millis() - ch2OnAt;
    if (elapsed < 250) delay(250 - elapsed);
  }
  wakepMBB32();

  t2.begin(callback_t2, t2CallbackRate);
  t3.begin(callback_t3, t3CallbackRate);

  sdInit();

  // IVT-S startup configuration
  msg2.id = 0x412; msg2.flags.extended = 0; msg2.len = 6;
  msg2.buf[0] = 0x34; msg2.buf[1] = 0; msg2.buf[2] = 1;
  can2.write(msg2); delay(1);
  msg2.buf[0] = 0x24; msg2.buf[1] = 2; msg2.buf[3] = 0xC8; can2.write(msg2); delay(1);
  msg2.buf[0] = 0x25; msg2.buf[3] = 0x64;                   can2.write(msg2); delay(1);
  msg2.buf[0] = 0x26;                                        can2.write(msg2); delay(1);
  msg2.buf[0] = 0x27; msg2.buf[3] = 0x64;                   can2.write(msg2); delay(1);
  msg2.buf[0] = 0x34; msg2.buf[1] = 1; msg2.buf[2] = 1;    can2.write(msg2); delay(1);

  // SIM100MOD startup
  msg2.id = 0xA100101; msg2.flags.extended = 1; msg2.len = 1;
  for (uint8_t n = 1; n <= 4; n++) { msg2.buf[0] = n; can2.write(msg2); delay(2); }

  // EMP WP29 — send Motor Off at startup; periodic commands via callback_t2()
  invPumpSetpoint  = 0;
  msg2.id          = EMP_WP29_CMD_ID; msg2.flags.extended = 1; msg2.len = 8;
  msg2.buf[0] = 0xFC; msg2.buf[1] = msg2.buf[2] = 0xFF;
  msg2.buf[3] = 0x00; msg2.buf[4] = msg2.buf[5] = msg2.buf[6] = msg2.buf[7] = 0xFF;
  can2.write(msg2); delay(1);

  battPumpSetpoint = 0;
  msg1.id          = EMP_WP29_CMD_ID; msg1.flags.extended = 1; msg1.len = 8;
  msg1.buf[0] = 0xFC; msg1.buf[1] = msg1.buf[2] = 0xFF;
  msg1.buf[3] = 0x00; msg1.buf[4] = msg1.buf[5] = msg1.buf[6] = msg1.buf[7] = 0xFF;
  can1.write(msg1); delay(1);

  // FSM transitions
  fsm.add_transition(&state_Off,        &state_PreCharge, KL15_ON,         &on_trans_Off_PreCharge);
  fsm.add_transition(&state_PreCharge,  &state_Idle,      PRECHARGE_OK,    &on_trans_PreCharge_Idle);
  fsm.add_transition(&state_PreCharge,  &state_Charge,    PRECHARGE_CHARGE,&on_trans_PreCharge_Charge);
  fsm.add_transition(&state_PreCharge,  &state_Fault,     PRECHARGE_FAIL,  &on_trans_PreCharge_Fault);
  fsm.add_transition(&state_PreCharge,  &state_Off,       KL15_OFF,        &on_trans_Idle_Off);
  fsm.add_transition(&state_Idle,       &state_Drive,     DRIVE_ON,        &on_trans_Idle_Drive);
  fsm.add_transition(&state_Drive,      &state_Idle,      DRIVE_OFF,       &on_trans_Drive_Idle);
  fsm.add_transition(&state_Idle,       &state_Off,       KL15_OFF,        &on_trans_Idle_Off);
  fsm.add_transition(&state_Drive,      &state_Off,       KL15_OFF,        &on_trans_Idle_Off);
  fsm.add_transition(&state_Charge,     &state_Off,       KL15_OFF,        &on_trans_Idle_Off);
  fsm.add_transition(&state_HeatPack,   &state_Off,       KL15_OFF,        &on_trans_Idle_Off);
  fsm.add_transition(&state_CoolPack,   &state_Off,       KL15_OFF,        &on_trans_Idle_Off);
  fsm.add_transition(&state_Drive,      &state_Fault,     FAULT_EV,        nullptr);
  fsm.add_transition(&state_Charge,     &state_Fault,     FAULT_EV,        nullptr);
  fsm.add_transition(&state_PreCharge,  &state_Fault,     FAULT_EV,        nullptr);
  fsm.add_transition(&state_Fault,      &state_Off,       FAULT_CLEAR,     &on_trans_Fault_Off);
  fsm.add_transition(&state_Charge,     &state_Off,       CHARGE_OFF,      nullptr);
  fsm.add_transition(&state_Idle,       &state_HeatPack,  TEMP_LOW,        nullptr);
  fsm.add_transition(&state_Drive,      &state_HeatPack,  TEMP_LOW,        nullptr);
  fsm.add_transition(&state_Charge,     &state_HeatPack,  TEMP_LOW,        nullptr);
  fsm.add_transition(&state_Idle,       &state_CoolPack,  TEMP_HIGH,       nullptr);
  fsm.add_transition(&state_Drive,      &state_CoolPack,  TEMP_HIGH,       nullptr);
  fsm.add_transition(&state_Charge,     &state_CoolPack,  TEMP_HIGH,       nullptr);
  fsm.add_transition(&state_HeatPack,   &state_Idle,      TEMP_OK,         nullptr);
  fsm.add_transition(&state_CoolPack,   &state_Idle,      TEMP_OK,         nullptr);
  fsm.add_transition(&state_Off,        &state_KL30C,     EXT_WAKE,        nullptr);
  fsm.add_transition(&state_KL30C,      &state_Off,       KL15_ON,         nullptr);
  fsm.add_transition(&state_KL30C,      &state_PreCharge, AC_CHARGE_START, nullptr);

  // Keypad: set backlight off, then amber
  msg2.id = 0x18EF2100; msg2.flags.extended = 1; msg2.len = 8;
  msg2.buf[0] = 0x04; msg2.buf[1] = 0x1B;
  msg2.buf[2] = KEYPAD_CMD_BACKLIGHT_BRIGHTNESS; msg2.buf[3] = KEYPAD_COLOR_OFF;
  msg2.buf[4] = msg2.buf[5] = msg2.buf[6] = msg2.buf[7] = 0xFF;
  can2.write(msg2);

  msg2.buf[2] = KEYPAD_CMD_BACKLIGHT_COLOR; msg2.buf[3] = KEYPAD_COLOR_AMBER;
  can2.write(msg2);

  Off_enter();

  if (extWakePending) { extWakePending = false; fsm.trigger(EXT_WAKE); }
}

/* Main loop */
void loop() {
  digitalWriteFast(3, HIGH);

  can1.events();
  can2.events();

  if (sdLogPending) { sdLogPending = false; sdLogData(); }
  sdDrainEvents();

  {
    static uint32_t klrLowSince = 0;
    if (digitalRead(KL15R_PIN)) {
      klrLowSince = millis();
    } else if (VCUstate == VCU_STATE_OFF && millis() - klrLowSince > 500) {
      enterSleep();
    }
  }

  // ADS1115 non-blocking read — throttle pot 1/2 and brake at 860 SPS
  {
    static const uint16_t adsMux[3] = {
      ADS1X15_REG_CONFIG_MUX_SINGLE_0,
      ADS1X15_REG_CONFIG_MUX_SINGLE_1,
      ADS1X15_REG_CONFIG_MUX_SINGLE_2,
    };
    static uint8_t  adsCh   = 0;
    static uint32_t adsNext = 0;
    if ((int32_t)(micros() - adsNext) >= 0) {
      int16_t val = ads.getLastConversionResults();
      if (val < 0) val = 0;
      switch (adsCh) {
        case 0: throttlePot1Raw = (uint16_t)val; break;
        case 1: throttlePot2Raw = (uint16_t)val; break;
        case 2: brakeRaw = val; brakePedal = (val > BRAKE_THRESHOLD); break;
      }
      adsCh = (adsCh + 1) % 3;
      ads.startADCReading(adsMux[adsCh], false);
      adsNext = micros() + 1300;
    }
  }

#ifdef UBLOX_GNSS
  myGNSS.checkUblox();
  myGNSS.checkCallbacks();
#endif

  fsm.run_machine();

  digitalWriteFast(3, LOW);
}
