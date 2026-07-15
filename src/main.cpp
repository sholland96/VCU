#include <Arduino.h>
#include <TeensyTimerTool.h>
#include <FlexCAN_T4.h>
#include <LIN_master_HardwareSerial.h>  // LIN master on Serial3 (RX3=pin7, TX3=pin8)
#include "pMBB32.h"
#include "defines.h"
#include "Fsm.h"

using namespace TeensyTimerTool;
IntervalTimer  t0;       // PIT channel — 10ms LDU torque command (highest priority)
PeriodicTimer t1(RTC);  // 62.5ms — PDU-8 keepalive, pMBB32 cell poll, RealDash update
PeriodicTimer t2(GPT1); // 200ms  — pMBB32 measurement broadcast, SIM100MOD, LIN valve
PeriodicTimer t3(GPT2); // 1000ms — heartbeat LED

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;// 500KHz - pMMB32, PDU-8  
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;// 500KHz - LDU, EVCC, SIM100MOD, IVT-MOD, EMP W29, keypad
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;// 1Mhz - Wireless Gateway, RealDash 

CAN_message_t msg1;//.buf[7] = {0x0D, 0x05, 0x05, 0x0D, 0, 0, 0, 0};
CAN_message_t msg2;
CAN_message_t msg3;
CAN_message_t PDUmsg1;
CAN_message_t PDUmsg2;
CAN_message_t LDUmsg;      // 0x201: pot (bytes 0-1) + canio (byte 4), sent every 10ms

// Used pins
#define LED_PIN 13
#define KL15R_PIN 2  // physical key position 1 (accessory) — wakes hardware, no FSM role

// FSM events — unique integers required by arduino-fsm
#define KL15_ON          1
#define KL15_OFF         2
#define DRIVE_ON         3
#define DRIVE_OFF        4
#define CHARGE_ON        5
#define CHARGE_OFF       6
#define PRECHARGE_OK     7
#define PRECHARGE_CHARGE 8  // pre-charge succeeded via EVCC path → go to Charge
#define PRECHARGE_FAIL   9
#define FAULT_EV         10
#define FAULT_CLEAR      11
#define TEMP_LOW         12
#define TEMP_HIGH        13
#define TEMP_OK          14
#define EXT_WAKE         15  // CAN/EVCC external wake without KL15R
#define AC_CHARGE_START  16  // AC plug detected in KL30C → close main contactors via PreCharge

#define KL30C_SLEEP_TIMEOUT_MS 60000UL  // sleep after 60 s of inactivity in KL30C

// DMAMEM places sleepMagic in OCRAM (.bss.dma, NOLOAD): valid writable RAM, not zeroed by CRT
// startup, survives AIRCR reset. Used in enterSleep() and setup() to detect CAN-wake vs key-on.
DMAMEM volatile uint32_t sleepMagic;
#define SLEEP_MAGIC_CAN_WAKE 0xC4A8B3E1UL

bool     extWakePending   = false; // set in setup() when CAN wake detected; consumed by FSM
uint32_t lastExtActivityMs = 0;    // last EVCC heartbeat or gateway frame (for KL30C timeout)
bool     kl30cKL15Rstate  = false; // tracks KL15R pin state inside KL30C for LED edge detect
bool     evccIsACSession  = false; // true when plug type is AC → VCU closes main contactors
bool     acReadyToDeliver = false; // set by AC_Control (0x601); EVCC grants AC power delivery

VCUStateEnum VCUstate = VCU_STATE_OFF;
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

/* t0 Callback — 10ms LDU fixed safety frame (IntervalTimer / PIT, highest priority)
 *
 * Sends the v5.32+ fixed control frame on 0x201 every 10ms.
 * OpenInverter shuts down after 5 consecutive invalid frames or 500ms silence.
 *
 * Bit packing (little-endian within each 32-bit word):
 *   buf[0]     = pot[7:0]
 *   buf[1]     = pot[11:8] | pot2[3:0]<<4
 *   buf[2]     = pot2[11:4]
 *   buf[3]     = canio[5:0] | ctr1[1:0]<<6
 *   buf[4]     = cruisespeed[7:0]          (0, not used)
 *   buf[5]     = cruisespeed[13:8] | ctr2[1:0]<<6   (ctr2 must == ctr1)
 *   buf[6]     = regenpreset[7:0]          (0, not used)
 *   buf[7]     = crc                       (0, controlcheck=0 on inverter)
 */
void callback_t0() {
  readThrottle(); // steps 1–4: updates LDUtorqueSetpoint (0-100)

  // Scale throttle to 12-bit pot value (potmax=4095 on inverter)
  uint16_t pot = (uint16_t)((uint32_t)LDUtorqueSetpoint * 4095 / 100);

  // Build canio 6-bit field
  uint8_t canio = 0;
  if (VCUstate == VCU_STATE_DRIVE) {
    canio |= LDU_CANIO_START;
    if (LDUdirection == LDU_DIR_FORWARD) canio |= LDU_CANIO_FORWARD;
    if (LDUdirection == LDU_DIR_REVERSE) canio |= LDU_CANIO_REVERSE;
  }
  if (brakePedal) canio |= LDU_CANIO_BRAKE;

  // Advance 2-bit sequence counter (must differ from the previous frame's value)
  LDUseqCounter = (LDUseqCounter + 1) & 0x03;
  uint8_t ctr = LDUseqCounter;

  // Pack data[0]: pot(12) | pot2(12) | canio(6) | ctr1(2)
  LDUmsg.buf[0] = pot & 0xFF;
  LDUmsg.buf[1] = ((pot >> 8) & 0x0F);          // pot2 = 0, upper nibble stays 0
  LDUmsg.buf[2] = 0;                             // pot2 high byte = 0
  LDUmsg.buf[3] = (canio & 0x3F) | (ctr << 6);

  // Pack data[1]: cruisespeed(14)=0 | ctr2(2) | regenpreset(8)=0 | crc(8)=0
  LDUmsg.buf[4] = 0;
  LDUmsg.buf[5] = ctr << 6;                      // ctr2 must equal ctr1
  LDUmsg.buf[6] = 0;
  LDUmsg.buf[7] = 0;                             // CRC disabled (controlcheck=0)
  can2.write(LDUmsg);
}

/* t1 Callback
* This runs every 62.5ms. 
* Tasks performed here:
*   1. Send PDU-8 driver settings every t1 period
*   2. Request min/max cells from pMBB32s
*   3. Update RealDash
*/
void callback_t1() {//send PDU-8 driver settings every t1 period
  //send PDU-8 driver settings every t1 period (125ms)
  can1.write(PDUmsg1);
  //delay(1);
  //can1.write(PDUmsg2);
  //delay(1);
  msg1.flags.extended = 1;
  switch (counter) {
    case 1:
      msg1.id = 0xCF0100; // request min/max cells from module 1
      msg1.len = 0;
      can1.write(msg1);
      break;
    case 2:
      msg1.id = 0xCF0200; // request min/max cells from module 2
      msg1.len = 0;
      can1.write(msg1);
      break;
    case 3:
      counter = 0;
      msg1.id = 0xCF0300; // request min/max cells from module 3
      msg1.len = 0;
      can1.write(msg1);
      break;
  }
  counter++;
  //delay(1);
  if (VCUstate != VCU_STATE_FAULT)//(VCUstate != VCU_STATE_OFF && VCUstate != VCU_STATE_FAULT)
    displayStatus(); // update RealDash

  // Advantics ADM-CS-EVCC Generic Power Modules protocol — send every 62.5ms
  {
    normalEndOfCharge = (highestCellV > 0u) && (highestCellV >= EVCC_CELL_V_FULL);

    // 0x60010 Power_Modules_Status: Present_V (LE uint16, 0.1V), Present_I (LE int16, 0.1A),
    //   Reserved (16b), System_Enable (8b 0/1), Insulation_R (8b, 2kΩ/bit from SIM100MOD Rp)
    uint16_t presentV = (uint16_t)(IVTpackVoltage / 100u);        // mV → 0.1V
    int16_t  presentA = (int16_t)(IVTpackCurrent / 100);           // mA → 0.1A
    uint8_t  insRes   = (SIM100MODRpKohms > 510u) ? 255u : (uint8_t)(SIM100MODRpKohms / 2u);
    msg2.flags.extended = 1;
    msg2.id   = EVCC_PWR_STATUS;
    msg2.len  = 8;
    msg2.buf[0] = presentV & 0xFF;
    msg2.buf[1] = (presentV >> 8) & 0xFF;
    msg2.buf[2] = (uint8_t)(presentA & 0xFF);
    msg2.buf[3] = (uint8_t)((uint16_t)presentA >> 8);
    msg2.buf[4] = 0;
    msg2.buf[5] = 0;
    msg2.buf[6] = EVCCsystemEnable ? 1u : 0u;
    msg2.buf[7] = insRes;
    can2.write(msg2);

    // 0x60011 Power_Modules_Limits: Max_Voltage (LE uint16, 0.1V), Max_Current (LE int16, 0.1A)
    msg2.id  = EVCC_PWR_LIMITS;
    msg2.len = 8;
    msg2.buf[0] = EVCC_MAX_VOLTAGE_x10 & 0xFF;
    msg2.buf[1] = (EVCC_MAX_VOLTAGE_x10 >> 8) & 0xFF;
    msg2.buf[2] = EVCC_MAX_CURRENT_x10 & 0xFF;
    msg2.buf[3] = (EVCC_MAX_CURRENT_x10 >> 8) & 0xFF;
    msg2.buf[4] = 0;
    msg2.buf[5] = 0;
    msg2.buf[6] = 0;
    msg2.buf[7] = 0;
    can2.write(msg2);

    // 0x60012 Sequence_Control (3 bytes):
    //   byte 0: b0=Start_Charge_Authorisation, b1=CHAdeMO_Start_Button
    //   byte 1: b0=CCS_Authorisation_Done, b1=CCS_Authorisation_Valid, b2=Charge_Parameters_Done
    //   byte 2: b0=User_Stop_Button (asserted when battery is full)
    bool chademoBtn = EVCCsystemEnable && (EVCCplugType == EVCC_PLUG_CHADEMO);
    msg2.id  = EVCC_SEQ_CTRL;
    msg2.len = 3;
    msg2.buf[0] = (EVCCsystemEnable ? 0x01u : 0x00u)  // Start_Charge_Authorisation
               | (chademoBtn       ? 0x02u : 0x00u);  // CHAdeMO_Start_Button
    msg2.buf[1] = EVCCsystemEnable ? 0x07u : 0x00u;   // CCS_Done | CCS_Valid | Params_Done
    msg2.buf[2] = normalEndOfCharge ? 0x01u : 0x00u;  // User_Stop_Button
    can2.write(msg2);

    // 0x611 AC_Status (v2.5 standard-ID): Ready_To_Charge bit 0
    // Ready only when EVCC has granted delivery, VCU is in Charge state, and battery is not full.
    msg2.flags.extended = 0;
    msg2.id  = EVCC_AC_STATUS;
    msg2.len = 1;
    msg2.buf[0] = (evccIsACSession && EVCCsessionActive
                   && VCUstate == VCU_STATE_CHARGE
                   && acReadyToDeliver && !normalEndOfCharge) ? 0x01u : 0x00u;
    can2.write(msg2);
    msg2.flags.extended = 1;
  }
}//end of callback_cells_pdu()

// Ghost SA flag: set by can1Sniff ISR when frames arrive with modNum > 3.
// Indicates one module's TOTAL_ICS was corrupted during init — it broadcasts on SA+0..SA+7.
// SA=1..3 frames still arrive so stale counters never trip, but cell data is garbage.
static volatile bool     pMBB32ghostSA   = false;
// Bitmask of frame types (ft=01..0C → bits 0..11) seen per module since last check.
static volatile uint16_t pMBB32ftSeen[3] = {0, 0, 0};

/* t2 Callback
* This runs every t2 period (200ms).
* Tasks performed here:
*   1. Request all cell and temperature measurements from pMBB32s
*   2. Request min/max cell voltages from pMBB32s
*   3. Check for stale pMBB32 CAN and restart if necessary
*   4. Read isolation state from SIM100MOD
*/
void callback_t2() {
  // request all cell and temperature measurements every t2 period
  msg1.id = 0xFF0000;
  msg1.flags.extended = 1;
  msg1.len = 0;
  can1.write(msg1);  
  //delay(1);
  // Invalidate data from any pMBB32 that has not responded within 400ms
  {
    uint32_t now = millis();
    if (now - lastUpdatePMBB1 > 400) { minCellV1 = 0xFFFF; maxCellV1 = 0; }
    if (now - lastUpdatePMBB2 > 400) { minCellV2 = 0xFFFF; maxCellV2 = 0; }
    if (now - lastUpdatePMBB3 > 400) { minCellV3 = 0xFFFF; maxCellV3 = 0; }
    lowestCellV  = std::min({minCellV1, minCellV2, minCellV3});
    highestCellV = std::max({maxCellV1, maxCellV2, maxCellV3});
  }
  // Increment stale counters every t2 period; reset in can1Sniff() on cell voltage frames.
  // uint8_t wraps at 255 — clamp to avoid silent overflow.
  if (pMBB32stale1 < 255) pMBB32stale1++;
  if (pMBB32stale2 < 255) pMBB32stale2++;
  if (pMBB32stale3 < 255) pMBB32stale3++;

  if(pMBB32stale1 > pMBB32staleMax) pMBB32staleMax = pMBB32stale1;
  if(pMBB32stale2 > pMBB32staleMax) pMBB32staleMax = pMBB32stale2;
  if(pMBB32stale3 > pMBB32staleMax) pMBB32staleMax = pMBB32stale3;



#ifdef PMBBB32_DEBUG
  {
    static uint8_t dbgTick = 0;
    if (++dbgTick >= 5) {
      dbgTick = 0;
      Serial.printf("pMBB32 stale: #1=%u  #2=%u  #3=%u\n",
                    pMBB32stale1, pMBB32stale2, pMBB32stale3);
    }
  }
#endif

  // Stale recovery: shutdown → 2 s → wake, escalating to PDU CH2 power cycle after 3 retries.
  // Skip for the first 1 s to allow modules time to boot after PDU-8 enables their 12V rail.
  static uint8_t startupGrace = 5; // 5 × 200 ms = 1 s
  if (startupGrace > 0) {
    if (--startupGrace == 0) {
      pMBB32stale1 = pMBB32stale2 = pMBB32stale3 = 0;
    }
  }
  if (startupGrace == 0) {
    static struct {
      uint8_t  *stale;
      uint32_t  id;
      uint8_t   num;
      uint8_t   phase;      // 0=idle, 1=awaiting 2s after shutdown
      uint32_t  phaseTime;
      uint8_t   retryCount;
    } mods[] = {
      {&pMBB32stale1, 0xAF0100, 1, 0, 0, 0},
      {&pMBB32stale2, 0xAF0200, 2, 0, 0, 0},
      {&pMBB32stale3, 0xAF0300, 3, 0, 0, 0},
    };

    // CH2 power cycle — state 0=idle, 1=CH2 off (1s), 2=CH2 on (3s boot wait)
    static struct { uint8_t state; uint32_t t; } ch2 = {0, 0};

    msg1.flags.extended = 1;

    // Ghost SA detection with 2 s debounce: TOTAL_ICS corruption makes one module broadcast on
    // SA+0..SA+7. Require sustained detection before cycling CH2 — a single transient frame
    // (e.g. residual bus traffic during boot wait) must not trigger a power cycle.
    {
      static uint32_t ghostSince = 0;
      noInterrupts(); bool ghostNow = pMBB32ghostSA; pMBB32ghostSA = false; interrupts();
      if (ghostNow) {
        if (ghostSince == 0) ghostSince = millis();
      } else {
        ghostSince = 0;
      }
      if (ghostSince && ch2.state == 0 && millis() - ghostSince >= 2000) {
        ghostSince = 0;
        noInterrupts(); pMBB32ftSeen[0] = pMBB32ftSeen[1] = pMBB32ftSeen[2] = 0; interrupts();
        Serial.println("pMBB32: ghost SA detected (TOTAL_ICS corruption) — cycling PDU CH2");
        PDUmsg1.buf[1] = 0;
        ch2.state = 1; ch2.t = millis();
        for (auto &mm : mods) { *mm.stale = 0; mm.phase = 0; mm.retryCount = 0; }
      }
    }

    // Incomplete frame-set detection: numChannels corruption suppresses ft=03/04/09/0A
    // (cells 9-16). The module still sends ft=01/02 so stale counters reset normally, making
    // the corruption silent. Detect by checking whether ft=01 arrived but ft=03 did not over
    // a 5 s window, then force the stale counter so recovery sends shutdown→wake.
    static uint32_t nextFtCheck = 0;
    if (ch2.state == 0 && millis() >= nextFtCheck) {
      nextFtCheck = millis() + 5000;
      noInterrupts();
      uint16_t snap[3] = {pMBB32ftSeen[0], pMBB32ftSeen[1], pMBB32ftSeen[2]};
      pMBB32ftSeen[0] = pMBB32ftSeen[1] = pMBB32ftSeen[2] = 0;
      interrupts();
      for (uint8_t i = 0; i < 3; i++) {
        bool seenFt01 = snap[i] & (1u << 0);  // module is live
        bool seenFt03 = snap[i] & (1u << 2);  // cells 9-12 present
        if (seenFt01 && !seenFt03) {
          Serial.printf("pMBB32 #%u: ft=03 absent (numChannels corrupted, seen=0x%03X)"
                        " — forcing recovery\n", i + 1, (unsigned)snap[i]);
          uint8_t *s = (i == 0 ? &pMBB32stale1 : i == 1 ? &pMBB32stale2 : &pMBB32stale3);
          *s = 255;
          nextFtCheck = millis() + 30000;  // 30 s gap; recovery takes 2-4 s + module boot
          break;
        }
      }
    }

    if (ch2.state == 1 && millis() - ch2.t >= 1000) {
      Serial.println("PDU CH2 restored — waiting for pMBB32 boot");
      PDUmsg1.buf[1] = 0x05;
      ch2.state = 2; ch2.t = millis();
    } else if (ch2.state == 2 && millis() - ch2.t >= 1000) {
      Serial.println("pMBB32 power cycle complete — waking all modules");
      for (auto &m : mods) {
        msg1.id = m.id; msg1.len = 3;
        msg1.buf[0] = wakeup; msg1.buf[1] = channelCount16; msg1.buf[2] = numberOfDevices;
        can1.write(msg1);
      }
      pMBB32stale1 = pMBB32stale2 = pMBB32stale3 = 0;
      noInterrupts();
      pMBB32ghostSA = false;  // discard any frames seen during the boot wait
      pMBB32ftSeen[0] = pMBB32ftSeen[1] = pMBB32ftSeen[2] = 0;
      interrupts();
      nextFtCheck = millis() + 10000;  // give modules 10 s to boot before checking ft=03
      for (auto &m : mods) { m.phase = 0; m.retryCount = 0; }
      ch2.state = 0;
    }

    bool anyWoke = false;
    if (ch2.state == 0) {
      for (auto &m : mods) {
        if (m.phase == 0 && *m.stale > 5) {
          if (m.retryCount >= 3) {
            Serial.printf("pMBB32 #%u exhausted retries — cycling PDU CH2\n", m.num);
            PDUmsg1.buf[1] = 0;
            ch2.state = 1; ch2.t = millis();
            for (auto &mm : mods) { *mm.stale = 0; mm.phase = 0; mm.retryCount = 0; }
            break;
          }
          *m.stale = 0;
          Serial.printf("pMBB32 #%u stale — sending shutdown\n", m.num);
          msg1.id = m.id; msg1.len = 1; msg1.buf[0] = shutdown;
          can1.write(msg1);
          m.phase = 1; m.phaseTime = millis();
        } else if (m.phase == 1 && millis() - m.phaseTime >= 500) {
          Serial.printf("pMBB32 #%u — sending wake\n", m.num);
          msg1.id = m.id; msg1.len = 3;
          msg1.buf[0] = wakeup; msg1.buf[1] = channelCount16; msg1.buf[2] = numberOfDevices;
          can1.write(msg1);
          *m.stale = 0; m.retryCount++; m.phase = 0; anyWoke = true;
        }
      }
    }
    if (anyWoke) { msg1.id = 0xFF0000; msg1.len = 0; can1.write(msg1); }
  }

  msg2.id = 0xA100101;//send SIM100MOD Request Isolation State command
  msg2.flags.extended = 1;
  msg2.len = 1;
  msg2.buf[0] = 0xE0;//request isolation state
  can2.write(msg2);
  //delay(2);
  /*
  msg2.buf[0] = 0xE1;//request isolation resistances
  can2.write(msg2);
  delay(2);
  msg2.buf[0] = 0xE2;//request isolation capacitances
  can2.write(msg2);
  delay(2);
  msg2.buf[0] = 0xE3;//request Vp and Vn voltages
  can2.write(msg2);
  delay(2);
  msg2.buf[0] = 0xE4;//request battery voltage
  can2.write(msg2);
  delay(2);
  msg2.buf[0] = 0xE5;//request error flags
  can2.write(msg2);
  delay(2); */
  msg2.buf[0] = 0x80;//request temperature
  can2.write(msg2);
  //delay(1);

  linReadValve();//poll BMW changeover valve over LIN (Serial3)

  // EMP WP29 inverter cooling pump (CAN2) — must be sent ≥ 1 Hz; t2 fires every 200 ms.
  msg2.id             = EMP_WP29_CMD_ID;
  msg2.flags.extended = 1;
  msg2.len            = 8;
  msg2.buf[0]         = (invPumpSetpoint > 0) ? 0xFD : 0xFC;
  msg2.buf[1]         = 0xFF;
  msg2.buf[2]         = 0xFF;
  msg2.buf[3]         = (uint8_t)(invPumpSetpoint * 2);
  msg2.buf[4]         = 0xFF;
  msg2.buf[5]         = 0xFF;
  msg2.buf[6]         = 0xFF;
  msg2.buf[7]         = 0xFF;
  can2.write(msg2);

  // EMP WP29 battery cooling pump (CAN1) — same protocol, separate bus
  msg1.id             = EMP_WP29_CMD_ID;
  msg1.flags.extended = 1;
  msg1.len            = 8;
  msg1.buf[0]         = (battPumpSetpoint > 0) ? 0xFD : 0xFC;
  msg1.buf[1]         = 0xFF;
  msg1.buf[2]         = 0xFF;
  msg1.buf[3]         = (uint8_t)(battPumpSetpoint * 2);
  msg1.buf[4]         = 0xFF;
  msg1.buf[5]         = 0xFF;
  msg1.buf[6]         = 0xFF;
  msg1.buf[7]         = 0xFF;
  can1.write(msg1);

}//end of callback_t2()

/* t3 Callback
* This runs every 1000ms. 
*/
void callback_t3() {
  //ReadDigitalStatuses();
  //ReadAnalogStatuses();
  //groundSpeed = myGNSS.getGroundSpeed() * 0.00223694;//convert mm/s to mph
  //GPSaltitude = myGNSS.getAltitudeMSL() / 3300;//feet
  digitalToggleFast(LED_BUILTIN);
  Serial.printf("pMBB32 stale=%u/%u/%u  lo=%u hi=%u mV\n",
                pMBB32stale1, pMBB32stale2, pMBB32stale3,
                lowestCellV, highestCellV);
}//end of callback1000ms()

/* CAN1 Receiver
* This function processes messages received on CAN1.
*   The pMBB32s are configured to broadcast cell voltages and temperatures at 100ms rate.
*/
void can1Sniff(const CAN_message_t &msg) {
  digitalWriteFast(4, HIGH);

  if ((msg.id & 0x00000F00) == 0x00000E00) {//received 0x18FF0Eyy    
    switch (msg.id) {
      case 0x18FF0E01:
        minCellV1 = (msg.buf[minCellHighByte]*256) + msg.buf[minCellLowByte];
        minCell1 = msg.buf[0];
        maxCellV1 = (msg.buf[maxCellHighByte]*256) + msg.buf[maxCellLowByte];
        maxCell1 = msg.buf[1];
        lastUpdatePMBB1 = millis();
        break;
      case 0x18FF0E02:
        minCellV2 = (msg.buf[minCellHighByte]*256) + msg.buf[minCellLowByte];
        minCell2 = msg.buf[0];
        maxCellV2 = (msg.buf[maxCellHighByte]*256) + msg.buf[maxCellLowByte];
        maxCell2 = msg.buf[1];
        lastUpdatePMBB2 = millis();
        break;
      case 0x18FF0E03:
        minCellV3 = (msg.buf[minCellHighByte]*256) + msg.buf[minCellLowByte];
        minCell3 = msg.buf[0];
        maxCellV3 = (msg.buf[maxCellHighByte]*256) + msg.buf[maxCellLowByte];
        maxCell3 = msg.buf[1];
        lastUpdatePMBB3 = millis();
        break;
    }
    // Calculate the lowest of minCellV1, minCellV2, and minCellV3
    lowestCellV = std::min({minCellV1, minCellV2, minCellV3});
    if(lowestCellV == minCellV1) {
      lowestCell = minCell1;
      //lowestModule = temp;
    }
    if(lowestCellV == minCellV2) {
      lowestCell = minCell2+32;
      //lowestModule = temp;
    }
    if(lowestCellV == minCellV3) {
      lowestCell = minCell3+64;
      //lowestModule = temp;
    }
    // Calculate the highest of maxCellV1, maxCellV2, and maxCellV3
    highestCellV = std::max({maxCellV1, maxCellV2, maxCellV3});
    if(highestCellV == maxCellV1) {
      highestCell = maxCell1;
      //highestModule = temp;
    }
    if(highestCellV == maxCellV2) {
      highestCell = maxCell2+32;
      //highestModule = temp;
    }
    if(highestCellV == maxCellV3) {
      highestCell = maxCell3+64;
      //highestModule = temp;
    }
  
    /*
    temp = (msg.buf[minCellHighByte]*256) + msg.buf[minCellLowByte];
    if(temp < minCellV){
      minCell = msg.buf[0];
      minCellV = temp;
      minModule = msg.id & 0x0000000F;
    } else {
      
    }
    temp = (msg.buf[maxCellHighByte]*256) + msg.buf[maxCellLowByte];
    if(temp > maxCellV){
      maxCell = msg.buf[1];
      maxCellV = temp;
      maxModule = msg.id & 0x0000000F;
    } else {
      
    }*/

    /*
    temp = (msg.buf[minCellHighByte]*256) + msg.buf[minCellLowByte];
    if(minCellV == 0){
      minCell = msg.buf[0];
      minCellV = temp;
      minModule = msg.id & 0x0000000F;
    } else {
      if(temp < minCellV){
        minCell = msg.buf[0];
        minCellV = temp;
        minModule = msg.id & 0x0000000F;
      }
    }
    temp = (msg.buf[maxCellHighByte]*256) + msg.buf[maxCellLowByte];
    if(maxCellV == 0){
      maxCell = msg.buf[0];
      maxCellV = temp;
      maxModule = msg.id & 0x0000000F;
    } else {
      if(temp > maxCellV){
        maxCell = msg.buf[0];
        maxCellV = temp;
        maxModule = msg.id & 0x0000000F;
      }
    } */
  }//end 0x18FF0Eyy receiver

  // Track pMBB32 cell/temp frames: reset stale, record frame-type coverage, detect ghost SAs.
  //
  // When a module's TOTAL_ICS or numChannels variable is corrupted during init (by a PDU-8
  // status frame arriving in the pMBB32 wake handler's ~8ms init window), two failure modes
  // occur:
  //
  //   TOTAL_ICS wrong (e.g. 16): module broadcasts on SA+0..SA+7. SA=1..3 frames still
  //   arrive, so stale counters never trip, but data is garbage and bus is flooded with
  //   96 frames per trigger. These show up as modNum=4..8 "ghost" frames.
  //
  //   numChannels wrong (e.g. 0): ft=03/04/09/0A (cells 9-16) are suppressed because the
  //   firmware gates them on (numChannels > 8). ft=01/02 still arrive, stale counters still
  //   reset, but cell data for cells 9-32 is absent and undetected.
  {
    uint8_t ft  = (msg.id >> 8) & 0xFF;
    uint8_t mod = msg.id & 0xFF;
    if ((msg.id >> 16) == 0x18FF && ft >= 0x01 && ft <= 0x0C) {
      if (mod >= 1 && mod <= 3) {
        pMBB32ftSeen[mod - 1] |= (1u << (ft - 1));
        switch (mod) {
          case 1: pMBB32stale1 = 0; break;
          case 2: pMBB32stale2 = 0; break;
          case 3: pMBB32stale3 = 0; break;
        }
      } else if (mod >= 4) {
        pMBB32ghostSA = true;
      }
    }
  }

#ifdef PMBBB32_DEBUG
  // Log every frame from the pMBB32 0x18FFxxxx family so we can see which module
  // is (or isn't) sending, and catch unexpected ID patterns.
  if ((msg.id >> 16) == 0x18FF) {
    Serial.printf("CAN1 0x%08X [%d]", msg.id, msg.len);
    for (uint8_t i = 0; i < msg.len; i++) Serial.printf(" %02X", msg.buf[i]);
    Serial.println();
  }
#endif

  // EMP WP29 battery cooling pump (CAN1) — same protocol as inverter pump on CAN2
  switch (msg.id) {
    case EMP_WP29_STATUS1:
      battPumpStatus = (msg.buf[0] >> 2) & 0x0F;
      battPumpSpeed  = ((uint16_t)msg.buf[2] << 8) | msg.buf[1];
      battPumpFaults = msg.buf[7] & 0x0F;
      break;
    case EMP_WP29_STATUS3:
      break;
  }

  digitalWriteFast(4, LOW);
}// end of can1Sniff(const CAN_message_t &msg)

/* CAN2 Receiver
* This function processes messages received on CAN2.
* 
* The IVT-S-1K-U3-I-CAN1-12V is configured for the following broadcast rates:
*   0x621: Current - 20ms
*   0x622: Pack+(U1) - 60ms
*   0x623: Pre-charge+(U2) - 60ms
*   0x624: U3 - 60ms
*   0x625: Temperature - 200ms
*   0x626: Power - 30ms
*   0x627: Coulomb Count (As) - 30ms
*   0x628: Energy (Wh) - 30ms
*
*   The keypad automatically broadcasts status at 100ms rate.
* 
*   The SIM100MOD broadcasts isolation state at 100ms rate.
*/
void can2Sniff(const CAN_message_t &msg) {
  digitalWriteFast(5, HIGH);
  switch (msg.id) {  
    case 0x621://IVT-S Current
      IVTpackCurrent = (msg.buf[2]<<24) | (msg.buf[3]<<16) | (msg.buf[4]<<8) | msg.buf[5];
      break;
    case 0x622:
      IVTpackVoltage = (msg.buf[2]<<24) | (msg.buf[3]<<16) | (msg.buf[4]<<8) | msg.buf[5];
      break;
    case 0x623:
      IVTpreChargeV = (msg.buf[2]<<24) | (msg.buf[3]<<16) | (msg.buf[4]<<8) | msg.buf[5];
      break;
    case 0x624:
      IVTvoltage3 = (msg.buf[2]<<24) | (msg.buf[3]<<16) | (msg.buf[4]<<8) | msg.buf[5];
      break;
    case 0x625:
      IVTtemp = (msg.buf[2]<<24) | (msg.buf[3]<<16) | (msg.buf[4]<<8) | msg.buf[5];
      break;
    case 0x626:
      IVTpower = (msg.buf[2]<<24) | (msg.buf[3]<<16) | (msg.buf[4]<<8) | msg.buf[5];
      break;
    case 0x627:
      IVTcoulombCounter = (msg.buf[2]<<24) | (msg.buf[3]<<16) | (msg.buf[4]<<8) | msg.buf[5];
      break;
    case 0x628:
      IVTenergyCounter = (msg.buf[2]<<24) | (msg.buf[3]<<16) | (msg.buf[4]<<8) | msg.buf[5];
      break;
    case 0x18EFFF21://CAN keypad
      if(msg.buf[2] == 0x01){ // Key Contact state (event-driven, enabled by default)
        bool pressed = (msg.buf[4] == 0x01);
        switch (msg.buf[3]) { // key number 1-8
          case 1://Park
            button_0x01_state = pressed ? HIGH : LOW;
            if (pressed) { LDUdirection = LDU_DIR_STOP; Serial.println("BTN: Park"); }
            break;
          case 2://Reverse
            button_0x02_state = pressed ? HIGH : LOW;
            if (pressed) { LDUdirection = LDU_DIR_REVERSE; Serial.println("BTN: Reverse"); }
            break;
          case 3://Neutral
            button_0x04_state = pressed ? HIGH : LOW;
            if (pressed) { LDUdirection = LDU_DIR_NEUTRAL; Serial.println("BTN: Neutral"); }
            break;
          case 4://Drive
            button_0x08_state = pressed ? HIGH : LOW;
            if (pressed) { LDUdirection = LDU_DIR_FORWARD; Serial.println("BTN: Drive"); }
            break;
          case 5: // KL15 — one-shot start, Park exits
            if (pressed && !button_0x10_state && !KL15state) {
              button_0x10_state = HIGH;
              KL15state = true;
              Serial.println("KL15: ON");
            } else if (!pressed) {
              button_0x10_state = LOW;
            }
            break;
          case 6://Speed Mode
            button_0x20_state = pressed ? HIGH : LOW;
            if (pressed) Serial.println("BTN: Speed Mode");
            break;
          case 7://AUX
            button_0x40_state = pressed ? HIGH : LOW;
            if (pressed) Serial.println("BTN: AUX");
            break;
          case 8://Drive Mode
            button_0x80_state = pressed ? HIGH : LOW;
            if (pressed) Serial.println("BTN: Drive Mode");
            break;
          default:
            Serial.printf("BTN: unknown key %u\n", msg.buf[3]);
            break;
        }
      }
      break;
    case 0xA100100://SIM100MOD Isolation Monitor
      switch (msg.buf[0]) {
        case 0xE0:
          SIM100MODohmsPerVolt = (msg.buf[2]<<8) | msg.buf[3];//Ω/V
          break;
        case 0xE1:
          SIM100MODRpKohms = (msg.buf[2]<<8) | msg.buf[3];//kΩ positive
          SIM100MODRnKohms = (msg.buf[5]<<8) | msg.buf[6];//kΩ negative
          break;
        case 0xE2:
          SIM100MODCpnF = (msg.buf[2]<<8) | msg.buf[3];//Cp nF
          SIM100MODCnnF = (msg.buf[5]<<8) | msg.buf[6];//Cn nF
          break;
        case 0x0E3:
          SIM100MODVp = (msg.buf[2]<<8) | msg.buf[3];//Vp
          SIM100MODVn = (msg.buf[5]<<8) | msg.buf[6];//Vn
          break;
        case 0xE4:
          SIM100MODVb = (msg.buf[2]<<8) | msg.buf[3];//Vb
          SIM100MODVbMax = (msg.buf[5]<<8) | msg.buf[6];//Vb max
          break;
        case 0xE5:
          SIMM100MODerrorFlags = msg.buf[1];//SIM100MOD error flags
          break;
        case 0x80:
          SIM100MODtemp = (msg.buf[3]<<8) | msg.buf[4];//
          break;
      }
      
      break;
    case 0x000A0610:

      break;

    /* OpenInverter Tesla LDU V2 --------------------------------
     * TODO: verify CAN IDs match device configuration page.
     * Default stm32-sine firmware IDs shown below.
     */
    case 0x19A://LDU status: speed, torque, temperatures, DC voltage
      LDUrpm          = (int32_t)((int16_t)((msg.buf[0]<<8) | msg.buf[1]));// TODO: confirm scaling
      LDUtorque       = (int16_t)((msg.buf[2]<<8) | msg.buf[3]);           // TODO: confirm scaling
      LDUmotorTemp    = (int16_t)((msg.buf[4]<<8) | msg.buf[5]);           // TODO: confirm byte map
      LDUinverterTemp = 0;                                                  // TODO: parse from correct byte
      LDUdcVoltage    = 0;                                                  // TODO: parse from correct byte
      LDUstatus       = msg.buf[6];
      break;
    case 0x55A://LDU faults
      LDUfaults = (msg.buf[0]<<8) | msg.buf[1];// TODO: confirm byte map
      break;

    /* EMP WP29-12V-CV-A Water Pump -------------------------------------------
     * Motor Status Message 1: ID = 0x18FF03{pump_addr}, 1 Hz — confirmed via DBC
     * Motor Status Message 3: ID = 0x18FF24{pump_addr}, 100 ms (voltage/current/HVIL)
     * EMP proprietary protocol — 9980001068 Rev. N
     */
    case EMP_WP29_STATUS1:
      // byte 0: bits[1:0]=direction, bits[5:2]=controller_status, bits[7:6]=command_src
      invPumpStatus = (msg.buf[0] >> 2) & 0x0F;
      // bytes 1-2: measured speed, little-endian uint16, 0.5 rpm/bit
      invPumpSpeed  = ((uint16_t)msg.buf[2] << 8) | msg.buf[1];
      // byte 7: bits[1:0]=service_indicator, bits[3:2]=operation_status
      invPumpFaults = msg.buf[7] & 0x0F;
      break;
    case EMP_WP29_STATUS3:
      // bytes 0-1: motor voltage (little-endian, 0.05 V/bit)
      // bytes 2-3: motor current (little-endian, 0.05 A/bit, offset -1600)
      // byte 4 bit 0: HVIL status (0=OK, 1=open)
      break;

    /* Advantics ADM-CS-EVCC Generic Power Modules protocol ---------------
     * EVCC drives DCFC contactors autonomously; VCU provides measurements,
     * limits, and authorisation. All IDs are 29-bit extended.
     */
    case EVCC_NEW_SESSION:  // 0x68001 — plug detected, EVSE parameters
      // buf[0]=Communication_Protocol, buf[1]=Plug_and_pins, buf[2:3]=EV_Max_V (0.1V LE),
      // buf[4:5]=EV_Max_I (0.1A LE), buf[6]=Battery_Capacity (2kWh/bit), buf[7]=SoC (%)
      EVCCplugType      = msg.buf[1]; // 0=CCS_DC_Core, 1=CCS_DC_Extended, 2=CHAdeMO
      EVCCsessionActive = true;
      EVCCsystemEnable  = true;
      EVCCemergencyStop = false;
      evccIsACSession   = !EVCC_PLUG_IS_DC(EVCCplugType); // AC plug → VCU closes main contactors
      chargeMode        = evccIsACSession; // true routes PreCharge → Charge (not Idle)
      // DC: EVCC handles its own pre-charge and contactors autonomously; VCU stays in KL30C.
      // AC: check_KL30C() detects evccIsACSession and fires AC_CHARGE_START → PreCharge → Charge.
      // TODO: update EVCC_PLUG_IS_DC macro when AC plug type values are confirmed.
      break;
    case EVCC_CHARGING_LOOP:  // 0x68005 — active charge targets from EVSE
      // buf[0:1]=Target_Voltage (0.1V LE), buf[2:3]=Target_Current (0.1A s LE), buf[4]=SoC
      // EVCC forwards these targets to the EVSE; VCU monitors only
      break;
    case EVCC_EMERG_STOP:  // 0x68006 — emergency condition from EVSE or EVCC
      EVCCemergencyStop = true;
      EVCCsessionActive = false;
      EVCCsystemEnable  = false;
      evccIsACSession   = false;
      chargeMode        = false;
      acReadyToDeliver  = false;
      // TODO: fsm.trigger(FAULT_EV) when charge FSM states are split
      break;
    case EVCC_SESSION_END:  // 0x68007 — charge session finished normally
      EVCCsessionActive = false;
      EVCCsystemEnable  = false;
      evccIsACSession   = false;
      chargeMode        = false;
      acReadyToDeliver  = false;
      // TODO: fsm.trigger(CHARGE_OFF) when charge FSM states are split
      break;
    case EVCC_CTRL_STATUS:  // 0x68009 — EVCC heartbeat (200ms timeout)
      EVCCstage         = msg.buf[0]; // 0=Booting … 8=Charging … 10=Finished
      lastExtActivityMs = millis();   // heartbeat resets KL30C inactivity timer
      break;
    case EVCC_INS_TEST:     // 0x68002 — insulation test in progress
    case EVCC_PRECHARGE:    // 0x68003 — precharge in progress
    case EVCC_STATUS_CHANGE: // 0x68004 — charge status change notification
      break;

  }

  // Advantics v2.5 PEV protocol — standard 11-bit IDs
  switch (msg.id) {
    case EVCC_EVSE_INFO:  // 0x600 — EVSE_Information: byte0=Stage, byte1=Protocol, byte2=Pins
      // Fallback session detection for v2.5: used when EVCC sends this instead of / in addition to 0x68001.
      // Pins: 1=CCS_AC, 2=CCS_AC_1PH, 3=CCS_AC_3PH, 4=CCS_DC_Core, 5=CCS_DC_Extended
      if (!EVCCsessionActive && EVCC_PINS_IS_AC(msg.buf[2])) {
        EVCCsessionActive = true;
        EVCCsystemEnable  = true;
        EVCCemergencyStop = false;
        evccIsACSession   = true;
        chargeMode        = true;
      }
      break;
    case EVCC_AC_CTRL:  // 0x601 — AC_Control: Ready_To_Deliver_Power (bit 0)
      acReadyToDeliver = (msg.buf[0] & 0x01u) != 0;
      break;
  }
  digitalWriteFast(5, LOW);
}//end of can2Sniff(const CAN_message_t &msg)

void can3Sniff(const CAN_message_t &msg) {
  lastExtActivityMs = millis(); // any gateway frame resets KL30C inactivity timer
  switch (msg.id) {
    case 500:
      break;
    default:
      break;
  }
}//end of can3Sniff(const CAN_message_t &msg)

void initCAN (int CAN1baud, int CAN2baud, int CAN3baud) {
  msg1.flags.extended = 1;
  msg1.len = 8;
  msg1.buf[0] = 0;
  msg1.buf[1] = 0;
  msg1.buf[2] = 0;
  msg1.buf[3] = 0;
  msg1.buf[4] = 0;
  msg1.buf[5] = 0;
  msg1.buf[6] = 0;
  msg1.buf[7] = 0;

  msg2.flags.extended = 1;
  msg2.len = 8;
  msg2.buf[0] = 0;
  msg2.buf[1] = 0;
  msg2.buf[2] = 0;
  msg2.buf[3] = 0;
  msg2.buf[4] = 0;
  msg2.buf[5] = 0;
  msg2.buf[6] = 0;
  msg2.buf[7] = 0;

  msg3.flags.extended = 1;
  msg3.len = 8;
  msg3.buf[0] = 0;
  msg3.buf[1] = 0;
  msg3.buf[2] = 0;
  msg3.buf[3] = 0;
  msg3.buf[4] = 0;
  msg3.buf[5] = 0;
  msg3.buf[6] = 0;
  msg3.buf[7] = 0;

  PDUmsg1.flags.extended = 1;
  PDUmsg1.len = 8;
  PDUmsg1.buf[0] = 0;
  PDUmsg1.buf[1] = 0;
  PDUmsg1.buf[2] = 0;
  PDUmsg1.buf[3] = 0;
  PDUmsg1.buf[4] = 0;
  PDUmsg1.buf[5] = 0;
  PDUmsg1.buf[6] = 0;
  PDUmsg1.buf[7] = 0;

  PDUmsg2.flags.extended = 1;
  PDUmsg2.len = 8;
  PDUmsg2.buf[0] = 0;
  PDUmsg2.buf[1] = 0;
  PDUmsg2.buf[2] = 0;
  PDUmsg2.buf[3] = 0;
  PDUmsg2.buf[4] = 0;
  PDUmsg2.buf[5] = 0;
  PDUmsg2.buf[6] = 0;
  PDUmsg2.buf[7] = 0;

  LDUmsg.id = LDU_CMD_ID;     // 0x201 — fixed safety frame, 10ms
  LDUmsg.flags.extended = 0;
  LDUmsg.len = 8;
  memset(LDUmsg.buf, 0, 8);

  can1.begin();
  can1.setBaudRate(CAN1baud);
  can1.setMaxMB(16);
  can1.enableFIFO();
  can1.enableFIFOInterrupt();
  //can1.setFIFOFilter(REJECT_ALL);
  //can1.setFIFOFilter(1, 0x18FF, EXT); // Set filter1 to allow EXTENDED CAN ID 0x456 to be collected by FIFO
  can1.onReceive(FIFO, can1Sniff);
  //can1.mailboxStatus();

  can2.begin();
  can2.setBaudRate(CAN2baud);
  can2.setMaxMB(16);
  can2.enableFIFO();
  can2.enableFIFOInterrupt();
  can2.onReceive(FIFO, can2Sniff);
  //can2.mailboxStatus();

  can3.begin();
  can3.setBaudRate(CAN3baud);
  can3.setMaxMB(16);
  can3.enableFIFO();
  can3.enableFIFOInterrupt();
  can3.onReceive(FIFO, can3Sniff);
  //can3.mailboxStatus();
}//end of initCAN (int CAN1baud, int CAN2baud, int CAN3baud)

void wakepMBB32(){
  msg1.flags.extended = 1;

  static const uint32_t modIds[] = {0xAF0100, 0xAF0200, 0xAF0300};

  for (uint8_t i = 0; i < 3; i++) {
    msg1.id = modIds[i];
    msg1.len = 3;
    msg1.buf[0] = wakeup;           // 0x01
    msg1.buf[1] = channelCount16;   // 0x10: 16-cell module
    msg1.buf[2] = numberOfDevices;  // 0x02
    can1.write(msg1);
    delay(10);
  }

  msg1.id = 0xFF0000;
  msg1.len = 0;
  can1.write(msg1);
}

void shutdownpMBB32() {
  //send shutdown to pMBB32 #3
  msg1.id = 0xAF0300; 
  msg1.len = 1;
  msg1.buf[0] = 0x55;
  can1.write(msg1);
  delay(1);
  can1.write(msg1);
  delay(1);

  //send shutdown to pMBB32 #2
  msg1.id = 0xAF0200;
  can1.write(msg1);
  delay(1);
  can1.write(msg1);
  delay(1);

  //send shutdown to pMBB32 #1
  msg1.id = 0xAF0100;
  can1.write(msg1);
  delay(5);
  can1.write(msg1);
  delay(5);
}

void ReadDigitalStatuses() {
  // read status of digital pins (1-13)
  digitalPins = 0;

  int bitposition = 0;
  for (int i=1; i<14; i++)
  {
    if (digitalRead(i) == HIGH) digitalPins |= (1 << bitposition);
    bitposition++;
  }
}//end of ReadDigitalStatuses()

void ReadAnalogStatuses() {
  // read analog pins (0-7)
  for (int i=0; i<7; i++)
  {
    analogPins[i] = analogRead(i);
  }
}//end of ReadAnalogStatuses()

void displayStatus() {
  digitalWriteFast(6, HIGH);
  /*
  Serial.print("keypadStatus: ");
  Serial.println(keypadStatus, BIN);
  Serial.print("Pack Current: ");
  //Serial.print(msg.buf[msg.buf[5]] < 16 ? "0" : ""); Serial.println(msg.buf[5], HEX); 
  Serial.print(packCurrent, HEX);
  Serial.println("mA ");
  Serial.print("Pack Voltage: ");
  Serial.print(packV, HEX);
  Serial.print("mV ");
  Serial.print("Pre-charge Voltage: ");
  Serial.print(preChargeV, HEX);
  Serial.print("mV ");
  Serial.print("U3 Voltage: ");
  Serial.print(U3V, HEX);
  Serial.println("mV ");
  Serial.print("Module ");
  Serial.print(minModule, DEC);
  Serial.print(" min cell #:  ");
  Serial.print(minCell);
  Serial.print(", 0x");
  Serial.print(minCellV, HEX);
  Serial.print(", ");
  Serial.print((float)minCellV*ADCres, 3);
  Serial.println("V ");
  Serial.print("Module ");
  Serial.print(maxModule, DEC);
  Serial.print(" max cell #:  ");
  Serial.print(maxCell, DEC);
  Serial.print(", 0x");
  Serial.print(maxCellV, HEX);
  Serial.print(", ");
  Serial.print((float)maxCellV*ADCres, 3);
  Serial.println("V ");
  Serial.print("CAN Staleness... #1:");
  Serial.print(pMBB32stale1, DEC);
  Serial.print(" #2:");
  Serial.print(pMBB32stale2, DEC);
  Serial.print(" #3:");
  Serial.println(pMBB32stale3, DEC);
  Serial.print("Fix? ");
  Serial.println(myGNSS.getGnssFixOk(), DEC);
  Serial.print("Number of Satellites: ");
  Serial.println(myGNSS.getSIV(), DEC);
  Serial.print("Altitude ");
  Serial.print(myGNSS.getAltitudeMSL() / 3300, DEC); Serial.println(" feet");
  Serial.print("Speed ");
  Serial.print(myGNSS.getGroundSpeed() * 0.00223694, DEC);  Serial.println(" mph");//convert mm/s to mph

  */

  //Serial.print("CAN Staleness... #1:");
  //Serial.print(pMBB32stale1, DEC);
  //Serial.print(" #2:");
  //Serial.print(pMBB32stale2, DEC);
  //Serial.print(" #3:");
  //Serial.print(pMBB32stale3, DEC);
  //Serial.print(" Max:");
  //Serial.println(pMBB32staleMax, DEC);

  rpm += 100;
  if(rpm >= 13000)
    rpm = 0;
  
  IVTpackVoltage = 3840;

  power += 1;
  IVTpackCurrent = power * 1000000 / IVTpackVoltage;
  if(power > 45)
    power = 0;
  
  throttle = 100;
  batteryVoltage = 1255;

#ifdef UBLOX_GNSS
  //groundSpeed = myGNSS.getGroundSpeed() * 0.00223694;
  //GPSaltitude = myGNSS.getAltitudeMSL() / 3300;
#endif

  msg3.flags.extended = 1;
  msg3.id = 0xc80;
  msg3.len = 8;
  msg3.buf[0] = rpm;//RPM = V
  msg3.buf[1] = rpm>>8;
  msg3.buf[2] = power*100*10*134/10000;//kW = V/10
  msg3.buf[3] = (power*100*10*134/10000)>>8;
  msg3.buf[4] = 21;//°C = V-100
  msg3.buf[5] = 0;
  msg3.buf[6] = throttle*10;//TPS = V/10
  msg3.buf[7] = (throttle*10)>>8;
  can3.write(msg3); 
  delay(1);

  msg3.id = 0xc81;
  msg3.len = 8;
  msg3.buf[0] = 0;
  msg3.buf[1] = 0;
  msg3.buf[2] = IVTpackVoltage;
  msg3.buf[3] = IVTpackVoltage>>8;
  msg3.buf[4] = IVTpackCurrent;
  msg3.buf[5] = IVTpackCurrent>>8;
  msg3.buf[6] = batteryVoltage;
  msg3.buf[7] = batteryVoltage>>8;
  can3.write(msg3);
  delay(1);

  msg3.id = 0xc82;
  msg3.len = 8;
  msg3.buf[0] = highestCellV;
  msg3.buf[1] = highestCellV>>8;
  msg3.buf[2] = lowestCellV;
  msg3.buf[3] = lowestCellV>>8;
  msg3.buf[4] = groundSpeed;
  msg3.buf[5] = 0;
  msg3.buf[6] = GPSaltitude;
  msg3.buf[7] = GPSaltitude>>8;
  can3.write(msg3);
  delay(1);

  msg3.id = 0xc83;
  msg3.len = 8;
  msg3.buf[0] = highestCellV - lowestCellV;
  msg3.buf[1] = (highestCellV - lowestCellV)>>8;
  msg3.buf[2] = SIM100MODohmsPerVolt;
  msg3.buf[3] = SIM100MODohmsPerVolt>>8;
  msg3.buf[4] = SIM100MODtemp;
  msg3.buf[5] = SIM100MODtemp>>8;
  msg3.buf[6] = fixType;
  msg3.buf[7] = 0;
  can3.write(msg3);

  digitalWriteFast(6, LOW);
}//end of displayStatus()
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
// State transition functions
void Off_enter()
{
  Serial.println("Entering Off state");
  VCUstate = VCU_STATE_OFF;
}

void Off_exit()
{
  Serial.println("Exiting Off state");
  //enable negative contactor
  PDUmsg2.buf[0] = 0xFE;//HS driver 1 PWM set to 99%- negative contactor
  PDUmsg1.buf[0] = 13;//HS driver 1 current limit 5A (5/0.4A = 13 or 0x0D) - negative contactor
  /* Check for welded contactor here*/

}

void Idle_enter()
{
  Serial.println("Entering Idle state");
  VCUstate = VCU_STATE_IDLE;
  //PDUmsg2.buf[3] = 0xFE;//HS driver 3 PWM set to 99%- positive pre-charge
  //PDUmsg1.buf[3] = 5;//HS driver 3 current limit 2A (2/0.4A = 5) - positive pre-charge

  msg2.id = 0x18EF2100;//keypad button color command
  msg2.flags.extended = 1;
  msg2.len = 8;
  msg2.buf[0] = 0x04;
  msg2.buf[1] = 0x1B;
  msg2.buf[2] = KEYPAD_CMD_SET_LED;
  msg2.buf[3] = 0x05;//button 5
  msg2.buf[4] = KEYPAD_COLOR_AMBER;
  msg2.buf[5] = KEYPAD_MODE_BLINK;
  msg2.buf[6] = 0x00;
  msg2.buf[7] = 0xFF;
  can2.write(msg2);
  //delay(1);

  // read IVT-MOD pack voltage U2, compare to pack voltage U1, if U2 < 95% of U1 after 2 seconds:
  // - send error message to keypad
  // - turn off pre-charge
  // - go back to off state

  //if pre-charge fails after 5 seconds, turn off pre-charge and send error message
  // delay(5000);
  // msg2.id = 0x18EF2100;//keypad button color command
  // msg2.flags.extended = 1;
  // msg2.len = 8;
  // msg2.buf[0] = 0x04;
  // msg2.buf[1] = 0x1B;
  // msg2.buf[2] = KEYPAD_CMD_SET_LED;
  // msg2.buf[3] = 0x05;//button 5
  // msg2.buf[4] = KEYPAD_COLOR_AMBER;
  // msg2.buf[5] = KEYPAD_MODE_ALT_BLINK;
  // msg2.buf[6] = KEYPAD_COLOR_RED;
  // msg2.buf[7] = 0xFF;
  // can2.write(msg2);
  //delay(1);

  // if pre-charge good, enable positive contactor
  //PDUmsg2.buf[4] = 0xFE;//HS driver 5 PWM set to 99%- positive contactor
  //PDUmsg1.buf[4] = 0;//HS driver 5 current limit 5A (5/0.4A = 13 or 0x0D) - positive contactor


  /* Send SMS
  0 = send "KLR on" message
  1 = send "KL15 on" message
  2 = send "Pre-charge failed..." message
  3 = send "Something happened..." message
  4 = send "Charging stopped..." message
  5 = send "Temperature warning..." message
  any other value  = send "Invalid request..." message
  */
  msg3.id = 0xC79;//send SMS
  msg3.len = 1;
  msg3.buf[0] = 1;//send "KL15 on" message
  can3.write(msg3);
  delay(1);
}

void Charge_enter()
{
  Serial.println("Entering Charge state");
  VCUstate = VCU_STATE_CHARGE;
}

void Drive_enter()
{
  Serial.println("Entering Drive state");
  VCUstate = VCU_STATE_DRIVE;
}

void Idle_exit()
{
  Serial.println("Exiting Idle state");
  //disable positive and negative contactors
  //PDUmsg1.buf[0] = 0;//channel 1 current limit 5A (5/0.4A = 13 or 0x0D) - negative contactor
  //PDUmsg1.buf[3] = 0;//channel 4 current limit 5A (5/0.4A = 13 or 0x0D) - positive contactor
}

void on_trans_Off_Idle()
{
  Serial.println("Transitioning from Off to Idle");
  

}

void on_trans_Idle_Off()
{
  Serial.println("Transitioning from Idle to Off");
  
}

void on_trans_Idle_Drive()
{
  Serial.println("Transitioning from Idle to Drive");
  
  
}

void on_trans_Drive_Idle()
{
  Serial.println("Transitioning from Drive to Idle");
  
}

void on_trans_Idle_Charge()
{
  Serial.println("Transitioning from Idle to Charge");
  
}

void on_trans_Charge_Idle()
{
  Serial.println("Transitioning from Charge to Idle");
  
}

// ── New state functions ──────────────────────────────────────────────────────

void PreCharge_enter() {
  Serial.println("Entering PreCharge state");
  VCUstate = VCU_STATE_PRECHARGE;
  preChargeStartTime = millis();
  PDUmsg1.buf[0] = 0x0D; // CH1 5A — negative contactor on
  PDUmsg1.buf[2] = 0x05; // CH3 2A — passive pre-charge relay (remove when active pre-charge fitted)
  digitalWrite(PRECHARGE_EN_PIN, HIGH); // TPS131PXQ1EVM-400 active pre-charge enable
}

void PreCharge_exit() {
  Serial.println("Exiting PreCharge state");
  PDUmsg1.buf[2] = 0x00; // CH3 off — passive pre-charge relay off
  digitalWrite(PRECHARGE_EN_PIN, LOW);  // disable active pre-charge
}

void check_PreCharge() {
  if (IVTpackVoltage > 0 &&
      IVTpreChargeV >= (IVTpackVoltage * 95 / 100)) {
    PDUmsg1.buf[3] = 0x0D; // CH4 5A — positive contactor on
    fsm.trigger(chargeMode ? PRECHARGE_CHARGE : PRECHARGE_OK);
  } else if (millis() - preChargeStartTime > PRECHARGE_TIMEOUT_MS) {
    fsm.trigger(PRECHARGE_FAIL);
  }
}

void check_Idle() {
  if (button_0x01_state && LDUrpm == 0) { // Park + stopped → exit On State
    button_0x01_state = 0;
    KL15state = false;
    fsm.trigger(KL15_OFF);
    return;
  }
  if (button_0x08_state) { // Drive button → enter Drive state
    fsm.trigger(DRIVE_ON);
    return;
  }
  // TODO: BMS out-of-bounds temp → fsm.trigger(TEMP_LOW) or fsm.trigger(TEMP_HIGH)
}

void Drive_exit() {
  Serial.println("Exiting Drive state");
  LDUdirection = LDU_DIR_STOP;
}

void check_DriveState() {
  if (button_0x01_state && LDUrpm == 0) { // Park button + stopped → Idle
    button_0x01_state = 0;
    fsm.trigger(DRIVE_OFF);
    return;
  }
  // TODO: isolation fault → fsm.trigger(FAULT_EV)
  // TODO: BMS temp out of bounds → fsm.trigger(TEMP_LOW) or fsm.trigger(TEMP_HIGH)
}

void Charge_exit() {
  Serial.println("Exiting Charge state");
  PDUmsg1.buf[0] = 0x00; // CH1 off — negative contactor
  PDUmsg1.buf[3] = 0x00; // CH4 off — positive contactor
  if (evccIsACSession) {
    msg2.flags.extended = 0;
    msg2.id  = EVCC_AC_STATUS;
    msg2.len = 1;
    msg2.buf[0] = 0x00;  // Ready_To_Charge = Not_Ready — EVCC opens AC relay
    can2.write(msg2);
    msg2.flags.extended = 1;
  }
}

void check_Charge() {
  if (button_0x01_state && LDUrpm == 0) { // Park + stopped → emergency exit
    button_0x01_state = 0;
    KL15state = false;
    fsm.trigger(KL15_OFF);
    return;
  }
  // TODO: EVCC stop charge request → fsm.trigger(CHARGE_OFF)
  // TODO: isolation fault → fsm.trigger(FAULT_EV)
  // TODO: BMS temp out of bounds → fsm.trigger(TEMP_LOW) or fsm.trigger(TEMP_HIGH)
}

void Fault_enter() {
  Serial.println("Entering Fault state");
  VCUstate = VCU_STATE_FAULT;
  PDUmsg1.buf[0] = 0x00; // CH1 off — negative contactor
  PDUmsg1.buf[2] = 0x00; // CH3 off — pre-charge relay
  PDUmsg1.buf[3] = 0x00; // CH4 off — positive contactor
  msg2.id = 0x18EF2100;
  msg2.flags.extended = 1;
  msg2.len = 8;
  msg2.buf[0] = 0x04; 
  msg2.buf[1] = 0x1B;
  msg2.buf[2] = KEYPAD_CMD_SET_LED;
  msg2.buf[3] = 0x05;
  msg2.buf[4] = KEYPAD_COLOR_RED;
  msg2.buf[5] = KEYPAD_MODE_BLINK;
  msg2.buf[6] = 0x00; 
  msg2.buf[7] = 0xFF;
  can2.write(msg2);
  msg3.id = 0xC79;
  msg3.len = 1;
  msg3.buf[0] = 2; // "Something happened..."
  can3.write(msg3);
}

void Fault_exit() {
  Serial.println("Exiting Fault state");
}

void check_Fault() {
  if (button_0x01_state && LDUrpm == 0) {
    button_0x01_state = 0;
    KL15state = false;
    fsm.trigger(FAULT_CLEAR);
  }
}

void HeatPack_enter() {
  Serial.println("Entering HeatPack state");
  VCUstate = VCU_STATE_HEAT_PACK;
  // TODO: set invPumpSetpoint/battPumpSetpoint; enable heater output
}

void HeatPack_exit() {
  Serial.println("Exiting HeatPack state");
  // TODO: disable heater; stop pump
}

void check_HeatPack() {
  if (button_0x01_state && LDUrpm == 0) { button_0x01_state = 0; KL15state = false; fsm.trigger(KL15_OFF); return; }
  // TODO: monitor pMBB32 temps; fsm.trigger(TEMP_OK) when in range
}

void CoolPack_enter() {
  Serial.println("Entering CoolPack state");
  VCUstate = VCU_STATE_COOL_PACK;
  // TODO: set invPumpSetpoint/battPumpSetpoint; enable AC exchanger
}

void CoolPack_exit() {
  Serial.println("Exiting CoolPack state");
  // TODO: disable AC exchanger; stop pump
}

void check_CoolPack() {
  if (button_0x01_state && LDUrpm == 0) { button_0x01_state = 0; KL15state = false; fsm.trigger(KL15_OFF); return; }
  // TODO: monitor pMBB32 temps; fsm.trigger(TEMP_OK) when in range
}

// ── New transition callbacks ─────────────────────────────────────────────────

void on_trans_Off_PreCharge()     { Serial.println("Off → PreCharge"); }
void on_trans_PreCharge_Idle()    { Serial.println("PreCharge OK → Idle"); }
void on_trans_PreCharge_Charge()  { Serial.println("PreCharge OK → Charge"); }
void on_trans_PreCharge_Fault()   { Serial.println("PreCharge FAILED → Fault"); }
void on_trans_Fault_Off()         { Serial.println("Fault cleared → Off"); }

// ── KL30C — external CAN wake standby ────────────────────────────────────────
// Entered when the VCU wakes from sleep via CAN2 activity (EVCC or wireless
// gateway) with KL15R still LOW (key not turned). No HV contactors are closed.
// Exits to Off on KL15_ON (button 5), which then immediately proceeds to
// PreCharge since KL15state is already true.
// Sleeps when EVCCsessionActive is false, KL15R is low, and no EVCC heartbeat
// or gateway frame has been seen for KL30C_SLEEP_TIMEOUT_MS.

void KL30C_enter() {
  Serial.println("Entering KL30C state");
  VCUstate          = VCU_STATE_KL30C;
  kl30cKL15Rstate   = false; // force LED edge detect on first check_KL30C() call
  lastExtActivityMs = millis(); // reset timeout — start counting from now
  PDUmsg1.buf[0]    = 0x00; // CH1 off — negative contactor
  PDUmsg1.buf[3]    = 0x00; // CH4 off — positive contactor
}

void check_KL30C() {
  // Flash button 1 (Park/P) LED while KL15R is high to signal standby mode.
  bool kl15rNow = digitalRead(KL15R_PIN);
  if (kl15rNow != kl30cKL15Rstate) {
    kl30cKL15Rstate     = kl15rNow;
    msg2.id             = 0x18EF2100;
    msg2.flags.extended = 1;
    msg2.len            = 8;
    msg2.buf[0]         = 0x04;
    msg2.buf[1]         = 0x1B;
    msg2.buf[2]         = KEYPAD_CMD_SET_LED;
    msg2.buf[3]         = 0x01; // button 1 — Park/P
    msg2.buf[4]         = kl15rNow ? KEYPAD_COLOR_AMBER : KEYPAD_COLOR_OFF;
    msg2.buf[5]         = kl15rNow ? KEYPAD_MODE_BLINK  : KEYPAD_MODE_SOLID;
    msg2.buf[6]         = 0x00;
    msg2.buf[7]         = 0xFF;
    can2.write(msg2);
  }

  // AC plug detected → close main contactors via pre-charge sequence.
  // DC sessions leave EVCC to handle its own contactors; VCU stays in KL30C.
  if (EVCCsessionActive && evccIsACSession) {
    fsm.trigger(AC_CHARGE_START);
    return;
  }

  // Button 5 (KL15) pressed → transition to Off; Off's check_KL15() will see
  // KL15state=true and immediately proceed to PreCharge.
  if (KL15state) {
    fsm.trigger(KL15_ON);
    return;
  }

  // Sleep when: no active EVCC session, KL15R is low, and no external activity
  // for KL30C_SLEEP_TIMEOUT_MS.
  if (!EVCCsessionActive && !kl15rNow &&
      millis() - lastExtActivityMs > KL30C_SLEEP_TIMEOUT_MS) {
    Serial.println("KL30C timeout — sleeping");
    enterSleep();
  }
}

void KL30C_exit() {
  Serial.println("Exiting KL30C state");
  msg2.id             = 0x18EF2100;
  msg2.flags.extended = 1;
  msg2.len            = 8;
  msg2.buf[0]         = 0x04;
  msg2.buf[1]         = 0x1B;
  msg2.buf[2]         = KEYPAD_CMD_SET_LED;
  msg2.buf[3]         = 0x01; // button 1 — Park/P
  msg2.buf[4]         = KEYPAD_COLOR_OFF;
  msg2.buf[5]         = KEYPAD_MODE_SOLID;
  msg2.buf[6]         = 0x00;
  msg2.buf[7]         = 0xFF;
  can2.write(msg2);
}

// ── Sleep / wake ─────────────────────────────────────────────────────────────

extern "C" uint32_t set_arm_clock(uint32_t frequency); // clockspeed.c

void enterSleep() {
  if (EVCCsessionActive) return;  // active EVCC charge session — stay awake

  Serial.println("KLR off — sleeping");
  Serial.flush();

  // 1. Power down USB PHY and gate its clock — restores automatically on AIRCR reset.
  USBPHY1_PWD    = 0xFFFFFFFF;
  CCM_CCGR6     &= ~CCM_CCGR6_USBOH3(3);

  t0.end();   // IntervalTimer uses end()
  t1.stop();  // TeensyTimerTool PeriodicTimer
  t2.stop();
  t3.stop();
  digitalWriteFast(LED_BUILTIN, LOW);

#ifdef UBLOX_GNSS
  // I2C cannot wake NEO-M8M from backup mode; EXTINT0 is the supported source.
  myGNSS.powerOffWithInterrupt(0, VAL_RXM_PMREQ_WAKEUPSOURCE_EXTINT0);
  delay(500);
#endif

  // Assert CAN transceiver standby — puts all three transceivers into low-power mode.
  digitalWrite(CAN_STBY_PIN, HIGH);

  // Gate FlexCAN peripheral clocks — stops internal CAN controller sampling.
  // AIRCR reset on wake restores all CCM clock gates to boot defaults.
  CCM_CCGR0 &= ~(CCM_CCGR0_CAN1(3) | CCM_CCGR0_CAN1_SERIAL(3) |
                 CCM_CCGR0_CAN2(3) | CCM_CCGR0_CAN2_SERIAL(3));
  CCM_CCGR7 &= ~(CCM_CCGR7_CAN3(3) | CCM_CCGR7_CAN3_SERIAL(3));

  // Reconfigure CAN2 RXD pin as GPIO input (CAN clock already gated).
  // MCP2562 in standby drives RXD low on dominant bus edges — triggers the interrupt below.
  pinMode(CAN2_RX_PIN, INPUT_PULLUP);

  // 2. Drop CPU to ARM PLL minimum (~16.2 MHz, 0.95 V DCDC).
  set_arm_clock(16000000);

  // Switch AHB to 24 MHz crystal, then bypass ARM PLL so the CPU also takes the
  // crystal reference (24 MHz / ARM_PODF ≈ 3 MHz), then power down the PLL VCO.
  // Mirrors the temporary switch inside set_arm_clock() — proven safe sequence.
  // AIRCR reset on wake restores all clock registers to boot state.
  CCM_CBCMR = (CCM_CBCMR & ~CCM_CBCMR_PERIPH_CLK2_SEL_MASK)
            | CCM_CBCMR_PERIPH_CLK2_SEL(1);           // PERIPH_CLK2 = 24 MHz OSC
  while (CCM_CDHIPR & CCM_CDHIPR_PERIPH2_CLK_SEL_BUSY);
  CCM_CBCDR &= ~CCM_CBCDR_PERIPH_CLK2_PODF_MASK;      // ÷1 (no divide on crystal)
  CCM_CBCDR |=  CCM_CBCDR_PERIPH_CLK_SEL;             // AHB from crystal
  while (CCM_CDHIPR & CCM_CDHIPR_PERIPH_CLK_SEL_BUSY);
  CCM_ANALOG_PLL_ARM |= CCM_ANALOG_PLL_ARM_BYPASS;    // CPU: crystal ref / ARM_PODF
  CCM_ANALOG_PLL_ARM |= CCM_ANALOG_PLL_ARM_POWERDOWN; // ARM PLL VCO off

  // 3. GPIO interrupt on KL15R rising edge; disable SysTick 1 ms tick.
  //    Single WFI then sleeps until the actual wake event instead of every 1 ms.
  //    If KL15R is already high when WFI executes, the pending interrupt returns it
  //    immediately — no race condition.
  attachInterrupt(digitalPinToInterrupt(KL15R_PIN),  [](){}, RISING);   // key-on
  attachInterrupt(digitalPinToInterrupt(CAN2_RX_PIN), [](){}, FALLING); // EVCC bus activity
  SYST_CSR &= ~1u;  // disable SysTick (bit 0 = ENABLE) — stops 1 ms wakeups

  // STOP mode — gates more internal domains than WAIT mode.
  // Wake sources: KL15R_PIN rising edge (key-on) or CAN2_RX_PIN falling edge (EVCC).
  // AIRCR reset on wake restores all registers, so no clock restore needed.
  // To revert to WAIT mode: delete the two lines below.
  CCM_CLPCR = (CCM_CLPCR & ~0x3u) | 0x2u;  // LPM = 0b10 (STOP)
  SCB_SCR   |= (1u << 2);                    // SLEEPDEEP — WFI enters STOP not WAIT

  asm volatile("dsb");
  asm volatile("isb");
  if (!digitalRead(KL15R_PIN)) {
    asm volatile("wfi");  // sleep until KL15R rising edge or CAN2 falling edge
  }

  asm volatile("dsb");

  // Set sleepMagic based on which signal woke us.
  // KL15R HIGH = key was turned → normal boot; KL15R still LOW = CAN2 woke us → KL30C.
  sleepMagic = digitalRead(KL15R_PIN) ? 0 : SLEEP_MAGIC_CAN_WAKE;

#ifdef UBLOX_GNSS
  // Rising edge on EXTINT0 wakes the GNSS module for a hot-start while the Teensy resets.
  // Use DWT cycle counter — independent of SysTick and the stale F_CPU_ACTUAL.
  // At ~3 MHz (crystal / ARM_PODF=8), 150 000 cycles ≈ 50 ms.
  digitalWrite(GNSS_EXTINT_PIN, HIGH);
  {
    uint32_t t = ARM_DWT_CYCCNT;
    while (ARM_DWT_CYCCNT - t < 150000u);
  }
#endif

  SCB_AIRCR = 0x05FA0004;
  while (1);
}

// ── Existing check functions ─────────────────────────────────────────────────

void check_KL15()
{
  if (KL15state) {
    fsm.trigger(KL15_ON);
  }
}

void check_KL17()
{
  if(KL17state) {//internal KL15 (switched 'ignition' state)
    fsm.trigger(KL15_ON);
  } else {
    //fsm.trigger(KL15_OFF);
  }
}
void check_Drive()
{
  if(button_0x08_state) {
    fsm.trigger(DRIVE_ON);
  } else {
    //fsm.trigger(DRIVE_OFF);
  }
  
}


/* BMW i4/i5/i7/iX Changeover Valve 64119462114 — LIN master on Serial3
 * LIN 2.x, 19200 baud. Uses gicking/LIN master portable (LIN_Master_HardwareSerial).
 * TODO: confirm node ID, response frame length, byte map, and checksum type
 *       from BMW LIN specification / ISTA service documentation.
 */
LIN_Master_HardwareSerial linBus(Serial3, "Valve"); // Serial3: RX3=pin7, TX3=pin8

void linInit() {
  linBus.begin(LIN_BAUD);
  //pinMode(LIN_EN_PIN, OUTPUT);    // TODO: uncomment when enable pin assigned
  //digitalWrite(LIN_EN_PIN, HIGH); // enable LIN transceiver
}

// Request status from valve (slave-response frame)
void linReadValve() {
  uint8_t buf[2] = {0};            // TODO: confirm response frame length from BMW docs
  LIN_Master_Base::error_t err = linBus.receiveSlaveResponseBlocking(LIN_Master_Base::LIN_V2, LIN_VALVE_ID, sizeof(buf), buf);
  if (err == LIN_Master_Base::NO_ERROR) {
    valveStatus   = buf[0];        // TODO: confirm byte map
    valvePosition = buf[1];        // TODO: confirm byte map
    valveOnline   = true;
  } else {
    valveOnline = false;
  }
}

// Command valve position (master-request frame)
// TODO: confirm frame length and byte map from BMW LIN spec / ISTA docs
void linWriteValve(uint8_t position) {
  valvePosition = position;
  // uint8_t data[] = {position};
  // linBus.sendMasterRequestBlocking(LIN_Master_Base::LIN_V2, LIN_VALVE_ID, sizeof(data), data);
}

/* Dual Pot Throttle (EVWest, OEM pedal)
 * Both pots are read and scaled independently. A plausibility check confirms
 * they agree within THROTTLE_PLAUSIBILITY_PCT. On fault, throttle is zeroed.
 * TODO: bench-calibrate the MIN/MAX constants, then remove the hardcoded
 *       throttle = 100 override in displayStatus().
 */
void readThrottle() {
  // ── 1. READ ─────────────────────────────────────────────────────────────
  // throttlePot1Raw, throttlePot2Raw, and brakePedal updated by ADS1115 in loop()
  regenActive = (LDUrpm > 50) && (LDUtorque < -5); // negative torque = generating
  brakePedal |= regenActive; // regen counts as braking: zeroes throttle + sets CANIO BRAKE

  // ── 2. VERIFY — 5 % plausibility window between tracks ──────────────────
  uint16_t pct1 = constrain(map(throttlePot1Raw, THROTTLE_POT1_MIN, THROTTLE_POT1_MAX, 0, 100), 0, 100);
  uint16_t pct2 = constrain(map(throttlePot2Raw, THROTTLE_POT2_MIN, THROTTLE_POT2_MAX, 0, 100), 0, 100);
  throttlePlausibility = (abs((int16_t)pct1 - (int16_t)pct2) <= THROTTLE_PLAUSIBILITY_PCT);
  uint16_t pedalPct = throttlePlausibility ? pct1 : 0; // track 1 primary; fault → 0

  // ── 3. ARBITRATE ─────────────────────────────────────────────────────────
  bool faultActive = IVTfaultActive || (SIMM100MODerrorFlags != 0);

  if (!throttlePlausibility || brakePedal) {
    pedalPct = 0;                                          // hard zero on sensor fault or brake
  } else if (faultActive) {
    pedalPct = min(pedalPct, (uint16_t)THROTTLE_FAULT_LIMIT); // limit to safe ceiling
  }
  throttle = pedalPct;

  // ── 4. MAP — linear 1:1 (TODO: replace with torque/slip curve) ───────────
  // Result written to LDUtorqueSetpoint (0-100); callback_t0 transmits on 0x201.
  LDUtorqueSetpoint = (VCUstate == VCU_STATE_DRIVE) ? (int16_t)pedalPct : 0;
}

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


/* Main */
void loop() {
  digitalWriteFast(3, HIGH);  

  can1.events();//Call to look for any input
  can2.events();//Call to look for any input
  //can3.events();//Output only

  // readThrottle() is called from callback_t0() at precise 10ms intervals
    //Serial.println("Shutting down");
    // test shutdown and wake
    //shutdownpMBB32();
    //delay(2);

    //alarm.setRtcTimer(0, 0, 20);// hour, min, sec 

    //timer.setTimer(30);// seconds

  // KL15R gone low (key off) while system is safe → hibernate.
  // Debounce: only sleep after KL15R has been continuously LOW for 500ms.
  // Gives the EVCC time to send New_Charge_Session (0x68001) after a CAN2 wake
  // before sleep is re-entered. Normal key-off behaviour is unchanged.
  // enterSleep() returns immediately if EVCCsessionActive (active charge session).
  {
    static uint32_t klrLowSince = 0;
    if (digitalRead(KL15R_PIN)) {
      klrLowSince = millis();
    } else if (VCUstate == VCU_STATE_OFF && millis() - klrLowSince > 500) {
      enterSleep();
    }
  }

  // ADS1115 non-blocking read — cycles through throttle pot 1/2 and brake at 860 SPS (~3.5ms/cycle).
  // Time-gated: only checks I2C once per conversion period to avoid starving the GNSS on Wire.
  {
    static const uint16_t adsMux[3] = {
      ADS1X15_REG_CONFIG_MUX_SINGLE_0,
      ADS1X15_REG_CONFIG_MUX_SINGLE_1,
      ADS1X15_REG_CONFIG_MUX_SINGLE_2,
    };
    static uint8_t  adsCh   = 0;
    static uint32_t adsNext = 0;   // micros() deadline for next read
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
      adsNext = micros() + 1300; // 1.16ms conversion + 150µs I2C margin
    }
  }

#ifdef UBLOX_GNSS
  myGNSS.checkUblox();
  myGNSS.checkCallbacks();

#endif

fsm.run_machine();

  digitalWriteFast(3, LOW);
}//end of loop()