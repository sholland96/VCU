#include <Arduino.h>
#include <TeensyTimerTool.h>
#include <FlexCAN_T4.h>
#include "pMBB32.h"
#include "defines.h"
#include "Fsm.h"
#include "throttle.h"
#include "lin.h"
#include "sdlog.h"
#include "display.h"
#include "sleep.h"
#include "fsm_states.h"
#include "can_handlers.h"

using namespace TeensyTimerTool;
IntervalTimer  t0;       // PIT channel — 10ms LDU torque command (highest priority)
PeriodicTimer t1(RTC);  // 62.5ms — PDU-8 keepalive, pMBB32 cell poll, RealDash update
PeriodicTimer t2(GPT1); // 200ms  — pMBB32 measurement broadcast, SIM100MOD, LIN valve
PeriodicTimer t3(GPT2); // 1000ms — heartbeat LED

CAN_message_t msg1;//.buf[7] = {0x0D, 0x05, 0x05, 0x0D, 0, 0, 0, 0};
CAN_message_t msg2;
CAN_message_t msg3;
CAN_message_t PDUmsg1;
CAN_message_t PDUmsg2;
CAN_message_t LDUmsg;      // 0x201: pot (bytes 0-1) + canio (byte 4), sent every 10ms

// Used pins
#define LED_PIN 13

// DMAMEM places sleepMagic in OCRAM (.bss.dma, NOLOAD): valid writable RAM, not zeroed by CRT
// startup, survives AIRCR reset. Used in enterSleep() and setup() to detect CAN-wake vs key-on.
DMAMEM volatile uint32_t sleepMagic;

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
volatile bool     pMBB32ghostSA   = false;
// Bitmask of frame types (ft=01..0C → bits 0..11) seen per module since last check.
volatile uint16_t pMBB32ftSeen[3] = {0, 0, 0};

/* t2 Callback
* This runs every t2 period (200ms).
* Tasks performed here:
*   1. Request all cell and temperature measurements from pMBB32s
*   2. Request min/max cell voltages from pMBB32s
*   3. Check for stale pMBB32 CAN and restart if necessary
*   4. Read isolation state from SIM100MOD
*/
void callback_t2() {
  if (VCUstate != VCU_STATE_FAULT)
    displayStatus(); // update RealDash

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
        sdQueueEventISR("GHOST_SA_CH2");
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
          static const char* const ft03Evt[] = {"MOD1_FT03_ABSENT","MOD2_FT03_ABSENT","MOD3_FT03_ABSENT"};
          sdQueueEventISR(ft03Evt[i]);
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
      sdQueueEventISR("CH2_WAKE_ALL");
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
            { static const char* const retryEvt[] = {"MOD1_RETRY_CH2","MOD2_RETRY_CH2","MOD3_RETRY_CH2"};
              sdQueueEventISR(retryEvt[m.num - 1]); }
            PDUmsg1.buf[1] = 0;
            ch2.state = 1; ch2.t = millis();
            for (auto &mm : mods) { *mm.stale = 0; mm.phase = 0; mm.retryCount = 0; }
            break;
          }
          *m.stale = 0;
          Serial.printf("pMBB32 #%u stale — sending shutdown\n", m.num);
          { static const char* const shutEvt[] = {"MOD1_SHUTDOWN","MOD2_SHUTDOWN","MOD3_SHUTDOWN"};
            sdQueueEventISR(shutEvt[m.num - 1]); }
          msg1.id = m.id; msg1.len = 1; msg1.buf[0] = shutdown;
          can1.write(msg1);
          m.phase = 1; m.phaseTime = millis();
        } else if (m.phase == 1 && millis() - m.phaseTime >= 500) {
          Serial.printf("pMBB32 #%u — sending wake\n", m.num);
          { static const char* const wakeEvt[] = {"MOD1_WAKE","MOD2_WAKE","MOD3_WAKE"};
            sdQueueEventISR(wakeEvt[m.num - 1]); }
          msg1.id = m.id; msg1.len = 3;
          msg1.buf[0] = wakeup; msg1.buf[1] = channelCount16; msg1.buf[2] = numberOfDevices;
          can1.write(msg1);
          *m.stale = 0; m.retryCount++; m.phase = 0; anyWoke = true;
        }
      }
    }
    if (anyWoke) { msg1.id = 0xFF0000; msg1.len = 0; can1.write(msg1); }
  }

  sdLogPending = true; // data row written in loop() — SD writes must not happen in ISR
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


/* Main */
void loop() {
  digitalWriteFast(3, HIGH);

  can1.events();//Call to look for any input
  can2.events();//Call to look for any input

  // SD logging — must run in main-loop context; SD writes are not ISR-safe.
  if (sdLogPending) { sdLogPending = false; sdLogData(); }
  sdDrainEvents();
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