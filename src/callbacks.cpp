/* callbacks.cpp — t0/t1/t2/t3 periodic timer callbacks. */
#include <Arduino.h>
#include <TeensyTimerTool.h>
#include <FlexCAN_T4.h>
#include "pMBB32.h"
#include "defines.h"
#include "throttle.h"
#include "lin.h"
#include "sdlog.h"
#include "display.h"
#include "callbacks.h"

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
  if (VCUstate != VCU_STATE_FAULT)
    displayStatus(); // update RealDash

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

// Set by can3Sniff() when a 0xC84 status request arrives from the wireless gateway;
// answered with a 0xC85 response the next time callback_t2() runs.
volatile bool gatewayStatusRequestPending = false;
// Set right after callback_t2() sends the 0xC85 response; consumed by loop().
volatile bool gatewayResponseSent = false;

/* t2 Callback
* This runs every t2 period (200ms).
* Tasks performed here:
*   1. Request all cell and temperature measurements from pMBB32s
*   2. Request min/max cell voltages from pMBB32s
*   3. Check for stale pMBB32 CAN and restart if necessary
*   4. Read isolation state from SIM100MOD
*/
void callback_t2() {
  // Wait for at least one real pMBB32 reading post-boot before answering — right after a
  // wake/reset, highestCellV/lowestCellV are still zero-initialized, not fresh BMS data,
  // until the modules have responded to a 0xFF0000 trigger at least once. Left pending
  // (retried every 200ms) rather than answering with obviously-invalid zeros.
  if (gatewayStatusRequestPending && highestCellV > 0) {
    gatewayStatusRequestPending = false;
    msg3.flags.extended = 1;
    msg3.id  = 0xC85; // status response to wireless gateway
    msg3.len = 8;
    uint16_t packV10 = (uint16_t)(IVTpackVoltage / 100); // mV -> 0.1V
    msg3.buf[0] = packV10 & 0xFF;
    msg3.buf[1] = (packV10 >> 8) & 0xFF;
    msg3.buf[2] = highestCellV & 0xFF;
    msg3.buf[3] = (highestCellV >> 8) & 0xFF;
    msg3.buf[4] = lowestCellV & 0xFF;
    msg3.buf[5] = (lowestCellV >> 8) & 0xFF;
    msg3.buf[6] = (uint8_t)VCUstate;
    msg3.buf[7] = 0; // reserved
    can3.write(msg3);
    gatewayResponseSent = true; // loop() sleeps promptly instead of waiting out the KLR debounce
    Serial.printf("CAN3: sent 0xC85 response (packV=%u x0.1V, hiCell=%u, loCell=%u, state=%u)\n",
                  packV10, highestCellV, lowestCellV, (uint8_t)VCUstate);
  }

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
