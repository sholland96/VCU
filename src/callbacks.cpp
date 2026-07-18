/* callbacks.cpp — Periodic timer callbacks (t0..t3). */
#include "vcu.h"
#include "callbacks.h"
#include "sdlog.h"
#include "can_handlers.h"
#include "throttle.h"
#include "display.h"
#include "lin.h"

/* t0 — 10ms LDU fixed safety frame (IntervalTimer/PIT, highest priority) */
void callback_t0() {
  readThrottle();

  uint16_t pot = (uint16_t)((uint32_t)LDUtorqueSetpoint * 4095 / 100);

  uint8_t canio = 0;
  if (VCUstate == VCU_STATE_DRIVE) {
    canio |= LDU_CANIO_START;
    if (LDUdirection == LDU_DIR_FORWARD) canio |= LDU_CANIO_FORWARD;
    if (LDUdirection == LDU_DIR_REVERSE) canio |= LDU_CANIO_REVERSE;
  }
  if (brakePedal) canio |= LDU_CANIO_BRAKE;

  LDUseqCounter = (LDUseqCounter + 1) & 0x03;
  uint8_t ctr = LDUseqCounter;

  LDUmsg.buf[0] = pot & 0xFF;
  LDUmsg.buf[1] = (pot >> 8) & 0x0F;
  LDUmsg.buf[2] = 0;
  LDUmsg.buf[3] = (canio & 0x3F) | (ctr << 6);
  LDUmsg.buf[4] = 0;
  LDUmsg.buf[5] = ctr << 6;
  LDUmsg.buf[6] = 0;
  LDUmsg.buf[7] = 0;
  can2.write(LDUmsg);
}

/* t1 — 62.5ms: PDU-8 keepalive, pMBB32 cell poll, RealDash, EVCC messages */
void callback_t1() {
  can1.write(PDUmsg1);

  msg1.flags.extended = 1;
  switch (counter) {
    case 1: msg1.id = 0xCF0100; msg1.len = 0; can1.write(msg1); break;
    case 2: msg1.id = 0xCF0200; msg1.len = 0; can1.write(msg1); break;
    case 3: counter = 0; msg1.id = 0xCF0300; msg1.len = 0; can1.write(msg1); break;
  }
  counter++;

  if (VCUstate != VCU_STATE_FAULT) displayStatus();

  // Advantics Generic Power Modules protocol — send every 62.5ms
  {
    normalEndOfCharge = (highestCellV > 0u) && (highestCellV >= EVCC_CELL_V_FULL);

    uint16_t presentV = (uint16_t)(IVTpackVoltage / 100u);
    int16_t  presentA = (int16_t)(IVTpackCurrent / 100);
    uint8_t  insRes   = (SIM100MODRpKohms > 510u) ? 255u : (uint8_t)(SIM100MODRpKohms / 2u);

    msg2.flags.extended = 1;
    msg2.id  = EVCC_PWR_STATUS; msg2.len = 8;
    msg2.buf[0] = presentV & 0xFF; msg2.buf[1] = (presentV >> 8) & 0xFF;
    msg2.buf[2] = (uint8_t)(presentA & 0xFF); msg2.buf[3] = (uint8_t)((uint16_t)presentA >> 8);
    msg2.buf[4] = 0; msg2.buf[5] = 0;
    msg2.buf[6] = EVCCsystemEnable ? 1u : 0u;
    msg2.buf[7] = insRes;
    can2.write(msg2);

    msg2.id  = EVCC_PWR_LIMITS; msg2.len = 8;
    msg2.buf[0] = EVCC_MAX_VOLTAGE_x10 & 0xFF; msg2.buf[1] = (EVCC_MAX_VOLTAGE_x10 >> 8) & 0xFF;
    msg2.buf[2] = EVCC_MAX_CURRENT_x10 & 0xFF; msg2.buf[3] = (EVCC_MAX_CURRENT_x10 >> 8) & 0xFF;
    msg2.buf[4] = msg2.buf[5] = msg2.buf[6] = msg2.buf[7] = 0;
    can2.write(msg2);

    bool chademoBtn = EVCCsystemEnable && (EVCCplugType == EVCC_PLUG_CHADEMO);
    msg2.id  = EVCC_SEQ_CTRL; msg2.len = 3;
    msg2.buf[0] = (EVCCsystemEnable ? 0x01u : 0x00u) | (chademoBtn ? 0x02u : 0x00u);
    msg2.buf[1] = EVCCsystemEnable ? 0x07u : 0x00u;
    msg2.buf[2] = normalEndOfCharge ? 0x01u : 0x00u;
    can2.write(msg2);

    // 0x611 AC_Status (v2.5 standard-ID): Ready_To_Charge bit 0
    msg2.flags.extended = 0;
    msg2.id     = EVCC_AC_STATUS;
    msg2.len    = 1;
    msg2.buf[0] = (evccIsACSession && EVCCsessionActive
                   && VCUstate == VCU_STATE_CHARGE
                   && acReadyToDeliver && !normalEndOfCharge) ? 0x01u : 0x00u;
    can2.write(msg2);
    msg2.flags.extended = 1;
  }
}

/* t2 — 200ms: pMBB32 measurement broadcast, stale recovery, SIM100MOD, pumps */
void callback_t2() {
  msg1.id             = 0xFF0000;
  msg1.flags.extended = 1;
  msg1.len            = 0;
  can1.write(msg1);

  // Invalidate stale pMBB32 data
  {
    uint32_t now = millis();
    if (now - lastUpdatePMBB1 > 400) { minCellV1 = 0xFFFF; maxCellV1 = 0; }
    if (now - lastUpdatePMBB2 > 400) { minCellV2 = 0xFFFF; maxCellV2 = 0; }
    if (now - lastUpdatePMBB3 > 400) { minCellV3 = 0xFFFF; maxCellV3 = 0; }
    lowestCellV  = std::min({minCellV1, minCellV2, minCellV3});
    highestCellV = std::max({maxCellV1, maxCellV2, maxCellV3});
  }

  if (pMBB32stale1 < 255) pMBB32stale1++;
  if (pMBB32stale2 < 255) pMBB32stale2++;
  if (pMBB32stale3 < 255) pMBB32stale3++;
  if (pMBB32stale1 > pMBB32staleMax) pMBB32staleMax = pMBB32stale1;
  if (pMBB32stale2 > pMBB32staleMax) pMBB32staleMax = pMBB32stale2;
  if (pMBB32stale3 > pMBB32staleMax) pMBB32staleMax = pMBB32stale3;

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

  static uint8_t startupGrace = 5; // 5 × 200ms = 1s
  if (startupGrace > 0) {
    if (--startupGrace == 0)
      pMBB32stale1 = pMBB32stale2 = pMBB32stale3 = 0;
  }

  if (startupGrace == 0) {
    static struct {
      uint8_t  *stale;
      uint32_t  id;
      uint8_t   num;
      uint8_t   phase;
      uint32_t  phaseTime;
      uint8_t   retryCount;
    } mods[] = {
      {&pMBB32stale1, 0xAF0100, 1, 0, 0, 0},
      {&pMBB32stale2, 0xAF0200, 2, 0, 0, 0},
      {&pMBB32stale3, 0xAF0300, 3, 0, 0, 0},
    };

    static struct { uint8_t state; uint32_t t; } ch2 = {0, 0};

    msg1.flags.extended = 1;

    // Ghost SA detection with 2s debounce
    {
      static uint32_t ghostSince = 0;
      noInterrupts(); bool ghostNow = pMBB32ghostSA; pMBB32ghostSA = false; interrupts();
      if (ghostNow) { if (ghostSince == 0) ghostSince = millis(); }
      else          { ghostSince = 0; }
      if (ghostSince && ch2.state == 0 && millis() - ghostSince >= 2000) {
        ghostSince = 0;
        noInterrupts(); pMBB32ftSeen[0] = pMBB32ftSeen[1] = pMBB32ftSeen[2] = 0; interrupts();
        Serial.println("pMBB32: ghost SA detected — cycling PDU CH2");
        sdQueueEventISR("GHOST_SA_CH2");
        PDUmsg1.buf[1] = 0;
        ch2.state = 1; ch2.t = millis();
        for (auto &mm : mods) { *mm.stale = 0; mm.phase = 0; mm.retryCount = 0; }
      }
    }

    // Incomplete frame-set detection (numChannels corruption)
    static uint32_t nextFtCheck = 0;
    if (ch2.state == 0 && millis() >= nextFtCheck) {
      nextFtCheck = millis() + 5000;
      noInterrupts();
      uint16_t snap[3] = {pMBB32ftSeen[0], pMBB32ftSeen[1], pMBB32ftSeen[2]};
      pMBB32ftSeen[0] = pMBB32ftSeen[1] = pMBB32ftSeen[2] = 0;
      interrupts();
      for (uint8_t i = 0; i < 3; i++) {
        bool seenFt01 = snap[i] & (1u << 0);
        bool seenFt03 = snap[i] & (1u << 2);
        if (seenFt01 && !seenFt03) {
          Serial.printf("pMBB32 #%u: ft=03 absent (seen=0x%03X) — forcing recovery\n",
                        i + 1, (unsigned)snap[i]);
          static const char* const ft03Evt[] = {"MOD1_FT03_ABSENT","MOD2_FT03_ABSENT","MOD3_FT03_ABSENT"};
          sdQueueEventISR(ft03Evt[i]);
          uint8_t *s = (i == 0 ? &pMBB32stale1 : i == 1 ? &pMBB32stale2 : &pMBB32stale3);
          *s = 255;
          nextFtCheck = millis() + 30000;
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
      noInterrupts(); pMBB32ghostSA = false; pMBB32ftSeen[0] = pMBB32ftSeen[1] = pMBB32ftSeen[2] = 0; interrupts();
      nextFtCheck = millis() + 10000;
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

  msg2.id             = 0xA100101;
  msg2.flags.extended = 1;
  msg2.len            = 1;
  msg2.buf[0]         = 0xE0; // request isolation state
  can2.write(msg2);
  msg2.buf[0] = 0x80; // request temperature
  can2.write(msg2);

  linReadValve();

  // EMP WP29 inverter cooling pump (CAN2) — must send ≥ 1 Hz
  msg2.id             = EMP_WP29_CMD_ID;
  msg2.flags.extended = 1;
  msg2.len            = 8;
  msg2.buf[0] = (invPumpSetpoint > 0) ? 0xFD : 0xFC;
  msg2.buf[1] = msg2.buf[2] = msg2.buf[4] = msg2.buf[5] = msg2.buf[6] = msg2.buf[7] = 0xFF;
  msg2.buf[3] = (uint8_t)(invPumpSetpoint * 2);
  can2.write(msg2);

  // EMP WP29 battery cooling pump (CAN1)
  msg1.id             = EMP_WP29_CMD_ID;
  msg1.flags.extended = 1;
  msg1.len            = 8;
  msg1.buf[0] = (battPumpSetpoint > 0) ? 0xFD : 0xFC;
  msg1.buf[1] = msg1.buf[2] = msg1.buf[4] = msg1.buf[5] = msg1.buf[6] = msg1.buf[7] = 0xFF;
  msg1.buf[3] = (uint8_t)(battPumpSetpoint * 2);
  can1.write(msg1);
}

/* t3 — 1000ms: heartbeat LED + stale diagnostic */
void callback_t3() {
  digitalToggleFast(LED_BUILTIN);
  Serial.printf("pMBB32 stale=%u/%u/%u  lo=%u hi=%u mV\n",
                pMBB32stale1, pMBB32stale2, pMBB32stale3,
                lowestCellV, highestCellV);
}
