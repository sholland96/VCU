/* can_handlers.cpp — CAN1/2/3 receive callbacks and bus initialisation. */
#include "vcu.h"
#include "can_handlers.h"
#include "sdlog.h"

volatile bool     pMBB32ghostSA   = false;
volatile uint16_t pMBB32ftSeen[3] = {0, 0, 0};

/* CAN1 Receiver — pMBB32 cell monitors, PDU-8, battery cooling pump */
void can1Sniff(const CAN_message_t &msg) {
  digitalWriteFast(4, HIGH);

  if ((msg.id & 0x00000F00) == 0x00000E00) { // 0x18FF0Eyy — min/max cell response
    switch (msg.id) {
      case 0x18FF0E01:
        minCellV1 = (msg.buf[minCellHighByte]*256) + msg.buf[minCellLowByte];
        minCell1  = msg.buf[0];
        maxCellV1 = (msg.buf[maxCellHighByte]*256) + msg.buf[maxCellLowByte];
        maxCell1  = msg.buf[1];
        lastUpdatePMBB1 = millis();
        break;
      case 0x18FF0E02:
        minCellV2 = (msg.buf[minCellHighByte]*256) + msg.buf[minCellLowByte];
        minCell2  = msg.buf[0];
        maxCellV2 = (msg.buf[maxCellHighByte]*256) + msg.buf[maxCellLowByte];
        maxCell2  = msg.buf[1];
        lastUpdatePMBB2 = millis();
        break;
      case 0x18FF0E03:
        minCellV3 = (msg.buf[minCellHighByte]*256) + msg.buf[minCellLowByte];
        minCell3  = msg.buf[0];
        maxCellV3 = (msg.buf[maxCellHighByte]*256) + msg.buf[maxCellLowByte];
        maxCell3  = msg.buf[1];
        lastUpdatePMBB3 = millis();
        break;
    }
    lowestCellV = std::min({minCellV1, minCellV2, minCellV3});
    if (lowestCellV == minCellV1) lowestCell = minCell1;
    if (lowestCellV == minCellV2) lowestCell = minCell2 + 32;
    if (lowestCellV == minCellV3) lowestCell = minCell3 + 64;
    highestCellV = std::max({maxCellV1, maxCellV2, maxCellV3});
    if (highestCellV == maxCellV1) highestCell = maxCell1;
    if (highestCellV == maxCellV2) highestCell = maxCell2 + 32;
    if (highestCellV == maxCellV3) highestCell = maxCell3 + 64;
  }

  // Track frame-type coverage and detect ghost SAs (TOTAL_ICS corruption).
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
  if ((msg.id >> 16) == 0x18FF) {
    Serial.printf("CAN1 0x%08X [%d]", msg.id, msg.len);
    for (uint8_t i = 0; i < msg.len; i++) Serial.printf(" %02X", msg.buf[i]);
    Serial.println();
  }
#endif

  // EMP WP29 battery cooling pump (CAN1)
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
}

/* CAN2 Receiver — IVT-S, keypad, SIM100MOD, LDU, EMP pump, EVCC */
void can2Sniff(const CAN_message_t &msg) {
  digitalWriteFast(5, HIGH);
  switch (msg.id) {
    case 0x621: IVTpackCurrent    = (msg.buf[2]<<24)|(msg.buf[3]<<16)|(msg.buf[4]<<8)|msg.buf[5]; break;
    case 0x622: IVTpackVoltage    = (msg.buf[2]<<24)|(msg.buf[3]<<16)|(msg.buf[4]<<8)|msg.buf[5]; break;
    case 0x623: IVTpreChargeV     = (msg.buf[2]<<24)|(msg.buf[3]<<16)|(msg.buf[4]<<8)|msg.buf[5]; break;
    case 0x624: IVTvoltage3       = (msg.buf[2]<<24)|(msg.buf[3]<<16)|(msg.buf[4]<<8)|msg.buf[5]; break;
    case 0x625: IVTtemp           = (msg.buf[2]<<24)|(msg.buf[3]<<16)|(msg.buf[4]<<8)|msg.buf[5]; break;
    case 0x626: IVTpower          = (msg.buf[2]<<24)|(msg.buf[3]<<16)|(msg.buf[4]<<8)|msg.buf[5]; break;
    case 0x627: IVTcoulombCounter = (msg.buf[2]<<24)|(msg.buf[3]<<16)|(msg.buf[4]<<8)|msg.buf[5]; break;
    case 0x628: IVTenergyCounter  = (msg.buf[2]<<24)|(msg.buf[3]<<16)|(msg.buf[4]<<8)|msg.buf[5]; break;

    case 0x18EFFF21: // CAN keypad — Key Contact state event
      if (msg.buf[2] == 0x01) {
        bool pressed = (msg.buf[4] == 0x01);
        switch (msg.buf[3]) {
          case 1: button_0x01_state = pressed ? HIGH : LOW;
                  if (pressed) { LDUdirection = LDU_DIR_STOP; Serial.println("BTN: Park"); }
                  break;
          case 2: button_0x02_state = pressed ? HIGH : LOW;
                  if (pressed) { LDUdirection = LDU_DIR_REVERSE; Serial.println("BTN: Reverse"); }
                  break;
          case 3: button_0x04_state = pressed ? HIGH : LOW;
                  if (pressed) { LDUdirection = LDU_DIR_NEUTRAL; Serial.println("BTN: Neutral"); }
                  break;
          case 4: button_0x08_state = pressed ? HIGH : LOW;
                  if (pressed) { LDUdirection = LDU_DIR_FORWARD; Serial.println("BTN: Drive"); }
                  break;
          case 5: // KL15 — one-shot start, Park exits
            if (pressed && !button_0x10_state && !KL15state) {
              button_0x10_state = HIGH; KL15state = true; Serial.println("KL15: ON");
            } else if (!pressed) {
              button_0x10_state = LOW;
            }
            break;
          case 6: button_0x20_state = pressed ? HIGH : LOW;
                  if (pressed) Serial.println("BTN: Speed Mode");
                  break;
          case 7: button_0x40_state = pressed ? HIGH : LOW;
                  if (pressed) Serial.println("BTN: AUX");
                  break;
          case 8: button_0x80_state = pressed ? HIGH : LOW;
                  if (pressed) Serial.println("BTN: Drive Mode");
                  break;
          default: Serial.printf("BTN: unknown key %u\n", msg.buf[3]); break;
        }
      }
      break;

    case 0xA100100: // SIM100MOD Isolation Monitor
      switch (msg.buf[0]) {
        case 0xE0: SIM100MODohmsPerVolt = (msg.buf[2]<<8)|msg.buf[3]; break;
        case 0xE1: SIM100MODRpKohms = (msg.buf[2]<<8)|msg.buf[3];
                   SIM100MODRnKohms = (msg.buf[5]<<8)|msg.buf[6]; break;
        case 0xE2: SIM100MODCpnF = (msg.buf[2]<<8)|msg.buf[3];
                   SIM100MODCnnF = (msg.buf[5]<<8)|msg.buf[6]; break;
        case 0x0E3: SIM100MODVp = (msg.buf[2]<<8)|msg.buf[3];
                    SIM100MODVn = (msg.buf[5]<<8)|msg.buf[6]; break;
        case 0xE4: SIM100MODVb    = (msg.buf[2]<<8)|msg.buf[3];
                   SIM100MODVbMax = (msg.buf[5]<<8)|msg.buf[6]; break;
        case 0xE5: SIMM100MODerrorFlags = msg.buf[1]; break;
        case 0x80: SIM100MODtemp = (msg.buf[3]<<8)|msg.buf[4]; break;
      }
      break;

    case 0x000A0610:
      break;

    // OpenInverter Tesla LDU V2
    case 0x19A:
      LDUrpm          = (int32_t)((int16_t)((msg.buf[0]<<8)|msg.buf[1]));
      LDUtorque       = (int16_t)((msg.buf[2]<<8)|msg.buf[3]);
      LDUmotorTemp    = (int16_t)((msg.buf[4]<<8)|msg.buf[5]);
      LDUinverterTemp = 0;
      LDUdcVoltage    = 0;
      LDUstatus       = msg.buf[6];
      break;
    case 0x55A:
      LDUfaults = (msg.buf[0]<<8)|msg.buf[1];
      break;

    // EMP WP29 inverter cooling pump (CAN2)
    case EMP_WP29_STATUS1:
      invPumpStatus = (msg.buf[0] >> 2) & 0x0F;
      invPumpSpeed  = ((uint16_t)msg.buf[2] << 8) | msg.buf[1];
      invPumpFaults = msg.buf[7] & 0x0F;
      break;
    case EMP_WP29_STATUS3:
      break;

    // Advantics Generic Power Modules protocol — 29-bit extended IDs
    case EVCC_NEW_SESSION:
      EVCCplugType      = msg.buf[1];
      EVCCsessionActive = true;
      EVCCsystemEnable  = true;
      EVCCemergencyStop = false;
      evccIsACSession   = !EVCC_PLUG_IS_DC(EVCCplugType);
      chargeMode        = evccIsACSession;
      sdLogEvent(evccIsACSession ? "EVCC_SESSION_AC" : "EVCC_SESSION_DC");
      break;
    case EVCC_CHARGING_LOOP:
      break;
    case EVCC_EMERG_STOP:
      EVCCemergencyStop = true;
      EVCCsessionActive = false;
      EVCCsystemEnable  = false;
      evccIsACSession   = false;
      chargeMode        = false;
      acReadyToDeliver  = false;
      sdLogEvent("EVCC_EMERG_STOP");
      break;
    case EVCC_SESSION_END:
      EVCCsessionActive = false;
      EVCCsystemEnable  = false;
      evccIsACSession   = false;
      chargeMode        = false;
      acReadyToDeliver  = false;
      sdLogEvent("EVCC_SESSION_END");
      break;
    case EVCC_CTRL_STATUS:
      EVCCstage         = msg.buf[0];
      lastExtActivityMs = millis();
      break;
    case EVCC_INS_TEST:
    case EVCC_PRECHARGE:
    case EVCC_STATUS_CHANGE:
      break;
  }

  // Advantics v2.5 PEV protocol — standard 11-bit IDs
  switch (msg.id) {
    case EVCC_EVSE_INFO: // 0x600 — byte2=Pins
      if (!EVCCsessionActive && EVCC_PINS_IS_AC(msg.buf[2])) {
        EVCCsessionActive = true;
        EVCCsystemEnable  = true;
        EVCCemergencyStop = false;
        evccIsACSession   = true;
        chargeMode        = true;
      }
      break;
    case EVCC_AC_CTRL: // 0x601 — Ready_To_Deliver_Power bit 0
      acReadyToDeliver = (msg.buf[0] & 0x01u) != 0;
      break;
  }
  digitalWriteFast(5, LOW);
}

/* CAN3 Receiver — wireless gateway */
void can3Sniff(const CAN_message_t &msg) {
  lastExtActivityMs = millis();
  switch (msg.id) {
    case 500: break;
    default:  break;
  }
}

void initCAN(int CAN1baud, int CAN2baud, int CAN3baud) {
  msg1.flags.extended = 1; msg1.len = 8; memset(msg1.buf, 0, 8);
  msg2.flags.extended = 1; msg2.len = 8; memset(msg2.buf, 0, 8);
  msg3.flags.extended = 1; msg3.len = 8; memset(msg3.buf, 0, 8);
  PDUmsg1.flags.extended = 1; PDUmsg1.len = 8; memset(PDUmsg1.buf, 0, 8);
  PDUmsg2.flags.extended = 1; PDUmsg2.len = 8; memset(PDUmsg2.buf, 0, 8);
  LDUmsg.id = LDU_CMD_ID; LDUmsg.flags.extended = 0; LDUmsg.len = 8; memset(LDUmsg.buf, 0, 8);

  can1.begin(); can1.setBaudRate(CAN1baud); can1.setMaxMB(16);
  can1.enableFIFO(); can1.enableFIFOInterrupt(); can1.onReceive(FIFO, can1Sniff);

  can2.begin(); can2.setBaudRate(CAN2baud); can2.setMaxMB(16);
  can2.enableFIFO(); can2.enableFIFOInterrupt(); can2.onReceive(FIFO, can2Sniff);

  can3.begin(); can3.setBaudRate(CAN3baud); can3.setMaxMB(16);
  can3.enableFIFO(); can3.enableFIFOInterrupt(); can3.onReceive(FIFO, can3Sniff);
}
