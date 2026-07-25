/* can_handlers.cpp — CAN1/2/3 receive callbacks and bus initialisation.
 *
 * The can1/can2/can3 objects are defined here (not main.cpp) because
 * FlexCAN_T4's constructor sets file-scope static routing pointers
 * (_CAN1/_CAN2/_CAN3 in FlexCAN_T4.tpp) that its begin()/ISR-trampoline
 * code also reads. Those statics have internal linkage, so every
 * translation unit that includes the header gets its own private copy.
 * If the objects were constructed in a different TU than the one that
 * calls begin() (which writes the real interrupt vector table entry),
 * the vector table would point at an ISR reading an unset (null)
 * routing pointer — the interrupt fires, the trampoline no-ops without
 * clearing the hardware flag, and the CPU re-enters it forever. Keeping
 * construction and begin() in the same TU keeps both copies consistent.
 */
#include <Arduino.h>
#include "defines.h"
#include "pMBB32.h"
#include "sdlog.h"
#include "can_handlers.h"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;// 500KHz - pMMB32, PDU-8
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;// 500KHz - LDU, EVCC, SIM100MOD, IVT-MOD, EMP W29, keypad
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;// 1Mhz - Wireless Gateway, RealDash

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
    case 0x18EFFF21://CAN keypad (PKP1600SI, 6 buttons — see dbc/PKP1600SI_J1939.dbc)
      if(msg.buf[2] == 0x01){ // Key Contact state (event-driven, enabled by default)
        bool pressed = (msg.buf[4] == 0x01);
        switch (msg.buf[3]) { // key number 1-6
          case KEYPAD_KEY_START_STOP: {
            // Rising edge only: first press while off starts (KL15state=true); a later
            // press while already on requests a stop, deferred until LDUrpm==0 lets the
            // relevant check_*() actually trigger KL15_OFF (same stopped-vehicle gate
            // already used for the Park button below).
            bool wasLow = !buttonStartStop;
            buttonStartStop = pressed ? HIGH : LOW;
            if (pressed && wasLow) {
              if (!KL15state) {
                KL15state = true;
                Serial.println("BTN: Start/Stop — start");
              } else {
                stopRequested = true;
                Serial.println("BTN: Start/Stop — stop requested");
              }
            }
            break;
          }
          case KEYPAD_KEY_PARK:
            buttonPark = pressed ? HIGH : LOW;
            if (pressed) { LDUdirection = LDU_DIR_STOP; Serial.println("BTN: Park"); }
            break;
          case KEYPAD_KEY_REVERSE:
            buttonReverse = pressed ? HIGH : LOW;
            if (pressed) { LDUdirection = LDU_DIR_REVERSE; Serial.println("BTN: Reverse"); }
            break;
          case KEYPAD_KEY_NEUTRAL:
            buttonNeutral = pressed ? HIGH : LOW;
            if (pressed) { LDUdirection = LDU_DIR_NEUTRAL; Serial.println("BTN: Neutral"); }
            break;
          case KEYPAD_KEY_DRIVE:
            buttonDrive = pressed ? HIGH : LOW;
            if (pressed) { LDUdirection = LDU_DIR_FORWARD; Serial.println("BTN: Drive"); }
            break;
          case KEYPAD_KEY_SPEED_MODE:
            buttonSpeedMode = pressed ? HIGH : LOW;
            if (pressed) Serial.println("BTN: Speed Mode");
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
      sdLogEvent(evccIsACSession ? "EVCC_SESSION_AC" : "EVCC_SESSION_DC");
      // DC: EVCC handles its own pre-charge and contactors autonomously; VCU stays in KL15C.
      // AC: check_KL15C() detects evccIsACSession and fires AC_CHARGE_START → PreCharge → Charge.
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
      sdLogEvent("EVCC_EMERG_STOP");
      // TODO: fsm.trigger(FAULT_EV) when charge FSM states are split
      break;
    case EVCC_SESSION_END:  // 0x68007 — charge session finished normally
      EVCCsessionActive = false;
      EVCCsystemEnable  = false;
      evccIsACSession   = false;
      chargeMode        = false;
      acReadyToDeliver  = false;
      sdLogEvent("EVCC_SESSION_END");
      // TODO: fsm.trigger(CHARGE_OFF) when charge FSM states are split
      break;
    case EVCC_CTRL_STATUS:  // 0x68009 — EVCC heartbeat (200ms timeout)
      EVCCstage         = msg.buf[0]; // 0=Booting … 8=Charging … 10=Finished
      lastExtActivityMs = millis();   // heartbeat resets KL15C inactivity timer
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
  lastExtActivityMs = millis(); // any gateway frame resets KL15C inactivity timer
  switch (msg.id) {
    case 0xC84: // status request from wireless gateway — answered (0xC85) by callback_t2()
      Serial.println("CAN3: 0xC84 status request received");
      gatewayStatusRequestPending = true;
      break;
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

  // CAN3 temporarily disabled: nothing is currently connected to it (wireless gateway
  // unplugged, Odroid has no CAN interface), and an unterminated/floating bus was
  // triggering spurious wake-from-sleep cycles via CAN3_RX_PIN (see sleep.cpp). One-line
  // re-enable once the gateway is back in service.
  // can3.begin();
  // can3.setBaudRate(CAN3baud);
  // can3.setMaxMB(16);
  // can3.enableFIFO();
  // can3.enableFIFOInterrupt();
  // can3.onReceive(FIFO, can3Sniff);
  //can3.mailboxStatus();
}//end of initCAN (int CAN1baud, int CAN2baud, int CAN3baud)
