#include <Arduino.h>
#include <TeensyTimerTool.h>
#include <FlexCAN_T4.h>
#include <LIN_master_HardwareSerial.h>  // LIN master on Serial3 (RX3=pin7, TX3=pin8)
#include "pMBB32.h"
#include "defines.h"
#include "Fsm.h"
//#include <Snooze.h>

//SnoozeDigital digital;
//SnoozeTimer timer;
//SnoozeAlarm  alarm;
//SnoozeBlock config_teensy40(alarm);

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
#define KLR_PIN 2  // physical key position 1 (accessory) — wakes hardware, no FSM role

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

VCUStateEnum VCUstate = VCU_STATE_OFF;
State state_Off(&Off_enter, &check_KL15, &Off_exit);
State state_PreCharge(&PreCharge_enter, &check_PreCharge, &PreCharge_exit);
State state_Idle(&Idle_enter, &check_Idle, &Idle_exit);
State state_Drive(&Drive_enter, &check_DriveState, &Drive_exit);
State state_Charge(&Charge_enter, &check_Charge, &Charge_exit);
State state_HeatPack(&HeatPack_enter, &check_HeatPack, &HeatPack_exit);
State state_CoolPack(&CoolPack_enter, &check_CoolPack, &CoolPack_exit);
State state_Fault(&Fault_enter, &check_Fault, &Fault_exit);
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
  if (VCUstate != VCU_STATE_OFF && VCUstate != VCU_STATE_FAULT)
    displayStatus(); // update RealDash
}//end of callback_cells_pdu()

/* t2 Callback
* This runs every t2 period (200ms).
* Tasks performed here:
*   1. Request all cell and temperature measurements from pMBB32s
*   2. Request min/max cell voltages from pMBB32s
*   3. Check for stale pMBB32 CAN and restart if necessary
*   4. Read isolation state from SIM100MOD
*/
void callback_t2() {
#ifdef debug_loop_timing
  callback_cell_sample_start = millis();
#endif
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

  // Stale recovery: non-blocking shutdown → 2 s delay → wake + contReportingEnable.
  // Phase 0 = idle, 1 = shutdown sent (waiting 2 s), 2 = wake pending.
  {
    static struct {
      uint8_t  *stale;
      uint32_t  id;
      uint8_t   num;
      uint8_t   phase;    // 0=idle, 1=awaiting 2s after shutdown
      uint32_t  phaseTime;
    } mods[] = {
      {&pMBB32stale1, 0xAF0100, 1, 0, 0},
      {&pMBB32stale2, 0xAF0200, 2, 0, 0},
      {&pMBB32stale3, 0xAF0300, 3, 0, 0},
    };
    bool anyWoke = false;
    msg1.flags.extended = 1;
    for (auto &m : mods) {
      if (m.phase == 0 && *m.stale > 5) {
        *m.stale = 0;
        Serial.printf("pMBB32 #%u stale — sending shutdown\n", m.num);
        msg1.id = m.id;
        msg1.len = 1;
        msg1.buf[0] = shutdown; // 0x55
        can1.write(msg1);
        m.phase = 1;
        m.phaseTime = millis();
      } else if (m.phase == 1 && millis() - m.phaseTime >= 2000) {
        Serial.printf("pMBB32 #%u — sending wake + contReportingEnable\n", m.num);
        msg1.id = m.id;
        msg1.len = 3;
        msg1.buf[0] = wakeup;
        msg1.buf[1] = channelCount16;
        msg1.buf[2] = numberOfDevices;
        can1.write(msg1);
        msg1.len = 1;
        msg1.buf[0] = contReportingEnable;
        can1.write(msg1);
        m.phase = 0;
        anyWoke = true;
      }
    }
    if (anyWoke) {
      msg1.id = 0xFF0000;
      msg1.len = 0;
      can1.write(msg1);
    }
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

  //SendCANFramesToEth(connectedClient);
#ifdef debug_loop_timing
  //callback_cell_sample_finish = millis() - callback_cell_sample_start;
  tft.setFont(Arial_16);
  tft.setCursor(160,152);
  tft.fillRect(160,152,80,20,ST77XX_BLUE);
  tft.println(millis() - callback_cell_sample_start);

  if ((millis() - callback_cell_sample_start) > callback_cell_sample_finish) {
    callback_cell_sample_finish = millis() - callback_cell_sample_start;
    tft.setCursor(160,172);
    tft.setFont(Arial_16);
    tft.fillRect(160,172,80,20,ST77XX_BLUE);
    tft.println(callback_cell_sample_finish);
  }
#endif
}//end of callback_cell_sample()

/* t3 Callback
* This runs every 1000ms. 
*/
void callback_t3() {
  //ReadDigitalStatuses();
  //ReadAnalogStatuses();
  //groundSpeed = myGNSS.getGroundSpeed() * 0.00223694;//convert mm/s to mph
  //GPSaltitude = myGNSS.getAltitudeMSL() / 3300;//feet
#ifndef TFT240_240_display
  digitalToggleFast(LED_BUILTIN);
#endif
}//end of callback1000ms()

#ifdef TCP_Interface
void teensyMAC(uint8_t *mac) {
    for(uint8_t by=0; by<2; by++) mac[by]=(HW_OCOTP_MAC1 >> ((1-by)*8)) & 0xFF;
    for(uint8_t by=0; by<4; by++) mac[by+2]=(HW_OCOTP_MAC0 >> ((3-by)*8)) & 0xFF;
    Serial.printf("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void startEthernet() {
    // start the Ethernet connection and the server:
    Ethernet.begin(mac, localIP);
    server.begin();
    //udpServer.begin(targetPort);
}
#endif
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

  // Reset stale counter when a cell voltage broadcast is received (0x18FF01yy–0x18FF0Cyy).
  // Min/max frames (0x18FF0Eyy) do NOT reset stale — they are explicitly polled and arrive
  // even when the 0xFF0000 trigger is missed. Stale only clears when the module is actually
  // broadcasting measurement data, which is what the recovery logic is meant to restore.
  {
    uint8_t frameType = (msg.id >> 8) & 0xFF;
    uint8_t modNum    = msg.id & 0xFF;
    if ((msg.id >> 16) == 0x18FF && frameType >= 0x01 && frameType <= 0x0C) {
      switch (modNum) {
        case 1: pMBB32stale1 = 0; break;
        case 2: pMBB32stale2 = 0; break;
        case 3: pMBB32stale3 = 0; break;
        default: break;
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

  digitalWriteFast(4, LOW);
}// end of can1Sniff(const CAN_message_t &msg)

/* CAN2 Receiver
* This function processes messages received on CAN2.
* 
* The IVT-MOD is configured by us for the following broadcast rates:
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
    case 0x621://IVT-MOD Current Sensor
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
      if(msg.buf[2] == 0xF9){
        switch (msg.buf[4]) {
          case 0:
            button_0x10_state = LOW; // reset so next button 5 press is detected as rising edge
            break;
          case 0x01://Park button pressed            
            new_0x01_state = HIGH;
            // If the switch changed, due to noise or pressing:
            if (new_0x01_state != last_0x01_state) {
              // reset the debouncing timer
              last_0x01_time = millis();
            }
            if ((millis() - last_0x01_time) > debounceDelay) {
              // whatever the reading is at, it's been there for longer than the debounce
              // delay, so take it as the actual current state.

              // if the button state has changed:
              if (new_0x01_state != button_0x01_state) {
                button_0x01_state = new_0x01_state;
                LDUdirection = LDU_DIR_STOP;
                Serial.println("BTN: Park");
              }
            }            
            break;
          case 0x02://Reverse button pressed
            new_0x02_state = HIGH;
            // If the switch changed, due to noise or pressing:
            if (new_0x02_state != last_0x02_state) {
              // reset the debouncing timer
              last_0x02_time = millis();
            }
            if ((millis() - last_0x02_time) > debounceDelay) {
              // whatever the reading is at, it's been there for longer than the debounce
              // delay, so take it as the actual current state.

              // if the button state has changed:
              if (new_0x02_state != button_0x02_state) {
                button_0x02_state = new_0x02_state;
                LDUdirection = LDU_DIR_REVERSE;
                Serial.println("BTN: Reverse");
              }
            }            
            break;
          case 0x04://Neutral button pressed
            new_0x04_state = HIGH;
            // If the switch changed, due to noise or pressing:
            if (new_0x04_state != last_0x04_state) {
              // reset the debouncing timer
              last_0x04_time = millis();
            }
            if ((millis() - last_0x04_time) > debounceDelay) {
              // whatever the reading is at, it's been there for longer than the debounce
              // delay, so take it as the actual current state.

              // if the button state has changed:
              if (new_0x04_state != button_0x04_state) {
                button_0x04_state = new_0x04_state;
                LDUdirection = LDU_DIR_NEUTRAL;
                Serial.println("BTN: Neutral");
              }
            }           
            break;
          case 0x08://Drive button pressed
            new_0x08_state = HIGH;
            // If the switch changed, due to noise or pressing:
            if (new_0x08_state != last_0x08_state) {
              // reset the debouncing timer
              last_0x08_time = millis();
            }
            if ((millis() - last_0x08_time) > debounceDelay) {
              // whatever the reading is at, it's been there for longer than the debounce
              // delay, so take it as the actual current state.

              // if the button state has changed:
              if (new_0x08_state != button_0x08_state) {
                button_0x08_state = new_0x08_state;
                LDUdirection = LDU_DIR_FORWARD;
                Serial.println("BTN: Drive");
              }
            }              
            break;
          case 0x10: // KL15 — Start button: enter On State (one-shot, Park exits)
            if (!button_0x10_state && !KL15state) {
              button_0x10_state = HIGH;
              KL15state = true;
              Serial.println("KL15: ON");
            }
            break;
          case 0x20://Speed mode button pressed
            new_0x20_state = HIGH;
            // If the switch changed, due to noise or pressing:
            if (new_0x20_state != last_0x20_state) {
              // reset the debouncing timer
              last_0x20_time = millis();
            }
            if ((millis() - last_0x20_time) > debounceDelay) {
              // whatever the reading is at, it's been there for longer than the debounce
              // delay, so take it as the actual current state.

              // if the button state has changed:
              if (new_0x20_state != button_0x20_state) {
                button_0x20_state = new_0x20_state;
                Serial.println("BTN: Speed Mode");
              }
            }            
            break;
          case 0x40://AUX button pressed
            new_0x40_state = HIGH;
            // If the switch changed, due to noise or pressing:
            if (new_0x40_state != last_0x40_state) {
              // reset the debouncing timer
              last_0x40_time = millis();
            }
            if ((millis() - last_0x40_time) > debounceDelay) {
              // whatever the reading is at, it's been there for longer than the debounce
              // delay, so take it as the actual current state.

              // if the button state has changed:
              if (new_0x40_state != button_0x40_state) {
                button_0x40_state = new_0x40_state;
                Serial.println("BTN: AUX");
              }
            }            
            break;
          case 0x80://Drive mode button pressed
            new_0x80_state = HIGH;
            // If the switch changed, due to noise or pressing:
            if (new_0x80_state != last_0x80_state) {
              // reset the debouncing timer
              last_0x80_time = millis();
            }
            if ((millis() - last_0x80_time) > debounceDelay) {
              // whatever the reading is at, it's been there for longer than the debounce
              // delay, so take it as the actual current state.

              // if the button state has changed:
              if (new_0x80_state != button_0x80_state) {
                button_0x80_state = new_0x80_state;
                Serial.println("BTN: Drive Mode");
              }
            }            
            break;
        default:
          Serial.println("Bad Button!");
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

    /* EMP WP29-12V-CV-A Water Pump -----------------------------
     * Status broadcast: 0xFBFE (64510) extended, 500kbps
     * TODO: confirm byte map / scaling from datasheet
     */
    case 0xFBFE://pump status broadcast
      pumpActualSpeed = (msg.buf[0]<<8) | msg.buf[1];// TODO: confirm byte map / scaling
      pumpStatus      = msg.buf[2];                  // TODO: confirm byte
      pumpFaults      = msg.buf[3];                  // TODO: confirm byte
      break;

  }

  switch (msg.id) {

  }
  digitalWriteFast(5, LOW);
}//end of can2Sniff(const CAN_message_t &msg)

void can3Sniff(const CAN_message_t &msg) {
  switch (msg.id) {
    case 500:
      //
      break;
    default:
      Serial.println(" Nothing ");
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

  // Step 1: wake each module
  for (uint8_t i = 0; i < 3; i++) {
    msg1.id = modIds[i];
    msg1.len = 3;
    msg1.buf[0] = wakeup;         // 0x01
    msg1.buf[1] = channelCount16; // 0x10: 16-cell module
    msg1.buf[2] = numberOfDevices;  // 0x02
    can1.write(msg1);
    delay(10);
  }

  // Step 2: enable continuous cell voltage reporting on each module (1-byte command)
  for (uint8_t i = 0; i < 3; i++) {
    msg1.id = modIds[i];
    msg1.len = 1;
    msg1.buf[0] = contReportingEnable; // 0x10
    can1.write(msg1);
    delay(10);
  }

  // Step 3: trigger first measurement cycle
  msg1.id = 0xFF0000;
  msg1.len = 0;
  can1.write(msg1);
  delay(1);
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
  
  power += 5;
  if(power >= 450)
    power = 0;
  
  IVTpackVoltage = 3840;
  //IVTpackCurrent = 5000;
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
  msg3.buf[2] = power*10;//kW = V/10
  msg3.buf[3] = (power*10)>>8;
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

#ifdef TFT240_240_display
  tft.setCursor(110,5);//Pack V is on line 5
  tft.setTextColor(ST77XX_YELLOW);
  tft.setFont(Arial_20);
  tft.fillRect(110,5,100,30,ST77XX_BLUE);
  tft.println((float)IVTpackVoltage/10, 1);//in mV
  tft.setCursor(110,35);//Pack A is on line 35
  tft.setFont(Arial_20);
  tft.fillRect(110,35,100,30,ST77XX_BLUE);
  tft.println((float)IVTpackCurrent/1000, 1);//in mA
  tft.setCursor(110,65);//Pack power is on line 65
  tft.setFont(Arial_20);
  tft.fillRect(110,65,100,30,ST77XX_BLUE);
  tft.println(IVTpower/1000, 2);//in Watts
  tft.setCursor(110,95);//12V battery is on line 95
  tft.setFont(Arial_20);
  tft.fillRect(110,95,100,30,ST77XX_BLUE);
  tft.println((float)batteryVoltage/100, 2);
#endif
  digitalWriteFast(6, LOW);
}//end of displayStatus()
#ifdef TCP_Interface
void SendCANFramesToEth(EthernetClient& client) {
  //client.beginFrame(clientIP, staticIP, 32)
      
  packet_t rxPacket;
  rxPacket.id = 0xC80;
  //rxPacket.rtr = CAN.packetRtr() ? 1 : 0;
  rxPacket.ide = msg3.flags.extended ? 1 : 0;
  rxPacket.dlc = 8;
  byte i = 0;
  for(i = 0; i < rxPacket.dlc; i++) {
    rxPacket.dataArray[i] = msg3.buf[i];
    if (i >= (sizeof(rxPacket.dataArray) / (sizeof(rxPacket.dataArray[0])))) {
      break;
    }
  }

  // Forward the received packet as RealDash '44' frame to the connected client
  forwardAsRD44Frame(&rxPacket, client);
}

void forwardAsRD44Frame(packet_t *packet, EthernetClient client) {
  // RealDash '44' frame format:
  // 4 bytes - 0x44,0x33,0x22,0x11
  // 4 bytes - CAN frame id number (32bit little endian value)
  // 8 bytes - CAN frame payload (data)

  byte header[] = {0x44, 0x33, 0x22, 0x11};
  client.write(header, 4);
  client.write((byte *)&packet->id, 4);
  client.write(packet->dataArray, 8);
}
#endif
#ifdef UBLOX_GNSS
void printPVTdata(UBX_NAV_PVT_data_t *ubxDataStruct)
{
    //Serial.println();

    // TODO: re-enable Serial printing once display/logging is implemented
    GPSaltitude = ubxDataStruct->hMSL; // Print the height above mean sea level
    //Serial.print(F(" Height above MSL: "));
    //Serial.print(GPSaltitude);
    //Serial.print(F(" (mm)"));

    groundSpeed = ubxDataStruct->gSpeed; // Print the ground speed
    //Serial.print(F(" Ground speed: "));
    //Serial.print(groundSpeed);
    //Serial.println(F(" (mm/s)"));
    fixType = ubxDataStruct->fixType; // Print the GNSS fix type
    //Serial.print(F(" Fix type: "));
    //Serial.print(fixType);
    //Serial.println(F(" (0=no fix, 1=dead reckoning only, 2=2D-fix, 3=3D-fix, 4=GNSS + dead reckoning, 5=time only fix)"));
}
#endif
// State transition functions
void Off_enter()
{
  Serial.println("Entering Off state");
  VCUstate = VCU_STATE_OFF;

  // Set keypad LEDs for Off state: buttons 1-7 off, button 8 amber blink (awaiting KL15)
  msg2.id = 0x18EF2100;
  msg2.flags.extended = 1;
  msg2.len = 8;
  msg2.buf[0] = 0x04;
  msg2.buf[1] = 0x1B;
  msg2.buf[5] = 0xFF;
  msg2.buf[6] = 0xFF;
  msg2.buf[7] = 0xFF;
  for (uint8_t btn = 1; btn <= 7; btn++) {
    msg2.buf[2] = KEYPAD_CMD_SET_LED;
    msg2.buf[3] = btn;
    msg2.buf[4] = KEYPAD_COLOR_OFF;
    can2.write(msg2);
  }
  msg2.buf[2] = KEYPAD_CMD_SET_LED;
  msg2.buf[3] = 0x08; // button 8 — drive mode, amber blink to indicate "waiting for KL15"
  msg2.buf[4] = KEYPAD_COLOR_AMBER;
  msg2.buf[5] = KEYPAD_MODE_BLINK;
  msg2.buf[6] = 0x00;
  msg2.buf[7] = 0xFF;
  can2.write(msg2);
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
  0 = send "KL30 connected" message
  1 = send "KL15 on" message
  2 = send "Something happened..." message
  3 = send "Charging stopped..." message
  4 = send "Charging started..." message
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
  PDUmsg1.buf[2] = 0x05; // CH3 2A — pre-charge relay on
}

void PreCharge_exit() {
  Serial.println("Exiting PreCharge state");
  PDUmsg1.buf[2] = 0x00; // CH3 off — pre-charge relay off
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
  // TODO: set pumpSetpoint and send to WP29; enable heater output
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
  // TODO: set pumpSetpoint and send to WP29; enable AC exchanger
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
  throttlePot1Raw = analogRead(THROTTLE_POT1_PIN);
  throttlePot2Raw = analogRead(THROTTLE_POT2_PIN);

  // ── 2. VERIFY — 5 % plausibility window between tracks ──────────────────
  uint16_t pct1 = constrain(map(throttlePot1Raw, THROTTLE_POT1_MIN, THROTTLE_POT1_MAX, 0, 100), 0, 100);
  uint16_t pct2 = constrain(map(throttlePot2Raw, THROTTLE_POT2_MIN, THROTTLE_POT2_MAX, 0, 100), 0, 100);
  throttlePlausibility = (abs((int16_t)pct1 - (int16_t)pct2) <= THROTTLE_PLAUSIBILITY_PCT);
  uint16_t pedalPct = throttlePlausibility ? pct1 : 0; // track 1 primary; fault → 0

  // ── 3. ARBITRATE ─────────────────────────────────────────────────────────
  brakePedal = (analogRead(BRAKE_PIN) > BRAKE_THRESHOLD);
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

  //digital.pinMode(KLR_PIN, INPUT_PULLDOWN, RISING);// KLR wake pin — TODO: Teensy sleep/wakeup

  initCAN(500000, 500000, 1000000);//start CAN1, CAN2, CAN3 at 500kbps, 500kbps, 1Mbps
  delay(100);

  t0.begin(callback_t0, 10000); // 10ms LDU torque command loop
  t0.priority(0);               // highest priority on ARM Cortex-M7 (0=highest, 255=lowest)

#ifndef TFT240_240_display
  pinMode(LED_BUILTIN, OUTPUT);//built-in LED or 240x240 TFT backlight
#else
  pinMode(LCD_BL, OUTPUT);
  digitalWrite(LCD_BL, HIGH);  // Turn LCD backlight on  
#endif

#ifdef UBLOX_GNSS
  Wire.setClock(400 * 1000);//for U-blox GPS
  Wire.begin();
  Wire.setTimeout(5);              // 5ms transaction timeout (I2CDriverWire API)
  
  while (myGNSS.begin() == false)
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    //while (1);
    myGNSS.hardReset();
    delay(500);
  }
  
  myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA); //Set the I2C port to output both NMEA and UBX messages
  //This will pipe all NMEA sentences to the serial port so we can see them
  //myGNSS.setNMEAOutputPort(Serial);

  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR
  //myGNSS.setNMEAOutputPort(SerialUSB);
  myGNSS.setNavigationFrequency(10); //Set output to 10 times a second
  //myGNSS.setAutoPVT(true);
  myGNSS.setAutoPVTcallbackPtr(&printPVTdata); // Enable automatic NAV PVT messages with callback to printPVTdata
  //myGNSS.setAutoNAVSATcallbackPtr(&newNAVSAT); // Enable automatic NAV SAT messages with callback to newNAVSAT
#endif

#ifdef TCP_Interface
  // get mac address
  //teensyMAC(mac);
  //startEthernet();
#endif

  //send PDU-8 driver settings - without sending 0x0A0630, this is simple ON/OFF control
  PDUmsg1.id = 0x0A0620;
  PDUmsg1.len = 8;
  PDUmsg1.buf[0] = 0;//channel 1 current limit 5A (5/0.4A = 13 or 0x0D) - negative contactor
  PDUmsg1.buf[1] = 0x05;//channel 2 current limit 2A (2/0.4A = 5) - pMBB32s
  PDUmsg1.buf[2] = 0;//channel 3 current limit 2A (2/0.4A = 5) - positive pre-charge
  PDUmsg1.buf[3] = 0;//channel 4 current limit 5A (5/0.4A = 13 or 0x0D) - positive contactor
  PDUmsg1.buf[4] = 0;
  PDUmsg1.buf[5] = 0;
  PDUmsg1.buf[6] = 0;
  PDUmsg1.buf[7] = 0; 
  can1.write(PDUmsg1);
  delay(1);

  //send PDU-8 driver outputs - only used for PWM control (unverified)
  PDUmsg2.id = 0x0A0630;
  PDUmsg2.len = 8;
  PDUmsg2.buf[0] = 0;//channel 1 - negative contactor
  PDUmsg2.buf[1] = 0xFE;//channel 2 - pMBB32
  PDUmsg2.buf[2] = 0;//channel 3 - positive pre-charge relay
  PDUmsg2.buf[3] = 0;//channel 4 - positive contac 
  PDUmsg2.buf[4] = 0;
  PDUmsg2.buf[5] = 0;
  PDUmsg2.buf[6] = 0;
  PDUmsg2.buf[7] = 0; 
  can1.write(PDUmsg2);

  t1.begin(callback_t1, t1CallbackRate);
  delay(100);
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
   * Command frame: 0x7A00 (31232) extended, 500kbps
   * Send initial speed setpoint (0 RPM / off) to pump.
   * TODO: confirm byte map / scaling from datasheet.
   */
  pumpSetpoint = 0;
  msg2.id = 0x7A00;
  msg2.flags.extended = 1;
  msg2.len = 8;
  msg2.buf[0] = pumpSetpoint >> 8;  // TODO: confirm byte map / scaling
  msg2.buf[1] = pumpSetpoint & 0xFF;
  msg2.buf[2] = 0;
  msg2.buf[3] = 0;
  msg2.buf[4] = 0;
  msg2.buf[5] = 0;
  msg2.buf[6] = 0;
  msg2.buf[7] = 0;
  can2.write(msg2);
  delay(1); 

#ifdef TFT240_240_display
  tft.init(240, 240);
  tft.setRotation(0);
  tft.fillScreen(ST77XX_BLUE);

  tft.setCursor(5,5);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setFont(Arial_20);
  tft.println("Pack V:");

  tft.setCursor(5,35);
  tft.println("Pack A:");
  tft.setCursor(60 ,65);
  tft.println("kW: ");
  tft.setCursor(60 ,95);
  tft.println("LV: ");

  tft.setFont(Arial_12);
  tft.setCursor(10,220);
  tft.println("EK9 EV VCU");
#endif
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

}//end of setup()


/* Main */
void loop() {
  // put your main code here, to run repeatedly:
  
  //callback_main_loop_start = millis();
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

    // Feed the sleep function its wakeup parameters, then go to sleep.
    //Snooze.sleep( config_teensy40 );// return module that woke processor


#ifdef UBLOX_GNSS
  myGNSS.checkUblox();    // returns on error if bus times out (5ms cap via setTimeout)
  myGNSS.checkCallbacks();
  // I2CDriverWire has no timeout-flag API; timeout errors surface as transaction failures
#endif

fsm.run_machine();

#ifdef debug_loop_timing
  //callback_main_loop_finish = millis() - callback_main_loop_start;
  tft.setFont(Arial_16);
  tft.setCursor(160,192);
  tft.fillRect(160,192,80,20,ST77XX_BLUE);
  tft.println(millis() - callback_main_loop_start);

  if ((millis() - callback_main_loop_start) > callback_main_loop_finish) {
    callback_main_loop_finish = millis() - callback_main_loop_start;
    tft.setCursor(160,212);
    tft.setFont(Arial_16);
    tft.fillRect(160,212,80,20,ST77XX_BLUE);
    tft.println(callback_main_loop_finish);
  }
#endif
  digitalWriteFast(3, LOW);
}//end of loop()