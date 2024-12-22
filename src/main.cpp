#include <Arduino.h>
//#include <i2c_driver_wire.h> //Needed for I2C to GNSS
#include <TeensyTimerTool.h>
#include <FlexCAN_T4.h>
#include "pMBB32.h"
#include <NativeEthernet.h>
//#include <Snooze.h>
#include "defines.h"
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //Click here to get the library: http://librarymanager/All#SparkFun_u-blox_GNSS

#include <SPI.h>
#include <ST7735_t3.h> // Hardware-specific library
#include <ST7789_t3.h> // Hardware-specific library
#include <ST7735_t3_font_Arial.h>
//#include <ST7735_t3_font_ArialBold.h>

#define TFT_RST    32   // chip reset
#define TFT_DC     9   // tells the display if you're sending data (D) or commands (C)   --> WR pin on TFT
#define TFT_MOSI   11  // Data out    (SPI standard)
#define TFT_SCLK   13  // Clock out   (SPI standard)
#define TFT_CS     10  // chip select (SPI standard)

int LCD_BL = 33;       // LCD back light control

//#define debug_loop_timing

ST7789_t3 tft = ST7789_t3(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

//SFE_UBLOX_GNSS myGNSS;

//SnoozeDigital digital;
//SnoozeTimer timer;
//SnoozeAlarm  alarm;
//SnoozeBlock config_teensy40(alarm);

using namespace TeensyTimerTool;
PeriodicTimer t1(RTC); // generate a timer from the pool (Pool: 2xGPT, 16xTMR(QUAD), 20xTCK)
PeriodicTimer t2(GPT1); // generate a timer from the pool (Pool: 2xGPT, 16xTMR(QUAD), 20xTCK)
//PeriodicTimer t3(GPT2); // generate a timer from the pool (Pool: 2xGPT, 16xTMR(QUAD), 20xTCK)

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;// pMMB32, PDU-8, keypad, EVCC
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;// Sendyne, IVT-S  
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;// RealDash 

CAN_message_t msg1;//.buf[7] = {0x0D, 0x05, 0x05, 0x0D, 0, 0, 0, 0};
CAN_message_t msg2;
CAN_message_t msg3;
CAN_message_t PDUmsg1;
CAN_message_t PDUmsg2;



void callback_cells_pdu() {
  //send PDU-8 driver settings every t1 period
  can1.write(PDUmsg1);
}//end of callback_cells_pdu()

void callback_cell_sample() {
  /*  This runs every t2 period (150ms) 
  *   1. Request all cell and temperature measurements from pMBB32s
  *   2. Request min/max cell voltages from pMBB32s
  *   3. Check for stale pMBB32 CAN and restart if necessary
  *   4. Update RealDash
  *   5. Read isolation state from SIM100MOD
  *   6. Update RealDash
  */
  callback_cell_sample_start = millis();
  // request all cell and temperature measurements every t2 period
  msg1.id = 0xFF0000;
  msg1.len = 0;
  can1.write(msg1);  //send sample command every 1 second
  delay(20);
  //check min/max cell voltages on each pMBB32 every 3 t2 periods
  switch (counter) {
    case 1: 
      msg1.id = 0xCF0100;//request min/max cells
      msg1.len = 0;
      can1.write(msg1);
      delay(20);
      msg1.id = 0xAF0100; 
      msg1.len = 3;
      msg1.buf[0] = 0x01;
      msg1.buf[1] = 0x10;
      msg1.buf[2] = 0x2;
      can1.write(msg1);
      break;
    case 2:
      msg1.id = 0xCF0200;//request min/max cells
      msg1.len = 0;
      can1.write(msg1);
      delay(20);
      msg1.id = 0xAF0200; 
      msg1.len = 3;
      msg1.buf[0] = 0x01;
      msg1.buf[1] = 0x10;
      msg1.buf[2] = 0x2;
      can1.write(msg1);
      break;
    case 3:
      counter = 0;
      msg1.id = 0xCF0300;//request min/max cells
      msg1.len = 0;
      can1.write(msg1);
      delay(20);
      msg1.id = 0xAF0300; 
      msg1.len = 3;
      msg1.buf[0] = 0x01;
      msg1.buf[1] = 0x10;
      msg1.buf[2] = 0x2;
      can1.write(msg1);
      break;
  }
  counter++;
  delay(20);
/* 
  if(PDUmsg2.buf[1] = pMBB32powerOff) { 
    PDUmsg2.buf[1] = pMBB32powerOn;
    can1.write(PDUmsg2);
    delay(100);
    wakepMBB32();
  }
  else {
        
    //check for stale pMBB32 CAN and restart if necessary
    if((pMBB32stale1 > 50)|(pMBB32stale2 > 50)|(pMBB32stale3 > 50)){
      PDUmsg2.buf[1] = pMBB32powerOff;
      can1.write(PDUmsg2);
      pMBB32stale1 = 0;
      pMBB32stale2 = 0;
      pMBB32stale3 = 0;
      PDUmsg2.buf[1] = pMBB32powerOn;
      can1.write(PDUmsg2);
      delay(500);
    }

  } */
  if(pMBB32stale1 > pMBB32staleMax)
    pMBB32staleMax = pMBB32stale1;
  if(pMBB32stale2 > pMBB32staleMax)
    pMBB32staleMax = pMBB32stale2;
  if(pMBB32stale3 > pMBB32staleMax)
    pMBB32staleMax = pMBB32stale3;
  
  if(pMBB32stale1 > 50){
    //send wakeup to pMBB32 #1
    msg1.id = 0xAF0100; 
    msg1.len = 3;
    msg1.buf[0] = 0x01;
    msg1.buf[1] = 0x10;
    msg1.buf[2] = 0x2;
    can1.write(msg1);
    delay(1);
  }
  if(pMBB32stale2 > 50){
    //send wakeup to pMBB32 #2
    msg1.id = 0xAF0200; 
    msg1.len = 3;
    msg1.buf[0] = 0x01;
    msg1.buf[1] = 0x10;
    msg1.buf[2] = 0x2;
    can1.write(msg1);
    delay(1);
  }
  if(pMBB32stale3 > 50){
    //send wakeup to pMBB32 #3
    msg1.id = 0xAF0300; 
    msg1.len = 3;
    msg1.buf[0] = 0x01;
    msg1.buf[1] = 0x10;
    msg1.buf[2] = 0x2;
    can1.write(msg1);
    delay(1);
  }

  msg2.id = 0xA100101;//send SIM100MOD Request Isolation State command
  msg2.flags.extended = 1;
  msg2.len = 1;
  msg2.buf[0] = 0xE0;//request isolation state
  can2.write(msg2);
  delay(1); 
  msg2.id = 0xA100101;//send SIM100MOD Request Isolation Resistances command
  msg2.len = 1;
  msg2.buf[0] = 0xE1;//request isolation resistances
  can2.write(msg2);
  delay(1);
  msg2.id = 0xA100101;//send SIM100MOD Request Isolation Capacitances command
  msg2.len = 1;
  msg2.buf[0] = 0xE2;//request isolation capacitances
  can2.write(msg2);
  delay(1);
  msg2.id = 0xA100101;//send SIM100MOD Request Voltages Vp and Vn command
  msg2.len = 1;
  msg2.buf[0] = 0xE3;//request Vp and Vn voltages
  can2.write(msg2);
  delay(1);
  msg2.id = 0xA100101;//send SIM100MOD Request Battery Voltage command
  msg2.len = 1;
  msg2.buf[0] = 0xE4;//request battery voltage
  can2.write(msg2);
  delay(1);
  msg2.id = 0xA100101;//send SIM100MOD Request Error Flags command
  msg2.len = 1;
  msg2.buf[0] = 0xE5;//request isolation resistances
  can2.write(msg2);
  delay(1);
  
  displayStatus();//update RealDash
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

void callback1000ms() {
  //ReadDigitalStatuses();
  //ReadAnalogStatuses();
  //groundSpeed = myGNSS.getGroundSpeed() * 0.00223694;//convert mm/s to mph
  //GPSaltitude = myGNSS.getAltitudeMSL() / 3300;//feet
  //digitalToggleFast(LED_BUILTIN);
}//end of callback1000ms()

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

void can1Sniff(const CAN_message_t &msg) {
  digitalWriteFast(3, HIGH);
  if ((msg.id & 0x00000F00) == 0x00000E00) {//received 0x18FF0Eyy    
    switch (msg.id & 0x0000000F) {
      case 1:
        minCellV1 = (msg.buf[minCellHighByte]*256) + msg.buf[minCellLowByte];
        minCell1 = msg.buf[0];
        maxCellV1 = (msg.buf[maxCellHighByte]*256) + msg.buf[maxCellLowByte];
        maxCell1 = msg.buf[1];
        break;
      case 2:
        minCellV2 = (msg.buf[minCellHighByte]*256) + msg.buf[minCellLowByte];
        minCell2 = msg.buf[0];
        maxCellV2 = (msg.buf[maxCellHighByte]*256) + msg.buf[maxCellLowByte];
        maxCell2 = msg.buf[1];
        break;
      case 3:
        minCellV3 = (msg.buf[minCellHighByte]*256) + msg.buf[minCellLowByte];
        minCell3 = msg.buf[0];
        maxCellV3 = (msg.buf[maxCellHighByte]*256) + msg.buf[maxCellLowByte];
        maxCell3 = msg.buf[1];
        break;
      default:
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

  // check for stale pMBB32 measurements requests. We should see a response to FF command regularly
  if (msg.id & 0x18FF0E00) {//(msg.id & 0x00000F00) == 0x00000E00
    switch (msg.id & 0x0000000F) {
        case 1:
            pMBB32stale1 = 0;
            pMBB32stale2++;
            pMBB32stale3++;
            break;
        case 2:
            pMBB32stale1++;
            pMBB32stale2 = 0;
            pMBB32stale3++;
            break;
        case 3:
            pMBB32stale1++;
            pMBB32stale2++;
            pMBB32stale3 = 0;
            break;
        default:
            break;
    }
  } else {
      pMBB32stale1++;
      pMBB32stale2++;
      pMBB32stale3++;
  }

  
  digitalWriteFast(3, LOW);
}// end of can1Sniff(const CAN_message_t &msg)

void can2Sniff(const CAN_message_t &msg) {
  digitalWriteFast(4, HIGH);
  switch (msg.id) {
    case 0x521:
      IVTpackCurrent = msg.buf[2]<<24 & msg.buf[3]<<16 & msg.buf[4]<<8 & msg.buf[5];
      break;
    case 0x522:
      IVTpackVoltage = msg.buf[2]<<24 & msg.buf[3]<<16 & msg.buf[4]<<8 & msg.buf[5];
      break;
    case 0x523:
      IVTpreChargeV = msg.buf[2]<<24 & msg.buf[3]<<16 & msg.buf[4]<<8 & msg.buf[5];
      break;
    case 0x524:
      IVTvoltage3 = msg.buf[2]<<24 & msg.buf[3]<<16 & msg.buf[4]<<8 & msg.buf[5];
      break;
    case 0x525:
      IVTtemp = msg.buf[2]<<24 & msg.buf[3]<<16 & msg.buf[4]<<8 & msg.buf[5];
      break;
    case 0x526:
      IVTpower = msg.buf[2]<<24 & msg.buf[3]<<16 & msg.buf[4]<<8 & msg.buf[5];
      break;
    case 0x527:
      IVTcoulombCounter = msg.buf[2]<<24 & msg.buf[3]<<16 & msg.buf[4]<<8 & msg.buf[5];
      break;
    case 0x528:
      IVTenergyCounter = msg.buf[2]<<24 & msg.buf[3]<<16 & msg.buf[4]<<8 & msg.buf[5];
      break;
    case 0x18EFFF21:
      //Serial.print(msg.buf[msg.buf[4]] < 16 ? "0" : ""); Serial.println(msg.buf[4], HEX);
      if(msg.buf[2] == 0xF9){
        switch (msg.buf[4]) {
          case 0:
            //
            break;
          case 0x01://Park button pressed            
            if((keypadStatus & 0x01) == 1){
              bitClear(keypadStatus, 0);
            }else{          
              bitSet(keypadStatus, 0);
            }            
            break;
          case 0x02://Reverse button pressed
            if((keypadStatus & 0x02) == 1){
              bitClear(keypadStatus, 1);
            }else{          
              bitSet(keypadStatus, 1);
            }            
            break;
          case 0x04://Neutral button pressed
            if((keypadStatus & 0x03) == 1){
              bitClear(keypadStatus, 2);
            }else{          
              bitSet(keypadStatus, 2);
            }            
            break;
          case 0x08://Drive button pressed
            if((keypadStatus & 0x04) == 1){
              bitClear(keypadStatus, 3);
            }else{          
              bitSet(keypadStatus, 3);
            }            
            break;
          case 0x10://Ignition button pressed
            if((keypadStatus & 0x05) == 1){
              bitClear(keypadStatus, 4);
            }else{          
              bitSet(keypadStatus, 4);
            }            
            break;
          case 0x20://Speed mode button pressed
            if((keypadStatus & 0x06) == 1){
              bitClear(keypadStatus, 5);
            }else{          
              bitSet(keypadStatus, 5);
            }            
            break;
          case 0x40://AUX button pressed
            if((keypadStatus & 0x07) == 1){
              bitClear(keypadStatus, 6);
            }else{          
              bitSet(keypadStatus, 6);
            }            
            break;
          case 0x80://Drive mode button pressed
            if((keypadStatus & 0x08) == 1){
              bitClear(keypadStatus, 7);
            }else{          
              bitSet(keypadStatus, 7);
            }            
            break;
        default:
          Serial.println("Bad Button!");
          break;
        }
      }
      break;
    case 0xA100100:
      //Serial.print("SIM100MOD Isolation State: "); Serial.println(msg.buf[0], HEX);
      switch (msg.buf[0]);{
        case 0xE0:
          SIM100MODohmsPerVolt = msg.buf[2]<<8 & msg.buf[3];//Ω/V
          break;
        case 0xE1:
          SIM100MODRpKohms = msg.buf[2]<<8 & msg.buf[3];//kΩ positive
          SIM100MODRnKohms = msg.buf[5]<<8 & msg.buf[6];//kΩ negative
          break;
        case 0xE2:
          SIM100MODCpnF = msg.buf[2]<<8 & msg.buf[3];//Cp nF
          SIM100MODCnnF = msg.buf[5]<<8 & msg.buf[6];//Cn nF
          break;
        case 0x0E3:
          SIM100MODVp = msg.buf[2]<<8 & msg.buf[3];//Vp
          SIM100MODVn = msg.buf[5]<<8 & msg.buf[6];//Vn
          break;
        case 0xE4:
          SIM100MODVb = msg.buf[2]<<8 & msg.buf[3];//Vb
          SIM100MODVbMax = msg.buf[5]<<8 & msg.buf[6];//Vb max
          break;
        case 0xE5:
          SIMM100MODerrorFlags = msg.buf[1];//SIM100MOD error flags
          break;
      }
      
      break;
    case 0x000A0610:

      break;
    
  }

  switch (msg.id) {
    
  }
  digitalWriteFast(4, LOW);
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
  //PDUmsg.buf[1] = pMBB32powerOn;
  //can1.write(PDUmsg1);
  //delay(200);
  
  //send wakeup to pMBB32 #1
  msg1.id = 0xAF0100; 
  msg1.len = 3;
  msg1.buf[0] = 0x01;
  msg1.buf[1] = 0x10;
  msg1.buf[2] = 0x2;
  can1.write(msg1);
  delay(20);
  
  //send wakeup to pMBB32 #2
  msg1.id = 0xAF0200;
  msg1.len = 3;
  msg1.buf[0] = 0x01;
  msg1.buf[1] = 0x10;
  msg1.buf[2] = 0x2;
  can1.write(msg1);
  delay(20);

  //send wakeup to pMBB32 #3
  msg1.id = 0xAF0300;
  msg1.len = 3;
  msg1.buf[0] = 0x01;
  msg1.buf[1] = 0x10;
  msg1.buf[2] = 0x2;
  can1.write(msg1);
  delay(20);

  //send broadcast start of measurement command to pMBB32
  msg1.id = 0xFF0000;
  msg1.len = 0;
  can1.write(msg1);
  delay(10);
}

void shutdownpMBB32() {
  //send shutdown to pMBB32 #3
  msg1.id = 0xAF0300; 
  msg1.len = 1;
  msg1.buf[0] = 0x55;
  can1.write(msg1);
  delay(5);
  can1.write(msg1);
  delay(5);

  //send shutdown to pMBB32 #2
  msg1.id = 0xAF0200;
  can1.write(msg1);
  delay(5);
  can1.write(msg1);
  delay(5);

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
  digitalWriteFast(5, HIGH);
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

  //groundSpeed = myGNSS.getGroundSpeed() * 0.00223694;
  //GPSaltitude = myGNSS.getAltitudeMSL() / 3300;
  
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
  delay(10);

  msg3.id = 0xc81;
  msg3.len = 8;
  msg3.buf[0] = 0xFF;
  msg3.buf[1] = 0x55;
  msg3.buf[2] = IVTpackVoltage;
  msg3.buf[3] = IVTpackVoltage>>8;
  msg3.buf[4] = IVTpackCurrent;
  msg3.buf[5] = IVTpackCurrent>>8;
  msg3.buf[6] = batteryVoltage;
  msg3.buf[7] = batteryVoltage>>8;
  can3.write(msg3);
  delay(10);

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
  delay(10);

  msg3.id = 0xc83;
  msg3.len = 8;
  msg3.buf[0] = highestCellV - lowestCellV;
  msg3.buf[1] = (highestCellV - lowestCellV)>>8;
  msg3.buf[2] = 0;
  msg3.buf[3] = 0;
  msg3.buf[4] = 0;
  msg3.buf[5] = 0;
  msg3.buf[6] = 0;
  msg3.buf[7] = 0;
  can3.write(msg3);

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
/*
  displayBuffer[0] = 0x44;
  displayBuffer[1] = 0x33;
  displayBuffer[2] = 0x22;
  displayBuffer[3] = 0x11;
  displayBuffer[4] = 0x00;
  displayBuffer[5] = 0x00;
  displayBuffer[6] = 0x0C;
  displayBuffer[7] = 0x80;
  displayBuffer[8] = rpm;//RPM = V
  displayBuffer[9] = rpm>>8;
  displayBuffer[10] = power*10;//kW = V/10
  displayBuffer[11] = (power*10)>>8;
  displayBuffer[12] = 21;//°C = V-100
  displayBuffer[13] = 0;
  displayBuffer[14] = throttle*10;//TPS = V/10
  displayBuffer[15] = (throttle*10)>>8;

  udpServer.beginPacket(targetIP, targetPort);
  udpServer.write(displayBuffer, 16);
  udpServer.endPacket();*/
  digitalWriteFast(5, LOW);
}//end of displayStatus()

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

/* Setup */
void setup() {
  // put your setup code here, to run once:
  pinMode(LCD_BL, OUTPUT);
  digitalWrite(LCD_BL, HIGH);  // Turn LCD backlight on

  //timing debug pins
  pinMode(2,OUTPUT);        //loop timing
  digitalWriteFast(2, LOW);
  pinMode(3,OUTPUT);        //CAN1 RX timing
  digitalWriteFast(3, LOW);
  pinMode(4,OUTPUT);        //CAN2 RX timing
  digitalWriteFast(4, LOW);
  pinMode(5,OUTPUT);        //display timing
  digitalWriteFast(5, LOW);
  
  Serial.begin(115200);
  delay(5);
  Serial.println("Hello Teensy 4.1 VCU with GPS");

  //digital.pinMode(2, INPUT_PULLUP, RISING);// wake pin

  initCAN(500000, 500000, 1000000);
  delay(100);
/*
  Wire.setClock(400 * 1000);//for U-blox GPS
  Wire.begin();
 
  if (myGNSS.begin() == false)
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }
  myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA); //Set the I2C port to output both NMEA and UBX messages
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR
*/
  //This will pipe all NMEA sentences to the serial port so we can see them
  //myGNSS.setNMEAOutputPort(Serial);

  // get mac address
  //teensyMAC(mac);
  //startEthernet();

  //send PDU-8 driver settings
  PDUmsg1.id = 0x0A0620;
  PDUmsg1.len = 8;
  PDUmsg1.buf[0] = 0x0D;//channel 1 current limit 5A (5/0.4A = 13 or 0x0D) - negative contactor
  PDUmsg1.buf[1] = 0x05;//channel 2 current limit 2A (2/0.4A = 5) - pMBB32s
  PDUmsg1.buf[2] = 0x05;//channel 3 current limit 2A (2/0.4A = 5) - positive pre-charge
  PDUmsg1.buf[3] = 0x0D;//channel 4 current limit 5A (5/0.4A = 13 or 0x0D) - positive contactor
  PDUmsg1.buf[4] = 0;
  PDUmsg1.buf[5] = 0;
  PDUmsg1.buf[6] = 0;
  PDUmsg1.buf[7] = 0; 
  can1.write(PDUmsg1);
  delay(2);

  //send PDU-8 driver outputs
  PDUmsg2.id = 0x0A0630;
  PDUmsg2.len = 8;
  PDUmsg2.buf[0] = 0;//channel 1 - negative contactor
  PDUmsg2.buf[1] = 0xFF;//channel 2 - pMBB32
  PDUmsg2.buf[2] = 0;//channel 3 - positive pre-charge relay
  PDUmsg2.buf[3] = 0;//channel 4 - positive contac 
  PDUmsg2.buf[4] = 0;
  PDUmsg2.buf[5] = 0;
  PDUmsg2.buf[6] = 0;
  PDUmsg2.buf[7] = 0; 
  can1.write(PDUmsg2);

  t1.begin(callback_cells_pdu, 62.5ms);//62.5ms);
  delay(500);
  wakepMBB32();

  t2.begin(callback_cell_sample, 150ms);
  //t3.begin(callback1000ms, 1000ms); 

  msg2.id = 0x411;//send IVT command
  msg2.flags.extended = 0;
  msg2.len = 6;
  msg2.buf[0] = 0x34;//set SET_MODE command 
  msg2.buf[1] = 0;//set actual mode: 0 = stop, 1 = run
  msg2.buf[2] = 1;//set startup operation mode: 0 = stop, 1 = run
  msg2.buf[3] = 0;
  msg2.buf[4] = 0;
  msg2.buf[5] = 0;
  can2.write(msg2);
  delay(1);
  msg2.id = 0x411;//send IVT command
  msg2.flags.extended = 0;
  msg2.len = 6;
  msg2.buf[0] = 0x24;//configuration of measurement 0x2x: 0 = I, 1 = U1, 2 = U2, 3 = U3, 4 = T, 5 = W, 6 = As, 7 = Wh
  msg2.buf[1] = 2;//low nibble = 0 for disabled, 1 for triggered, 2 for cyclic running
  msg2.buf[2] = 0;
  msg2.buf[3] = 0xC8;//200ms
  msg2.buf[4] = 0;
  msg2.buf[5] = 0;
  can2.write(msg2);
  delay(1);
  msg2.id = 0x411;//send IVT command
  msg2.flags.extended = 0;
  msg2.len = 6;
  msg2.buf[0] = 0x25;//configuration of measurement 0x2x: 0 = I, 1 = U1, 2 = U2, 3 = U3, 4 = T, 5 = W, 6 = As, 7 = Wh
  msg2.buf[1] = 2;//low nibble = 0 for disabled, 1 for triggered, 2 for cyclic running
  msg2.buf[2] = 0;
  msg2.buf[3] = 0x64;//100ms
  msg2.buf[4] = 0;
  msg2.buf[5] = 0;
  can2.write(msg2);
  delay(1);
  msg2.id = 0x411;//send IVT command
  msg2.flags.extended = 0;
  msg2.len = 6;
  msg2.buf[0] = 0x26;//configuration of measurement 0x2x: 0 = I, 1 = U1, 2 = U2, 3 = U3, 4 = T, 5 = W, 6 = As, 7 = Wh
  msg2.buf[1] = 2;//low nibble = 0 for disabled, 1 for triggered, 2 for cyclic running
  msg2.buf[2] = 0;
  msg2.buf[3] = 0x64;//100ms
  msg2.buf[4] = 0;
  msg2.buf[5] = 0;
  can2.write(msg2);
  delay(1);
  msg2.id = 0x411;//send IVT command
  msg2.flags.extended = 0;
  msg2.len = 6;
  msg2.buf[0] = 0x27;//configuration of measurement 0x2x: 0 = I, 1 = U1, 2 = U2, 3 = U3, 4 = T, 5 = W, 6 = As, 7 = Wh
  msg2.buf[1] = 2;//low nibble = 0 for disabled, 1 for triggered, 2 for cyclic running
  msg2.buf[2] = 0;
  msg2.buf[3] = 0x64;//100ms
  msg2.buf[4] = 0;
  msg2.buf[5] = 0;
  can2.write(msg2);
  delay(1);
  msg2.id = 0x411;//send IVT command
  msg2.flags.extended = 0;
  msg2.len = 6;
  msg2.buf[0] = 0x34;//set SET_MODE command 
  msg2.buf[1] = 1;//set actual mode: 0 = stop, 1 = run
  msg2.buf[2] = 1;//set startup operation mode: 0 = stop, 1 = run
  msg2.buf[3] = 0;
  msg2.buf[4] = 0;
  msg2.buf[5] = 0;
  can2.write(msg2);
  delay(1);

  msg2.id = 0xA100101;//send SIM100MOD Request Part Name command
  msg2.flags.extended = 1;
  msg2.len = 1;
  msg2.buf[0] = 1;//request part name 0
  can2.write(msg2);
  delay(1);
  msg2.id = 0xA100101;//send SIM100MOD Request Part Name command
  msg2.len = 1;
  msg2.buf[0] = 2;//request part name 1
  can2.write(msg2);
  delay(1);
  msg2.id = 0xA100101;//send SIM100MOD Request Part Name command
  msg2.len = 1;
  msg2.buf[0] = 3;//request part name 2
  can2.write(msg2);
  delay(1);
  msg2.id = 0xA100101;//send SIM100MOD Request Part Name command
  msg2.len = 1;
  msg2.buf[0] = 4;//request part name 3
  can2.write(msg2);
  delay(1); 

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

  tft.setFont(Arial_16);
  tft.setCursor(10,212);
  tft.println("EK9 EV VCU");
}//end of setup()


/* Main */
void loop() {
  // put your main code here, to run repeatedly:
  callback_main_loop_start = millis();
  digitalWriteFast(2, HIGH);  

  can1.events();//Call to look for any input
  can2.events();//Call to look for any input
  //can3.events();//Output only
 
    //Serial.println("Shutting down");
    // test shutdown and wake
    //shutdownpMBB32();
    //delay(2);

    //alarm.setRtcTimer(0, 0, 20);// hour, min, sec 

    //timer.setTimer(30);// seconds

    // Feed the sleep function its wakeup parameters, then go to sleep.
    //Snooze.sleep( config_teensy40 );// return module that woke processor

/*  
  //SendCANFrameToClient(3200);
  // if an incoming client connects, there will be bytes available to read:
  
  EthernetClient client = server.available();

  if (client) {
    while (client.connected()) {
      if (client.available()) {
        //char c = client.read();
  // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
//        if (c == '\n' && currentLineIsBlank) {
          
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/html");
          client.println("Connection: close");  // the connection will be closed after completion of the response
          client.println("Refresh: 2");  // refresh the page automatically every 5 sec
          client.println();
          client.println("<!DOCTYPE HTML>");
          client.println("<html>");
          client.println("Hello World!");
          client.print("<br />");
          client.print("Max Cell Voltage = ");
          client.print((double)highestCellV*0.000076293945);
          client.print("<br />");
          client.print("Min Cell Voltage = ");
          client.print((double)lowestCellV*0.000076293945);
          client.print("<br />");
          client.print("12V Battery Voltage = ");
          client.print((double)batteryVoltage/100);
          client.println("<br />"); 
          client.println("</html>");

          byte header[] = {0x44, 0x33, 0x22, 0x11};
          byte frameID[] = {0x80, 0xC, 0, 0};
          client.write(header, 4);
          client.write(frameID, 4);
          client.write(msg3.buf, 8);

          //client.print(displayBuffer, 16);
//          break;
//        }
//        if (c == '\n') {
          // you're starting a new line
//          currentLineIsBlank = true;
//        } else if (c != '\r') {
          // you've gotten a character on the current line
//          currentLineIsBlank = false;
//        }
      }
    }
  }
  // give the web browser time to receive the data
  delay(10);
  client.stop();
*/
  //myGNSS.checkUblox(); //See if new data is available. Process bytes as they come in.
  //groundSpeed = myGNSS.getGroundSpeed() * 0.00223694;//convert mm/s to mph
  //GPSaltitude = myGNSS.getAltitudeMSL() / 3300;//feet
  //Serial.print("Altitude ");
  //Serial.print(myGNSS.getAltitudeMSL() / 3300, DEC); Serial.println(" feet");
  //Serial.print("Speed ");
  //Serial.print(myGNSS.getGroundSpeed() * 0.00223694, DEC);  Serial.println(" mph");//convert mm/s to mph
  //delay(50);
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
  digitalWriteFast(2, LOW);
}//end of loop()