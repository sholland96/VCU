#include <Arduino.h>
#include <Wire.h> //Needed for I2C to GNSS
#include <TeensyTimerTool.h>
#include <FlexCAN_T4.h>
#include "pMBB32.h"
#include <NativeEthernet.h>

//#include <Snooze.h>
#include "defines.h"
#include "helperFunctions.h"
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //Click here to get the library: http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

//SnoozeDigital digital;
//SnoozeTimer timer;
//SnoozeAlarm  alarm;
//SnoozeBlock config_teensy40(alarm);

using namespace TeensyTimerTool;
PeriodicTimer t1(RTC); // generate a timer from the pool (Pool: 2xGPT, 16xTMR(QUAD), 20xTCK)
PeriodicTimer t2(GPT1); // generate a timer from the pool (Pool: 2xGPT, 16xTMR(QUAD), 20xTCK)
PeriodicTimer t3(GPT2); // generate a timer from the pool (Pool: 2xGPT, 16xTMR(QUAD), 20xTCK)

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;// 29-bit CAN: pMMB32, PDU-8, keypad
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;// 11-bit CAN: IVT-S  
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;// EVCC 

CAN_message_t msg1;//.buf[7] = {0x0D, 0x05, 0x05, 0x0D, 0, 0, 0, 0};
CAN_message_t msg2;
CAN_message_t msg3;
CAN_message_t PDUmsg1;
CAN_message_t PDUmsg2;

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN,OUTPUT);
  Serial.begin(115200);
  delay(5);
  Serial.println("Hello Teensy 4.1 VCU with GPS");

  //digital.pinMode(2, INPUT_PULLUP, RISING);// wake pin

  initCAN(500000, 500000, 1000000);
  delay(100);

  Wire.begin();
  Wire.setClock(400000);
  if (myGNSS.begin() == false)
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }
  myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA); //Set the I2C port to output both NMEA and UBX messages
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR
  //This will pipe all NMEA sentences to the serial port so we can see them
  //myGNSS.setNMEAOutputPort(Serial);

  // get mac address
  teensyMAC(mac);
  startEthernet();

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
  delay(200);
  wakepMBB32();
  t2.begin(callback_cell_sample, 100ms);
  t3.begin(callback500ms, 1000ms);
  //delay(100);
  //threads.addThread(thread_cells_pdu);
  
  //delay(200);
  //threads.addThread(thread_cell_sample);
  //threads.addThread(thread_5000ms);
  
}//end of setup()

void loop() {
  // put your main code here, to run repeatedly:

  can1.events();
  can2.events();
  can3.events();
 
    //Serial.println("Shutting down");
    // test shutdown and wake
    //shutdownpMBB32();
    //delay(2);

    //alarm.setRtcTimer(0, 0, 20);// hour, min, sec 

    //timer.setTimer(30);// seconds

    // Feed the sleep function its wakeup parameters, then go to sleep.
    //Snooze.sleep( config_teensy40 );// return module that woke processor
  
  //SendCANFrameToClient(3200);
  // if an incoming client connects, there will be bytes available to read:
  
  EthernetClient client = server.available();
  SendCANFramesToEth(client);

  myGNSS.checkUblox(); //See if new data is available. Process bytes as they come in.

  groundSpeed = myGNSS.getGroundSpeed() * 0.00223694;//convert mm/s to mph
  GPSaltitude = myGNSS.getAltitudeMSL() / 3300;//feet
  //Serial.print("Altitude ");
  //Serial.print(myGNSS.getAltitudeMSL() / 3300, DEC); Serial.println(" feet");
  //Serial.print("Speed ");
  //Serial.print(myGNSS.getGroundSpeed() * 0.00223694, DEC);  Serial.println(" mph");//convert mm/s to mph
  //delay(50);
  
}//end of loop()

void callback_cells_pdu() {
  //send PDU-8 driver settings every t1 period
  can1.write(PDUmsg1);
}//end of callback_cells_pdu()

void callback_cell_sample() {
  // request all cell and temperature measurements every t2 period
  msg1.id = 0xFF0000;
  msg1.len = 0;
  can1.write(msg1);  //send sample command every 1 second
  //delay(5);
  //check min/max cell voltages on each pMBB32 every 3 t2 periods
  if (counter == 1) {
    msg1.id = 0xCF0100;
    msg1.len = 0;
    can1.write(msg1);
  }
  if (counter == 2) {
    msg1.id = 0xCF0200;
    msg1.len = 0;
    can1.write(msg1);
  }
  if (counter == 3) {
    counter = 0;
    msg1.id = 0xCF0300;
    msg1.len = 0;
    can1.write(msg1);
  }
  counter++;
  //delay(2);
/*  
  if(PDUmsg.buf[1] == pMBB32powerOff){
    PDUmsg.buf[1] = pMBB32powerOn;
    can1.write(PDUmsg);
    delay(100);
    wakepMBB32();
  }*/
//  else{
    
/*    
    //check for stale pMBB32 CAN and restart if necessary
    if((pMBB32stale1 > 50)|(pMBB32stale2 > 50)|(pMBB32stale3 > 50)){
      PDUmsg.buf[1] = pMBB32powerOff;
      can1.write(PDUmsg);
      pMBB32stale1 = 0;
      pMBB32stale2 = 0;
      pMBB32stale3 = 0;
      delay(200);
      
    }*/
//  } 
  
/*  if(pMBB32stale1 > 5){
    //send wakeup to pMBB32 #1
    msg1.id = 0xAF0100; 
    msg1.len = 3;
    msg1.buf[0] = 0x01;
    msg1.buf[1] = 0x10;
    msg1.buf[2] = 0x2;
    can1.write(msg1);
    delay(5);
  }
  if(pMBB32stale2 > 5){
    //send wakeup to pMBB32 #2
    msg1.id = 0xAF0200; 
    msg1.len = 3;
    msg1.buf[0] = 0x01;
    msg1.buf[1] = 0x10;
    msg1.buf[2] = 0x2;
    can1.write(msg1);
    delay(5);
  }
  if(pMBB32stale3 > 5){
    //send wakeup to pMBB32 #3
    msg1.id = 0xAF0300; 
    msg1.len = 3;
    msg1.buf[0] = 0x01;
    msg1.buf[1] = 0x10;
    msg1.buf[2] = 0x2;
    can1.write(msg1);
    delay(5);
  }*/

  displayStatus();//update RealDash

  
}//end of callback_cell_sample()

void callback500ms() {
  //ReadDigitalStatuses();
  //ReadAnalogStatuses();
  //SendCANFramesToEth();
  
  digitalToggleFast(LED_BUILTIN);
}//end of callback5000ms()

void teensyMAC(uint8_t *mac) {
    for(uint8_t by=0; by<2; by++) mac[by]=(HW_OCOTP_MAC1 >> ((1-by)*8)) & 0xFF;
    for(uint8_t by=0; by<4; by++) mac[by+2]=(HW_OCOTP_MAC0 >> ((3-by)*8)) & 0xFF;
    Serial.printf("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void startEthernet() {
    // start the Ethernet connection and the server:
    Ethernet.begin(mac, localIP);
    server.begin();
    udpServer.begin(targetPort);
}

void can1Sniff(const CAN_message_t &msg) {
  if ((msg.id & 0x00000F00) == 0x00000E00) {//received 0x18FF0Eyy    
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
    }
  }

  // chack for stale pMBB32 measurements requests. We should see a response to FF command regularly
  if((msg.id & 0xFFFF000F) == 0x18FF0000){
    temp = msg.id & 0x0000000F;
   
    if(temp == 1){
      pMBB32stale1 = 0;
      pMBB32stale2++;
      pMBB32stale3++;
    } 
    if(temp == 2){
      pMBB32stale1++;
      pMBB32stale2 = 0;
      pMBB32stale3++;
    } 
    if(temp == 3){
      pMBB32stale1++;
      pMBB32stale2++;
      pMBB32stale3 = 0;
    }
  // else {
    //pMBB32stale1++;
    //pMBB32stale2++;
    //pMBB32stale3++;
  }

  switch (msg.id) {
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
    case 0x000A0610:
      
    //  break;
    default:
      //
      break;
  }
}// end of can1Sniff(const CAN_message_t &msg)

void can2Sniff(const CAN_message_t &msg) {
  switch (msg.id) {
    case 0x521:
      packCurrent = msg.buf[2]<<24 & msg.buf[3]<<16 & msg.buf[4]<<8 & msg.buf[5];
      break;
    case 0x522:
      packVoltage = msg.buf[2]<<24 & msg.buf[3]<<16 & msg.buf[4]<<8 & msg.buf[5];
      break;
    case 0x523:
      preChargeV = msg.buf[2]<<24 & msg.buf[3]<<16 & msg.buf[4]<<8 & msg.buf[5];
      break;
    case 0x524:
      U3V = msg.buf[2]<<24 & msg.buf[3]<<16 & msg.buf[4]<<8 & msg.buf[5];
      break;
    default:
      Serial.println(" Nothing ");
      break;
  }
}//end of can2Sniff(const CAN_message_t &msg)

void can3Sniff(const CAN_message_t &msg) {
  switch (msg.id) {
    case 500:
      packCurrent = msg.buf[1]<<24 & msg.buf[2]<<16 & msg.buf[3]<<8 & msg.buf[4];
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

  msg2.flags.extended = 0;
  msg2.len = 8;
  msg2.buf[0] = 0;
  msg2.buf[1] = 0;
  msg2.buf[2] = 0;
  msg2.buf[3] = 0;
  msg2.buf[4] = 0;
  msg2.buf[5] = 0;
  msg2.buf[6] = 0;
  msg2.buf[7] = 0;

  msg3.flags.extended = 0;
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
  can1.write(PDUmsg1);
  delay(20);
  
  //send wakeup to pMBB32 #1
  msg1.id = 0xAF0100; 
  msg1.len = 3;
  msg1.buf[0] = 0x01;
  msg1.buf[1] = 0x10;
  msg1.buf[2] = 0x2;
  can1.write(msg1);
  delay(2);
  
  //send wakeup to pMBB32 #2
  msg1.id = 0xAF0200;
  msg1.len = 3;
  msg1.buf[0] = 0x01;
  msg1.buf[1] = 0x10;
  msg1.buf[2] = 0x2;
  can1.write(msg1);
  delay(2);

  //send wakeup to pMBB32 #3
  msg1.id = 0xAF0300;
  msg1.len = 3;
  msg1.buf[0] = 0x01;
  msg1.buf[1] = 0x10;
  msg1.buf[2] = 0x2;
  can1.write(msg1);
  delay(2);

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



void sendTestData() {
  /*
  for (int i = 0; i < 250; i++) {
    // 102-byte string (println appends CRLF)
    size_t written = client.println("1234567890"
                                    "1234567890"
                                    "1234567890"
                                    "1234567890"
                                    "1234567890"
                                    "1234567890"
                                    "1234567890"
                                    "1234567890"
                                    "1234567890"
                                    "1234567890");
    if (written != 102) {
      // This is not an error!
      Serial.println("Didn't write fully");
    }
  }*/
  // send a reply to the IP address and port that sent us the packet we received
  


}


/*
void SendCANFrameToClient(unsigned long canFrameId)
{
    // the 4 byte identifier at the beginning of each CAN frame
    // this is required for RealDash to 'catch-up' on ongoing stream of CAN frames
//    server.write((const byte*)&serialBlockTag, 4);
    
    // the CAN frame id number
//    server.write((const byte*)&canFrameId, 4);
    
    // CAN frame payload
//    server.write(frameData, 8);

  if (client.connect(targetIP, targetPort)) { // Connect to the target device
    // the 4 byte identifier at the beginning of each CAN frame
    // this is required for RealDash to 'catch-up' on ongoing stream of CAN frames
    client.write((const byte*)&serialBlockTag, 4);
    
    // the CAN frame id number
    client.write((const byte*)&frameID, 2);

    // CAN frame payload
    client.write(msg1.buf, 8);

    client.println("Hello from Teensy!"); // Send data to the target device

  } 
  client.connected();
  delay(1);
  // close the connection:
  client.stop();
}*/

void displayStatus() {
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

 rpm += 100;
  if(rpm >= 13000)
    rpm = 0;
  
  power += 5;
  if(power >= 450)
    power = 0;
  
  packVoltage = 3840;
  packCurrent = 5000;
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
  //if(msg3.buf[6] >= 90){
  //  msg3.buf[6] = 5;
  //  msg3.buf[7] = 0;}
  can3.write(msg3); 

  msg3.id = 0xc81;
  msg3.len = 8;
  msg3.buf[0] = 0xFF;
  msg3.buf[1] = 0x55;
  msg3.buf[2] = packVoltage;
  msg3.buf[3] = packVoltage>>8;
  msg3.buf[4] = packCurrent;
  msg3.buf[5] = packCurrent>>8;
  msg3.buf[6] = batteryVoltage;
  msg3.buf[7] = batteryVoltage>>8;
  can3.write(msg3);

  //temp = maxCellV*ADCres;
  msg3.id = 0xc82;
  msg3.len = 8;
  msg3.buf[0] = maxCellV;
  msg3.buf[1] = maxCellV>>8;
  msg3.buf[2] = minCellV;
  msg3.buf[3] = minCellV>>8;
  msg3.buf[4] = groundSpeed;
  msg3.buf[5] = 0;
  msg3.buf[6] = GPSaltitude;
  msg3.buf[7] = GPSaltitude>>8;
  can3.write(msg3);
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
}//end of displayStatus()





