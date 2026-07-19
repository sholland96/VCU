/* lin.cpp — BMW i4/i5/i7/iX Changeover Valve 64119462114, LIN master on Serial3.
 * LIN 2.x, 19200 baud. Uses gicking/LIN master portable (LIN_Master_HardwareSerial).
 * TODO: confirm node ID, response frame length, byte map, and checksum type
 *       from BMW LIN specification / ISTA service documentation.
 */
#include <Arduino.h>
#include <LIN_master_HardwareSerial.h>
#include "defines.h"
#include "lin.h"

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
