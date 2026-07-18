/* lin.cpp — BMW i4/i5/i7/iX changeover valve LIN master (Serial3). */
#include <Arduino.h>
#include <LIN_master_HardwareSerial.h>
#include "defines.h"
#include "lin.h"

// LIN master on Serial3: RX3=pin7, TX3=pin8 (SK Pang board GNSS is on Serial2)
LIN_Master_HardwareSerial linBus(Serial3, "Valve");

void linInit() {
  linBus.begin(LIN_BAUD);
  // pinMode(LIN_EN_PIN, OUTPUT);    // TODO: uncomment when enable pin assigned
  // digitalWrite(LIN_EN_PIN, HIGH);
}

void linReadValve() {
  uint8_t buf[2] = {0};
  LIN_Master_Base::error_t err = linBus.receiveSlaveResponseBlocking(
      LIN_Master_Base::LIN_V2, LIN_VALVE_ID, sizeof(buf), buf);
  if (err == LIN_Master_Base::NO_ERROR) {
    valveStatus   = buf[0];
    valvePosition = buf[1];
    valveOnline   = true;
  } else {
    valveOnline = false;
  }
}

void linWriteValve(uint8_t position) {
  valvePosition = position;
  // uint8_t data[] = {position};
  // linBus.sendMasterRequestBlocking(LIN_Master_Base::LIN_V2, LIN_VALVE_ID, sizeof(data), data);
}
