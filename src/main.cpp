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
#include "callbacks.h"
#include "realdash_tcp.h"

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

bool     extWakePending   = false; // set in setup() when CAN wake detected; consumed by FSM
uint32_t lastExtActivityMs = 0;    // last EVCC heartbeat or gateway frame (for KL15C timeout)
uint32_t klrLowSince      = 0;     // reset in Off_enter() — start of Off-state KLR debounce
bool     gnssInitialized  = false; // set in setup() only if GNSS bring-up actually ran this boot
bool     kl15cKL15Rstate  = false; // tracks KL15R pin state inside KL15C for LED edge detect
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
State state_KL15C(&KL15C_enter, &check_KL15C, &KL15C_exit);
Fsm fsm(&state_Off);

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
  // Also stays awake while a wireless-gateway status request is still waiting on a fresh
  // pMBB32 reading (see callback_t2()) — otherwise it can re-sleep before ever answering —
  // and sleeps promptly (skipping the debounce wait) right after that response is sent.
  {
    if (digitalRead(KL15R_PIN)) {
      klrLowSince = millis();
    } else if (VCUstate == VCU_STATE_OFF) {
      if (gatewayResponseSent) {
        gatewayResponseSent = false;
        enterSleep();
      } else if (!gatewayStatusRequestPending && millis() - klrLowSince > 500) {
        enterSleep();
      }
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
  if (gnssInitialized) {
    myGNSS.checkUblox();
    myGNSS.checkCallbacks();
  }
#endif

  realdashService(); // accept/replace the RealDash TCP client (Odroid dashboard)

fsm.run_machine();

  digitalWriteFast(3, LOW);
}//end of loop()