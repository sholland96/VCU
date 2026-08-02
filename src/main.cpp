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
#include "odroid_shutdown.h"
#include "note_alerts.h"

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
uint32_t klrHighSince     = 0;     // mirror of klrLowSince — see defines.h
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

  // Debounced KL15R read, shared by the relay block and the sleep-trigger block below.
  // klrStableLow requires a full 500ms of continuous LOW before either block treats the key
  // as genuinely off (fixes an earlier bug: brief noise/bounce repeatedly reset the Odroid
  // shutdown sequence before its 15s timer could ever elapse). klrStableHigh is the mirror
  // for the reverse direction — confirmed on hardware that relay-driver switching noise can
  // couple onto the KL15R sense line right as RELAY_ODROID_PIN de-energizes, which the raw
  // (non-debounced) read was treating as a genuine key-on and immediately re-energizing the
  // relay again — symptom: Odroid display goes dark then immediately reboots. Only a KL15R
  // read that's been stably high for 500ms now counts as a real key-on for relay purposes.
  bool klrRaw = digitalRead(KL15R_PIN);
  if (klrRaw) klrLowSince  = millis(); else klrHighSince = millis();
  bool klrStableLow  = !klrRaw && (millis() - klrLowSince  > 500);
  bool klrStableHigh =  klrRaw && (millis() - klrHighSince > 500);
  bool klrHigh = klrRaw; // raw value, still used below only where debounce isn't the concern

  // 12V relay power control, staged relative to KL15R/EVCC state.
  {
    // Relay #1: PDU-8/IVT-S/SIM100MOD/keypad — on whenever KL15R is stably high or an EVCC
    // charge session is active (mirrors EVCCsessionActive, which already keeps the VCU
    // itself awake during charging). No graceful-shutdown handling needed for these; still
    // drops immediately (no debounce) the instant KL15R reads low and EVCC isn't active.
    digitalWrite(RELAY_PDU_PIN, (klrStableHigh || EVCCsessionActive) ? HIGH : LOW);

    // Relay #2: Odroid + VU12. Once KL15R has been stably low, signal the Odroid to shut
    // down gracefully over the dedicated TCP link (odroid_shutdown.cpp — separate from the
    // RealDash feed) and wait ODROID_SHUTDOWN_DELAY_MS before actually cutting power, so it
    // has time to unmount/poweroff cleanly instead of losing power hot. The KLR sleep
    // debounce below waits for this relay to actually be off before enterSleep() can fire,
    // keeping the VCU (and its Ethernet link) awake for the whole sequence.
    if (klrStableHigh) {
      digitalWrite(RELAY_ODROID_PIN, HIGH);
      odroidShutdownSignaled = false;
    } else if (klrStableLow && digitalRead(RELAY_ODROID_PIN)) {
      // Hard ceiling on the whole sequence (measured from klrLowSince, when KL15R first
      // went low) — if the Odroid is off/unreachable, odroidShutdownSignal() can never
      // succeed, and without this the relay (and therefore enterSleep(), which waits on
      // it) would never fire at all. Confirmed on hardware: the VCU stayed fully awake
      // indefinitely with no fallback. The VCU's own ability to sleep must never be
      // permanently hostage to a peripheral being unavailable.
      if (millis() - klrLowSince >= ODROID_SHUTDOWN_MAX_WAIT_MS) {
        digitalWrite(RELAY_ODROID_PIN, LOW);
      } else if (!odroidShutdownSignaled) {
        // Retry at most once/second until actually delivered (a connected client at the
        // moment of the call) — a single one-shot attempt could silently miss if the
        // Odroid-side watcher's TCP connection happens to be mid-reconnect right then.
        if (millis() - odroidShutdownLastAttempt >= 1000) {
          odroidShutdownLastAttempt = millis();
          if (odroidShutdownSignal()) {
            odroidShutdownSignaled = true;
            odroidShutdownSignalTime = millis();
          }
        }
      } else if (millis() - odroidShutdownSignalTime >= ODROID_SHUTDOWN_DELAY_MS) {
        digitalWrite(RELAY_ODROID_PIN, LOW);
      }
    }
  }

  // KL15R gone low (key off) while system is safe → hibernate.
  // Debounce: only sleep after KL15R has been continuously LOW for 500ms (klrStableLow).
  // Gives the EVCC time to send New_Charge_Session (0x68001) after a CAN2 wake
  // before sleep is re-entered. Normal key-off behaviour is unchanged.
  // enterSleep() returns immediately if EVCCsessionActive (active charge session).
  // Also stays awake while a wireless-gateway status request is still waiting on a fresh
  // pMBB32 reading (see callback_t2()) — otherwise it can re-sleep before ever answering —
  // and sleeps promptly (skipping the debounce wait) right after that response is sent.
  // Also stays awake until RELAY_ODROID_PIN reads LOW — see the relay block above.
  {
    // klrLowSince is refreshed above whenever klrHigh — nothing left to do in that case.
    if (!klrHigh && VCUstate == VCU_STATE_OFF && !digitalRead(RELAY_ODROID_PIN)) {
      if (gatewayResponseSent) {
        gatewayResponseSent = false;
        enterSleep();
      } else if (!gatewayStatusRequestPending && klrStableLow) {
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
  odroidShutdownService(); // accept the Odroid-side shutdown-watcher TCP client
  notecardCheckStatus(); // rate-limited to 1/min internally — see note_alerts.h


fsm.run_machine();

  digitalWriteFast(3, LOW);
}//end of loop()