/* fsm_states.cpp — arduino-fsm state enter/exit/check callbacks and
 * transition callbacks for the VCU state machine (Off/PreCharge/Idle/
 * Drive/Charge/Fault/HeatPack/CoolPack/KL30C).
 */
#include <Arduino.h>
#include "defines.h"
#include "sdlog.h"
#include "sleep.h"
#include "fsm_states.h"

void Off_enter()
{
  Serial.println("Entering Off state");
  VCUstate = VCU_STATE_OFF;
  sdLogEvent("STATE:OFF");
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
  sdLogEvent("STATE:IDLE");
  
  /* Send SMS
  0 = send "KL15R on" message
  1 = send "KL15C on" message
  2 = send "Pre-charge failed..." message
  3 = send "Something happened..." message
  4 = send "Charging stopped..." message
  5 = send "Temperature warning..." message
  any other value  = send "Invalid request..." message
  */
  msg3.id = 0xC79;//send SMS
  msg3.len = 1;
  msg3.buf[0] = 0;//send "KL15R on" message
  can3.write(msg3);

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

    /* Send SMS
    0 = send "KL15R on" message
    1 = send "KL15C on" message
    2 = send "Pre-charge failed..." message
    3 = send "Something happened..." message
    4 = send "Charging stopped..." message
    5 = send "Temperature warning..." message
    any other value  = send "Invalid request..." message
    */
    msg3.id = 0xC79;//send SMS
    msg3.len = 1;
    msg3.buf[0] = 2;//send "Pre-charge failed..." message
    can3.write(msg3);

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
}

void Charge_enter()
{
  Serial.println("Entering Charge state");
  VCUstate = VCU_STATE_CHARGE;
  sdLogEvent("STATE:CHARGE");
}

void Drive_enter()
{
  Serial.println("Entering Drive state");
  VCUstate = VCU_STATE_DRIVE;
  sdLogEvent("STATE:DRIVE");
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
  sdLogEvent("STATE:PRECHARGE");
  preChargeStartTime = millis();
  PDUmsg1.buf[0] = 0x0D; // CH1 5A — negative contactor on
  PDUmsg1.buf[2] = 0x05; // CH3 2A — passive pre-charge relay (remove when active pre-charge fitted)
  digitalWrite(PRECHARGE_EN_PIN, HIGH); // TPS131PXQ1EVM-400 active pre-charge enable
}

void PreCharge_exit() {
  Serial.println("Exiting PreCharge state");
  PDUmsg1.buf[2] = 0x00; // CH3 off — passive pre-charge relay off
  digitalWrite(PRECHARGE_EN_PIN, LOW);  // disable active pre-charge
}

void check_PreCharge() {
  if (IVTpackVoltage > 0 &&
      IVTpreChargeV >= (IVTpackVoltage * 95 / 100)) {
    PDUmsg1.buf[3] = 0x0D; // CH4 5A — positive contactor on
    sdLogEvent("PRECHARGE_OK");
    fsm.trigger(chargeMode ? PRECHARGE_CHARGE : PRECHARGE_OK);
  } else if (millis() - preChargeStartTime > PRECHARGE_TIMEOUT_MS) {
    sdLogEvent("PRECHARGE_FAIL");
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
  if (evccIsACSession) {
    msg2.flags.extended = 0;
    msg2.id  = EVCC_AC_STATUS;
    msg2.len = 1;
    msg2.buf[0] = 0x00;  // Ready_To_Charge = Not_Ready — EVCC opens AC relay
    can2.write(msg2);
    msg2.flags.extended = 1;
  }
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
  sdLogEvent("STATE:FAULT");
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
  msg3.buf[0] = 3; // "Something happened..."
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
  sdLogEvent("STATE:HEAT");
  // TODO: set invPumpSetpoint/battPumpSetpoint; enable heater output
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
  sdLogEvent("STATE:COOL");
  // TODO: set invPumpSetpoint/battPumpSetpoint; enable AC exchanger
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

// ── KL30C — external CAN wake standby ────────────────────────────────────────
// Entered when the VCU wakes from sleep via CAN2 activity (EVCC or wireless
// gateway) with KL15R still LOW (key not turned). No HV contactors are closed.
// Exits to Off on KL15_ON (button 5), which then immediately proceeds to
// PreCharge since KL15state is already true.
// Sleeps when EVCCsessionActive is false, KL15R is low, and no EVCC heartbeat
// or gateway frame has been seen for KL30C_SLEEP_TIMEOUT_MS.

void KL30C_enter() {
  Serial.println("Entering KL30C state");
  VCUstate          = VCU_STATE_KL30C;
  sdLogEvent("STATE:KL30C");
  kl30cKL15Rstate   = false; // force LED edge detect on first check_KL30C() call
  lastExtActivityMs = millis(); // reset timeout — start counting from now
  PDUmsg1.buf[0]    = 0x00; // CH1 off — negative contactor
  PDUmsg1.buf[3]    = 0x00; // CH4 off — positive contactor
}

void check_KL30C() {
  // Flash button 1 (Park/P) LED while KL15R is high to signal standby mode.
  bool kl15rNow = digitalRead(KL15R_PIN);
  if (kl15rNow != kl30cKL15Rstate) {
    kl30cKL15Rstate     = kl15rNow;
    msg2.id             = 0x18EF2100;
    msg2.flags.extended = 1;
    msg2.len            = 8;
    msg2.buf[0]         = 0x04;
    msg2.buf[1]         = 0x1B;
    msg2.buf[2]         = KEYPAD_CMD_SET_LED;
    msg2.buf[3]         = 0x01; // button 1 — Park/P
    msg2.buf[4]         = kl15rNow ? KEYPAD_COLOR_AMBER : KEYPAD_COLOR_OFF;
    msg2.buf[5]         = kl15rNow ? KEYPAD_MODE_BLINK  : KEYPAD_MODE_SOLID;
    msg2.buf[6]         = 0x00;
    msg2.buf[7]         = 0xFF;
    can2.write(msg2);
  }

  // AC plug detected → close main contactors via pre-charge sequence.
  // DC sessions leave EVCC to handle its own contactors; VCU stays in KL30C.
  if (EVCCsessionActive && evccIsACSession) {
    fsm.trigger(AC_CHARGE_START);
    return;
  }

  // Button 5 (KL15) pressed → transition to Off; Off's check_KL15() will see
  // KL15state=true and immediately proceed to PreCharge.
  if (KL15state) {
    fsm.trigger(KL15_ON);
    return;
  }

  // Sleep when: no active EVCC session, KL15R is low, and no external activity
  // for KL30C_SLEEP_TIMEOUT_MS.
  if (!EVCCsessionActive && !kl15rNow &&
      millis() - lastExtActivityMs > KL30C_SLEEP_TIMEOUT_MS) {
    Serial.println("KL30C timeout — sleeping");
    enterSleep();
  }
}

void KL30C_exit() {
  Serial.println("Exiting KL30C state");
  msg2.id             = 0x18EF2100;
  msg2.flags.extended = 1;
  msg2.len            = 8;
  msg2.buf[0]         = 0x04;
  msg2.buf[1]         = 0x1B;
  msg2.buf[2]         = KEYPAD_CMD_SET_LED;
  msg2.buf[3]         = 0x01; // button 1 — Park/P
  msg2.buf[4]         = KEYPAD_COLOR_OFF;
  msg2.buf[5]         = KEYPAD_MODE_SOLID;
  msg2.buf[6]         = 0x00;
  msg2.buf[7]         = 0xFF;
  can2.write(msg2);
}

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
