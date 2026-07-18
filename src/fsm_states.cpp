/* fsm_states.cpp — FSM state enter/exit/check callbacks and transition handlers. */
#include "vcu.h"
#include "fsm_states.h"
#include "sdlog.h"
#include "sleep.h"

// ── State: Off ────────────────────────────────────────────────────────────────
void Off_enter() {
  Serial.println("Entering Off state");
  VCUstate = VCU_STATE_OFF;
  sdLogEvent("STATE:OFF");
}

void Off_exit() {
  Serial.println("Exiting Off state");
  PDUmsg2.buf[0] = 0xFE; // HS driver 1 PWM — negative contactor
  PDUmsg1.buf[0] = 13;   // HS driver 1 current limit 5A
}

void check_KL15() {
  if (KL15state) fsm.trigger(KL15_ON);
}

// ── State: PreCharge ──────────────────────────────────────────────────────────
void PreCharge_enter() {
  Serial.println("Entering PreCharge state");
  VCUstate = VCU_STATE_PRECHARGE;
  sdLogEvent("STATE:PRECHARGE");
  preChargeStartTime = millis();
  PDUmsg1.buf[0] = 0x0D; // CH1 5A — negative contactor on
  PDUmsg1.buf[2] = 0x05; // CH3 2A — passive pre-charge relay
  digitalWrite(PRECHARGE_EN_PIN, HIGH);
}

void PreCharge_exit() {
  Serial.println("Exiting PreCharge state");
  PDUmsg1.buf[2] = 0x00;
  digitalWrite(PRECHARGE_EN_PIN, LOW);
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

// ── State: Idle ───────────────────────────────────────────────────────────────
void Idle_enter() {
  Serial.println("Entering Idle state");
  VCUstate = VCU_STATE_IDLE;
  sdLogEvent("STATE:IDLE");

  msg2.id             = 0x18EF2100;
  msg2.flags.extended = 1;
  msg2.len            = 8;
  msg2.buf[0] = 0x04; msg2.buf[1] = 0x1B;
  msg2.buf[2] = KEYPAD_CMD_SET_LED;
  msg2.buf[3] = 0x05;
  msg2.buf[4] = KEYPAD_COLOR_AMBER;
  msg2.buf[5] = KEYPAD_MODE_BLINK;
  msg2.buf[6] = 0x00; msg2.buf[7] = 0xFF;
  can2.write(msg2);

  msg3.id     = 0xC79;
  msg3.len    = 1;
  msg3.buf[0] = 1; // "KL15 on"
  can3.write(msg3);
  delay(1);
}

void Idle_exit() {
  Serial.println("Exiting Idle state");
}

void check_Idle() {
  if (button_0x01_state && LDUrpm == 0) {
    button_0x01_state = 0; KL15state = false; fsm.trigger(KL15_OFF); return;
  }
  if (button_0x08_state) { fsm.trigger(DRIVE_ON); return; }
}

// ── State: Drive ──────────────────────────────────────────────────────────────
void Drive_enter() {
  Serial.println("Entering Drive state");
  VCUstate = VCU_STATE_DRIVE;
  sdLogEvent("STATE:DRIVE");
}

void Drive_exit() {
  Serial.println("Exiting Drive state");
  LDUdirection = LDU_DIR_STOP;
}

void check_DriveState() {
  if (button_0x01_state && LDUrpm == 0) {
    button_0x01_state = 0; fsm.trigger(DRIVE_OFF); return;
  }
}

// ── State: Charge ─────────────────────────────────────────────────────────────
void Charge_enter() {
  Serial.println("Entering Charge state");
  VCUstate = VCU_STATE_CHARGE;
  sdLogEvent("STATE:CHARGE");
}

void Charge_exit() {
  Serial.println("Exiting Charge state");
  PDUmsg1.buf[0] = 0x00; // CH1 off — negative contactor
  PDUmsg1.buf[3] = 0x00; // CH4 off — positive contactor
  if (evccIsACSession) {
    msg2.flags.extended = 0;
    msg2.id     = EVCC_AC_STATUS;
    msg2.len    = 1;
    msg2.buf[0] = 0x00; // Ready_To_Charge = Not_Ready
    can2.write(msg2);
    msg2.flags.extended = 1;
  }
}

void check_Charge() {
  if (button_0x01_state && LDUrpm == 0) {
    button_0x01_state = 0; KL15state = false; fsm.trigger(KL15_OFF); return;
  }
}

// ── State: Fault ──────────────────────────────────────────────────────────────
void Fault_enter() {
  Serial.println("Entering Fault state");
  VCUstate = VCU_STATE_FAULT;
  sdLogEvent("STATE:FAULT");
  PDUmsg1.buf[0] = 0x00;
  PDUmsg1.buf[2] = 0x00;
  PDUmsg1.buf[3] = 0x00;
  msg2.id             = 0x18EF2100;
  msg2.flags.extended = 1;
  msg2.len            = 8;
  msg2.buf[0] = 0x04; msg2.buf[1] = 0x1B;
  msg2.buf[2] = KEYPAD_CMD_SET_LED;
  msg2.buf[3] = 0x05;
  msg2.buf[4] = KEYPAD_COLOR_RED;
  msg2.buf[5] = KEYPAD_MODE_BLINK;
  msg2.buf[6] = 0x00; msg2.buf[7] = 0xFF;
  can2.write(msg2);
  msg3.id     = 0xC79;
  msg3.len    = 1;
  msg3.buf[0] = 2; // "Something happened..."
  can3.write(msg3);
}

void Fault_exit() { Serial.println("Exiting Fault state"); }

void check_Fault() {
  if (button_0x01_state && LDUrpm == 0) {
    button_0x01_state = 0; KL15state = false; fsm.trigger(FAULT_CLEAR);
  }
}

// ── State: HeatPack ──────────────────────────────────────────────────────────
void HeatPack_enter() {
  Serial.println("Entering HeatPack state");
  VCUstate = VCU_STATE_HEAT_PACK;
  sdLogEvent("STATE:HEAT");
}
void HeatPack_exit() { Serial.println("Exiting HeatPack state"); }
void check_HeatPack() {
  if (button_0x01_state && LDUrpm == 0) { button_0x01_state = 0; KL15state = false; fsm.trigger(KL15_OFF); }
}

// ── State: CoolPack ──────────────────────────────────────────────────────────
void CoolPack_enter() {
  Serial.println("Entering CoolPack state");
  VCUstate = VCU_STATE_COOL_PACK;
  sdLogEvent("STATE:COOL");
}
void CoolPack_exit() { Serial.println("Exiting CoolPack state"); }
void check_CoolPack() {
  if (button_0x01_state && LDUrpm == 0) { button_0x01_state = 0; KL15state = false; fsm.trigger(KL15_OFF); }
}

// ── State: KL30C — external CAN wake standby ──────────────────────────────────
void KL30C_enter() {
  Serial.println("Entering KL30C state");
  VCUstate          = VCU_STATE_KL30C;
  sdLogEvent("STATE:KL30C");
  kl30cKL15Rstate   = false;
  lastExtActivityMs = millis();
  PDUmsg1.buf[0]    = 0x00;
  PDUmsg1.buf[3]    = 0x00;
}

void check_KL30C() {
  bool kl15rNow = digitalRead(KL15R_PIN);
  if (kl15rNow != kl30cKL15Rstate) {
    kl30cKL15Rstate     = kl15rNow;
    msg2.id             = 0x18EF2100;
    msg2.flags.extended = 1;
    msg2.len            = 8;
    msg2.buf[0] = 0x04; msg2.buf[1] = 0x1B;
    msg2.buf[2] = KEYPAD_CMD_SET_LED;
    msg2.buf[3] = 0x01;
    msg2.buf[4] = kl15rNow ? KEYPAD_COLOR_AMBER : KEYPAD_COLOR_OFF;
    msg2.buf[5] = kl15rNow ? KEYPAD_MODE_BLINK  : KEYPAD_MODE_SOLID;
    msg2.buf[6] = 0x00; msg2.buf[7] = 0xFF;
    can2.write(msg2);
  }

  if (EVCCsessionActive && evccIsACSession) { fsm.trigger(AC_CHARGE_START); return; }
  if (KL15state)                            { fsm.trigger(KL15_ON);         return; }

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
  msg2.buf[0] = 0x04; msg2.buf[1] = 0x1B;
  msg2.buf[2] = KEYPAD_CMD_SET_LED;
  msg2.buf[3] = 0x01;
  msg2.buf[4] = KEYPAD_COLOR_OFF;
  msg2.buf[5] = KEYPAD_MODE_SOLID;
  msg2.buf[6] = 0x00; msg2.buf[7] = 0xFF;
  can2.write(msg2);
}

// ── Dead code — kept for reference ───────────────────────────────────────────
void check_KL17() {
  if (KL17state) fsm.trigger(KL15_ON);
}
void check_Drive() {
  if (button_0x08_state) fsm.trigger(DRIVE_ON);
}

// ── Transition callbacks ──────────────────────────────────────────────────────
void on_trans_Off_Idle()          { Serial.println("Off → Idle"); }
void on_trans_Off_PreCharge()     { Serial.println("Off → PreCharge"); }
void on_trans_Idle_Off()          { Serial.println("→ Off"); }
void on_trans_Idle_Drive()        { Serial.println("Idle → Drive"); }
void on_trans_Drive_Idle()        { Serial.println("Drive → Idle"); }
void on_trans_Idle_Charge()       { Serial.println("Idle → Charge"); }
void on_trans_Charge_Idle()       { Serial.println("Charge → Idle"); }
void on_trans_PreCharge_Idle()    { Serial.println("PreCharge OK → Idle"); }
void on_trans_PreCharge_Charge()  { Serial.println("PreCharge OK → Charge"); }
void on_trans_PreCharge_Fault()   { Serial.println("PreCharge FAILED → Fault"); }
void on_trans_Fault_Off()         { Serial.println("Fault cleared → Off"); }
