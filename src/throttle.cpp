/* throttle.cpp — EVWest dual-pot throttle read/arbitration. */
#include <Arduino.h>
#include "defines.h"
#include "throttle.h"

/* Dual Pot Throttle (EVWest, OEM pedal)
 * Both pots are read and scaled independently. A plausibility check confirms
 * they agree within THROTTLE_PLAUSIBILITY_PCT. On fault, throttle is zeroed.
 * TODO: bench-calibrate the MIN/MAX constants, then remove the hardcoded
 *       throttle = 100 override in displayStatus().
 */
void readThrottle() {
  // ── 1. READ ─────────────────────────────────────────────────────────────
  // throttlePot1Raw, throttlePot2Raw, and brakePedal updated by ADS1115 in loop()
  regenActive = (LDUrpm > 50) && (LDUtorque < -5); // negative torque = generating
  brakePedal |= regenActive; // regen counts as braking: zeroes throttle + sets CANIO BRAKE

  // ── 2. VERIFY — 5 % plausibility window between tracks ──────────────────
  uint16_t pct1 = constrain(map(throttlePot1Raw, THROTTLE_POT1_MIN, THROTTLE_POT1_MAX, 0, 100), 0, 100);
  uint16_t pct2 = constrain(map(throttlePot2Raw, THROTTLE_POT2_MIN, THROTTLE_POT2_MAX, 0, 100), 0, 100);
  throttlePlausibility = (abs((int16_t)pct1 - (int16_t)pct2) <= THROTTLE_PLAUSIBILITY_PCT);
  uint16_t pedalPct = throttlePlausibility ? pct1 : 0; // track 1 primary; fault → 0

  // ── 3. ARBITRATE ─────────────────────────────────────────────────────────
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
