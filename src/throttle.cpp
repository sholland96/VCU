/* throttle.cpp — Dual pot throttle read, plausibility check, and demand arbitration. */
#include "vcu.h"
#include "throttle.h"

void readThrottle() {
  // throttlePot1Raw, throttlePot2Raw, brakePedal updated by ADS1115 in loop()
  regenActive = (LDUrpm > 50) && (LDUtorque < -5);
  brakePedal |= regenActive;

  uint16_t pct1 = constrain(map(throttlePot1Raw, THROTTLE_POT1_MIN, THROTTLE_POT1_MAX, 0, 100), 0, 100);
  uint16_t pct2 = constrain(map(throttlePot2Raw, THROTTLE_POT2_MIN, THROTTLE_POT2_MAX, 0, 100), 0, 100);
  throttlePlausibility = (abs((int16_t)pct1 - (int16_t)pct2) <= THROTTLE_PLAUSIBILITY_PCT);
  uint16_t pedalPct = throttlePlausibility ? pct1 : 0;

  bool faultActive = IVTfaultActive || (SIMM100MODerrorFlags != 0);
  if (!throttlePlausibility || brakePedal) {
    pedalPct = 0;
  } else if (faultActive) {
    pedalPct = min(pedalPct, (uint16_t)THROTTLE_FAULT_LIMIT);
  }
  throttle = pedalPct;

  LDUtorqueSetpoint = (VCUstate == VCU_STATE_DRIVE) ? (int16_t)pedalPct : 0;
}
