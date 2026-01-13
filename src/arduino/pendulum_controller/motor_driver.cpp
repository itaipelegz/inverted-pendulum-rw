#include <Arduino.h>

#include "config.h"
#include "motor_driver.h"
#include "state.h"

void allStopPWM() {
  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, 0);
}

void enableDriver() {
  digitalWrite(R_EN_PIN, HIGH);
  digitalWrite(L_EN_PIN, HIGH);
}

void disableDriver() {
  allStopPWM();
  digitalWrite(R_EN_PIN, LOW);
  digitalWrite(L_EN_PIN, LOW);
}

// u in [-255, +255]
void setMotorU(int u) {
  // ---- GLOBAL motor direction fix (single point of truth) ----
  u = -u;  // <--- flips the motor direction for ALL control modes

  // ---- Global absolute PWM clamp ----
  u = constrain(u, -PWM_LIMIT, PWM_LIMIT);

  // ---- Drive BTS7960 ----
  if (u > 0) {
    analogWrite(RPWM_PIN, (uint8_t)u);
    analogWrite(LPWM_PIN, 0);
  } else if (u < 0) {
    analogWrite(RPWM_PIN, 0);
    analogWrite(LPWM_PIN, (uint8_t)(-u));
  } else {
    allStopPWM();
  }
}

// ------------------ Motor output smoothing (direction-safe ramp) ------------------
//
// Purpose:
//   Prevent violent direction changes by:
//   1) limiting PWM slew-rate (ramp)
//   2) forcing a short zero-PWM dead-time before reversing direction
//
// Apply motor command with ramp + safe direction change
//
// This function MUST be used instead of calling setMotorU() directly
void applyMotorWithRamp(int u_target) {
  unsigned long now = millis();

  // If we are inside a forced zero-PWM window (direction change dead-time)
  if (now < dirChangeHoldUntilMs) {
    u_applied = 0;
    setMotorU(0);
    return;
  }

  // Detect direction change request
  if ((u_target > 0 && u_applied < 0) ||
      (u_target < 0 && u_applied > 0)) {

    // Force PWM = 0 briefly before reversing direction
    u_applied = 0;
    setMotorU(0);
    dirChangeHoldUntilMs = now + DIR_CHANGE_ZERO_HOLD_MS;
    return;
  }

  // Slew-rate limit (PWM ramp)
  int diff = u_target - u_applied;
  if (diff >  PWM_STEP_PER_CTRL) diff =  PWM_STEP_PER_CTRL;
  if (diff < -PWM_STEP_PER_CTRL) diff = -PWM_STEP_PER_CTRL;

  u_applied += diff;
  setMotorU(u_applied);
}
