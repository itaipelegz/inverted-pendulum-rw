#include <Arduino.h>

#include "config.h"
#include "control.h"
#include "io.h"
#include "state.h"

int sgn(float x, float eps = 1e-4f) {
  if (x > eps) return +1;
  if (x < -eps) return -1;
  return 0;
}

int pendulumVelSignStable(float vel) {
  // Updates latched sign only when |vel| is above deadband.
  if (vel >  P_VEL_DEADBAND) p_vel_sign_latched = +1;
  if (vel < -P_VEL_DEADBAND) p_vel_sign_latched = -1;
  return p_vel_sign_latched; // holds last sign inside deadband
}

float wrapToPi(float a) {
  a = fmod(a + PI, TWO_PI);
  if (a < 0) a += TWO_PI;
  return a - PI;
}

void updateEstimates() {
  unsigned long now = millis();
  unsigned long dtMs = now - lastEstMs;
  if (dtMs < EST_PERIOD_MS) return;

  float dt = dtMs / 1000.0f;
  if (dt <= 0.0f) return;

  // Snapshot counts
  long pC = getPendulumCount();
  long mC = getMotorCount();

  long dP = pC - lastPCountSnap;
  long dM = mC - lastMCountSnap;

  // Raw velocities
  float p_vel_raw = ((float)dP / SHAFT_COUNTS_PER_REV) * TWO_PI / dt; // rad/s
  float m_vel_raw = ((float)dM / MOTOR_COUNTS_PER_REV) * TWO_PI / dt; // rad/s

  // Filtered velocities
  p_vel = (1.0f - P_VEL_ALPHA) * p_vel + P_VEL_ALPHA * p_vel_raw;
  m_vel = (1.0f - M_VEL_ALPHA) * m_vel + M_VEL_ALPHA * m_vel_raw;

  // Angle from pendulum count (ZERO = DOWN)
  float p_angle_unwrapped = ((float)pC / SHAFT_COUNTS_PER_REV) * TWO_PI;
  p_angle = wrapToPi(p_angle_unwrapped);

  lastPCountSnap = pC;
  lastMCountSnap = mC;
  lastEstMs = now;
}

// Restriction #3: Swing-up should complete ONLY from CCW direction (due to mechanical restriction).
// Assumption: positive p_vel corresponds to CCW (if not, we can flip ISR sign).
// We will allow "stabilize near UP" ONLY if:
//   - close to UP (up_err small)
//   - approaching from CCW side (p_angle < 0) AND moving CCW (p_vel > 0)
//
// Otherwise we keep using swing-up.
//   - approaching from CCW side (p_angle < 0) AND moving CCW (p_vel > 0)
int computeU() {
  float up_err = p_angle;
  bool nearUp = (fabs(up_err) < THETA_STAB_RAD);
  // Latch entry to only evaluate CCW condition once per stability-zone entry.
  static bool wasInStabZone = false;
  bool enteringStabZone = nearUp && !wasInStabZone;
  wasInStabZone = nearUp;

  // CCW-only completion condition:
  bool ccwSide = (p_angle > 0.0f);   // CCW approach side is the +angle branch
  bool ccwVel  = (p_vel < 0.0f);    // CCW gives NEGATIVE velocity (measured)


  if (enteringStabZone && ccwSide && ccwVel) {
    float u = -KP * up_err - KD * p_vel - KW * m_vel;
    int ui = (int)lroundf(u);
    ui = constrain(ui, -PWM_MAX_STAB, PWM_MAX_STAB);
    return ui;
  }

  int v = pendulumVelSignStable(p_vel);
  if (v == 0) return 0;
  return -v * PWM_MAX_SWING;
}
