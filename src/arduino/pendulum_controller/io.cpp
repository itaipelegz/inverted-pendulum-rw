#include <Arduino.h>

#include "config.h"
#include "io.h"
#include "motor_driver.h"
#include "safety.h"
#include "state.h"

long getMotorCount() {
  noInterrupts();
  long c = g_m_count;
  interrupts();
  return c;
}

long getPendulumRawCount() {
  noInterrupts();
  long c = g_p_count;
  interrupts();
  return c;
}

long getPendulumCount() {
  return getPendulumRawCount() - p_zeroOffset;
}

// ------------------ ISRs ------------------
void motorA_isr() {
  // If direction is reversed, swap ++/--
  if (digitalRead(BM_PIN) == HIGH) g_m_count++;
  else g_m_count--;
}

void pendulumA_isr() {
  // If direction is reversed, swap ++/--
  if (digitalRead(BS_PIN) == HIGH) g_p_count++;
  else g_p_count--;
}

// Calibration is performed when pendulum is DOWN,
// but we want p_angle = 0 when pendulum is UP.
//
// So when we press 'z' at DOWN, we shift the offset by half a turn.
void setZeroAtUpUsingDownCalibration() {
  long rawDownCount = getPendulumRawCount();
  p_calibRawCount = rawDownCount;

  // half revolution in counts
  long halfTurnCounts = (long)(SHAFT_COUNTS_PER_REV / 2.0f);

  noInterrupts();
  p_zeroOffset = rawDownCount - halfTurnCounts;
  interrupts();

  // Reset estimator state safely (avoid velocity spike)
  long pC_now = getPendulumCount();
  long mC_now = getMotorCount();

  lastPCountSnap = pC_now;
  lastMCountSnap = mC_now;
  lastEstMs = millis();

  p_angle = PI;   // because we're currently DOWN
  p_vel = 0.0f;
  m_vel = 0.0f;

  // IMPORTANT: stop motion after calibration
  paused = true;                 // require user to press 'g' again
  allStopPWM();
  u_applied = 0;                 // reset ramp state
  dirChangeHoldUntilMs = 0;
  p_vel_sign_latched = 0;        // reset swing-up sign latch

  Serial.println("Calibrated at DOWN. Now p_angle=0 is UP (DOWN reads ~+PI). PAUSED (press 'g' to run).");
}

void printHelp() {
  Serial.println("Commands: z=calibrate at DOWN (sets p_angle=0 at UP), g=go, s=pause, x=ESTOP, r=release ESTOP, p=print, ?=help");
  Serial.println("Safety: full turn => ESTOP, and stabilize only if approaching UP from CCW.");
}

void printSnapshot() {
  bool nearUp = (fabs(p_angle) < THETA_STAB_RAD);
  const char* mode = nearUp ? "STAB" : "SWING";

  Serial.print("t_ms="); Serial.print(millis());
  // Serial.print(" | pC="); Serial.print(pC); // debug
  // Serial.print(" | rawDelta="); Serial.print(rawDelta); // debug
  Serial.print(" | mode="); Serial.print(mode);
  Serial.print(" | p_angle(rad)="); Serial.print(p_angle, 6);
  Serial.print(" | p_angle(deg)="); Serial.print(p_angle * RAD_TO_DEG, 2);
  Serial.print(" | p_vel(rad/s)="); Serial.print(p_vel, 6);
  Serial.print(" | m_vel(rad/s)="); Serial.print(m_vel, 6);
  Serial.print(" | PWM="); Serial.print(u_applied);
  Serial.print(" | state=");
  if (estop) Serial.println("ESTOP");
  else if (paused) Serial.println("PAUSED");
  else Serial.println("RUN");
}

void handleSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'z' || c == 'Z') setZeroAtUpUsingDownCalibration();
    else if (c == 'g' || c == 'G') {
      if(!estop){
        paused = false;
        Serial.println("RUN.");
      }
    }
    else if (c == 's' || c == 'S') { paused = true; allStopPWM(); Serial.println("PAUSED."); }
    else if (c == 'x' || c == 'X') { estop = true; paused = true; disableDriver(); Serial.println("!!! ESTOP !!!"); }
    else if (c == 'r' || c == 'R') {
      estop = false;
      paused = true;
      fullTurnTripped = false;
      enableDriver();
      allStopPWM();
      Serial.println("ESTOP released. Still paused.");
    }
    else if (c == 'p' || c == 'P') printSnapshot();
    else if (c == '?') printHelp();
  }
}
