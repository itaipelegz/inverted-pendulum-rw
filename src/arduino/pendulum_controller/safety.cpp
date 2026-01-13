#include <Arduino.h>

#include "config.h"
#include "safety.h"
#include "state.h"
#include "motor_driver.h"
#include "io.h"

// --- E-STOP helper used by safety rules ---
void triggerEstop(const char* reason) {
  estop = true;
  paused = true;
  fullTurnTripped = true;     // latch until 'r'
  disableDriver();
  Serial.print("!!! ESTOP TRIGGERED: ");
  Serial.print(reason);
  Serial.println(" !!!");
}

// ------------------ Safety checks (added) ------------------
void safetyChecks() {
  // Restriction #2: if pendulum completes a full turn -> stop immediately.
  // Using count since last zero:
  long rawNow = getPendulumRawCount();
  long rawDelta = rawNow - p_calibRawCount;

  if (!fullTurnTripped && (labs(rawDelta) >= (long)(FULL_TURN_THRESHOLD_FRACTION * SHAFT_COUNTS_PER_REV))) {
    triggerEstop("Pendulum full turn detected (>= ~324deg since calibration)");
  }

}
