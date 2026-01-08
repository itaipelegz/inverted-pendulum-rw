// ===== Reaction Wheel Pendulum (UNO) =====
// Updated wiring:
//
// Motor encoder:
//   Am -> D2 (INT0)
//   Bm -> D4
//
// Shaft (pendulum) encoder:
//   As -> D3 (INT1)
//   Bs -> D5
//
// BTS7960:
//   RPWM -> D9 (~)
//   LPWM -> D10(~)
//   R_EN -> D7
//   L_EN -> D8
//
// Signals naming:
//   p_angle = pendulum angle (rad), wrapped to [-PI, +PI]
//   p_vel   = pendulum angular velocity (rad/s), filtered
//   m_vel   = motor angular velocity (rad/s), filtered
//
// Serial commands:
//   z = set current pendulum position as ZERO (NOW: DOWN reference)
//   g = go/run
//   s = pause (PWM=0, EN stays HIGH)
//   x = E-STOP (EN LOW, PWM=0)
//   r = release E-STOP (EN HIGH, still paused)
//   p = print snapshot
//   ? = help

#include <Arduino.h>

#include "config.h"
#include "control.h"
#include "io.h"
#include "motor_driver.h"
#include "safety.h"
#include "state.h"

void setup() {
  Serial.begin(BAUDRATE);

  // BTS7960
  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  pinMode(R_EN_PIN, OUTPUT);
  pinMode(L_EN_PIN, OUTPUT);
  enableDriver();
  allStopPWM();

  // Encoders
  pinMode(AM_PIN, INPUT_PULLUP);
  pinMode(BM_PIN, INPUT_PULLUP);
  pinMode(AS_PIN, INPUT_PULLUP);
  pinMode(BS_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(AM_PIN), motorA_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(AS_PIN), pendulumA_isr, SHAFT_USE_CHANGE ? CHANGE : RISING);

  lastEstMs = millis();
  lastCtrlMs = millis();

  Serial.println("RWP control: swing-up by sign(p_vel) + stabilize near UP (ZERO = UP).");
  Serial.print("Motor CPR (1X) = "); Serial.println(MOTOR_COUNTS_PER_REV, 3);
  Serial.print("Shaft CPR = "); Serial.println(SHAFT_COUNTS_PER_REV, 1);
  Serial.print("PWM_LIMIT = "); Serial.println(PWM_LIMIT);
  Serial.println("Start paused. Put pendulum DOWN, send 'z', then 'g'.");
  printHelp();
}

void loop() {
  handleSerial();
  updateEstimates();

  // Added safety checks (restriction #2)
  safetyChecks();

  if (estop) { delay(10); return; }
  if (paused) { allStopPWM(); delay(5); return; }

  unsigned long now = millis();
  if (now - lastCtrlMs >= CTRL_PERIOD_MS) {
    int u = computeU();
    applyMotorWithRamp(u);
    // setMotorU(u);
    lastCtrlMs = now;
  }

  static unsigned long lastPrint = 0;
  if (now - lastPrint >= 200) {
    printSnapshot("STREAM");
    lastPrint = now;
  }
}
