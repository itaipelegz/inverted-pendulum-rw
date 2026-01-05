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

// ------------------ BTS7960 pins ------------------
const uint8_t RPWM_PIN = 9;
const uint8_t LPWM_PIN = 10;
const uint8_t R_EN_PIN = 7;
const uint8_t L_EN_PIN = 8;

// ------------------ Encoders pins ------------------
// Motor encoder
const uint8_t AM_PIN = 2;  // INT0
const uint8_t BM_PIN = 4;  // direction

// Shaft (pendulum) encoder
const uint8_t AS_PIN = 3;  // INT1
const uint8_t BS_PIN = 5;  // direction

// ------------------ Encoder calibration ------------------
// Motor encoder: you measured 10 turns = 17179 counts using 4X decoding previously.
// In our "1 interrupt on A rising" scheme, effective CPR is /4:
const float MOTOR_COUNTS_PER_REV_4X = 1717.9f;
const float MOTOR_DECODE_FACTOR = 4.0f; // 4X -> 1X (A rising)
const float MOTOR_COUNTS_PER_REV = MOTOR_COUNTS_PER_REV_4X / MOTOR_DECODE_FACTOR; // ~429.475

// Shaft encoder: advertised 360 PPR. With 1 interrupt on A rising => 1X => 360 counts/rev.
// If you switch to CHANGE on A, set SHAFT_DECODE_FACTOR=2 and attachInterrupt CHANGE.
const bool SHAFT_USE_CHANGE = false;
const float SHAFT_PPR = 360.0f;
const float SHAFT_DECODE_FACTOR = SHAFT_USE_CHANGE ? 2.0f : 1.0f;
const float SHAFT_COUNTS_PER_REV = SHAFT_PPR * SHAFT_DECODE_FACTOR;

// ------------------ Estimator settings ------------------
const unsigned long EST_PERIOD_MS = 5; // ~200Hz updates
const float P_VEL_ALPHA = 0.25f;
const float M_VEL_ALPHA = 0.25f;

// ------------------ State (counts) ------------------
volatile long g_m_count = 0; // motor encoder count
volatile long g_p_count = 0; // pendulum/shaft encoder count

long p_zeroOffset = 0;
long p_calibRawCount = 0;   // raw count at the moment we pressed 'z' (pendulum DOWN)


// ------------------ Estimated signals ------------------
float p_angle = 0.0f; // rad, wrapped [-PI, +PI]
//   p_angle = 0   means UP
//   p_angle = +PI or -PI means DOWN
float p_vel = 0.0f; // rad/s filtered
float m_vel = 0.0f; // rad/s filtered

unsigned long lastEstMs = 0;
long lastPCountSnap = 0;
long lastMCountSnap = 0;

// ------------------ Helpers ------------------
int sgn(float x, float eps = 1e-4f) {
  if (x > eps) return +1;
  if (x < -eps) return -1;
  return 0;
}

float wrapToPi(float a) {
  a = fmod(a + PI, TWO_PI);
  if (a < 0) a += TWO_PI;
  return a - PI;
}

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

// ------------------ Estimator update ------------------
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

// ------------------ Motor driver ------------------
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

// ------------------ PWM LIMIT (restriction #4) ------------------
// This is the GLOBAL absolute PWM clamp for ANY command.
// Set it low at first (e.g., 40-80) and raise carefully.
const int PWM_LIMIT = 180;

// u in [-255, +255]
void setMotorU(int u) {
  // Restriction #4: limit PWM HERE (single place affects everything)
  u = constrain(u, -PWM_LIMIT, PWM_LIMIT);

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

// ------------------ Control (swing-up + stabilize) ------------------
// We now have ZERO = DOWN.
// Therefore "UP" is at angle ~ +/-PI.
// We'll stabilize around UP using an error:
//   up_err = wrapToPi(p_angle - PI)   (also equivalent near -PI)
const float THETA_STAB_RAD = 0.35f; // smaller = safer. start 0.25..0.5

// Swing-up PWM magnitude (START LOW!)
int PWM_SWING = 120;       // still here (restriction #4 clamps anyway)

// PD stabilize near UP (START LOW!)
float KP = 60.0f;
float KD = 8.0f;

// Keep this too; it is an internal limit for stabilize output BEFORE global PWM_LIMIT
int PWM_MAX_STAB = 120;

const unsigned long CTRL_PERIOD_MS = 20;
unsigned long lastCtrlMs = 0;

// ------------------ Serial control ------------------
bool estop = false;
bool paused = true;

// Restriction #2 full-turn protection:
bool fullTurnTripped = false;

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

  // Reset estimator state to avoid spikes
  lastPCountSnap = 0;
  p_angle = PI;   // because we're currently DOWN
  p_vel = 0.0f;

  Serial.println("Calibrated at DOWN. Now p_angle=0 is UP (DOWN reads ~+PI).");
}


void printHelp() {
  Serial.println("Commands: z=calibrate at DOWN (sets p_angle=0 at UP), g=go, s=pause, x=ESTOP, r=release ESTOP, p=print, ?=help");
  Serial.println("Safety: full turn => ESTOP, and stabilize only if approaching UP from CCW.");
}

void printSnapshot(const char* tag="SNAP") {
  // Up error around PI (UP)
  float up_err = p_angle;

  Serial.print(tag);
  Serial.print(" | p_angle(rad)="); Serial.print(p_angle, 6);
  Serial.print(" | p_angle(deg)="); Serial.print(p_angle * RAD_TO_DEG, 2);
  Serial.print(" | up_err(rad)="); Serial.print(up_err, 6);
  Serial.print(" | p_vel(rad/s)="); Serial.print(p_vel, 6);
  Serial.print(" | m_vel(rad/s)="); Serial.print(m_vel, 6);
  Serial.print(" | PWM_LIMIT="); Serial.print(PWM_LIMIT);
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
    else if (c == 'p' || c == 'P') printSnapshot("PRINT");
    else if (c == '?') printHelp();
  }
}

// ------------------ Safety checks (added) ------------------
void safetyChecks() {
  // Restriction #2: if pendulum completes a full turn -> stop immediately.
  // Using count since last zero:
  long rawNow = getPendulumRawCount();
  long rawDelta = rawNow - p_calibRawCount;
  
  if (!fullTurnTripped && (labs(rawDelta) >= (long)(0.90f * SHAFT_COUNTS_PER_REV))) {
    triggerEstop("Pendulum full turn detected (>= ~324deg since calibration)");
  }
}

// Restriction #3: Swing-up should complete ONLY from CCW direction.
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

  // CCW-only completion condition:
  bool ccwSide = (p_angle > 0.0f);   // near -PI branch (UP approached via negative wrap)
  bool ccwVel  = (p_vel < 0.0f);     // CCW gives positive velocity in your convention

  if (nearUp && ccwSide && ccwVel) {
    float u = KP * up_err + KD * p_vel;
    int ui = (int)lroundf(u);
    ui = constrain(ui, -PWM_MAX_STAB, PWM_MAX_STAB);
    return ui;
  }

  int v = sgn(p_vel);
  if (v == 0) return 0;
  return -v * PWM_SWING;
}


// ------------------ Setup/loop ------------------
void setup() {
  Serial.begin(115200);

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
    setMotorU(u);
    lastCtrlMs = now;
  }

  static unsigned long lastPrint = 0;
  if (now - lastPrint >= 200) {
    printSnapshot("STREAM");
    lastPrint = now;
  }
}
