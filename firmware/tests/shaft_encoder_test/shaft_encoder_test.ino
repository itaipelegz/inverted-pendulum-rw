// ===== Shaft/Pendulum Encoder (UNO) - theta in [-PI, +PI], zero at UP =====
// Mapping you chose:
//   As (shaft A) -> D3 (INT1)
//   Bs (shaft B) -> D5 (digital input)
// Encoder: AB 2-phase NPN, 360 PPR
//
// Decoding: 1 interrupt only (A), RISING edges (1X) for low CPU.
// If you want more resolution: change to CHANGE (2X) below.
//
// Serial commands:
//   z = set current position as ZERO (UP)
//   p = print snapshot

#include <Arduino.h>

const uint8_t AS_PIN = 3;   // INT1
const uint8_t BS_PIN = 5;   // any digital pin

// Keep advertised PPR:
const float PPR = 360.0f;

// Choose decoding mode (1 interrupt):
// - RISING: 1X (360 counts/rev)
// - CHANGE: 2X (720 counts/rev)
const bool USE_CHANGE = false;

const float DECODE_FACTOR = USE_CHANGE ? 2.0f : 1.0f;
const float COUNTS_PER_REV = PPR * DECODE_FACTOR;

const unsigned long UPDATE_PERIOD_MS = 10; // 100 Hz update for theta_dot
const float OMEGA_ALPHA = 0.2f;            // LPF 0..1

volatile long g_count = 0;
long g_zeroOffset = 0;

unsigned long lastUpdateMs = 0;
long lastCountSnapshot = 0;

float theta = 0.0f;      // rad, wrapped to [-PI, +PI]
float thetaDot = 0.0f;   // rad/s filtered

// Wrap angle to [-PI, +PI]
float wrapToPi(float a) {
  // Using fmod to avoid slow loops
  a = fmod(a + PI, TWO_PI);
  if (a < 0) a += TWO_PI;
  return a - PI;
}

void shaftA_isr() {
  // Direction from B
  // If direction is reversed, swap ++/--
  if (digitalRead(BS_PIN) == HIGH) g_count++;
  else g_count--;
}

long getRawCount() {
  noInterrupts();
  long c = g_count;
  interrupts();
  return c;
}

long getCount() {
  return getRawCount() - g_zeroOffset;
}

void setZeroHere() {
  long raw = getRawCount();
  noInterrupts();
  g_zeroOffset = raw;
  interrupts();

  // Reset estimator state to avoid spikes
  lastCountSnapshot = 0;
  theta = 0.0f;
  thetaDot = 0.0f;

  Serial.println("ZERO set: current position is theta=0 (UP).");
}

void updateEstimator() {
  unsigned long now = millis();
  unsigned long dtMs = now - lastUpdateMs;
  if (dtMs < UPDATE_PERIOD_MS) return;

  long c = getCount();
  long delta = c - lastCountSnapshot;

  float dt = dtMs / 1000.0f;
  float omegaRaw = 0.0f;
  if (dt > 0.0f) {
    omegaRaw = ((float)delta / COUNTS_PER_REV) * TWO_PI / dt; // rad/s (signed)
  }

  // Filter angular velocity
  thetaDot = (1.0f - OMEGA_ALPHA) * thetaDot + OMEGA_ALPHA * omegaRaw;

  // Compute theta from absolute count and wrap to [-PI, +PI]
  float thetaUnwrapped = ((float)c / COUNTS_PER_REV) * TWO_PI;
  theta = wrapToPi(thetaUnwrapped);

  lastCountSnapshot = c;
  lastUpdateMs = now;
}

void printSnapshot(const char* tag="SNAP") {
  long c = getCount();
  float rev = (float)c / COUNTS_PER_REV;

  Serial.print(tag);
  Serial.print(" | count=");
  Serial.print(c);
  Serial.print(" | rev=");
  Serial.print(rev, 6);
  Serial.print(" | theta(rad)=");
  Serial.print(theta, 6);
  Serial.print(" | theta(deg)=");
  Serial.print(theta * RAD_TO_DEG, 2);
  Serial.print(" | thetaDot(rad/s)=");
  Serial.println(thetaDot, 6);
}

void handleSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'z' || c == 'Z') setZeroHere();
    else if (c == 'p' || c == 'P') printSnapshot("PRINT");
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(AS_PIN, INPUT_PULLUP);
  pinMode(BS_PIN, INPUT_PULLUP);

  attachInterrupt(
    digitalPinToInterrupt(AS_PIN),
    shaftA_isr,
    USE_CHANGE ? CHANGE : RISING
  );

  lastUpdateMs = millis();
  lastCountSnapshot = 0;

  Serial.println("Shaft encoder started: theta in [-PI, +PI], zero=UP.");
  Serial.println("Commands: z=zero at UP, p=print snapshot");
  Serial.print("Mode: ");
  Serial.println(USE_CHANGE ? "CHANGE (2X)" : "RISING (1X)");
  Serial.print("COUNTS_PER_REV = ");
  Serial.println(COUNTS_PER_REV, 1);
}

void loop() {
  handleSerial();
  updateEstimator();

  // Stream at ~20Hz
  static unsigned long lastPrint = 0;
  unsigned long now = millis();
  if (now - lastPrint >= 50) {
    printSnapshot("STREAM");
    lastPrint = now;
  }
}
