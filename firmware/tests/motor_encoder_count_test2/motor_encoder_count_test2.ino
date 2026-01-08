// ===== Encoder Control Module (UNO) + Serial Commands =====
// - ONE interrupt (channel A on D2)
// - Direction from channel B (D4)
// - Provides:
//    * count (signed)
//    * angle (rad) unwrapped
//    * omega (rad/s) filtered
//
// Added (as requested):
//   Serial commands:
//     z = zero/reset count (and angle baseline)
//     p = print a one-line snapshot immediately
//   Print also:
//     revolutions (rev)
//     angle degrees within [0..360) (wrapped)
//
// Wiring:
//   Encoder A -> D2  (INT0)
//   Encoder B -> D4
//   Vcc -> 5V (if supported)
//   GND -> GND
//
// Calibrated from your measurement:
//   10 turns = 17179 counts  => 1717.9 counts/rev

#include <Arduino.h>

const uint8_t ENC_A_PIN = 2;  // interrupt pin
const uint8_t ENC_B_PIN = 4;  // direction pin

// Encoder calibration (measured with full quadrature decoding = 4X)
const float COUNTS_PER_REV_4X = 1717.9f;

// Current decoding mode:
// We use ONLY channel A, RISING edge  -> 1X decoding
const float DECODE_FACTOR = 4.0f;

// Effective counts per mechanical revolution
const float COUNTS_PER_REV = COUNTS_PER_REV_4X / DECODE_FACTOR;

const unsigned long UPDATE_PERIOD_MS = 10;      // speed estimate update ~100Hz
const float SPEED_ALPHA = 0.2f;                 // LPF: 0..1 (higher = faster response)

volatile long g_count = 0;                      // updated in ISR

// Baseline offset so you can "zero" wherever you want
long g_zeroOffset = 0;

// ISR: triggered on A rising edge only (low CPU load)
void encoderA_rise_isr() {
  // Direction from B:
  // If your direction feels reversed, flip the sign here (swap ++/--).
  if (digitalRead(ENC_B_PIN) == HIGH) {
    g_count++;   // one direction
  } else {
    g_count--;   // other direction
  }
}

long getRawCount() {
  noInterrupts();
  long c = g_count;
  interrupts();
  return c;
}

// Count relative to last zero command
long getCount() {
  return getRawCount() - g_zeroOffset;
}

float countToRevs(long count) {
  return (float)count / COUNTS_PER_REV;
}

float countToRad(long count) {
  return countToRevs(count) * TWO_PI;
}

float wrapDeg360(float deg) {
  // Wrap to [0, 360)
  float w = fmod(deg, 360.0f);
  if (w < 0.0f) w += 360.0f;
  return w;
}

// State for speed/angle calculation
unsigned long lastUpdateMs = 0;
long lastCountSnapshot = 0;

float omegaFilt = 0.0f;   // rad/s filtered
float angleRad = 0.0f;    // rad (unwrapped, relative to zero)

void updateEncoderEstimates() {
  unsigned long now = millis();
  unsigned long dtMs = now - lastUpdateMs;
  if (dtMs < UPDATE_PERIOD_MS) return;

  long c = getCount();
  long delta = c - lastCountSnapshot;

  float dt = dtMs / 1000.0f;  // seconds
  float omegaRaw = 0.0f;

  if (dt > 0.0f) {
    omegaRaw = ((float)delta / COUNTS_PER_REV) * TWO_PI / dt; // rad/s (signed)
  }

  // Low-pass filter for omega
  omegaFilt = (1.0f - SPEED_ALPHA) * omegaFilt + SPEED_ALPHA * omegaRaw;

  // Angle (unwrapped) from absolute (relative) count
  angleRad = countToRad(c);

  lastCountSnapshot = c;
  lastUpdateMs = now;
}

float getAngleRad() {
  return angleRad;
}

float getOmegaRadS() {
  return omegaFilt;
}

float getRevs() {
  return countToRevs(getCount());
}

float getAngleDegWrapped() {
  float deg = getAngleRad() * RAD_TO_DEG;
  return wrapDeg360(deg);
}

void doZero() {
  // Make current raw count become zero
  long raw = getRawCount();

  // Update offset atomically-ish
  noInterrupts();
  g_zeroOffset = raw;
  interrupts();

  // Reset derivative bookkeeping so omega doesn't spike after zero
  lastCountSnapshot = 0;
  omegaFilt = 0.0f;
  angleRad = 0.0f;

  Serial.println("ZERO: count set to 0 (baseline updated).");
}

void printSnapshot(const char* prefix = "SNAP") {
  long c = getCount();
  float rev = countToRevs(c);
  float angRad = countToRad(c);
  float angDegW = wrapDeg360(angRad * RAD_TO_DEG);

  Serial.print(prefix);
  Serial.print(" | count=");
  Serial.print(c);
  Serial.print(" | rev=");
  Serial.print(rev, 6);
  Serial.print(" | angle_rad=");
  Serial.print(angRad, 6);
  Serial.print(" | angle_deg_wrapped=");
  Serial.print(angDegW, 2);
  Serial.print(" | omega_rad_s=");
  Serial.println(getOmegaRadS(), 6);
}

void handleSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'z' || c == 'Z') {
      doZero();
    } else if (c == 'p' || c == 'P') {
      printSnapshot("PRINT");
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), encoderA_rise_isr, RISING);

  lastUpdateMs = millis();
  lastCountSnapshot = 0;

  Serial.println("Encoder control module started (+serial z/p).");
  Serial.println("Commands: z=zero, p=print snapshot");
  Serial.println("Stream: ms | count | rev | angle_deg(0..360) | omega(rad/s)");
}

void loop() {
  handleSerial();
  updateEncoderEstimates();

  // Stream at ~20Hz for monitoring
  static unsigned long lastPrint = 0;
  unsigned long now = millis();
  if (now - lastPrint >= 50) {
    long c = getCount();
    Serial.print(now);
    Serial.print(" | ");
    Serial.print(c);
    Serial.print(" | ");
    Serial.print(getRevs(), 6);
    Serial.print(" | ");
    Serial.print(getAngleDegWrapped(), 2);
    Serial.print(" | ");
    Serial.println(getOmegaRadS(), 6);
    lastPrint = now;
  }

  // Later: controller goes here
}
