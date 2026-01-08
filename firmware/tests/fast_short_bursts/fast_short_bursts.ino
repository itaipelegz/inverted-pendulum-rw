// ===== BTS7960 "Fast Short Bursts" Test + Serial E-STOP/Resume =====
// Uses your UPDATED wiring:
//
// Motor encoder (not used here):
//   Am -> D2 (INT0)
//   Bm -> D4
//
// Shaft encoder (not used here):
//   As -> D3 (INT1)
//   Bs -> D5
//
// BTS7960:
//   RPWM -> D9  (~)
//   LPWM -> D10 (~)
//   R_EN -> D7
//   L_EN -> D8
//
// Behavior:
// - Alternates short bursts: RPWM then LPWM
// - Burst PWM = 250 (out of 255)
// - "Fast and short": default 80ms ON, 120ms OFF (tweak below)
// - Serial control:
//     x = E-STOP (disable EN + PWM=0) and pause
//     r = resume (enable EN) and continue
//     s = stop (PWM=0 but EN stays enabled) and pause
//     g = go (continue)
//     ? = print help

#include <Arduino.h>

// --- BTS7960 pins (updated wiring) ---
const uint8_t RPWM_PIN = 9;
const uint8_t LPWM_PIN = 10;
const uint8_t R_EN_PIN = 7;
const uint8_t L_EN_PIN = 8;

// --- Burst parameters ---
const uint8_t PWM_BURST = 250;
const unsigned long ON_MS  = 80;   // "short"
const unsigned long OFF_MS = 120;  // "fast"

// --- State ---
bool estop = false;   // hard stop: EN low + PWM 0
bool paused = false;  // soft pause: PWM 0, EN high
bool phaseRPWM = true;

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

void printHelp() {
  Serial.println("Commands:");
  Serial.println("  x = E-STOP (EN=LOW, PWM=0)");
  Serial.println("  r = Resume from E-STOP (EN=HIGH)");
  Serial.println("  s = Stop/Pause (PWM=0, EN stays HIGH)");
  Serial.println("  g = Go/Continue");
  Serial.println("  ? = Help");
}

void handleSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == 'x' || c == 'X') {
      estop = true;
      paused = true;
      disableDriver();
      Serial.println("!!! E-STOP ACTIVATED !!!");
    } else if (c == 'r' || c == 'R') {
      estop = false;
      enableDriver();
      Serial.println("E-STOP released (EN=HIGH).");
    } else if (c == 's' || c == 'S') {
      paused = true;
      allStopPWM();
      // keep EN as-is (usually HIGH)
      Serial.println("Paused: PWM=0 (EN unchanged).");
    } else if (c == 'g' || c == 'G') {
      if (!estop) {
        paused = false;
        Serial.println("Running.");
      } else {
        Serial.println("Cannot run: E-STOP is active. Send 'r' first.");
      }
    } else if (c == '?') {
      printHelp();
    }
  }
}

// Delay helper that stays responsive to serial commands
bool waitResponsive(unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    handleSerial();
    if (estop) return false;
    if (paused) return false;
    delay(2);
  }
  return true;
}

void setup() {
  Serial.begin(115200);

  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  pinMode(R_EN_PIN, OUTPUT);
  pinMode(L_EN_PIN, OUTPUT);

  enableDriver();
  allStopPWM();

  Serial.println("BTS7960 Fast Short Bursts test (updated wiring).");
  Serial.print("PWM_BURST=");
  Serial.print(PWM_BURST);
  Serial.print(" | ON_MS=");
  Serial.print(ON_MS);
  Serial.print(" | OFF_MS=");
  Serial.println(OFF_MS);
  printHelp();

  paused = true; // start safe: wait for 'g'
  Serial.println("Starting paused. Send 'g' to run.");
}

void loop() {
  handleSerial();

  if (estop) {
    // Stay here until released
    delay(20);
    return;
  }

  if (paused) {
    allStopPWM();
    delay(20);
    return;
  }

  // Ensure driver enabled while running
  enableDriver();

  // Phase: burst in one direction
  if (phaseRPWM) {
    analogWrite(RPWM_PIN, PWM_BURST);
    analogWrite(LPWM_PIN, 0);
    // Serial.println("Burst: RPWM"); // uncomment if you want spammy logs
  } else {
    analogWrite(RPWM_PIN, 0);
    analogWrite(LPWM_PIN, PWM_BURST);
    // Serial.println("Burst: LPWM");
  }

  if (!waitResponsive(ON_MS)) {
    allStopPWM();
    return;
  }

  // Off time
  allStopPWM();
  if (!waitResponsive(OFF_MS)) {
    return;
  }

  // Toggle direction for next burst
  phaseRPWM = !phaseRPWM;
}
