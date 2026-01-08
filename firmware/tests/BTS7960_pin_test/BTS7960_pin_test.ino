// ===== BTS7960 Pin-Test (NO MOTOR / SAFE) + SERIAL E-STOP =====
// Purpose: Verify Arduino -> BTS7960 control pins behave as expected.
// It alternates RPWM and LPWM at low PWM while keeping both EN pins HIGH.
// Added: Serial E-STOP:
//   - Send 'x' to activate E-STOP (disable EN + stop PWM)
//   - Send 'r' to release E-STOP (enable EN + resume)
//
// Wiring (adjust if you used different pins):
// RPWM -> D5 (PWM)
// LPWM -> D6 (PWM)
// R_EN -> D7
// L_EN -> D8
//
// Notes:
// - For a BTS7960 board, EN pins HIGH usually enables both half-bridges.
// - Keep PWM low (20–40) for first tests.
// - If you later connect the motor: start with PWM 20–40 and short bursts.

const int RPWM_PIN = 5;   // RPWM (PWM pin)
const int LPWM_PIN = 6;   // LPWM (PWM pin)
const int R_EN_PIN = 7;   // R_EN
const int L_EN_PIN = 8;   // L_EN

const int PWM_LOW = 30;         // 20–40 recommended for first tests
const unsigned long ON_MS = 1000;
const unsigned long OFF_MS = 800;

bool estop = false;

void allStop() {
  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, 0);
}

void printState(const char* label, int ren, int len, int rpwm, int lpwm) {
  Serial.print(label);
  Serial.print(" | R_EN=");
  Serial.print(ren);
  Serial.print(" L_EN=");
  Serial.print(len);
  Serial.print(" RPWM=");
  Serial.print(rpwm);
  Serial.print(" LPWM=");
  Serial.println(lpwm);
}

// --- E-STOP helpers (added) ---
void disableDriver() {
  allStop();
  digitalWrite(R_EN_PIN, LOW);
  digitalWrite(L_EN_PIN, LOW);
}

void enableDriver() {
  digitalWrite(R_EN_PIN, HIGH);
  digitalWrite(L_EN_PIN, HIGH);
}

void checkSerialEstop() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'x' || c == 'X') {
      estop = true;
      disableDriver();
      Serial.println("!!! E-STOP ACTIVATED: EN=LOW, PWM=0 !!!");
    } else if (c == 'r' || c == 'R') {
      estop = false;
      enableDriver();
      Serial.println("E-STOP released: EN=HIGH, resuming.");
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  pinMode(R_EN_PIN, OUTPUT);
  pinMode(L_EN_PIN, OUTPUT);

  // Enable both sides
  digitalWrite(R_EN_PIN, HIGH);
  digitalWrite(L_EN_PIN, HIGH);

  allStop();

  Serial.println("BTS7960 pin-test started.");
  Serial.println("Alternating RPWM/LPWM at low PWM. EN pins kept HIGH.");
  Serial.println("E-STOP: send 'x' to stop, 'r' to resume.");
}

void loop() {
  checkSerialEstop();
  if (estop) {
    // Stay safe in E-STOP state
    delay(50);
    return;
  }

  // Phase A: "Right" direction (RPWM on, LPWM off)
  digitalWrite(R_EN_PIN, HIGH);
  digitalWrite(L_EN_PIN, HIGH);
  analogWrite(RPWM_PIN, PWM_LOW);
  analogWrite(LPWM_PIN, 0);
  printState("PHASE A (RPWM ON)", digitalRead(R_EN_PIN), digitalRead(L_EN_PIN), PWM_LOW, 0);

  // Keep checking serial during the wait (so E-STOP is responsive)
  unsigned long t0 = millis();
  while (millis() - t0 < ON_MS) {
    checkSerialEstop();
    if (estop) return;
    delay(5);
  }

  // Stop
  allStop();
  printState("STOP", digitalRead(R_EN_PIN), digitalRead(L_EN_PIN), 0, 0);

  t0 = millis();
  while (millis() - t0 < OFF_MS) {
    checkSerialEstop();
    if (estop) return;
    delay(5);
  }

  // Phase B: "Left" direction (LPWM on, RPWM off)
  digitalWrite(R_EN_PIN, HIGH);
  digitalWrite(L_EN_PIN, HIGH);
  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, PWM_LOW);
  printState("PHASE B (LPWM ON)", digitalRead(R_EN_PIN), digitalRead(L_EN_PIN), 0, PWM_LOW);

  t0 = millis();
  while (millis() - t0 < ON_MS) {
    checkSerialEstop();
    if (estop) return;
    delay(5);
  }

  // Stop
  allStop();
  printState("STOP", digitalRead(R_EN_PIN), digitalRead(L_EN_PIN), 0, 0);

  t0 = millis();
  while (millis() - t0 < OFF_MS) {
    checkSerialEstop();
    if (estop) return;
    delay(5);
  }
}
