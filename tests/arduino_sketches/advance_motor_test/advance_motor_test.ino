// ===== BTS7960 Test: Ramp + Serial control =====

const int RPWM_PIN = 5;   // RPWM  (~)
const int LPWM_PIN = 6;   // LPWM  (~)
const int R_EN_PIN = 7;   // R_EN
const int L_EN_PIN = 8;   // L_EN

int currentPWM = 0;       // 0..255
int targetPWM  = 0;       // 0..255
int direction  = 0;       // +1 = right(RPWM), -1 = left(LPWM), 0 = stop
bool enabled   = true;

const int RAMP_STEP = 5;          // PWM step per tick
const int RAMP_DELAY_MS = 30;     // time between steps

void applyOutput(int dir, int pwm) {
  pwm = constrain(pwm, 0, 255);
  if (!enabled || dir == 0 || pwm == 0) {
    analogWrite(RPWM_PIN, 0);
    analogWrite(LPWM_PIN, 0);
    return;
  }

  if (dir > 0) { // right
    analogWrite(RPWM_PIN, pwm);
    analogWrite(LPWM_PIN, 0);
  } else {       // left
    analogWrite(RPWM_PIN, 0);
    analogWrite(LPWM_PIN, pwm);
  }
}

void setEnable(bool en) {
  enabled = en;
  digitalWrite(R_EN_PIN, enabled ? HIGH : LOW);
  digitalWrite(L_EN_PIN, enabled ? HIGH : LOW);
  if (!enabled) {
    currentPWM = 0;
    targetPWM = 0;
    direction = 0;
    applyOutput(0, 0);
  }
}

void stopMotor() {
  direction = 0;
  targetPWM = 0;
}

void rampTick() {
  if (currentPWM == targetPWM) {
    applyOutput(direction, currentPWM);
    return;
  }

  if (currentPWM < targetPWM) currentPWM += RAMP_STEP;
  else currentPWM -= RAMP_STEP;

  // avoid overshoot
  if ((RAMP_STEP > 0) && abs(currentPWM - targetPWM) < RAMP_STEP) currentPWM = targetPWM;

  applyOutput(direction, currentPWM);
  delay(RAMP_DELAY_MS);
}

void printHelp() {
  Serial.println("Commands:");
  Serial.println("  r <0-255>  : rotate RIGHT (RPWM) to PWM value");
  Serial.println("  l <0-255>  : rotate LEFT  (LPWM) to PWM value");
  Serial.println("  s          : stop");
  Serial.println("  e          : enable driver");
  Serial.println("  d          : disable driver");
  Serial.println("Example: r 60");
}

void setup() {
  Serial.begin(115200);

  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  pinMode(R_EN_PIN, OUTPUT);
  pinMode(L_EN_PIN, OUTPUT);

  setEnable(true);
  applyOutput(0, 0);

  Serial.println("BTS7960 ramp+serial test ready.");
  printHelp();
}

void loop() {
  // Handle serial input
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() == 0) return;

    char c = cmd.charAt(0);

    if (c == 's') {
      stopMotor();
      Serial.println("Stop");
    } else if (c == 'e') {
      setEnable(true);
      Serial.println("Enabled");
    } else if (c == 'd') {
      setEnable(false);
      Serial.println("Disabled");
    } else if (c == 'r' || c == 'l') {
      int spaceIdx = cmd.indexOf(' ');
      if (spaceIdx < 0) {
        Serial.println("Missing value. Example: r 60");
      } else {
        int val = cmd.substring(spaceIdx + 1).toInt();
        val = constrain(val, 0, 255);

        direction = (c == 'r') ? +1 : -1;
        targetPWM = val;

        Serial.print(c == 'r' ? "RIGHT PWM target=" : "LEFT PWM target=");
        Serial.println(targetPWM);
      }
    } else {
      Serial.println("Unknown command.");
      printHelp();
    }
  }

  // Smoothly move toward target
  rampTick();
}
