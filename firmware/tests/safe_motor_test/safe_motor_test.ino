// ===== BTS7960 Safe Motor Test =====
// Pin mapping based on your driver labeling

const int RPWM_PIN = 5;   // RPWM
const int LPWM_PIN = 6;   // LPWM
const int R_EN_PIN = 7;   // R_EN
const int L_EN_PIN = 8;   // L_EN

const int PWM_TEST = 30;  // Start LOW (20–40 is safe)

void stopMotor() {
  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, 0);
}

void setup() {
  Serial.begin(115200);

  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  pinMode(R_EN_PIN, OUTPUT);
  pinMode(L_EN_PIN, OUTPUT);

  // Enable both directions
  digitalWrite(R_EN_PIN, HIGH);
  digitalWrite(L_EN_PIN, HIGH);

  stopMotor();

  Serial.println("BTS7960 motor test started");
  Serial.println("PWM is LOW for safety. Motor must be free.");
  delay(1000);
}

void loop() {
  // Rotate one direction
  Serial.println("Rotate RIGHT");
  analogWrite(RPWM_PIN, PWM_TEST);
  analogWrite(LPWM_PIN, 0);
  delay(2000);

  // Stop
  Serial.println("Stop");
  stopMotor();
  delay(2000);

  // Rotate other direction
  Serial.println("Rotate LEFT");
  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, PWM_TEST);
  delay(2000);

  // Stop
  Serial.println("Stop");
  stopMotor();
  delay(2000);
}
