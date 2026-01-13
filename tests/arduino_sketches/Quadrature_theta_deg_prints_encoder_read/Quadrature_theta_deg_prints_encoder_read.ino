volatile long encoderPos = 0;

void ISR_A() {
  if (digitalRead(2) == digitalRead(3))
    encoderPos++;
  else
    encoderPos--;
}

void ISR_B() {
  if (digitalRead(2) != digitalRead(3))
    encoderPos++;
  else
    encoderPos--;
}

void setup() {
  Serial.begin(115200);

  pinMode(2, INPUT_PULLUP); // A
  pinMode(3, INPUT_PULLUP); // B

  attachInterrupt(digitalPinToInterrupt(2), ISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), ISR_B, CHANGE);
}

void loop() {
  long count;

  noInterrupts();          // 🔒 atomic copy
  count = encoderPos;
  interrupts();            // 🔓

  double theta_rad = count * (2.0 * PI / 1440.0);
  double theta_deg = theta_rad * 180.0 / PI;

  Serial.print("count: ");
  Serial.print(count);
  Serial.print(" | rad: ");
  Serial.print(theta_rad, 6);
  Serial.print(" | deg: ");
  Serial.println(theta_deg, 2);

  delay(50);
}
