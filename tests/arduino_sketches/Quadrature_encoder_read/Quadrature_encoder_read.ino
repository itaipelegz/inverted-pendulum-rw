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
  Serial.println(encoderPos);
  delay(50);
}
