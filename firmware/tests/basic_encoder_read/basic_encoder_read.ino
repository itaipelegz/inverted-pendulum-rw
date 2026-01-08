volatile long encoderPos = 0;

void ISR_A() {
  if (digitalRead(3) == HIGH)
    encoderPos++;
  else
    encoderPos--;
}

void setup() {
  Serial.begin(9600);

  pinMode(2, INPUT_PULLUP); // A
  pinMode(3, INPUT_PULLUP); // B

  attachInterrupt(digitalPinToInterrupt(2), ISR_A, RISING);
}

void loop() {
  Serial.println(encoderPos);
  delay(200);
}
