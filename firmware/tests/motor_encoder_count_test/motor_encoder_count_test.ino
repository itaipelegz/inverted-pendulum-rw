// ===== Encoder "1 Revolution" Test (UNO) =====
// A -> D2, B -> D3
// Commands in Serial Monitor:
//   z = zero/reset count
//   p = print current count

const int ENC_A_PIN = 2;
const int ENC_B_PIN = 3;

volatile long encoderCount = 0;

void encoderA_ISR() {
  bool A = digitalRead(ENC_A_PIN);
  bool B = digitalRead(ENC_B_PIN);
  if (A == B) encoderCount++;
  else encoderCount--;
}

void encoderB_ISR() {
  bool A = digitalRead(ENC_A_PIN);
  bool B = digitalRead(ENC_B_PIN);
  if (A != B) encoderCount++;
  else encoderCount--;
}

void setup() {
  Serial.begin(115200);
  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), encoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_PIN), encoderB_ISR, CHANGE);

  Serial.println("1-Revolution test ready.");
  Serial.println("Send 'z' to zero, rotate exactly 1 turn, send 'p' to print counts.");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();

    if (c == 'z' || c == 'Z') {
      noInterrupts();
      encoderCount = 0;
      interrupts();
      Serial.println("Count reset to 0.");
    }

    if (c == 'p' || c == 'P') {
      noInterrupts();
      long cSnapshot = encoderCount;
      interrupts();
      Serial.print("Counts = ");
      Serial.println(cSnapshot);
    }
  }
}
