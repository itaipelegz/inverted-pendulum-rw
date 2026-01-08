// ===== Incremental Encoder Test (Arduino UNO) =====
// Prints:
//  - count (total pulses)
//  - direction (CW / CCW)
//  - pulses per second (speed)
//
// Encoder wiring:
//  A -> D2 (INT0)
//  B -> D3 (INT1)
//  Vcc -> 5V (if your encoder supports 5V)
//  GND -> GND

const int ENC_A_PIN = 2;
const int ENC_B_PIN = 3;

volatile long encoderCount = 0;
volatile int lastDirection = 0; // +1 = CW, -1 = CCW

unsigned long lastPrintTime = 0;
long lastCountSnapshot = 0;

const unsigned long PRINT_INTERVAL_MS = 500; // print twice per second

void encoderA_ISR() {
  bool A = digitalRead(ENC_A_PIN);
  bool B = digitalRead(ENC_B_PIN);

  if (A == B) {
    encoderCount++;
    lastDirection = +1;
  } else {
    encoderCount--;
    lastDirection = -1;
  }
}

void encoderB_ISR() {
  bool A = digitalRead(ENC_A_PIN);
  bool B = digitalRead(ENC_B_PIN);

  if (A != B) {
    encoderCount++;
    lastDirection = +1;
  } else {
    encoderCount--;
    lastDirection = -1;
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), encoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_PIN), encoderB_ISR, CHANGE);

  Serial.println("Encoder test started (UNO)");
  Serial.println("count | direction | pulses/sec");
}

void loop() {
  unsigned long now = millis();

  if (now - lastPrintTime >= PRINT_INTERVAL_MS) {
    noInterrupts();
    long countSnapshot = encoderCount;
    int dirSnapshot = lastDirection;
    interrupts();

    unsigned long dt = now - lastPrintTime;
    long delta = countSnapshot - lastCountSnapshot;
    float pulsesPerSecond = (dt > 0) ? (delta * 1000.0f) / dt : 0.0f;

    Serial.print(countSnapshot);
    Serial.print(" | ");

    if (dirSnapshot > 0) Serial.print("CW");
    else if (dirSnapshot < 0) Serial.print("CCW");
    else Serial.print("-");

    Serial.print(" | ");
    Serial.println(pulsesPerSecond);

    lastCountSnapshot = countSnapshot;
    lastPrintTime = now;
  }
}
