const int buttonPin = 2;
int lastState = HIGH;

void setup() {
  pinMode(buttonPin, INPUT_PULLUP);
  Serial.begin(115200);
}

void loop() {
  bool state = digitalRead(buttonPin);

  if (lastState != state) {
    if (state == LOW) {
      Serial.println("PRESSED");
    } else {
      Serial.println("RELEASED");
    };
    delay(10);
  }

  lastState = state;
}
