const int buttonPin = 2;

void setup() {
  pinMode(buttonPin, INPUT_PULLUP);
  Serial.begin(115200);
}

void loop() {
  bool state = digitalRead(buttonPin);
  Serial.println(state == LOW ? "PRESSED" : "RELEASED");
  delay(10);
}
