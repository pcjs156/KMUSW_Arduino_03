#define PIN_LED 13

void setup() {
  pinMode(PIN_LED, OUTPUT);
}

void loop() {
  // digitalWrite(PIN_LED, 0);
  digitalWrite(PIN_LED, 1);
}
