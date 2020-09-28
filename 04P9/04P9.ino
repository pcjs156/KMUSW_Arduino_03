#define PIN_LED 13
unsigned int count, toggle;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  Serial.println("Hello World!");
  count = 0;
  toggle = LOW; // 아두이노 내부적으로 HIGH: 1 / LOW: 0으로 지정됨
  digitalWrite(PIN_LED, toggle);
}

void loop() {
  Serial.println(++count);
  toggle = toggle_state(toggle); // 상태 변경
  digitalWrite(PIN_LED, toggle); // LEF 켜기/끄기
  delay(1000);
}

int toggle_state(int toggle) {
  // 꺼져 있으면 켜고, 켜져 있으면 끈다.
  return (toggle == LOW) ? HIGH : LOW;
}
