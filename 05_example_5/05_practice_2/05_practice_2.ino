#define PIN_LED 7

unsigned int toggle;
unsigned int turn_on_cnt = 0;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200);
  while(!Serial) {
    ;
  }
  
  // 1초간 켬
  digitalWrite(PIN_LED, 0);
  delay(1000);

  toggle = 1;
}

void loop() {
  while (turn_on_cnt <= 5) {
    delay(100);

    digitalWrite(PIN_LED, toggle);

    if (toggle == 0)
      turn_on_cnt++;

    toggle = toggle_state(toggle);
  }

  digitalWrite(PIN_LED, 1);
}

int toggle_state(int toggle) {
  return (toggle == 1) ? 0 : 1;
}
