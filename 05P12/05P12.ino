#define PIN_LED 7

unsigned int toggle;
unsigned int turn_on_cnt = 0;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200);
  while(!Serial) {
    ;
  }
  
  
}

void loop() {
  // 1초간 켬
  digitalWrite(PIN_LED, 0);
  delay(1000);

  // 꺼진 상태로 
  toggle = 1;

  // 5번 깜빡임
  while (turn_on_cnt < 5) {
    digitalWrite(PIN_LED, toggle);
    delay(100);

    if (toggle == 0)
      turn_on_cnt++;

    toggle = toggle_state(toggle);
  }
  
  // 동작을 완료하면 LED를 끔
  digitalWrite(PIN_LED, 1);

  // 무한 루프로 loop()를 정지시킴
  while (1) {
    ;
  }
}

int toggle_state(int toggle) {
  return (toggle == 1) ? 0 : 1;
}
