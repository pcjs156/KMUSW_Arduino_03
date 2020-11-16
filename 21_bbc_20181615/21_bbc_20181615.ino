#include <Servo.h>

// Arduino pin assignment
#define PIN_IR A0
#define PIN_SERVO 10

// Configurable parameters
#define INTERVAL 25    // 샘플링 주기
#define _DIST_MIN 70   // 측정 가능한 최소 거리 (unit: mm)
#define _DIST_MAX 450  // 측정 가능한 최대 거리 (unit: mm)
#define _DUTY_MIN 1065 // 최고 각도
#define _DUTY_NEU 1520 // 수평
#define _DUTY_MAX 2100 // 최저 각도

// Global variables
float dist_min, dist_max, dist_raw; // unit: mm
unsigned long last_sampling_time;   // unit: ms
float last_reading;                 // 마지막 reading 값

// EMA Filter
#define ALPHA 0.3
float filtered_distance;

Servo myservo;

void setup() {
  // GPIO 핀 초기화
  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(_DUTY_NEU);

  // 거리 측정 관련 변수 초기화
  dist_min = _DIST_MIN;
  dist_max = _DIST_MAX;
  dist_raw = 0.0;                    // USS로부터 읽어온 처리 전의 거리 (unit: mm)
  filtered_distance = 0.0;

  Serial.begin(115200);

  last_sampling_time = 0;
  last_reading = 0;
}

void loop() {
  if (millis() < last_sampling_time + INTERVAL)
    return;

  dist_raw = ir_distance();

  // 정상입력: 가장 최근의 거리값을 현재 거리값으로 갱신
  if (dist_min <= dist_raw && dist_raw <= dist_max)
    last_reading = dist_raw;
  // 비정상입력: 현재 거리값을 가장 최근의 거리값으로 대체
  else
    dist_raw = last_reading;

  filtered_distance = (ALPHA * dist_raw) + ((1-ALPHA) * filtered_distance);

  Serial.print("Min:0,");
  Serial.print("raw:");
  Serial.print(dist_raw);
  Serial.print(", ");
  Serial.print("filtereed:");
  Serial.print(filtered_distance);
  Serial.print(", ");
  Serial.println("Max:400");

  if (filtered_distance < 255)
    myservo.writeMicroseconds(_DUTY_MAX);
  else
    myservo.writeMicroseconds(_DUTY_MIN);

  last_sampling_time += INTERVAL;
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-1.5) * 10.0;
  return val;
}
