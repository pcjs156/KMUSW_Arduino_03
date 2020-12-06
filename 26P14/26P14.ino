#include <Servo.h>

/////////////////////////////
// Configurable parameters //
/////////////////////////////
// Arduino pin assignment
#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_IR A0

// Framework setting
#define _DIST_TARGET 255 // 중간값
#define _DIST_MIN 100 // 측정 최솟값
#define _DIST_MAX 410 // 측정 최댓값

// Distance sensor
#define EMA_ALPHA 0.4

// ir filter by 박우혁
#define _INTERVAL_DIST 30  // DELAY_MICROS * samples_num^2 의 값이 최종 거리측정 인터벌임. 넉넉하게 30ms 잡음.
#define DELAY_MICROS  1500 // 필터에 넣을 샘플값을 측정하는 딜레이(고정값!)
float ema_dist = 0;          // EMA 필터에 사용할 변수
float filtered_dist;       // 최종 측정된 거리값을 넣을 변수. loop()안에 filtered_dist = filtered_ir_distance(); 형태로 사용하면 됨.
float samples_num = 3;     // 스파이크 제거를 위한 부분필터에 샘플을 몇개 측정할 것인지. 3개로 충분함! 가능하면 수정하지 말 것.

// Servo range
#define _DUTY_MIN 1050   // 서보 제어 펄스 폭: 최고 각도 / 212mm
#define _DUTY_NEU 1550   // 서보 제어 펄스 폭: 수평 / 188mm
#define _DUTY_MAX 2090  // 서보 제어 펄스 폭: 최저 각도 / 170mm

// Servo speed control
#define _SERVO_ANGLE 30
#define _SERVO_SPEED 250

// Event periods
#define _INTERVAL_DIST 30    // 거리측정주기 (ms)
#define _INTERVAL_SERVO 30   // 서보제어주기 (ms)
#define _INTERVAL_SERIAL 100 // Serial제어주기 (ms)

// PID parameters
// control을 duty로 완벽히 매칭시키지 못해 정지 지점이 gain에 따라 달라지는 문제를 상수로 해결
#define CONTROL_CONST -40
#define _UP_KP 0.9
#define _DOWN_KP 0.9
#define _UP_KD 30
#define _DOWN_KD 30

#define _K_UP 1.8
#define _K_DOWN 1.8

// filter by 추헌준
const float coE[] = { -0.0000253, 0.0152458, -1.4843685, 139.5061287};

//////////////////////
// global variables //
//////////////////////
// Servo instance
Servo myservo;

// Distance sensor
float dist_target;         // 공을 보낼 위치
float dist_raw, dist_ema;  // 센서가 인식한 거리 / ema 필터링된 거리
float last_sampling_dist;  // 마지막으로 정상 측정된 거리

// Event periods
// 마지막으로 샘플링/서보제어/시리얼출력한 시각
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial;
// 샘플링/서보제어/시리얼출력 제어 플래그
bool event_dist, event_servo, event_serial;

// Servo speed control
int duty_chg_per_interval; // 서보 업데이트 1주기에 증감 가능한 duty 의 최댓값
int duty_target, duty_curr;

// PID variables
float error_curr;
float error_prev, control, pterm, dterm, iterm;


void setup() {
  // initialize GPIO pins for LED and attach servo
  pinMode(PIN_LED, OUTPUT);  // PIN_LED 핀을 OUTPUT으로 설정
  myservo.attach(PIN_SERVO); // PIN_SERVO 핀을 servo를 쓰는 핀으로 설정

  // initialize global variables
  last_sampling_dist = 0;
  dist_raw = 0;
  dist_ema = 0;
  dist_target = _DIST_TARGET;
  control = _DUTY_NEU;
  duty_curr = _DUTY_NEU;

  // move servo to neutral position
  myservo.writeMicroseconds(_DUTY_NEU);
  // 서보 업데이트 1주기에 증감 가능한 duty 의 최댓값
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / 180.0) * (_INTERVAL_SERVO / 1000.0);

  // 마지막 이벤트 발생 시각 초기화
  last_sampling_time_dist = 0;
  last_sampling_time_servo = 0;
  last_sampling_time_serial = 0;

  // 이벤트 제어 플래그 비활성화
  event_dist = false;
  event_servo = false;
  event_serial = false;

  // initialize serial port
  Serial.begin(115200);
}

void loop() {
  /////////////////////
  // Event generator //
  /////////////////////
  // 거리 측정 주기가 되었는지 검사
  if (millis() >= last_sampling_time_dist + _INTERVAL_DIST)
    event_dist = true;

  // 서보 제어 주기가 되었는지 검사
  if (millis() >= last_sampling_time_servo + _INTERVAL_SERVO)
    event_servo = true;

  // Serial 제어 주기가 되었는지 검사
  if (millis() >= last_sampling_time_serial + _INTERVAL_SERIAL)
    event_serial = true;

  ////////////////////
  // Event handlers //
  ////////////////////

  if (event_dist) {
    event_dist = false;

    // 필터링된 측정 거리
    dist_ema = filtered_ir_distance();

    // 오차 = 목표까지의 거리 - 탁구공까지의 거리
    error_curr = dist_target - dist_ema;

    // control이 음수: 목표까지의 거리 < 탁구공까지의 거리
    // 각도를 올려야 함 -> duty_target을 낮춰야 함
    if (control < 0) {
      // p항(proportional)
      pterm = _UP_KP * error_curr;

      // d항(derivative)
      dterm = _UP_KD * (error_curr - error_prev);

      // 제어량: P + I + D 예정
      control = _K_UP * (pterm + dterm);

      duty_target = _DUTY_NEU + control + CONTROL_CONST;

      // 이게 원본
//      duty_target = map(control, _DIST_TARGET - _DIST_MAX, 0, _DUTY_MIN, _DUTY_NEU);
      
//      duty_target = map(control, (_UP_KP + _UP_KD) * (_DIST_TARGET - _DIST_MAX), 0, _DUTY_MIN, _DUTY_NEU);
    }

    // control이 양수: 목표까지의 거리 > 탁구공까지의 거리
    // 각도를 내려야 함 -> duty_target을 높여야 함
    else {
      // p항(proportional)
      pterm = _DOWN_KP * error_curr;

      // d항(derivative)
      dterm = _DOWN_KD * (error_curr - error_prev);

      // 제어량: P + I + D 예정
      control = _K_DOWN * (pterm + dterm);
      
      duty_target = _DUTY_NEU + control + CONTROL_CONST;

      // 이게 원본
//      duty_target = map(control, 0, _DIST_TARGET - _DIST_MIN, _DUTY_NEU, _DUTY_MAX);

//      duty_target = map(control, 0, (_DOWN_KP + _DOWN_KD) * (_DIST_TARGET - _DIST_MIN), _DUTY_NEU, _DUTY_MAX);
    }

    // control을 duty로 완벽히 매칭시키지 못해 정지 지점이 gain에 따라 달라지는 문제를 상수로 해결
    duty_target = max(min(duty_target, _DUTY_MAX), _DUTY_MIN);

    error_prev = error_curr;

    // 마지막 샘플링 시각 업데이트
    last_sampling_time_dist += _INTERVAL_DIST;
  }

  if (event_servo) {
    event_servo = false;

    // 움직여야 하는 duty가 최대 이동 가능 duty보다 큰 경우
    if (abs(duty_target - duty_curr) > duty_chg_per_interval) {
      // 목표 duty_target이 duty_curr보다 작은 경우 duty_curr을 duty_chg_per_interval만큼 낮춰줌
      // (현재 duty값에서 내려줘야 하므로)
      if (duty_target < duty_curr) {
        duty_curr -= duty_chg_per_interval;
        if (duty_curr < duty_target) duty_curr = duty_target;
      }

      // 목표 duty_target이 duty_curr보다 큰 경우 duty_curr을 duty_chg_per_interval만큼 높여줌
      // (현재 duty값에서 올려줘야 하므로)
      else {
        duty_curr += duty_chg_per_interval;
        if (duty_curr > duty_target) duty_curr = duty_target;
      }
    }
    // 움직여야 하는 duty가 최대 이동 가능 duty보단 작은 경우
    else {
      // 그냥 그대로 갱신해줌
      duty_curr = duty_target;
    }

    myservo.writeMicroseconds(duty_curr);

    // 마지막 서보 제어 시각 업데이트
    last_sampling_time_servo += _INTERVAL_SERVO;
  }

  if (event_serial) {
    event_serial = false;

    Serial.print("dist_ir:");
  Serial.print(dist_raw);
  Serial.print(",pterm:");
  Serial.print(map(pterm,-1000,1000,510,610));
  Serial.print(",dterm:");
  Serial.print(map(dterm,-1000,1000,510,610));
  Serial.print(",duty_target:");
  Serial.print(map(duty_target,1000,2000,410,510));
  Serial.print(",duty_curr:");
  Serial.print(map(duty_curr,1000,2000,410,510));
  Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");

//    Serial.print("dist_ir:");
//    Serial.print(dist_raw);
//    Serial.print(",pterm:");
//    Serial.print(map(pterm, -1000, 1000, 510, 610));
//    Serial.print(",dterm:");
//    Serial.print(map(dterm, -1000, 1000, 510, 610));
//    Serial.print(",duty_target:");
//    Serial.print(duty_target);
//    Serial.print(",duty_curr:");
//    Serial.print(duty_curr);
//    Serial.print(",control:");
//    Serial.print(control);
//    Serial.print(",error:");
//    Serial.print(error_curr);
//    Serial.println();
    
    // 마지막 Serial 출력 시각 업데이트
    last_sampling_time_serial += _INTERVAL_SERIAL;
  }
}

float ir_distance(void) { // return value unit: mm
  float val, raw;
  float volt = float(analogRead(PIN_IR));
  raw = ((6762.0 / (volt - 9.0)) - 4.0) * 10.0; // 변환 식 보정 필요
  dist_raw = coE[0] * pow(raw, 3) + coE[1] * pow(raw, 2) + coE[2] * raw + coE[3];

  if (dist_raw >= _DIST_MIN && dist_raw <= _DIST_MAX) {
    last_sampling_dist = raw;
  }
  else {
    dist_raw = last_sampling_dist;
  }

  return dist_raw;
}

// ir filter by 박우혁
float under_noise_filter(void) { // 아래로 떨어지는 형태의 스파이크를 제거해주는 필터
  int currReading;
  int largestReading = 0;
  for (int i = 0; i < samples_num; i++) {
    currReading = ir_distance();
    if (currReading > largestReading) {
      largestReading = currReading;
    }
    // Delay a short time before taking another reading
    delayMicroseconds(DELAY_MICROS);
  }
  return largestReading;
}

// ir filter by 박우혁
float filtered_ir_distance(void) { // 아래로 떨어지는 형태의 스파이크를 제거 후, 위로 치솟는 스파이크를 제거하고 EMA필터를 적용함.
  // under_noise_filter를 통과한 값을 upper_nosie_filter에 넣어 최종 값이 나옴.
  int currReading;
  int lowestReading = 1024;
  for (int i = 0; i < samples_num; i++) {
    currReading = under_noise_filter();
    if (currReading < lowestReading) {
      lowestReading = currReading;
    }
  }
  
  // eam 필터 추가
  ema_dist = EMA_ALPHA * lowestReading + (1 - EMA_ALPHA) * ema_dist;
  // 시리얼 모니터에 보정된 값 갱신
  dist_raw = ema_dist;
  return ema_dist;
}
