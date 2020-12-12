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

// ir filter by 김태완
#define LENGTH 30
#define k_LENGTH 8
#define Horizontal_Angle 2160
#define Max_Variable_Angle 100

int a, b; // unit: mm
int correction_dist, iter;
float dist_list[LENGTH], sum, dist_ema, alpha;

// Servo range
#define _DUTY_MIN 1050   // 서보 제어 펄스 폭: 최고 각도 / 212mm
#define _DUTY_NEU 1500   // 서보 제어 펄스 폭: 수평 / 188mm
#define _DUTY_MAX 2090  // 서보 제어 펄스 폭: 최저 각도 / 170mm

// Servo speed control
#define _SERVO_ANGLE 30
#define _SERVO_SPEED 300

// Event periods
#define _INTERVAL_DIST 20    // 거리측정주기 (ms)
#define _INTERVAL_SERVO 20   // 서보제어주기 (ms)
#define _INTERVAL_SERIAL 100 // Serial제어주기 (ms)

// PID parameters
// control을 duty로 완벽히 매칭시키지 못해 정지 지점이 gain에 따라 달라지는 문제를 상수로 해결
#define CONTROL_CONST 0
#define _KP 2.0
#define _KI 0.015
#define _KD 140.0
#define _K 1.0

// filter by 추헌준
const float coE[] = {-0.0000185, 0.0106246, -0.5796819, 98.6079364};

//////////////////////
// global variables //
//////////////////////
// Servo instance
Servo myservo;

// Distance sensor
float dist_target;         // 공을 보낼 위치
float dist_raw;  // 센서가 인식한 거리
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

  correction_dist = 0;
  iter = 0; sum = 0;
  alpha = EMA_ALPHA;

  iterm = 0;

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
    dist_ema = ir_distence_filter();

    // 오차 = 목표까지의 거리 - 탁구공까지의 거리
    error_curr = dist_target - dist_ema;

    // p항(proportional)
    pterm = _KP * error_curr;

    // I항(integral)
    iterm += _KI * error_curr;

    // d항(derivative)
    dterm = _KD * (error_curr - error_prev);

    // 제어량: P + I + D 예정
    control = _K * (pterm + dterm + iterm);
    
    duty_target = _DUTY_NEU + control + CONTROL_CONST;

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

    Serial.print("IR:");
    Serial.print(dist_raw);
    Serial.print(",T:");
    Serial.print(dist_target);
    Serial.print(",P:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",D:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",I:");
    Serial.print(map(iterm,-1000,1000,510,610));
    Serial.print(",DTT:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",DTC:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",-G:245,+G:265,m:0,M:800");
    
    // 마지막 Serial 출력 시각 업데이트
    last_sampling_time_serial += _INTERVAL_SERIAL;
  }
}

float ir_distance(void) { // return value unit: mm
  float val, raw;
  float volt = float(analogRead(PIN_IR));
  float result;
  
  raw = ((6762.0 / (volt - 9.0)) - 4.0) * 10.0; // 변환 식 보정 필요
  result = coE[0] * pow(raw, 3) + coE[1] * pow(raw, 2) + coE[2] * raw + coE[3];

  if (result >= _DIST_MIN && result <= _DIST_MAX) {
    last_sampling_dist = raw;
  }
  else {
    result = last_sampling_dist;
  }

  return result;
}

// ir filter by 김태완
float ir_distence_filter() {
  sum = 0;
  iter = 0;
  
  while (iter < LENGTH)
  {
    dist_list[iter] = ir_distance();
    sum += dist_list[iter];
    iter++;
  }

  for (int i = 0; i < LENGTH-1; i++){
    for (int j = i+1; j < LENGTH; j++){
      if (dist_list[i] > dist_list[j]) {
        float tmp = dist_list[i];
        dist_list[i] = dist_list[j];
        dist_list[j] = tmp;
      }
    }
  }
  
  for (int i = 0; i < k_LENGTH; i++) {
    sum -= dist_list[i];
  }
  for (int i = 1; i <= k_LENGTH; i++) {
    sum -= dist_list[LENGTH-i];
  }

  float dist_cali = sum/(LENGTH-2*k_LENGTH);

  dist_raw = alpha*dist_cali + (1-alpha)*dist_ema;
  
  return dist_raw;
}
