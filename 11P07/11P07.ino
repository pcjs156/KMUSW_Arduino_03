#include <Servo.h>

// Arduino pin assignment
#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_TRIG 12
#define PIN_ECHO 13

// configurable parameters
#define SND_VEL 346.0 // sound velocity at 24 celsius degree (unit: m/s)
#define INTERVAL 25 // sampling interval (unit: ms)
#define _DIST_MIN 180 // minimum distance to be measured (unit: mm)
#define _DIST_MAX 360 // maximum distance to be measured (unit: mm)

#define _DUTY_MIN 553 // servo full clockwise position (0 degree)
#define _DUTY_NEU 1476 // servo neutral position (90 degree)
#define _DUTY_MAX 2399 // servo full counterclockwise position (180 degree)

#define ALPHA 0.5

// global variables
float timeout; // unit: us
float dist_min, dist_max, dist_raw, dist_prev; // unit: mm
unsigned long last_sampling_time; // unit: ms
float scale; // used for pulse duration to distance conversion
Servo myservo;

float dist_ema;

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  pinMode(PIN_TRIG,OUTPUT);
  digitalWrite(PIN_TRIG, LOW); 
  pinMode(PIN_ECHO,INPUT);

  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_NEU);

// initialize USS related variables
  dist_min = _DIST_MIN; 
  dist_max = _DIST_MAX;
  timeout = (INTERVAL / 2) * 1000.0; // precalculate pulseIn() timeout value. (unit: us)
  dist_raw = dist_prev = dist_ema = 0.0; // raw distance output from USS (unit: mm)
  scale = 0.001 * 0.5 * SND_VEL;

// initialize serial port
  Serial.begin(57600);

// initialize last sampling time
  last_sampling_time = 0;
  }

void loop() {
// wait until next sampling time. 
// millis() returns the number of milliseconds since the program started. Will overflow after 50 days.
  if(millis() < last_sampling_time + INTERVAL) return;

// get a distance reading from the USS
  dist_raw = USS_measure(PIN_TRIG,PIN_ECHO);
  dist_ema = (ALPHA * dist_raw) + ((1-ALPHA)*dist_ema);

// output the read value to the serial port
  Serial.print("Min:100,raw:");
  Serial.print(dist_raw);
  Serial.print(",ema:");
  Serial.print(dist_ema);
  Serial.print(",servo:");
  Serial.print(myservo.read());
  Serial.println(",Max:400");

// adjust servo position according to the USS read value

  // add your code here!

  // 측정 범위 미만
  if(dist_ema <= _DIST_MIN) {
     myservo.writeMicroseconds(_DUTY_MIN);
     digitalWrite(PIN_LED, 255);
  }
  // 측정 범위 안
  // ema가 raw보다 커질 수 없으므로 이를 보정
  else if(dist_ema < _DIST_MAX){
     myservo.writeMicroseconds(get_servo_degree(dist_ema+2));
     digitalWrite(PIN_LED, 0);
  }
  // 측정 범위 초과
  else {
    myservo.writeMicroseconds(_DUTY_MAX);
    digitalWrite(PIN_LED, 255);
  }
   
// update last sampling time
  last_sampling_time += INTERVAL;
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO)
{
  float reading;
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  reading = pulseIn(ECHO, HIGH, timeout) * scale; // unit: mm
//  if(reading < dist_min || reading > dist_max)
//    reading = 0.0; // return 0 when out of range.
//
  if(reading == 0.0) {
    reading = dist_prev;
  }
  else {
    if (reading < dist_min) {
      reading = _DIST_MIN;
      dist_prev = reading;
    }
    else if (reading > dist_max) {
      reading = _DIST_MAX;
      dist_prev = reading;
    }
  }

  return reading;
}

float get_servo_degree(float distance) {
  return (_DUTY_MAX - _DUTY_MIN) / (_DIST_MAX - _DIST_MIN) * (distance - _DIST_MIN) + _DUTY_MIN;
}
