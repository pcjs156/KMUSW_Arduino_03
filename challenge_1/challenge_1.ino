/*
 주기성을 재연하는데까진 성공했지만, 값이 바뀌면 이마저도 안됨 -> 단위환산 다시 해야 함
 10000 ->   100 | 500
 1000  ->  1000 | 5000
 100   -> 10000 | 50000
*/

#define LED_PIN 7

unsigned long cur_p; // 단위 : ms
unsigned long p;
unsigned long d;

float turn_on_ratio;

void setup()
{
  Serial.begin(9600);
  
  pinMode(LED_PIN, OUTPUT);

  p = us_to_ms(10000);
  d = 100;
}

unsigned long us_to_ms(int us) {
  return 0.001 * us;
}

void loop()
{
  cur_p = 0;
  while (cur_p <= 500) {
    turn_on_ratio = ((float)d/(float)500) * (float)cur_p / 100;
    
    digitalWrite(LED_PIN, HIGH);
    delayMicroseconds((float)p * turn_on_ratio);
    
    digitalWrite(LED_PIN, LOW);
    delayMicroseconds((float)p * (1 - turn_on_ratio));
    
    cur_p += p;

    Serial.print("Ascending... ");
    Serial.print((float)p * turn_on_ratio);
    Serial.print(" ");
    Serial.println((float)p * (1 - turn_on_ratio));
  }

  Serial.println("HIGH-------------------------");

  cur_p = 0;
  while (cur_p <= 500) {
    turn_on_ratio = ((float)d/(float)500) * (float)cur_p / 100;

    digitalWrite(LED_PIN, HIGH);
    delayMicroseconds((float)p * turn_on_ratio);

    digitalWrite(LED_PIN, LOW);
    delayMicroseconds((float)p * (1 - turn_on_ratio));

    cur_p += p;
  
    Serial.print("Descending... ");
    Serial.print((float)p * turn_on_ratio);
    Serial.print(" ");
    Serial.println((float)p * (1 - turn_on_ratio));
  }

  Serial.println("LOW--------------------------");
}
