// 이하 상수 선언 =============================

// 큐의 크기(N)를 지정
#define QUEUE_SIZE 30

// Arduino의 PIN 번호에 이름을 붙임
#define PIN_LED 9
#define PIN_TRIG 12
#define PIN_ECHO 13

// 측정을 위한 상수
#define SND_VEL 346.0 // 25℃에서의 음속(m/s)
#define INTERVAL 25 // 샘플링 주기(ms)
#define _DIST_MIN 100 // 측정 거리 하한(mm)
#define _DIST_MAX 300  // 측정 거리 상한(mm)

// 이상 상수 선언 =============================
// 이하 구조체 정의 ============================

// 큐의 연결 리스트를 구성하는 각 요소(노드)
typedef struct Node {
  float data; // 측정값을 기록할 변수
  struct Node *next; // 다음 노드를 가리키는 포인터 변수
} Node;

// Singly Linked List로 구현한 큐
// 큐의 각 요소들은 모두 오름차순으로 정렬됨이 보장된다.
typedef struct MedianQueue {
  int MAX_CNT; // 큐에 저장할 수 있는 최대 개수
  int cnt; // 큐에 저장된 현재 개수
  Node *front; // 큐의 맨 앞 노드
  Node *rear; // 큐의 맨 뒤 노드
} MedianQueue;

// 이상 구조체 정의 ============================
// 이하 전역변수 선언 ==========================

float timeout; // unit: us
float dist_min, dist_max, dist_raw; // unit: mm
unsigned long last_sampling_time; // unit: ms
float scale; // used for pulse duration to distance conversion

MedianQueue queue;

float median;

// 이상 전역변수 선언 ===========================
// 이하 함수 정의 ==============================

// 버블 정렬
void bubble_sort(float *arr, int size) {
  float tmp_for_swap;
  
  for(int i=size; i>1; i--) {
    for(int j=1; j<i; j++) {
      if(arr[j-1] > arr[j]) {
        float tmp_for_swap = arr[j-1];
        arr[j-1] = arr[j];
        arr[j] = tmp_for_swap;
      }
    }
  }
}

// 큐 초기화
void init_queue(MedianQueue *queue, int queue_capacity) {
  queue->MAX_CNT = queue_capacity;
  queue->cnt = 0;
  queue->front = NULL;
  queue->rear = NULL;
}

// 큐가 비어 있는지 확인
int is_empty(MedianQueue *queue) {
  return (queue->cnt == 0);
}

// 큐가 가득 찼는지 확인
int is_full(MedianQueue *queue) {
  return (queue->MAX_CNT <= queue->cnt);
}

// 새 데이터를 삽입
void enqueue(MedianQueue *queue, float data) {
  // 삽입할 새 노드 생성
  Node *new_node = (Node*)malloc(sizeof(Node));
  new_node->data = data;
  new_node->next = NULL;

  // 큐가 비어 있는 경우
  if (is_empty(queue)) {
    // 새 노드가 곧 큐의 시작이자 끝이 됨
    queue->front = new_node;    
    queue->rear = new_node; 
  }
  // 큐가 비어 있지 않은 경우
  else {
    // 기존의 끝 노드의 다음 노드를 연결
    queue->rear->next = new_node;
    // 큐의 마지막 노드를 새 노드로 지정
    queue->rear = new_node;
  }

  // 갯수 업데이트
  queue->cnt++;
}

// 맨 앞의 데이터(가장 오래된 데이터)를 버림
// 버린 값은 더 이상 사용하지 않으므로, 반환에 성공하면 1, 실패하면 0을 반환
int dequeue(MedianQueue *queue) {
  // 큐가 비어 있을 경우 0을 반환(반환 실패)
  if (is_empty(queue))
      return 0;

  // 큐가 비어 있지 않은 경우 큐의 맨 앞 요소의 값을 반환
  Node *target_node;

  // 큐의 첫 노드를 선택
  target_node = queue->front;
  // 'deque 이전 시점의 첫 노드의 다음 노드'로 큐의 첫 번째 노드를 갱신
  // 만약 큐가 비어 있었을 경우 첫 노드가 NULL이 됨 
  queue->front = target_node->next;
  // 메모리 해제 : 더 이상 target_node를 가리키는 노드가 없으므로
  free(target_node);

  // 갯수 업데이트
  queue->cnt--;

  // deque에 성공했으므로 1을 반환
  return 1;
}

void update_queue(MedianQueue *queue, float data) {
  // 만약 최대 저장 가능 갯수보다 많거나 같게 저장되어 있다면
  if(is_full(queue)) {
    // 우선 가장 오래된 데이터를 버린다
    dequeue(queue);
  }
  
  // 적절한 위치에 새 데이터를 삽입한다.
  enqueue(queue, data);
}

float get_median_val(MedianQueue *que) {
  int length = que->cnt;
  
  // 값들을 오름차순으로 정렬해 저장하기 위한 배열
  float value_arr[length];
  // 배열 초기화
  Node *cur_node = que->front;
  for(int i=0; i<length; i++) {
    value_arr[i] = cur_node->data;
    cur_node = cur_node->next;
  }

  // 배열을 오름차순으로 정렬
  bubble_sort(value_arr, length);
  
  // 만약 큐의 길이가 홀수이면
  if (length%2 !=0) {
    // 정 가운데 있는 값을 반환
    return value_arr[int(length/2)];
  }

  // 만약 큐의 길이가 짝수이면
  else {
    // 가운데 있는 두 노드의 평균을 반환;
    return (value_arr[int(length/2)] + value_arr[int(length/2)-1]) / 2;
  }
}

// 큐의 데이터를 맨 앞부터 출력
void print_queue(MedianQueue *que) {
  if (is_empty(que)) {
    Serial.println("EMPTY QUEUE");
    return;
  }
  
  // 큐의 맨 앞부터 순회
  Node *cur_node = que->front;

  // 큐의 맨 뒤를 제외한 모든 노드를 출력
  for(int i=0; i<(que->cnt)-1; i++) {
    Serial.print(cur_node->data);
    Serial.print(" -> ");
    cur_node = cur_node->next;
  }

  // 큐의 맨 뒤 노드를 출력하고 개행
  Serial.println(cur_node->data);
}

// 이상 함수 정의 ==============================

void setup()
{
  // initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  pinMode(PIN_TRIG,OUTPUT);
  digitalWrite(PIN_TRIG, LOW); 
  pinMode(PIN_ECHO,INPUT);

  // initialize USS related variables
  dist_min = _DIST_MIN; 
  dist_max = _DIST_MAX;
  timeout = (INTERVAL / 2) * 1000.0; // precalculate pulseIn() timeout value. (unit: us)
  dist_raw = 0.0; // raw distance output from USS (unit: mm)
  scale = 0.001 * 0.5 * SND_VEL;

  int queue_capacity = QUEUE_SIZE;
  init_queue(&queue, queue_capacity);//큐 초기화
  
  Serial.begin(57600);
}

void loop() {
  // wait until next sampling time. 
  // millis() returns the number of milliseconds since the program started. Will overflow after 50 days.
  if(millis() < last_sampling_time + INTERVAL) return;

  // get a distance reading from the USS
  dist_raw = USS_measure(PIN_TRIG,PIN_ECHO);

  update_queue(&queue, dist_raw);
  median = get_median_val(&queue);
//    print_queue(&queue);
      
  // output the read value to the serial port
  Serial.print("Min:0,");
  Serial.print("raw:");
  Serial.print(dist_raw);
  Serial.print(",");
  Serial.print("median:");
  Serial.print(map(median,0,400,100,500));
  Serial.print(",");
  Serial.println("Max:500");

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
  if(reading < dist_min || reading > dist_max) reading = 0.0; // return 0 when out of range.
  return reading;
  // Pulse duration to distance conversion example (target distance = 17.3m)
  // - round trip distance: 34.6m
  // - expected pulse duration: 0.1 sec, or 100,000us
  // - pulseIn(ECHO, HIGH, timeout) * 0.001 * 0.5 * SND_VEL
  //           = 100,000 micro*sec * 0.001 milli/micro * 0.5 * 346 meter/sec
  //           = 100,000 * 0.001 * 0.5 * 346 * micro * sec * milli * meter
  //                                           ----------------------------
  //                                           micro * sec
  //           = 100 * 173 milli*meter = 17,300 mm = 17.3m
  // pulseIn() returns microseconds.
}
