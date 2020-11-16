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

// 중위수 필터 관련
#define QUEUE_SIZE 8  // 큐 사이즈 지정
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

Servo myservo;

MedianQueue queue;
float median;

void setup() {
  // GPIO 핀 초기화
  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(_DUTY_NEU);

  // 거리 측정 관련 변수 초기화
  dist_min = _DIST_MIN;
  dist_max = _DIST_MAX;
  dist_raw = 0.0;                    // USS로부터 읽어온 처리 전의 거리 (unit: mm)

  // 큐 초기화
  int queue_capacity = QUEUE_SIZE;
  init_queue(&queue, queue_capacity);

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

  update_queue(&queue, dist_raw);
  median = get_median_val(&queue);
  
  Serial.print("Min:0,");
  Serial.print("raw:");
  Serial.print(dist_raw);
  Serial.print(", ");
  Serial.print("median:");
  Serial.print(median);
  Serial.print(", ");
  Serial.println("Max:400");

  if (median < 255)
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

// 이하 중위수 필터 관련 함수 정의 =============================
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
