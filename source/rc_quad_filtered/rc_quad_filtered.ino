// Define filter parameters
#define WINDOW_SIZE 5

// Define control input pins
const int RUD = 5;
const int THR = 4;
const int ELE = 3;
const int AIL = 2;

typedef struct {
  int index;
  int value;
  int sum;
  int window[WINDOW_SIZE];
  int averaged_value;
} moving_average_filter_t;

static void filter_init(moving_average_filter_t* filter) {
  filter->index = 0;
  filter->value = 0;
  filter->sum = 0;
  filter->averaged_value = 0;
}

static void filter_update(moving_average_filter_t* filter, int new_value) {
  filter->sum = (filter->sum - filter->window[filter->index]);   // Remove the oldest entry from the sum
  filter->window[filter->index] = new_value;
  filter->sum = filter->sum + new_value;
  filter->index = (filter->index + 1) % WINDOW_SIZE;
  filter->averaged_value = filter->sum / WINDOW_SIZE;
}

// Control values
static moving_average_filter_t rud_value;
static moving_average_filter_t thr_value;
static moving_average_filter_t ele_value;
static moving_average_filter_t ail_value;

void setup() {
  // Initialize pins
  pinMode(RUD, INPUT);
  pinMode(THR, INPUT);
  pinMode(ELE, INPUT);
  pinMode(AIL, INPUT);

  // Initialize filters
  filter_init(&rud_value);
  filter_init(&thr_value);
  filter_init(&ele_value);
  filter_init(&ail_value);

  // Initialize serial comm
  Serial.begin(9600);
}

void loop() {
  // Read input values and update filters
  filter_update(&rud_value, pulseIn(RUD, HIGH));
  filter_update(&thr_value, pulseIn(THR, HIGH));
  filter_update(&ele_value, pulseIn(ELE, HIGH));
  filter_update(&ail_value, pulseIn(AIL, HIGH));

  // Delay for stability
  delay(50);

  // DEBUGGING - PRINT TO SERIAL
  Serial.print(rud_value.averaged_value);
  Serial.print(',');
  Serial.print(thr_value.averaged_value);
  Serial.print(',');
  Serial.print(ele_value.averaged_value);
  Serial.print(',');
  Serial.print(ail_value.averaged_value);
  Serial.print(',');
  Serial.println();
}
