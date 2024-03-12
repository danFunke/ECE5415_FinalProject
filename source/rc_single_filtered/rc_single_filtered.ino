// Define filter parameters
#define WINDOW_SIZE 5

// Define control input pins
const int ELE = 3;

// variables for filter
int INDEX = 0;
int VALUE = 0;
int SUM = 0;
int READINGS[WINDOW_SIZE];
int AVERAGED = 0;

void setup() {
  pinMode(ELE, INPUT);
  Serial.begin(9600);
}

void loop() {
  SUM = SUM - READINGS[INDEX]; // Remove the oldest entry from the sum
  VALUE = pulseIn(ELE, HIGH);
  READINGS[INDEX] = VALUE;
  SUM = SUM + VALUE;
  INDEX = (INDEX + 1) % WINDOW_SIZE;  // Increment the index, and wrap to zero if exceeds window size

  AVERAGED = SUM / WINDOW_SIZE;

  Serial.print(VALUE);
  Serial.print(",");
  Serial.println(AVERAGED);

  delay(50);

}
