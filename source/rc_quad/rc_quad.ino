// Define control input pins
const int RUD = 5;
const int THR = 4;
const int ELE = 3;
const int AIL = 2;

// Variables to hold control values
unsigned long rud_duration;
unsigned long thr_duration;
unsigned long ele_duration;
unsigned long ail_duration;

void setup() {
  // Initialize input pins
  pinMode(RUD, INPUT);
  pinMode(THR, INPUT);
  pinMode(ELE, INPUT);
  pinMode(AIL, INPUT);

  // Initialize serial input
  Serial.begin(9600);
}

void loop() {
  // Read input values
  rud_duration = pulseIn(RUD, HIGH);
  thr_duration = pulseIn(THR, HIGH);
  ele_duration = pulseIn(ELE, HIGH);
  ail_duration = pulseIn(AIL, HIGH);

  // DEBUGGING - PRINT TO SERIAL
  Serial.print(rud_duration);
  Serial.print(',');
  Serial.print(thr_duration);
  Serial.print(',');
  Serial.print(ele_duration);
  Serial.print(',');
  Serial.print(ail_duration);
  Serial.print(',');
  Serial.println();

  // Delay for stability
  delay(50);
}
