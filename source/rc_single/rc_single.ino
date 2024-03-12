const int receiver_pin = 2;
unsigned long pulse_duration;

void setup() {
  // put your setup code here, to run once:
  pinMode(receiver_pin, INPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  pulse_duration = pulseIn(receiver_pin, HIGH);
  Serial.println(pulse_duration);
}
