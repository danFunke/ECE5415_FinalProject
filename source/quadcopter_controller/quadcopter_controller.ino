/**
 *
 *
 *
 */

#include "imu.h"
#include "motor_controller.h"
#include "rc_receiver.h"

#define STATUS_LED_RED    13
#define STATUS_LED_GREEN  12

/*** Global variables ***/
const uint32_t FRAME_DURATION = 4000;


uint32_t frame_timer;

void setup(void) 
{
  // Initialize status LEDs
  pinMode(STATUS_LED_RED, OUTPUT);
  pinMode(STATUS_LED_GREEN, OUTPUT);
  Serial.begin(115200);

  // Initialize hardware
  digitalWrite(STATUS_LED_RED, HIGH);
  digitalWrite(STATUS_LED_GREEN, LOW);
  Serial.print("Beginning Initialization");
  for(int i=0; i<5; i++){
    Serial.print(".");
    delay(250);
  }
  Serial.println(" ");
  Serial.println("Initializing IMU.");
  imu_init();
  Serial.println("IMU Initialized.");
  delay(500);
  Serial.println("Initializing motor controller.");
  motor_controller_init();
  Serial.println("Motor controller initialized.");
  delay(500);
  Serial.println("Initializing RC receiver.");
  rc_receiver_init();
  Serial.println("Motor controller initialized.");
  delay(500);
  Serial.println("Initialization complete.");
  digitalWrite(STATUS_LED_RED, LOW);
  digitalWrite(STATUS_LED_GREEN, HIGH);

  // Dev/Debug
  pinMode(8, OUTPUT);

  // Initialize frame timer
  frame_timer = micros();
}

int print_count = 0;
void loop() 
{
  // Enforce a 250 Hz frame
  while(micros() - frame_timer < FRAME_DURATION) {}
  frame_timer = micros();

  motor_controller_update();
  rc_receiver_update();
  imu_update();


  // Debug
  if (print_count == 125) {
    // Serial.print("MOTOR 0:");
    // Serial.print(motor_controller_get_inputs(0));
    // Serial.print(",");
    // Serial.print("\tMOTOR 1:");
    // Serial.print(motor_controller_get_inputs(1));
    // Serial.print(",");
    // Serial.print("\tMOTOR 2:");
    // Serial.print(motor_controller_get_inputs(2));
    // Serial.print(",");
    // Serial.print("\tMOTOR 3:");
    // Serial.print(motor_controller_get_inputs(3));
    // Serial.println();

    Serial.print("ROLL:");
    Serial.print(rc_receiver_get_value(RC_RECEIVER_CHANNEL_1));
    Serial.print(",");
    Serial.print("     PITCH:");
    Serial.print(rc_receiver_get_value(RC_RECEIVER_CHANNEL_2));
    Serial.print(",");
    Serial.print("     YAW:");
    Serial.print(rc_receiver_get_value(RC_RECEIVER_CHANNEL_4));
    Serial.print(",");
    Serial.print("     THROTTLE:");
    Serial.print(rc_receiver_get_value(RC_RECEIVER_CHANNEL_3));
    Serial.print(",");
    Serial.print("    CHANNEL 5:");
    Serial.print(rc_receiver_get_value(RC_RECEIVER_CHANNEL_5));
    Serial.print(",");
    Serial.print("     CHANNEL 6:");
    Serial.print(rc_receiver_get_value(RC_RECEIVER_CHANNEL_6));
    Serial.print(",");
    Serial.print("     CHANNEL 7:");
    Serial.print(rc_receiver_get_value(RC_RECEIVER_CHANNEL_7));
    Serial.print(",");
    Serial.print("     CHANNEL 8:");
    Serial.print(rc_receiver_get_value(RC_RECEIVER_CHANNEL_8)); 
    Serial.println();
    print_count = 0;
  }
  print_count++;
}
