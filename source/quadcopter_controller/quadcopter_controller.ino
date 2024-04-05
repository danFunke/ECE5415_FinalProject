/**
 *
 *
 *
 */

#include "imu.h"
#include "motor_controller.h"
#include "rc_receiver.h"

/*** Global variables ***/
const uint32_t FRAME_DURATION = 4000;

/*** Static functions ***/

uint32_t frame_timer;

void setup(void) 
{
  imu_init();
  motor_controller_init();
  rc_receiver_init();

  

  // Dev/Debug
  pinMode(8, OUTPUT);
  Serial.begin(115200);

  // Initialize frame timer
  frame_timer = micros();
}

void loop() 
{
  imu_update();
  motor_controller_update();
  rc_receiver_update();

  // Delay until end of frame
  while(micros() - frame_timer < FRAME_DURATION) {}
  frame_timer = micros();

  // Debug
  // Serial.print("Channel 1:");
  // Serial.print(rc_receiver_get_value(1));
  // Serial.print(",");
  // Serial.print("Channel 2:");
  // Serial.print(rc_receiver_get_value(2));
  // Serial.print(",");
  // Serial.print("Channel 3:");
  // Serial.print(rc_receiver_get_value(3));
  // Serial.print(",");
  // Serial.print("Channel 4:");
  // Serial.print(rc_receiver_get_value(4));
  // Serial.println();
}
