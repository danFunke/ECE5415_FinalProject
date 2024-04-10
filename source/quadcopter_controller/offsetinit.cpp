#include "offsetinit.h"
#include "imu.h"
#include <Arduino.h>

#define ITERATIONS 1000

float pitch_offset;
float roll_offset;

// Assuming we start on a level surface, calculate offset values
// for the roll and pitch values coming out of the kalman filter
void offset_init(void){
  float time;
  float sum_pitch = 0.0;
  float sum_roll = 0.0;
  roll_offset = 0.0;
  pitch_offset = 0.0;
  float pitch, roll;
  for(int i=0; i<ITERATIONS; i++){
    time = micros();
    imu_update1();
    imu_update2();
    pitch = imu_get_pitch_angle();
    sum_pitch += pitch;
    // Serial.print("Pitch = ");
    // Serial.print(pitch);
    roll = imu_get_roll_angle();
    sum_roll += roll;
    // Serial.print(" Roll = ");
    // Serial.println(roll);
    // Wait for our update frequency of 250Hz
    while(micros() - time < 4000);
  }
  pitch_offset = sum_pitch / (float)ITERATIONS;
  roll_offset = sum_roll / (float)ITERATIONS;
}

float get_pitch_offset(void){
  return(pitch_offset);
}

float get_roll_offset(void){
  return(roll_offset);
}