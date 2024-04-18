#include "motor_controller.h"
#include "imu.h"
#include "pid.h"
#include "rc_receiver.h"
#include <EEPROM.h>

#include <Arduino.h>

// #define UNO
#define MEGA

// Define to make thrust controlled by PID
// #define Z_RATE_CONTROL

#define MOTOR_CONTROL_PINS B11110000

#define H_PINS B00011000
#define E_PINS B00001000
#define G_PINS B00100000

#define MAX_ANGLE 20.0
#define CONTROL_ANGLE_RATIO MAX_ANGLE/500.0

#define THROTTLE_MAX  1800
#define THROTTLE_IDLE 1200
#define THROTTLE_MIN  1050
#define THROTTLE_OFFSET 200

#define MOTOR_INPUT_MAX   1800
#define MOTOR_INPUT_IDLE  1100
#define MOTOR_INPUT_MIN   1000

#define DEADZONE 75 // + or - from 1500

#define EEPROM_LENGTH 1024 // number of bytes of memory in EEPROM

enum PIDNames {
  PID_ROLL,
  PID_PITCH,
  PID_YAW,
  PID_THRUST,
  NUM_PIDS
};

// TODO: fill in appropriate gain values; gains for roll and pitch will be equal
const float K_P[] = {2.5, 2.5, 12, 10};// {1.3, 1.3, 4.0};
const float K_I[] = {.1, .1, 0, 0};// {0.04, 0.04, 0.02};
const float K_D[] = {1, 1, .2, .5};// {18, 18, 1};
const float dT = 0.005;
pid_t pids[NUM_PIDS];

int motor_inputs[NUM_MOTORS];

bool throttle_low = false;
float throttle_offset = 0.0;

int log_count = 0;
bool logged = false;
bool log_start = false;
float data_log[EEPROM_LENGTH];

static void write_pulses(void)
{
  // Get current time
  unsigned long pulse_timer = micros();

  #ifdef UNO
  // Write ones to all motor control output pins
  PORTD |= MOTOR_CONTROL_PINS;
  #endif
  #ifdef MEGA
  PORTH |= H_PINS; // 6 = PH3; 7 = PH4;
  PORTE |= E_PINS; // 5 = PE3;
  PORTG |= G_PINS; // 4 = PG5;
  #endif

  // Calculate pulse durations
  unsigned long pulse_durations[NUM_MOTORS];
  for (int i = 0; i < NUM_MOTORS; ++i) {
    pulse_durations[i] = motor_inputs[i] + pulse_timer;
  }

  imu_update2();

  // Write pulses over output pins
  unsigned long loop_timer;
  #ifdef UNO
  while (PORTD >= 16) {
    loop_timer = micros();

    if (pulse_durations[0] <= loop_timer) {
      PORTD &= B11101111;
    }

    if (pulse_durations[1] <= loop_timer) {
      PORTD &= B11011111;
    }

    if (pulse_durations[2] <= loop_timer) {
      PORTD &= B10111111;
    }

    if (pulse_durations[3] <= loop_timer) {
      PORTD &= B01111111;
    }
  }
  #endif
  #ifdef MEGA
  
  while ((PORTH >= 16) || (PORTE >= 16) || (PORTG >= 16)) {
    loop_timer = micros();

    if (pulse_durations[0] <= loop_timer) {
      PORTG &= ~G_PINS;
    }

    if (pulse_durations[1] <= loop_timer) {
      PORTE &= B11110111;
    }

    if (pulse_durations[2] <= loop_timer) {
      PORTH &= B11101111;
    }

    if (pulse_durations[3] <= loop_timer) {
      PORTH &= ~H_PINS;
    }
  }

  #endif
}

void motor_controller_init(void)
{
  // Set motor control pins as output
  #ifdef UNO
  DDRD |= MOTOR_CONTROL_PINS;
  #endif
  #ifdef MEGA
  DDRH |= H_PINS; // 6 = PH3; 7 = PH4;
  DDRE |= E_PINS; // 5 = PE3;
  DDRG |= G_PINS; // 4 = PG5;
  #endif;

  // Wait 5 seconds; send 0 signal to ESCs so they stay quiet
  for (int i = 0; i < 1250; ++i) {
    #ifdef UNO
    PORTD |= MOTOR_CONTROL_PINS;
    delayMicroseconds(1000);
    PORTD &= ~MOTOR_CONTROL_PINS;
    delayMicroseconds(3000);
    #endif
    #ifdef MEGA
    PORTH |= H_PINS; // 6 = PH3; 7 = PH4;
    PORTE |= E_PINS; // 5 = PE3;
    PORTG |= G_PINS; // 4 = PG5;
    delayMicroseconds(1000);
    PORTH &= ~H_PINS; // 6 = PH3; 7 = PH4;
    PORTE &= ~E_PINS; // 5 = PE3;
    PORTG &= ~G_PINS; // 4 = PG5;
    delayMicroseconds(3000);
    #endif
  }

  // Initialize PIDs
  for (int i = 0; i < NUM_PIDS; ++i) {
    pid_init(&pids[i], K_P[i], K_I[i], K_D[i], dT);
  }

  // Initialize motor control values
  for (int i = 0; i < NUM_MOTORS; ++i) {
    motor_inputs[i] = 1200;
  }

}

void motor_controller_update(void)
{
  // Get inputs from RC receiver and map
  float controller_roll_input = rc_receiver_get_value(RC_RECEIVER_CHANNEL_1);
  float controller_pitch_input = rc_receiver_get_value(RC_RECEIVER_CHANNEL_2);
  float controller_yaw_input = rc_receiver_get_value(RC_RECEIVER_CHANNEL_4);
  float controller_thrust_input = rc_receiver_get_value(RC_RECEIVER_CHANNEL_3);

  // Serial.print("roll_input: "); Serial.print(controller_roll_input);
  // Serial.print(" pitch_input: "); Serial.println(controller_pitch_input);

  // Add deadzones to pitch for better control
  if((controller_pitch_input >= (1500 - DEADZONE)) && (controller_pitch_input <= (1500 + DEADZONE))){
    controller_pitch_input = 1500;
  }else if(controller_pitch_input > (1500 + DEADZONE)){
    controller_pitch_input = controller_pitch_input - DEADZONE;
  }else{
    controller_pitch_input = controller_pitch_input + DEADZONE;
  }

  // Invert pitch to be more intuitive
  float temp = 2000 - controller_pitch_input;
  controller_pitch_input = 1000 + temp;

  // Add deadzones to roll
  if((controller_roll_input >= (1500 - DEADZONE)) && (controller_roll_input <= (1500 + DEADZONE))){
    controller_roll_input = 1500;
  }else if(controller_roll_input > (1500 + DEADZONE)){
    controller_roll_input = controller_roll_input - DEADZONE;
  }else{
    controller_roll_input = controller_roll_input + DEADZONE;
  }

  // Add deadzones to yaw
  if((controller_yaw_input >= (1500 - DEADZONE)) && (controller_yaw_input <= (1500 + DEADZONE))){
    controller_yaw_input = 1500;
  }else if(controller_yaw_input > (1500 + DEADZONE)){
    controller_yaw_input = controller_yaw_input - DEADZONE;
  }else{
    controller_yaw_input = controller_yaw_input + DEADZONE;
  }

  #ifdef Z_RATE_CONTROL
    // Add deadzones to thrust
    if((controller_thrust_input >= (1500 - DEADZONE)) && (controller_thrust_input <= (1500 + DEADZONE))){
      controller_thrust_input = 1500;
    }else if(controller_thrust_input > (1500 + DEADZONE)){
      controller_thrust_input = controller_thrust_input - DEADZONE;
    }else{
      controller_thrust_input = controller_thrust_input + DEADZONE;
    }
  #endif

  // Serial.print("roll_input: "); Serial.print(controller_roll_input);
  // Serial.print(" pitch_input: "); Serial.println(controller_pitch_input);

  // Get reference values from RC receiver
  float reference_angle_roll = CONTROL_ANGLE_RATIO * (controller_roll_input - 1500);
  float reference_angle_pitch = CONTROL_ANGLE_RATIO * (controller_pitch_input - 1500);
  float reference_rate_yaw = 0.15 * (controller_yaw_input - 1500);
  #ifdef Z_RATE_CONTROL
  float reference_throttle = (controller_thrust_input - 1500) / 25;
  #endif
  #ifndef Z_RATE_CONTROL
  float reference_throttle = controller_thrust_input - THROTTLE_OFFSET;
  #endif
  float start_switch = rc_receiver_get_value(RC_RECEIVER_CHANNEL_7);
  float log_switch = rc_receiver_get_value(RC_RECEIVER_CHANNEL_5);

  // Calculate angle error values
  float error_angle_roll = reference_angle_roll - imu_get_roll_angle();
  float error_angle_pitch = reference_angle_pitch - imu_get_pitch_angle();
  float error_rate_yaw  = reference_rate_yaw - imu_get_yaw_rate();
  float error_rate_z = reference_throttle - imu_get_z_rate() + throttle_offset;

  // Serial.print(reference_throttle);Serial.print("\t");Serial.print(error_rate_z);Serial.print("\t");

  // Serial.print("Desired Roll = ");Serial.print(reference_angle_roll);
  // Serial.print(" Desired Pitch = ");Serial.print(reference_angle_pitch);

  // Serial.print(" Roll Error = "); Serial.print(error_angle_roll);
  // Serial.print(" Pitch Error = "); Serial.print(error_angle_pitch);
  // Serial.print(" Yaw Error = "); Serial.println(error_rate_yaw);

  // Update PIDs
  pid_update(&pids[PID_ROLL], error_angle_roll); 
  pid_update(&pids[PID_PITCH], error_angle_pitch);
  pid_update(&pids[PID_YAW], error_rate_yaw);
  pid_update(&pids[PID_THRUST], error_rate_z);

  #ifdef Z_RATE_CONTROL
  reference_throttle = 1300 + pids[PID_THRUST].output;
  // Serial.print(pids[PID_THRUST].output);Serial.print("\t");Serial.println(reference_throttle);
  #endif

  // Logging data for storage to the EEPROM
  // UNO Rev3 has 1024 bytes of storage, each float
  // takes 4 bytes
  // We can store 256 datapoints
  // Storing 50 per second, we can log about 20 seconds
  // of flight data for one parameter
  if(log_start && (log_count*4 < EEPROM_LENGTH)){
    // data_log[log_count] = error_angle_roll;
    log_count++;
    logged = true;
  }

  // Check for start switch on
  if((start_switch < 1250) && throttle_low && (reference_throttle > 500)){
    // Check for saturation of throttle
    if (reference_throttle > THROTTLE_MAX) {
      reference_throttle = THROTTLE_MAX;
    }

    // if(log_switch < 1250){
    //   log_start = true;
    // }

    // pids[PID_YAW].output = 0.0;

    // Serial.print("PID_PITCH = "); Serial.print(pids[PID_PITCH].output);
    // Serial.print("\tPID_ROLL = "); Serial.print(pids[PID_ROLL].output);
    // Serial.print("    \tPID_YAW = "); Serial.println(pids[PID_YAW].output);

    // Calculate motor control inputs
    motor_inputs[MOTOR_1] = (reference_throttle - pids[PID_PITCH].output - pids[PID_ROLL].output - pids[PID_YAW].output);
    motor_inputs[MOTOR_2] = (reference_throttle + pids[PID_PITCH].output - pids[PID_ROLL].output + pids[PID_YAW].output);
    motor_inputs[MOTOR_3] = (reference_throttle + pids[PID_PITCH].output + pids[PID_ROLL].output - pids[PID_YAW].output);
    motor_inputs[MOTOR_4] = (reference_throttle - pids[PID_PITCH].output + pids[PID_ROLL].output + pids[PID_YAW].output);

    // Check for saturation/idle conditions of motor inputs
    for (int i = 0; i < NUM_MOTORS; ++i) {
      // Saturation
      if (motor_inputs[i] > MOTOR_INPUT_MAX) {
        motor_inputs[i] = MOTOR_INPUT_MAX - 1;
      } 
      
      // Idle
      if (motor_inputs[i] < MOTOR_INPUT_IDLE) {
        motor_inputs[i] = MOTOR_INPUT_IDLE;
      }
    }

    // Check for cutoff condition
    if (reference_throttle < THROTTLE_MIN) {
      for (int i = 0; i < NUM_MOTORS; ++i) {
        motor_inputs[i] = MOTOR_INPUT_MIN;
      }

      // Reset PIDs
      for (int i = 0; i < NUM_PIDS; ++i) {
        pid_reset(&pids[i]);
      }
    }
  }else{
    if(start_switch < 1250){  // If start switch is on but throttle never went low
      if(!throttle_low && reference_throttle<1100){ // If throttle is low, enable flight
        throttle_low = true;
      }
    }else{  // Start switch isn't on, clear throttle low flag
      throttle_low = false;
      if(logged){
        // If start switch is off and data is logged, store in EEPROM.
        for(int i = 0; i*4 < EEPROM_LENGTH; i ++){
          EEPROM.put(i*4,data_log[i]);
        }
        while(1){
          // This takes ~3ms per write, will destroy timing loop. Hence we will lock the system here.
          digitalWrite(12,HIGH);
          digitalWrite(13,LOW);
          delay(500);
          digitalWrite(12,LOW);
          digitalWrite(13,HIGH);
          delay(500);
        }
      }
      
      // Z_rate drifts, so while sitting still grab the z_rate to use as an offset.
      throttle_offset = imu_get_z_rate();
    }
    // Start switch isn't on so turn off motors
    motor_inputs[MOTOR_1] = MOTOR_INPUT_MIN;
    motor_inputs[MOTOR_2] = MOTOR_INPUT_MIN;
    motor_inputs[MOTOR_3] = MOTOR_INPUT_MIN;
    motor_inputs[MOTOR_4] = MOTOR_INPUT_MIN;
  }

  // Write motor control values
  write_pulses();
}

int motor_controller_get_inputs(int motor_index)
{
  return motor_inputs[motor_index];
}