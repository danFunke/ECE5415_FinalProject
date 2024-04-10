#include "motor_controller.h"
#include "imu.h"
#include "pid.h"
#include "rc_receiver.h"

#include <Arduino.h>

#define MOTOR_CONTROL_PINS B11110000

#define MAX_ANGLE 20
#define CONTROL_ANGLE_RATIO MAX_ANGLE/500

#define THROTTLE_MAX  1800
#define THROTTLE_IDLE 1200
#define THROTTLE_MIN  1050

#define MOTOR_INPUT_MAX   1600
#define MOTOR_INPUT_IDLE  1100
#define MOTOR_INPUT_MIN   1000

enum PIDNames {
  PID_ROLL,
  PID_PITCH,
  PID_YAW,
  NUM_PIDS
};

// TODO: fill in appropriate gain values; gains for roll and pitch will be equal
const float K_P[] = {1.3, 1.3, 4.0};
const float K_I[] = {0.04, 0.04, 0.02};
const float K_D[] = {18, 18, 1};
const float dT = 0.004;
pid_t pids[NUM_PIDS];

int motor_inputs[NUM_MOTORS];

bool throttle_low = false;

static void write_pulses(void)
{
  // Get current time
  unsigned long pulse_timer = micros();

  // Write ones to all motor control output pins
  PORTD |= MOTOR_CONTROL_PINS;

  // Calculate pulse durations
  unsigned long pulse_durations[NUM_MOTORS];
  for (int i = 0; i < NUM_MOTORS; ++i) {
    pulse_durations[i] = motor_inputs[i] + pulse_timer;
  }
  
  // Write pulses over output pins
  unsigned long loop_timer;
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
}

void motor_controller_init(void)
{
  // Set motor control pins as output
  DDRD |= MOTOR_CONTROL_PINS;

  // Wait 5 seconds; send 0 signal to ESCs so they stay quiet
  for (int i = 0; i < 1250; ++i) {
    PORTD |= MOTOR_CONTROL_PINS;
    delayMicroseconds(1000);
    PORTD &= ~MOTOR_CONTROL_PINS;
    delayMicroseconds(3000);
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
  // Get reference values from RC receiver
  float reference_angle_roll = CONTROL_ANGLE_RATIO * (rc_receiver_get_value(RC_RECEIVER_CHANNEL_1) - 1500);
  float reference_angle_pitch = CONTROL_ANGLE_RATIO * (rc_receiver_get_value(RC_RECEIVER_CHANNEL_2) - 1500);
  float reference_rate_yaw = 0.15 * (rc_receiver_get_value(RC_RECEIVER_CHANNEL_4) - 1500);
  float reference_throttle = rc_receiver_get_value(RC_RECEIVER_CHANNEL_3);
  float start_switch = rc_receiver_get_value(RC_RECEIVER_CHANNEL_7);

  // Calculate angle error values
  float error_angle_roll = reference_angle_roll - imu_get_roll_angle();
  float error_angle_pitch = reference_angle_pitch - imu_get_pitch_angle();
  float error_rate_yaw  = reference_rate_yaw - imu_get_yaw_rate();

  // Update PIDs
  pid_update(&pids[PID_ROLL], error_angle_roll); 
  pid_update(&pids[PID_PITCH], error_angle_pitch);
  pid_update(&pids[PID_YAW], error_rate_yaw);

  // Check for start switch on
  if((start_switch < 1250) && throttle_low){
    // Check for saturation of throttle
    if (reference_throttle > THROTTLE_MAX) {
      reference_throttle = THROTTLE_MAX;
    }

    // Calculate motor control inputs
    motor_inputs[MOTOR_1] = (reference_throttle - pids[PID_PITCH].output + pids[PID_ROLL].output - pids[PID_YAW].output);
    motor_inputs[MOTOR_2] = (reference_throttle + pids[PID_PITCH].output + pids[PID_ROLL].output + pids[PID_YAW].output);
    motor_inputs[MOTOR_3] = (reference_throttle + pids[PID_PITCH].output - pids[PID_ROLL].output - pids[PID_YAW].output);
    motor_inputs[MOTOR_4] = (reference_throttle - pids[PID_PITCH].output - pids[PID_ROLL].output + pids[PID_YAW].output);

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
    if(start_switch < 1250){
      if(!throttle_low && reference_throttle<1050){
        throttle_low = true;
      }
    }else{
      throttle_low = false;
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