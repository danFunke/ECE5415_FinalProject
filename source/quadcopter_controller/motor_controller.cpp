#include "motor_controller.h"
#include "imu.h"
#include "pid.h"
#include "rc_receiver.h"
#include <Servo.h>

#define THROTTLE_MAX    1800
#define THROTTLE_CUTOFF 1050

#define MOTOR_INPUT_MAX   2000
#define MOTOR_INPUT_IDLE  1200
#define MOTOR_INPUT_MIN   1000

enum AnglePIDNames {
  ANGLE_PID_ROLL,
  ANGLE_PID_PITCH,
  NUM_ANGLE_PIDS
};

// TODO: fill in appropriate gain values; gains for roll and pitch will be equal
const float ANGLE_K_P[] = {1, 1, 1};
const float ANGLE_K_I[] = {1, 1, 1};
const float ANGLE_K_D[] = {1, 1, 1};
const float dT = 0.004;
pid_t angle_pids[NUM_ANGLE_PIDS];

enum RatePIDNames {
  RATE_PID_ROLL,
  RATE_PID_PITCH,
  RATE_PID_YAW,
  NUM_RATE_PIDS
};

// TODO: fill in appropriate gain values; gains for roll and pitch will be equal
const float RATE_K_P[] = {1, 1, 1};
const float RATE_K_I[] = {1, 1, 1};
const float RATE_K_D[] = {1, 1, 1};
pid_t rate_pids[NUM_RATE_PIDS];

enum MotorNames {
  MOTOR_1,
  MOTOR_2,
  MOTOR_3,
  MOTOR_4,
  NUM_MOTORS
};

// TODO: correct pin numbers for drone arduino
const int MOTOR_PINS[] = {11, 10, 9, 6};
Servo motors[NUM_MOTORS];

void motor_controller_init(void)
{
  // Initialize angle PIDs
  for (int i = 0; i < NUM_ANGLE_PIDS; ++i) {
    pid_init(&angle_pids[i], ANGLE_K_P[i], ANGLE_K_I[i], ANGLE_K_D[i], dT);
  }

  // Initialize rate PIDs
  for (int i = 0; i < NUM_RATE_PIDS; ++i) {
    pid_init(&rate_pids[i], RATE_K_P[i], RATE_K_I[i], RATE_K_D[i], dT);
  }

  // Initialize motors
  for (int i = 0; i < NUM_MOTORS; ++i) {
    motors[i].attach(MOTOR_PINS[i]);
    motors[i].write(MOTOR_INPUT_MIN);
  }
}

void motor_controller_update(void)
{
  // Get reference values from RC receiver
  float reference_angle_roll = 0.10 * (rc_receiver_get_value(RC_RECEIVER_CHANNEL_1) - 1500);
  float reference_angle_pitch = 0.10 * (rc_receiver_get_value(RC_RECEIVER_CHANNEL_2) - 1500);
  float reference_rate_yaw = 0.15 * (rc_receiver_get_value(RC_RECEIVER_CHANNEL_4) - 1500);
  float reference_throttle = rc_receiver_get_value(RC_RECEIVER_CHANNEL_3);

  // Calculate angle error values
  float error_angle_roll = reference_angle_roll - imu_get_roll_angle();
  float error_angle_pitch = reference_angle_pitch - imu_get_pitch_angle();

  // Update angle PIDs
  pid_update(&angle_pids[ANGLE_PID_ROLL], error_angle_roll);
  pid_update(&angle_pids[ANGLE_PID_PITCH], error_angle_pitch);

  // Calculate rate error values
  float error_rate_roll = angle_pids[ANGLE_PID_ROLL].output - imu_get_roll_rate();
  float error_rate_pitch = angle_pids[ANGLE_PID_PITCH].output - imu_get_pitch_rate();
  float error_rate_yaw = reference_rate_yaw - imu_get_yaw_rate();

  // Update rate PIDs
  pid_update(&rate_pids[RATE_PID_ROLL], error_rate_roll);
  pid_update(&rate_pids[RATE_PID_PITCH], error_rate_pitch);
  pid_update(&rate_pids[RATE_PID_YAW], error_rate_yaw);

  // Check for saturation of throttle
  if (reference_throttle > THROTTLE_MAX) {
    reference_throttle = THROTTLE_MAX;
  }

  // Calculate motor control inputs
  float motor_inputs[NUM_MOTORS];
  motor_inputs[MOTOR_1] = 1.024 * (reference_throttle - rate_pids[RATE_PID_ROLL].output - rate_pids[RATE_PID_PITCH].output - rate_pids[RATE_PID_YAW].output);
  motor_inputs[MOTOR_2] = 1.024 * (reference_throttle - rate_pids[RATE_PID_ROLL].output + rate_pids[RATE_PID_PITCH].output + rate_pids[RATE_PID_YAW].output);
  motor_inputs[MOTOR_3] = 1.024 * (reference_throttle + rate_pids[RATE_PID_ROLL].output + rate_pids[RATE_PID_PITCH].output - rate_pids[RATE_PID_YAW].output);
  motor_inputs[MOTOR_4] = 1.024 * (reference_throttle + rate_pids[RATE_PID_ROLL].output - rate_pids[RATE_PID_PITCH].output + rate_pids[RATE_PID_YAW].output);

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
  if (reference_throttle < THROTTLE_CUTOFF) {
    for (int i = 0; i < NUM_MOTORS; ++i) {
      motor_inputs[i] = MOTOR_INPUT_MIN;
    }

    // Reset PIDs
    for (int i = 0; i < NUM_ANGLE_PIDS; ++i) {
      pid_reset(&angle_pids[i]);
    }

    for (int i = 0; i < NUM_RATE_PIDS; ++i) {
      pid_reset(&rate_pids[i]);
    }
  }

  // Write motor control values
  for (int i = 0; i < NUM_MOTORS; ++i) {
    motors[i].write(motor_inputs[i]);
  }
  
}