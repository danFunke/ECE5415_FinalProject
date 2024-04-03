#include "pid.h"

void pid_init(pid_t* pid, float kp, float ki, float kd, float dt)
{
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->dt = dt;
  pid->i_term_prev = 0;
  pid->error_prev = 0;
}

void pid_update(pid_t* pid, float error)
{
  // Calculate p term
  float p_term = pid->kp * error;
  
  // Calculate i term; Check for saturation
  float i_term = pid->i_term_prev + (pid->ki * (error + pid->error_prev) * pid->dt / 2);

  if (i_term > 400) {
    i_term = 400;
  } else if (i_term < -400) {
    i_term = -400;
  }

  // Calculate d term
  float d_term = pid->kd * (error - pid->error_prev) / pid->dt;

  // Calculate PID output; check for saturation
  pid->output = p_term + i_term + d_term;
  
  if (pid->output > 400) {
    pid->output = 400;
  } else if (pid->output < -400) {
    pid->output = -400;
  }

  // Update i_term_prev and error_prev
  pid->i_term_prev = i_term;
  pid->error_prev = error;
}