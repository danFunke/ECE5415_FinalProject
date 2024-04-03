#ifndef PID_H_
#define PID_H_

typedef struct {
  float kp;
  float ki;
  float kd;
  float i_term_prev;
  float error_prev;
  float dt;
  float output;
} pid_t;

void pid_init(pid_t* pid, float kp, float ki, float kd, float dt);

void pid_update(pid_t* pid, float error);

#endif /* PID_H_ */