#ifndef MOTOR_CONTROLLER_H_
#define MOTOR_CONTROLLER_H_

enum MotorNames {
  MOTOR_1,
  MOTOR_2,
  MOTOR_3,
  MOTOR_4,
  NUM_MOTORS
};

void motor_controller_init(void);

void motor_controller_update(void);

int motor_controller_get_inputs(int motor_index);

#endif /* MOTOR_CONTROLLER_H_ */