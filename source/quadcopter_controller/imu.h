/**
 *
 *
 *
 */

#ifndef IMU_H_
#define IMU_H_

void imu_init(void);

void imu_update(void);

float imu_get_roll_rate(void);

float imu_get_pitch_rate(void);

float imu_get_yaw_rate(void);

float imu_get_roll_angle(void);

float imu_get_pitch_angle(void);

 #endif /* IMU_H_ */