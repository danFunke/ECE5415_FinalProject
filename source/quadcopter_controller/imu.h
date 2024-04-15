/**
 *
 *
 *
 */

#ifndef IMU_H_
#define IMU_H_

void imu_init(void);

void imu_update1(void);

void imu_update2(void);

void imu_update3(void);

float imu_get_roll_rate(void);

float imu_get_pitch_rate(void);

float imu_get_yaw_rate(void);

float imu_get_roll_angle(void);

float imu_get_pitch_angle(void);

float imu_get_z_rate(void);

 #endif /* IMU_H_ */