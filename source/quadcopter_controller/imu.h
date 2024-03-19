/**
 *
 *
 *
 */

#ifndef IMU_H_
#define IMU_H_

#include <Wire.h>

const int MPU_ADDR = 0x68;  // I2C address of MPU board
const float x_calibration = 0.0;
const float y_calibration = 0.0;
const float z_calibration = -0.02;

// Angular velocity variables
float pitch_rate;
float roll_rate;
float yaw_rate;

// Linear acceleration variables
float x_acc;
float y_acc;
float z_acc;

// Angular displacement variables
float pitch_angle;
float roll_angle;

float LoopTimer; // not sure what this is for yet

void gyro_signals(void) {
  // Turn on low-pass filter
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  // Configure accelerometer output
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);                   // Accelerometer configuration register address
  Wire.write(0x10);                   // Full scale range of 8g
  Wire.endTransmission();

  // Pull accelerometer measurements from sensor
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);                   // Accelerometer output registers start address (XOUT[15:8])
  Wire.endTransmission();
  Wire.requestFrom(MPU_ADDR, 6);      // Request 6 bytes from IMU
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  // Configure gyroscope outputs
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();

  // Pull gyroscope measurements from the sensor
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(MPU_ADDR, 6);      // Request 6 bytes from IMU
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();
  roll_rate = (float)GyroX / 65.5;
  pitch_rate = (float)GyroY / 65.5;
  yaw_rate = (float)GyroZ / 65.5;

  x_acc = ((float)AccXLSB / 4096) + x_calibration;
  y_acc = ((float)AccYLSB / 4096) + y_calibration;
  z_acc = ((float)AccZLSB / 4096) + z_calibration;

  roll_angle = atan(y_acc / sqrt(x_acc * x_acc + z_acc * z_acc));
  pitch_angle = -atan(x_acc / sqrt(y_acc * y_acc + z_acc * z_acc));

  // Wire.beginTransmission(MPU_ADDR);
  // Wire.write(0x3B);
  // Wire.endTransmission(false);
  // Wire.requestFrom(MPU_ADDR, 6, true);

  // x_acc = Wire.read() << 8 | Wire.read();
  // y_acc = Wire.read() << 8 | Wire.read();
  // z_acc = Wire.read() << 8 | Wire.read();
}

void imu_init(void)
{
  Wire.begin();
  delay(250);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
}

// float imu_get_pitch_angle(void)
// {

// }

// float imu_get_roll_angle(void)
// {

// }

// float imu_get_z_acc(void)
// {

// }


 #endif /* IMU_H_ */