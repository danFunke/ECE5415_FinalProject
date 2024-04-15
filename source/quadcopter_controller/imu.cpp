#include "imu.h"
#include "kalman_filter.h"
#include "offsetinit.h"
#include <Arduino.h>
#include <Wire.h>

const float dT = .005;

const int MPU_ADDR = 0x68;  // I2C address of MPU board
const int RATE_CALIBRATION_ITERATIONS = 2000;

// Global variables
float rate_roll;
float rate_pitch;
float rate_yaw;
float rate_roll_calibration;
float rate_pitch_calibration;
float rate_yaw_calibration;
bool gyro_is_calibrated = false;
float acc_x;
float acc_y;
float acc_z;
float acc_x_calibration = -0.08;
float acc_y_calibration = 0;
float acc_z_calibration = 0.15;

float z_rate = 0.0;
float x_comp, y_comp, z_comp;

// Output variables
kalman_filter_1d_t roll_angle;
kalman_filter_1d_t pitch_angle;

// Made global to share between update1 and update2
int16_t acc_x_bin;
int16_t acc_y_bin;
int16_t acc_z_bin;

void imu_update1(void){
  // This function call takes approx 950us

  // Get accelerometer data (6 bytes) from MPU
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(MPU_ADDR, 14);
  acc_x_bin = Wire.read() << 8 | Wire.read();
  acc_y_bin = Wire.read() << 8 | Wire.read();
  acc_z_bin = Wire.read() << 8 | Wire.read();
  Wire.read() << 8 | Wire.read(); // This is temperature data, we don't care about it
}

void imu_update2(void)
{
  // Get gyroscope data (6 bytes) from MPU
  // Wire.beginTransmission(MPU_ADDR);
  // Wire.write(0x43);
  // Wire.endTransmission();
  // Wire.requestFrom(MPU_ADDR, 6);
  int16_t gyro_x_bin = Wire.read() << 8 | Wire.read();
  int16_t gyro_y_bin = Wire.read() << 8 | Wire.read();
  int16_t gyro_z_bin = Wire.read() << 8 | Wire.read();

  // Convert binary acc/gyro values to decimal
  acc_x = ((float)acc_x_bin / 4096) + acc_x_calibration;
  acc_y = ((float)acc_y_bin / 4096) + acc_y_calibration;
  acc_z = ((float)acc_z_bin / 4096) + acc_z_calibration;
  rate_roll = (float)gyro_x_bin / 65.5;
  rate_pitch = (float)gyro_y_bin / 65.5;
  rate_yaw = (float)gyro_z_bin / 65.5;

  if (gyro_is_calibrated) {
    rate_roll -= rate_roll_calibration;
    rate_pitch -= rate_pitch_calibration;
    rate_yaw -= rate_yaw_calibration;
  }

  // // Calculate roll and pitch angles in radians
  // float roll_angle_raw = atan(acc_y / sqrt(acc_x * acc_x + acc_z * acc_z));
  // float pitch_angle_raw = -atan(acc_x / sqrt(acc_y * acc_y + acc_z * acc_z));

  // Calculate roll and pitch angles in degrees
  float roll_angle_raw = atan(acc_y / sqrt(acc_x * acc_x + acc_z * acc_z)) * 1/(3.142/180);
  float pitch_angle_raw = -atan(acc_x / sqrt(acc_y * acc_y + acc_z * acc_z)) * 1/(3.142/180);

  // Update filtered output values
  kalman_filter_update(&roll_angle, rate_roll, roll_angle_raw);
  kalman_filter_update(&pitch_angle, rate_pitch, pitch_angle_raw);
}

void imu_update3(void){
  // Calculate Z velocity
  float roll_angle_rad = imu_get_roll_angle() * (3.142/180);
  float pitch_angle_rad = imu_get_pitch_angle() * (3.142/180);
  x_comp = (-acc_x)*sin(pitch_angle_rad);
  y_comp = acc_y*sin(roll_angle_rad);
  z_comp = acc_z*cos(roll_angle_rad)*cos(pitch_angle_rad);
  z_rate += (x_comp + y_comp + z_comp - 1) * 981 * dT; // * dT; cm/s
}

void imu_init(void)
{
  // Initialize I2C
  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  Serial.println("Configuring IMU");
  // Wake up MPU and set clock source to internal 8MHz oscillator
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Enable digital low-pass filter; Set bandwith to 10Hz
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  // Configure Accelerometer; set full scale range to +/- 8g
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  // Configure gyroscope; set full scale range to +/- 500 deg/s
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  // Calibrate Rate signals
  Serial.println("Calibrating rate signals...");
  for (int i = 0; i < RATE_CALIBRATION_ITERATIONS; ++i) {
    imu_update1();
    imu_update2();
    rate_roll_calibration += rate_roll;
    rate_pitch_calibration += rate_pitch;
    rate_yaw_calibration += rate_yaw;
    delay(1);
  }
  rate_roll_calibration = (rate_roll_calibration / RATE_CALIBRATION_ITERATIONS);
  rate_pitch_calibration = (rate_pitch_calibration / RATE_CALIBRATION_ITERATIONS);
  rate_yaw_calibration = (rate_yaw_calibration / RATE_CALIBRATION_ITERATIONS);
  gyro_is_calibrated = true;

  // Initialize output filters
  Serial.println("Initializing Kalman Filters...");
  kalman_filter_init(&roll_angle);
  kalman_filter_init(&pitch_angle);

  // Get offsets for roll and pitch values
  Serial.println("Getting roll and pitch offsets.");
  offset_init();
}

float imu_get_roll_rate(void)
{
  return rate_roll;
}

float imu_get_pitch_rate(void)
{
  return rate_pitch;
}

float imu_get_yaw_rate(void)
{
  return rate_yaw;
}

float imu_get_roll_angle(void)
{
  return(roll_angle.kalman_value - get_roll_offset());
}

float imu_get_pitch_angle(void)
{
  return(pitch_angle.kalman_value - get_pitch_offset());
}

float imu_get_z_rate(void)
{
  return z_rate;
}