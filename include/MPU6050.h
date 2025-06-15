#pragma once

// MPU6050 datasheet : MPU-6050 :Register Map and Descriptions Revision 4.2 ==> https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
#include <Arduino.h>
#include <Wire.h>

#define MPU_ADDR        0x68  // Default I2C address of the MPU6050
#define PWR_MGMT_1      0x6B  // Power Management 1 register
#define ACCEL_CONFIG    0x1C  // Accelerometer Configuration register
#define GYRO_CONFIG     0x1B  // Gyroscope Configuration register
#define CALIBRATION_SAMPLES 1000

const float GYRO_SENSITIVITY = 65.5;   // for ±500 dps
const float ACCEL_SENSITIVITY = 4096.0; // for ±8g

//----------------------------------------------------------------
extern float gyro_raw[3];
extern float gyro_angle[3];
extern float gyro_bias[3];

extern float accel_raw[3];
extern float accel_angle[3];
extern float accel_bias[3];

extern float fused_angle_x, fused_angle_y, fused_angle_z;
 
extern float current_time, last_time;
extern float delta_time;

void mpu_init();
void IMU_calibration();
void Acc_data();
void Gyro_data();
void get_angle();