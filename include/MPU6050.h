#pragma once

// MPU6050 datasheet : MPU-6050 :Register Map and Descriptions Revision 4.2 ==> https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf

#include "Wire.h"

const float PI = 3.14159265358979323846f;
const float RAD_TO_DEG = 180.0 / PI;

#define MPU_ADDR        0x68  // Default I2C address of the MPU6050
#define PWR_MGMT_1      0x6B  // Power Management 1 register
#define ACCEL_CONFIG    0x1C  // Accelerometer Configuration register
#define GYRO_CONFIG     0x1B  // Gyroscope Configuration register
#define CALIBRATION_SAMPLES 1000

const float GYRO_SENSITIVITY = 65.5;   // for ±500 dps
const float ACCEL_SENSITIVITY = 4096.0; // for ±8g

//----------------------------------------------------------------
float GyroX = 0, GyroY = 0, GyroZ = 0;
float Gyro_X_angle = 0, Gyro_Y_angle = 0, Gyro_Z_angle = 0;
float Gyro_X_error = 0, Gyro_Y_error = 0, Gyro_Z_error = 0;


float AccelX = 0, AccelY = 0, AccelZ = 0;
float Accel_X_angle = 0, Accel_Y_angle = 0; Accel_Z_angle = 0;
float Accel_X_error = 0, Accel_Y_error = 0; Accel_Z_error = 0;

float angle_x = 0, angle_y = 0, angle_z = 0;
float t = 0, prev = 0;
float elapsed_time = 0;

void mpu_init();
void IMU_calibration();
void Acc_data();
void Gyro_data();
void get_angle();

#endif