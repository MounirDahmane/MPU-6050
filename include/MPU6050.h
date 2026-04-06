/**
 * @file MPU6050.h
 * @author Mounir Dahmane
 * @brief Header file for MPU6050 6-DOF IMU driver and sensor fusion.
 * * Includes register definitions, sensitivity constants, and global 
 * telemetry variables for Motion Processing Unit (MPU) interfacing.
 * * Datasheet: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>

// --- Register Definitions ---
#define MPU_ADDR        0x68  ///< Default I2C address of the MPU6050
#define PWR_MGMT_1      0x6B  ///< Power Management 1 (Wake/Reset)
#define ACCEL_CONFIG    0x1C  ///< Accelerometer Configuration (Scale)
#define GYRO_CONFIG     0x1B  ///< Gyroscope Configuration (Scale)

// --- Calibration Configuration ---
#define CALIBRATION_SAMPLES 1000 ///< Number of samples used to calculate static bias

// --- Sensitivity Constants ---
/** @brief Gyro sensitivity for ±500 dps range (65.5 LSB per °/s) */
const float GYRO_SENSITIVITY = 65.5;   
/** @brief Accel sensitivity for ±8g range (4096 LSB per g) */
const float ACCEL_SENSITIVITY = 4096.0; 

// --- Global Telemetry Data ---
// Index notation: [0]=X, [1]=Y, [2]=Z

extern float gyro_raw[3];   ///< Raw compensated angular velocity (°/s)
extern float gyro_bias[3];  ///< Calculated gyroscope zero-drift offsets

extern float accel_raw[3];  ///< Raw compensated linear acceleration (g)
extern float accel_bias[3]; ///< Calculated accelerometer static offsets

/** @brief Filtered Euler angles calculated via Complementary Filter */
extern float fused_angle_x, fused_angle_y, fused_angle_z;
 
/** @brief High-precision timing variables for discrete integration */
extern unsigned long current_time, last_time; 
extern float delta_time;

// --- Function Prototypes ---

/** @brief Initialize I2C and configure MPU6050 operational modes */
void mpu_init();

/** @brief Sample and average static offsets for sensor compensation */
void IMU_calibration();

/** @brief Acquisition and scaling of raw accelerometer data */
void Acc_data();

/** @brief Acquisition and scaling of raw gyroscope data */
void Gyro_data();

/** @brief Execute sensor fusion to estimate spatial orientation */
void get_angle();