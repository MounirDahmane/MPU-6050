/**
 * @file MPU6050.cpp
 * @author Mounir Dahmane
 * @brief Implementation of MPU6050 driver, calibration, and sensor fusion.
 * * This file handles I2C communication, raw data processing with bias 
 * compensation, and a Complementary Filter to estimate Roll, Pitch, and Yaw.
 */

#include "MPU6050.h"

// --- Global Data Structures ---
enum {X, Y, Z};

float gyro_raw[3] = {0, 0, 0};
float gyro_bias[3] = {0, 0, 0};

float accel_raw[3] = {0, 0, 0};
float accel_bias[3] = {0, 0, 0};

float fused_angle_x = 0, fused_angle_y = 0, fused_angle_z = 0;

unsigned long current_time = 0, last_time = 0;
float delta_time = 0;

/**
 * @brief Configures MPU6050 registers for high-performance IMU sampling.
 * Sets I2C to 400kHz, wakes the device, and configures Full Scale Ranges.
 */
void mpu_init() {
    Wire.begin();
    Wire.setClock(400000); // 400kHz Fast Mode I2C

    // Wake up the MPU6050 (Internal 8MHz oscillator)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(PWR_MGMT_1);
    Wire.write(0x00); 
    Wire.endTransmission();

    // DLPF Configuration: 42Hz Bandwidth to filter out high-frequency motor noise
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1A); 
    Wire.write(0x03); 
    Wire.endTransmission();

    // Gyroscope Configuration: ±500 deg/s
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(GYRO_CONFIG);
    Wire.write(0x08);
    Wire.endTransmission();

    // Accelerometer Configuration: ±8g
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(ACCEL_CONFIG);
    Wire.write(0x10);
    Wire.endTransmission();
}

/**
 * @brief Performs static bias calibration for Gyro and Accelerometer.
 * @note The IMU must be perfectly level and motionless during execution.
 */
void IMU_calibration() {
    Serial.println(F("Starting IMU Calibration... Keep level."));
    float sum_accel[3] = {0, 0, 0};
    float sum_gyro[3] = {0, 0, 0};

    for(int i = 0; i < CALIBRATION_SAMPLES; i++) {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x3B);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU_ADDR, 14); // Burst read: Accel(6), Temp(2), Gyro(6)

        sum_accel[X] += (int16_t)(Wire.read() << 8 | Wire.read());
        sum_accel[Y] += (int16_t)(Wire.read() << 8 | Wire.read());
        sum_accel[Z] += (int16_t)(Wire.read() << 8 | Wire.read());
        
        Wire.read(); Wire.read(); // Discard Temperature data

        sum_gyro[X] += (int16_t)(Wire.read() << 8 | Wire.read());
        sum_gyro[Y] += (int16_t)(Wire.read() << 8 | Wire.read());
        sum_gyro[Z] += (int16_t)(Wire.read() << 8 | Wire.read());
        delay(3);
    }

    // Average raw offsets
    accel_bias[X] = sum_accel[X] / CALIBRATION_SAMPLES;
    accel_bias[Y] = sum_accel[Y] / CALIBRATION_SAMPLES;
    // Expected Z is 1g (4096 LSB at ±8g). Offset = Reading - 4096.
    accel_bias[Z] = (sum_accel[Z] / CALIBRATION_SAMPLES) - 4096.0f; 

    gyro_bias[X] = (sum_gyro[X] / CALIBRATION_SAMPLES) / GYRO_SENSITIVITY;
    gyro_bias[Y] = (sum_gyro[Y] / CALIBRATION_SAMPLES) / GYRO_SENSITIVITY;
    gyro_bias[Z] = (sum_gyro[Z] / CALIBRATION_SAMPLES) / GYRO_SENSITIVITY;

    // Initialize fused angles with initial gravity vector to prevent filter "climb"
    Acc_data();
    fused_angle_x = atan2(accel_raw[Y], sqrt(accel_raw[X]*accel_raw[X] + accel_raw[Z]*accel_raw[Z])) * RAD_TO_DEG;
    fused_angle_y = atan2(-accel_raw[X], sqrt(accel_raw[Y]*accel_raw[Y] + accel_raw[Z]*accel_raw[Z])) * RAD_TO_DEG;
}

/**
 * @brief Reads raw accelerometer data and applies bias compensation.
 */
void Acc_data() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6);

    accel_raw[X] = ((int16_t)(Wire.read() << 8 | Wire.read()) - accel_bias[X]) / ACCEL_SENSITIVITY;
    accel_raw[Y] = ((int16_t)(Wire.read() << 8 | Wire.read()) - accel_bias[Y]) / ACCEL_SENSITIVITY;
    accel_raw[Z] = ((int16_t)(Wire.read() << 8 | Wire.read()) - accel_bias[Z]) / ACCEL_SENSITIVITY;
}

/**
 * @brief Reads raw gyroscope data and applies bias compensation.
 */
void Gyro_data() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6);

    gyro_raw[X] = ((int16_t)(Wire.read() << 8 | Wire.read()) / GYRO_SENSITIVITY) - gyro_bias[X];
    gyro_raw[Y] = ((int16_t)(Wire.read() << 8 | Wire.read()) / GYRO_SENSITIVITY) - gyro_bias[Y];
    gyro_raw[Z] = ((int16_t)(Wire.read() << 8 | Wire.read()) / GYRO_SENSITIVITY) - gyro_bias[Z];
}

/**
 * @brief Primary sensor fusion routine.
 * Integrates gyro rates and fuses them with accelerometer angles 
 * via a Complementary Filter (Alpha = 0.98).
 */
void get_angle() {
    Acc_data();
    Gyro_data();

    // 1. Calculate static tilt angles from accelerometer
    float accel_angle_x = atan2(accel_raw[Y], sqrt(accel_raw[X]*accel_raw[X] + accel_raw[Z]*accel_raw[Z])) * RAD_TO_DEG;
    float accel_angle_y = atan2(-accel_raw[X], sqrt(accel_raw[Y]*accel_raw[Y] + accel_raw[Z]*accel_raw[Z])) * RAD_TO_DEG;

    // 2. High-precision delta_time for integration
    current_time = micros();
    delta_time = (current_time - last_time) / 1000000.0f;
    last_time = current_time;

    // 3. COMPLEMENTARY FILTER: Angle = 0.98 * (Angle + Gyro_Rate * dt) + 0.02 * (Accel_Angle)
    // The filter trusts the gyro for fast motion and the accelerometer for long-term stability.
    fused_angle_x = 0.98f * (fused_angle_x + gyro_raw[X] * delta_time) + 0.02f * accel_angle_x;
    fused_angle_y = 0.98f * (fused_angle_y + gyro_raw[Y] * delta_time) + 0.02f * accel_angle_y;
    
    // 4. Z-Axis (Yaw) Integration: Pure integration, subject to gyro drift.
    fused_angle_z += gyro_raw[Z] * delta_time;
}