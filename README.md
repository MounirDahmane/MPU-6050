# MPU6050 Raw I2C Interface (No Libraries)

This project provides a clean, low-level interface to the MPU6050 sensor (accelerometer + gyroscope) using raw I2C communication. It performs data acquisition, calibration, and sensor fusion.

## Features

- **Direct I2C communication** (no third-party libraries)
- **Manual register setup** for accelerometer and gyroscope
- **Calibration routines** for bias/error correction
- **Complementary filter** to estimate pitch and roll angles
- Compatible with **Arduino** or similar microcontrollers with `Wire.h`

## Project Structure

- `MPU6050.h`: Header with all constants, globals, and function declarations
- `MPU6050.cpp`: Implementation file with:
  - Sensor initialization (`mpu_init`)
  - Calibration (`IMU_calibration`)
  - Data reading (`Acc_data`, `Gyro_data`)
  - Angle estimation (`get_angle`)

## Usage

1. **Include the files** in your Arduino or C++ project.
2. **Call** the following functions in order:
   ```cpp
   mpu_init();           // Initialize sensor
   IMU_calibration();    // Calibrate accelerometer and gyroscope
   get_angle();          // Call it in your loop, it Updates global pitch/roll values (angle_x, angle_y)

