/**
 * @file main.cpp
 * @author Mounir Dahmane
 * @brief Main execution loop for MPU6050 IMU data fusion and telemetry.
 * * This module handles the high-frequency sampling of the MPU6050 sensor 
 * and provides filtered Euler angles (Roll, Pitch, Yaw) via a non-blocking 
 * serial interface.
 */

#include "MPU6050.h"

// --- Telemetry Configuration ---
unsigned long last_print_time = 0;
const unsigned long print_interval = 100; // Output rate: 10Hz (100ms)

/**
 * @brief Standard Arduino setup function.
 * Initializes hardware peripherals and performs IMU bias calibration.
 */
void setup() {
    Serial.begin(115200);
    
    // Initialize I2C and MPU6050 registers
    mpu_init();
    
    // Critical: Calculate Gyro and Accel offsets. 
    // Device must remain stationary during this phase.
    IMU_calibration();
    
    Serial.println(F("--- MPU6050 System Online ---"));
    
    // Synchronize timing for the first integration step
    last_time = micros(); 
}

/**
 * @brief Main execution loop.
 * Runs at the maximum CPU frequency to maintain high integration accuracy.
 */
void loop() {
    // 1. SENSOR FUSION DATA ACQUISITION
    // get_angle() calculates delta_time and updates fused_angle variables.
    // High loop frequency is vital to minimize gyro integration drift.
    get_angle();

    // 2. NON-BLOCKING TELEMETRY
    // We use a timer check instead of delay() to ensure get_angle() 
    // is never interrupted, maintaining real-time performance.
    if (millis() - last_print_time >= print_interval) {
        last_print_time = millis();

        // Display Filtered Euler Angles
        Serial.print(F("R: ")); Serial.print(fused_angle_x, 2);
        Serial.print(F(" | P: ")); Serial.print(fused_angle_y, 2);
        Serial.print(F(" | Y: ")); Serial.print(fused_angle_z, 2);
        
        // Loop frequency diagnostics (f = 1 / dt)
        // Values > 200Hz are recommended for stable flight control.
        Serial.print(F(" | Freq: "));
        Serial.print(1.0f / delta_time, 0);
        Serial.println(F(" Hz"));
    }
}