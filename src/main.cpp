#include "Mpu6050.h"


void setup() {
  Serial.begin(115200);
  mpu_init();
  IMU_calibration();
  Serial.println("MPU6050 initialized and calibrated.");
  Serial.println("Starting main loop...");
}

void loop() {
  get_angle();
  Serial.print("Angle X: ");
  Serial.print(fused_angle_x);
  Serial.print(" | Angle Y: ");
  Serial.print(fused_angle_y);
  Serial.print(" | Angle Z: ");
  Serial.println(fused_angle_z);
  delay(100); // Adjust the delay as needed for your application
  // This delay allows for smoother output and prevents flooding the Serial Monitor
}

