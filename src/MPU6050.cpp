#include "MPU6050.h"

void mpu_init()
{
  Wire.begin();                       // begin the Wire communication

  Wire.beginTransmission(MPU_ADDR);   // send the slave address 
  Wire.write(PWR_MGMT_1);             // reset ==> placing 0 into the 6B register
  Wire.write(0x00);                   // Wake up MPU6050
  Wire.endTransmission(false);        // This allows repeated starts (important for I²C efficiency)

  delay(100);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);               // This register configures the external Frame Synchronization (FSYNC) pin sampling 
  Wire.write(0x05);               // and the Digital Low Pass Filter (DLPF) setting for both the gyroscopes and accelerometers                
  Wire.endTransmission(false);  

  // Gyro Configuration
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(GYRO_CONFIG);        // Gyro specfic configuration register  
  Wire.write(0x08);              // set the range to ±500 °/s ==> // FS_SEL = 1
  Wire.endTransmission(false);
  
  // Accel Configuration
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_CONFIG);       // Accel specfic configuration register  
  Wire.write(0x10);              // set the range to ±8g
  Wire.endTransmission(false);

  delay(20);
}

void IMU_calibration()
{
  for(int i = 0; i < CALIBRATION_SAMPLES; i++)
  {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43);                 
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6);  

    float Gyro_X = Wire.read() << 8 | Wire.read(); // reading the high and low gyro data, by or and shift operations 
    float Gyro_Y = Wire.read() << 8 | Wire.read();
    float Gyro_Z = Wire.read() << 8 | Wire.read();

    Gyro_X_error += Gyro_X / GYRO_SENSITIVITY;
    Gyro_Y_error += Gyro_Y / GYRO_SENSITIVITY;
    Gyro_Z_error += Gyro_Z / GYRO_SENSITIVITY;
  }
  
  Gyro_X_error /= CALIBRATION_SAMPLES;
  Gyro_Y_error /= CALIBRATION_SAMPLES;
  Gyro_Z_error /= CALIBRATION_SAMPLES;

  for(int i = 0; i < CALIBRATION_SAMPLES; i++)
  {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);                 
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);   

    float Accel_X = (Wire.read() << 8 | Wire.read()) / ACCEL_SENSITIVITY; 
    float Accel_Y = (Wire.read() << 8 | Wire.read()) / ACCEL_SENSITIVITY;
    float Accel_Z = (Wire.read() << 8 | Wire.read()) / ACCEL_SENSITIVITY;

    Accel_X_error += atan2( Accel_Y, sqrt(Accel_X*Accel_X + Accel_Z*Accel_Z)) * RAD_TO_DEG;
    Accel_Y_error += atan2(-Accel_X, sqrt(Accel_Y*Accel_Y + Accel_Z*Accel_Z)) * RAD_TO_DEG;

     // Z-axis bias = how much it differs from 1g, If you're working in m/s² instead of g, expectedZ = 9.81.
    float expectedZ = 1.0; // 1g in normalized units
    Accel_Z_error += (Accel_Z - expectedZ);
  }
  
  Accel_X_error /= CALIBRATION_SAMPLES;
  Accel_Y_error /= CALIBRATION_SAMPLES;
  Accel_Z_error /= CALIBRATION_SAMPLES;

}

void Acc_data()
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);                 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);  

  AccelX = (Wire.read() << 8 | Wire.read()) / ACCEL_SENSITIVITY ; // reading the high and low acceleration data, by or op and shift op 
  AccelY = (Wire.read() << 8 | Wire.read()) / ACCEL_SENSITIVITY ;
  AccelZ = (Wire.read() << 8 | Wire.read()) / ACCEL_SENSITIVITY ;

  AccelX -= Accel_X_error;
  AccelY -= Accel_Y_error;
  AccelZ -= Accel_Z_error;
}

void Gyro_data()
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43);                 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6);

  GyroX = Wire.read() << 8 | Wire.read(); // reading the high and low gyro data, by or op and shift op 
  GyroY = Wire.read() << 8 | Wire.read();
  GyroZ = Wire.read() << 8 | Wire.read();

  GyroX = (GyroX / GYRO_SENSITIVITY) ;    // to obtain the real values which are degrees per sec we have to divide by 65.5 for the range ±500 °/s (from the datasheet)
  GyroY = (GyroY / GYRO_SENSITIVITY) ;    // and we substract the error for the calibration
  GyroZ = (GyroZ / GYRO_SENSITIVITY) ; 
  
  GyroX -= Gyro_X_error;
  GyroY -= Gyro_Y_error;
  GyroZ -= Gyro_Z_error;
}

void get_angle() 
{
  Gyro_data(); 
  Acc_data();
      // euler angle formula
  Accel_X_angle = atan2( AccelY, sqrt(AccelX*AccelX + AccelZ*AccelZ)) * RAD_TO_DEG;
  Accel_Y_angle = atan2(-AccelX, sqrt(AccelY*AccelY + AccelZ*AccelZ)) * RAD_TO_DEG;

  prev = t;
  t = millis();
  elapsed_time = (t - prev) / 1000.0; // to get time in seconds

  // integration applied each loop to get the degree values (multiplication)
  Gyro_X_angle = GyroX * elapsed_time; 
  Gyro_Y_angle = GyroY * elapsed_time; 
  Gyro_Z_angle = GyroZ * elapsed_time; 

  // for better results, we apply a complementary filter
  angle_x = 0.98 * (angle_x + Gyro_X_angle) + 0.02 * Accel_X_angle; // roll
  angle_y = 0.98 * (angle_y + Gyro_Y_angle) + 0.02 * Accel_Y_angle; // pitch
  angle_z += Gyro_Z_angle;
}