#include "MPU6050.h"

enum{X, Y, Z};

float gyro_raw[3] = {0, 0, 0};
float gyro_angle[3] = {0, 0, 0};
float gyro_bias[3] = {0, 0, 0};

float accel_raw[3] = {0, 0, 0};
float accel_angle[3] = {0, 0, 0};
float accel_bias[3] = {0, 0, 0};

float fused_angle_x = 0, fused_angle_y = 0, fused_angle_z = 0;


float current_time = 0, last_time = 0;
float delta_time = 0;


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

    gyro_bias[X] += Gyro_X / GYRO_SENSITIVITY;
    gyro_bias[Y] += Gyro_Y / GYRO_SENSITIVITY;
    gyro_bias[Z] += Gyro_Z / GYRO_SENSITIVITY;
  }
  
  gyro_bias[X] /= CALIBRATION_SAMPLES;
  gyro_bias[Y] /= CALIBRATION_SAMPLES;
  gyro_bias[Z] /= CALIBRATION_SAMPLES;

  for(int i = 0; i < CALIBRATION_SAMPLES; i++)
  {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);                 
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)6, true);  

    float Accel_X = (Wire.read() << 8 | Wire.read()) / ACCEL_SENSITIVITY; 
    float Accel_Y = (Wire.read() << 8 | Wire.read()) / ACCEL_SENSITIVITY;
    float Accel_Z = (Wire.read() << 8 | Wire.read()) / ACCEL_SENSITIVITY;

    accel_bias[X] += atan2( Accel_Y, sqrt(Accel_X*Accel_X + Accel_Z*Accel_Z)) * RAD_TO_DEG;
    accel_bias[Y] += atan2(-Accel_X, sqrt(Accel_Y*Accel_Y + Accel_Z*Accel_Z)) * RAD_TO_DEG;

     // Z-axis bias = how much it differs from 1g, If you're working in m/s² instead of g, expectedZ = 9.81.
    float expectedZ = 1.0; // 1g in normalized units
    accel_bias[Z] += (Accel_Z - expectedZ);
  }
  
  accel_bias[X] /= CALIBRATION_SAMPLES;
  accel_bias[Y] /= CALIBRATION_SAMPLES;
  accel_bias[Z] /= CALIBRATION_SAMPLES;

}

void Acc_data()
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);                 
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)6, true); 

  accel_raw[X] = (float)((int16_t)(Wire.read() << 8 | Wire.read())) / ACCEL_SENSITIVITY ; // reading the high and low acceleration data, by or op and shift op 
  accel_raw[Y] = (float)((int16_t)(Wire.read() << 8 | Wire.read())) / ACCEL_SENSITIVITY ;
  accel_raw[Z] = (float)((int16_t)(Wire.read() << 8 | Wire.read())) / ACCEL_SENSITIVITY ;

  accel_raw[X] -= accel_bias[X];
  accel_raw[Y] -= accel_bias[Y];
  accel_raw[Z] -= accel_bias[Z];
}

void Gyro_data()
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43);                 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6);

  gyro_raw[X] = (float)((int16_t)(Wire.read() << 8 | Wire.read())) / GYRO_SENSITIVITY; // reading the high and low gyro data, by or op and shift op 
  gyro_raw[Y] = (float)((int16_t)(Wire.read() << 8 | Wire.read())) / GYRO_SENSITIVITY; // to obtain the real values which are degrees per sec we have to divide by 65.5 for the range ±500 °/s (from the datasheet)
  gyro_raw[Z] = (float)((int16_t)(Wire.read() << 8 | Wire.read())) / GYRO_SENSITIVITY; // and we substract the error for the calibration

  gyro_raw[X] -= gyro_bias[X];
  gyro_raw[Y] -= gyro_bias[Y];
  gyro_raw[Z] -= gyro_bias[Z];
}

void get_angle() 
{
  Gyro_data(); 
  Acc_data();
      // euler angle formula
  accel_angle[X] = atan2( accel_raw[Y], sqrt(accel_raw[X]*accel_raw[X] + accel_raw[Z]*accel_raw[Z])) * RAD_TO_DEG;
  accel_angle[Y] = atan2(-accel_raw[X], sqrt(accel_raw[Y]*accel_raw[Y] + accel_raw[Z]*accel_raw[Z])) * RAD_TO_DEG;

  current_time = millis();
  delta_time = (current_time - last_time) / 1000.0; // to get time in seconds
  last_time = current_time;

  // integration applied each loop to get the degree values (multiplication)
  gyro_angle[X] = gyro_raw[X] * delta_time; 
  gyro_angle[Y] = gyro_raw[Y] * delta_time; 
  gyro_angle[Z] = gyro_raw[Z] * delta_time; 

  // for better results, we apply a complementary filter
  fused_angle_x = 0.98 * (fused_angle_x + gyro_angle[X]) + 0.02 * accel_angle[X]; // roll
  fused_angle_y = 0.98 * (fused_angle_y + gyro_angle[Y]) + 0.02 * accel_angle[Y]; // pitch
  fused_angle_z += gyro_angle[Z];
}