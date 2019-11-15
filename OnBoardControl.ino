#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS1.h>
#include <Wire.h>
#include <WireIMXRT.h>
#include <WireKinetis.h>
#include <EEPROM.h>
#define SDA0 18
#define SDA1 17
#define SCL0 19
#define SCL1 16
#define TVC1 2
#define TVC2 3
#define iris 1
#define reac 0
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(); // i2c sensor
const float g=9.80665;   //m/s^2
const float wndspd=0.0;    //m/s


void setup() {
  Serial.begin(115200);
  pinMode(SDA0,INPUT);    //SDA0
  pinMode(SDA1,INPUT);    //SDA1
  pinMode(SCL0,INPUT);    //SCL0
  pinMode(SCL1,INPUT);    //SCL1
  pinMode(reac,OUTPUT);    //reaction wheel motor
  pinMode(iris,OUTPUT);    //iris servo
  pinMode(TVC1,OUTPUT);    //TVC servo 1
  pinMode(TVC2,OUTPUT);    //TVC servo 2
  lsm.begin(SDA0);
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);  //setup acceleration range --> 2g's
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);   //setup magnetometer range --> 4 Gauss
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);    //setup gryroscope range --> 245 degrees per second
  
  

  float A[49] = { 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
                  0.0000, 0.0000, 0.0000, 0.0000, 0.5000, 0.0000, 0.0000,
                  0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.5000, 0.0000,
                  0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.5000,
                  0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
                  0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
                  0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000 };
  
  float B[21] = { 0.0000, 0.0000, 0.0000,
                  0.0000, 0.0000, 0.0000,
                  0.0000, 0.0000, 0.0000,
                  0.0000, 0.0000, 0.0000,
                  986.6935, 0.0000, 0.0000,
                  0.0000, 82.9891, 0.0000,
                  0.0000, 0.0000, 82.9891 };

  float K[21] = { 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
                  0.0000, 0.4000, 0.5000, 0.5000, 0.5000, 0.5000, 0.5000,
                  0.0000, 1.5000, 0.0500, 0.2500, 0.2500, 0.1500, 0.3500 };   //tentative gain matrix. Negative real eigenvalues for A-BK. Can be tuned further if needed.
}

void loop() {
  sensors_event_t accel, mag, gyro, temp;   //read IMU data
  lsm.getEvent(&accel, &mag, &gyro, &temp);   //takes snapshot at time t(i)
  w_x = gyro.gyro.x;   //angular velocity around x-axis
  w_y = gyro.gyro.y;   //angular velocity around y-axis
  w_z = gyro.gyro.z;   //angular velocity around z-axis
  mag_x = mag.magnetic.x;   //x-comp magnetic field
  mag_y = mag.magnetic.y;   //y-comp
  mag_z = mag.magnetic.z;   //z-comp
  
  

}
