#include <MatrixMath.h>
#include <Adafruit_BMP3XX.h>
#include <bmp3.h>
#include <bmp3_defs.h>
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
#define TVC1 13
#define TVC2 14
#define iris 15
#define reac 23
#define sealvl_P (69)     //Pa                                //**CHANGE ON DAY OF LAUNCH**
#define MAX_EEPROM_ADDR 65536
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(); // i2c sensor
Adafruit_BMP3XX bmp; // I2C
const float g=9.80665;   //m/s^2
const float wndspd=0.0;    //m/s                              //**CHANGE ON DAY OF LAUNCH**
//equilibrium state variables
const float w_xe = 0.0;   //rad/s
const float w_ye = 0.0;
const float w_ze = 0.0;
const float q_0e = 1.0;   //rad
const float q_1e = 0.0;
const float q_2e = 0.0;
const float q_3e = 0.0;
bool read_mode = false;
float u[3];         //initialize input vector
float x_c[7];       //initialize state vector
float R_zx[9];      //initialize Rz*Rx
float R_zxz[9];     //initialize full Rz*Rx*Rz rotation matrix
float K[21] = { 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
                  0.0000, 0.4000, 0.5000, 0.5000, 0.5000, 0.5000, 0.5000,
                  0.0000, 1.5000, 0.0500, 0.2500, 0.2500, 0.1500, 0.3500 };   //tentative gain matrix. Negative real eigenvalues for A-BK. Can be tuned further if needed.

int curr_address = 0;
int num_logs = 0;
int data_size = 0;
meta_data_node data_info;
int log_count = 0;

struct dataNode{
  float roll;
  float yaw;
  float roll;
  float h;
  //Add whatever other data to be logged 
};
typedef struct dataNode data_node;

struct metaDataNode{
  int num_logs;
  int data_size;
};
typedef struct metaDataNode meta_data_node;

void setup() {
  Serial.begin(115200);
  pinMode(SDA0,INPUT);    //SDA0    //altimeter
  pinMode(SDA1,INPUT);    //SDA1    //IMU
  pinMode(SCL0,INPUT);    //SCL0    //altimeter
  pinMode(SCL1,INPUT);    //SCL1    //IMU
  pinMode(reac,OUTPUT);    //reaction wheel motor
  pinMode(iris,OUTPUT);    //iris servo
  pinMode(TVC1,OUTPUT);    //TVC servo 1
  pinMode(TVC2,OUTPUT);    //TVC servo 2
  lsm.begin(SDA0);
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);  //setup acceleration range --> 2g's
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);   //setup magnetometer range --> 4 Gauss
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);    //setup gryroscope range --> 245 degrees per second

  if(!read_mode){
    data_size = sizeof(data_node);
    curr_address = sizeof(meta_data_node); //Start data logging after meta_data node
    data_info.data_size = data_size;
    data_info.num_logs = 0;
    EEPROM.put(0, data_info);
  }else{
    EEPROM.get(0,data_info);
    data_size = data_info.data_size;
    curr_address = data_size;
    log_count = data_info.num_logs;
  }

  //float A[49] = { 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
  //                0.0000, 0.0000, 0.0000, 0.0000, 0.5000, 0.0000, 0.0000,
  //                0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.5000, 0.0000,
  //                0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.5000,
  //                0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
  //                0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
  //                0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000 };
  
 //float B[21] = { 0.0000, 0.0000, 0.0000,
 //                0.0000, 0.0000, 0.0000,
 //                 0.0000, 0.0000, 0.0000,
 //                 0.0000, 0.0000, 0.0000,
 //                 986.6935, 0.0000, 0.0000,
 //                 0.0000, 82.9891, 0.0000,
 //                 0.0000, 0.0000, 82.9891 };

  
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
  P = bmp.pressure;         //Pressure in Pa
  alt = bmp.readAltitude(sealvl_P);   //altitude in meters
  //difference between real and desired ang. vel.
  w_xc = w_x - w_xe;    
  w_yc = w_y - w_ye;
  w_zc = w_z - w_ze;
  
  //insert filter here
  //using Kalman filter
  //then integrate angular velocities to get angles

  //generate rotation matrices based on integrated angular velocities
  R_z1 = {cos(theta1), -sin(theta1), 0,
          sin(theta1), cos(theta1), 0,
          0, 0, 1};
  R_x2 = {1, 0, 0,
          0, cos(theta2), sin(theta2),
          0, sin(theta2), cos(theta2)};
  R_z3 = {cos(theta3), -sin(theta3), 0,
          sin(theta3), cos(theta3), 1};
  void Multiply(R_z1,R_x2, 3, 3, 3, R_zx);
  void Multiply(R_zx, R_z3, 3, 3, 3, R_zxz);      //Euler angle rotation matrix
  //calculate quaternions based on current orientation angles
  q0 = 0.5*sqrt(R_zxz[0] + R_zxz[4] + R_zxz[8] + 1);
  q1 = 0.25*(1/q0)*(R_zxz[7] - R_zxz[5]);
  q2 = 0.25*(1/q0)*(R_zxz[2] - R_zxz[6]);
  q3 = 0.25*(1/q0)*(R_zxz[3] - R_zxz[1]);
  //find difference between current and desired orientation
  q_0c = q0 - q_0e;
  q_1c = q1 - q_1e;
  q_2c = q2 - q_2e;
  q_3c = q3 - q_3c;
  //populate state vector
  x_c[0] = q_0c;
  x_c[1] = q_1c;
  x_c[2] = q_2c;
  x_c[3] = q_3c;
  x_c[4] = w_xc;
  x_c[5] = w_yc;
  x_c[6] = w_zc;
  x_c = -1*x_c;
  void Multiply(K,x_c,3,7,1,u);

  //need to figure out how a PWM value maps to an angular value

  //also need to determine how to write to EEPROM
  if(!read_mode){ //If in operation mode, only writing to EEPROM will occur
    data_node to_log;
    to_log.roll = roll;
    to_log.yaw = yaw;
    to_log.pitch = pitch;
    to_log.h = h;
    if(curr_address <= (MAX_EEPROM_ADDR - data_size)){
      EEPROM.put(curr_address, to_log);
      curr_address += data_size;
      data_info.num_logs++;
    }else{
      //EEPROM overflow whoops handle somehow
    }
  }else{ //If in collection mode, only reading from EEPROM will occur
    if(log_count > 0){
      meta_data_node data_buffer;
      EEPROM.get(curr_address, data_buffer);
      float roll_data = data_buffer.roll;
      float yaw_data = data_buffer.yaw;
      float pitch_data = data_buffer.pitch;
      float z_data = data_buffer.z;
      //Eventually will have to serially communicate data to a computer, writing to some output file
      curr_address += data_info.data_size;
      log_count--;
    }else{
      //Done with data retrieval, close file on computer
    }
    
  }
  
  //probably need a delay in here
}

void 
