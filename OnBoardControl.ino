#include <Servo.h>
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
#define LOG_SKIP 100

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
float dt = something;       //find this out
float theta[3];             //Euler angle vector
float theta_dot[3];         //time derivatives of Euler angles
bool read_mode = false;  //SET TO FALSE IF DOING DATA GENERATION, TRUE IF DOING DATA COLLECTION
float u[3];         //initialize input vector
float x_c[7];       //initialize state vector
float R_zy[9];      //initialize Rz*Rx
float R_zyx[9];     //initialize full Rz*Rx*Rz rotation matrix     
float N1[9];         //part of ang vel to ang rates matrix
float N11[3];
float N2[9];
float N[9];
float Ninv[9];
float z_id[3] = {0, 
                 0,
                 1};
float y_id[3] = {0,
                 1,
                 0};
float x_id[3] = {1,
                 0,
                 0};
float K[21] = { 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
                  0.0000, 0.4000, 0.5000, 0.5000, 0.5000, 0.5000, 0.5000,
                  0.0000, 1.5000, 0.0500, 0.2500, 0.2500, 0.1500, 0.3500 };   //tentative gain matrix. Negative real eigenvalues for A-BK. Can be tuned further if needed.

int curr_address = 0;
int num_logs = 0;
int data_size = 0;
meta_data_node data_info;
int log_count = 0;
unsigned long loop_count = 0;

struct dataNode{
  float roll; //Roll of body
  float yaw; //Yaw of body
  float roll; //Roll of body
  float h; //Altitude of body
  unsigned long t; //Time of measurement
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
  if(!lsm.begin(SDA0)){
    Serial.println("IMU didn't open up right. Infinite looping now.");
    while(1);
  }
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);  //setup acceleration range --> 2g's
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);   //setup magnetometer range --> 4 Gauss
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);    //setup gryroscope range --> 245 degrees per second

  if (!bmp.begin()) {
    Serial.println("Altimeter didnt' open up right. Infinite looping now.");
    while (1);
  }

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
    Serial.print("Time(ms), Yaw(deg), Pitch(deg), Roll(deg)");

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
  w[0] = gyro.gyro.x - w_xe;   //angular velocity around x-axis
  w[1] = gyro.gyro.y - w_ye;   //angular velocity around y-axis
  w[2] = gyro.gyro.z - wze;   //angular velocity around z-axis
  mag_x = mag.magnetic.x;   //x-comp magnetic field
  mag_y = mag.magnetic.y;   //y-comp
  mag_z = mag.magnetic.z;   //z-comp
  P = bmp.pressure;         //Pressure in Pa
  alt = bmp.readAltitude(sealvl_P);   //altitude in meters

  //calculate N here from previous angles
  void Multiply(R_y2, R_x2, 3, 3, 3 N1);
  void Multiply(N1, z_id, 3, 3, 1, N11);
  void Multiply(R_x, y_id, 3, 3, 1, N2);
  N[0] = N11[0];
  N[1} = N11[1];
  N[2] = N11[2];
  N[3] = N2[0];
  N[4] = N2[1];
  N[5] = N2[2];
  N[6] = x_id[0];
  N[7] = x_id[1];
  N[8] = x_id[2];
  int MatrixInversion(N, 3, Ninv);
  void Multiply(Ninv, w, 3, 3, 1, theta_dot);

  //integrate to get Euler angles
  theta[0] = dt*theta_dot[0];
  theta[1] = dt*theta_dot[1];
  theta[2] = dt*theta_dot[2];
  
  //generate rotation matrices based on integrated angular velocities
  float R_z1[9] = {cos(theta[0]), -sin(theta[0]), 0,
          sin(theta[0]), cos(theta[0]), 0,
          0, 0, 1};
  float R_y2[9] = {cos(theta[1]), 0, sin(theta[1]),
          0, 1, 0,
          -sin(theta[1]), 0, cos(theta[1])};
  float R_x3[9] = {1, 0, 0,
          0, cos(theta[2]), -sin(theta[2]),
          0, cos(theta[2]), sin(theta[2])};

  float R_zx[9];
  Multiply(R_z1,R_y2, 3, 3, 3, R_zy);
  float R_zxz[9];
  Multiply(R_zy, R_x3, 3, 3, 3, R_zyx);      //Euler angle rotation matrix

  //calculate quaternions based on current orientation angles
  float q0 = 0.5*sqrt(R_zxz[0] + R_zxz[4] + R_zxz[8] + 1);
  float q1 = 0.25*(1/q0)*(R_zyx[7] - R_zyx[5]);
  float q2 = 0.25*(1/q0)*(R_zyx[2] - R_zyx[6]);
  float q3 = 0.25*(1/q0)*(R_zyx[3] - R_zyx[1]);
  //find difference between current and desired orientation
  float q_0c = q0 - q_0e;
  float q_1c = q1 - q_1e;
  float q_2c = q2 - q_2e;
  float q_3c = q3 - q_3c;
  //populate state vector
  x_c[0] = q_0c;
  x_c[1] = q_1c;
  x_c[2] = q_2c;
  x_c[3] = q_3c;
  x_c[4] = w_xc;
  x_c[5] = w_yc;
  x_c[6] = w_zc;
  float neg_one[1] = -1;
  Multiply(x_c, neg_one, 7,1,1, x_c); // x_c = -1 * x_c
  Multiply(K,x_c,3,7,1,u);

  //need to figure out how a PWM value maps to an angular value
  //should be a library to do this for us

  //also need to determine how to write to EEPROM
  if(!read_mode && (loop_count % LOG_SKIP == 0) ){ //If in operation mode, only writing to EEPROM will occur
    if(curr_address <= (MAX_EEPROM_ADDR - data_size)){
      data_node to_log;
      to_log.roll = roll;
      to_log.yaw = yaw;
      to_log.pitch = pitch;
      to_log.h = h;
      to_log.t = millis();
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
      unsigned long t_data = data_buffer.t;
      //Logging a line to the serial monitor
      print_data_line(t_data, yaw_data, pitch_data, roll_data);
      curr_address += data_info.data_size;
      log_count--;
    }else{
      //Done with data retrieval, do any closing remarks
    }

  }
  loop_count++;
  //probably need a delay in here
}

void print_data_line(unsigned long t, float yaw, float pitch, float roll){
  Serial.print(t, DEC);
  Serial.print(',');
  Serial.print(yaw, DEC);
  Serial.print(',');
  Serial.print(pitch, DEC);
  Serial.print(',');
  Serial.print(roll, DEC);
  Serial.print(',');
  Serial.println();
}
