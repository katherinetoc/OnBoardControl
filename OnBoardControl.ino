#include <Adafruit_PWMServoDriver.h>
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
#include <Math.h>
#define PI 3.1415926535897932384626433832795
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
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

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
float dt = 1.0/200.0;       //guess for right now, in seconds
double theta[3] = {0.0, 0.0, 0.0};             //Euler angle vector assumed initially at all 0 deg
double theta_dot[3];         //time derivatives of Euler angles
bool read_mode = false;  //SET TO FALSE IF DOING DATA GENERATION, TRUE IF DOING DATA COLLECTION
double w[3];          //angular velocity vector
float mag_x;         //x-comp magnetic field
float mag_y;         //y-comp magnetic field
float mag_z;         //z-comp magnetic field
float roll;
float pitch;
float yaw;
float P;              //Pressure at current rocket altitude
float alt;            //current rocket altitude 
double u[3];         //initialize input vector
double x_c[7];       //initialize state vector
float w_xc;
float w_yc;
float w_zc;
double R_y2[9];      
double R_x2[9];
double R_zyx[9];     //initialize full Rz*Rx*Rz rotation matrix     
double N1[9];         //part of ang vel to ang rates matrix
double N11[3];
double N2[9];
double N[9];
double z_id[3] = {0, 
                 0,
                 1};
double y_id[3] = {0,
                 1,
                 0};
double x_id[3] = {1,
                 0,
                 0};
double K[21] = { 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
                  0.0000, 0.4000, 0.5000, 0.5000, 0.5000, 0.5000, 0.5000,
                  0.0000, 1.5000, 0.0500, 0.2500, 0.2500, 0.1500, 0.3500 };   //tentative gain matrix. Negative real eigenvalues for A-BK. Can be tuned further if needed.

int curr_address = 0;
int num_logs = 0;
int data_size = 0;
//meta_data_node data_info;
int log_count = 0;
unsigned long loop_count = 0;

struct dataNode{
  float roll; //Roll of body
  float yaw; //Yaw of body
  float pitch; //Roll of body
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

meta_data_node data_info;

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
  if(!lsm.begin(/*SDA0*/)){
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
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);  // The int.osc. is closer to 27MHz  
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

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
  w[2] = gyro.gyro.z - w_ze;   //angular velocity around z-axis
  mag_x = mag.magnetic.x;   //x-comp magnetic field
  mag_y = mag.magnetic.y;   //y-comp
  mag_z = mag.magnetic.z;   //z-comp
  P = bmp.pressure;         //Pressure in Pa
  alt = bmp.readAltitude(sealvl_P);   //altitude in meters
  //add in if statement to close iris.

  //calculate N here from previous angles
  //generate rotation matrices based on integrated angular velocities
  double R_z1[9] = {cos(theta[0]), -sin(theta[0]), 0.0,
          sin(theta[0]), cos(theta[0]), 0.0,
          0.0, 0.0, 1.0};
  double R_y2[9] = {cos(theta[1]), 0, sin(theta[1]),
          0.0, 1.0, 0.0,
          -sin(theta[1]), 0.0, cos(theta[1])};
  double R_x3[9] = {1.0, 0.0, 0.0,
          0.0, cos(theta[2]), -sin(theta[2]),
          0.0, cos(theta[2]), sin(theta[2])};
  
  Matrix.Multiply((mtx_type*)R_y2, (mtx_type*)R_x3, 3, 3, 3, (mtx_type*)N1);
  Matrix.Multiply((mtx_type*)N1, (mtx_type*)z_id, 3, 3, 1, (mtx_type*)N11);
  Matrix.Multiply((mtx_type*)R_x3, (mtx_type*)y_id, 3, 3, 1, (mtx_type*)N2);
  N[0] = N11[0];
  N[1] = N11[1];
  N[2] = N11[2];
  N[3] = N2[0];
  N[4] = N2[1];
  N[5] = N2[2];
  N[6] = x_id[0];
  N[7] = x_id[1];
  N[8] = x_id[2];
  Matrix.Invert(N, 3);
  Matrix.Multiply((mtx_type*)N, (mtx_type*)w, 3, 3, 1, (mtx_type*)theta_dot);

  //integrate to get Euler angles
  theta[0] = dt*theta_dot[0];
  theta[1] = dt*theta_dot[1];
  theta[2] = dt*theta_dot[2];
  
  

  float R_zy[9];
  Matrix.Multiply((mtx_type*)R_z1, (mtx_type*)R_y2, 3, 3, 3, (mtx_type*)R_zy);
  Matrix.Multiply((mtx_type*)R_zy, (mtx_type*)R_x3, 3, 3, 3, (mtx_type*)R_zyx);      //Euler angle rotation matrix

  //calculate quaternions based on current orientation angles
  float q0 = 0.5*sqrt(R_zyx[0] + R_zyx[4] + R_zyx[8] + 1); //CHANGED FROM ZXZ to ZYX
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
  double neg_one[1] = {-1.0};
  Matrix.Multiply((mtx_type*)x_c, (mtx_type*)neg_one, 7,1,1, (mtx_type*)x_c); // x_c = -1 * x_c
  Matrix.Multiply((mtx_type*)K, (mtx_type*)x_c,3,7,1,(mtx_type*)u);
  //convert back to euler angles
  theta[1] = (PI/180.0)*atan((2*(x_c[0]*x_c[1] + x_c[2]*x_c[3]))/(1 - 2*(sq(x_c[1]) + sq(x_c[2]))));
  theta[2] = (PI/180.0)*asin(2*(x_c[0]*x_c[2] - x_c[3]*x_c[1]));
  theta[3] = (PI/180.0)*atan((2*(x_c[0]*x_c[3] + x_c[1]*x_c[2]))/(1 - 2*(sq(x_c[2]) + sq(x_c[3]))));
  
  
  //need to figure out how a PWM value maps to an angular value
  long pulselength1 = map((long)theta[1], 0, 180, SERVOMIN, SERVOMAX);
  long pulselength2 = map((long)theta[2], 0, 180, SERVOMIN, SERVOMAX);
  long pulselength3 = map((long)theta[3], 0, 180, SERVOMIN, SERVOMAX);
  //pwm.setPWM(servonum_placehold, 0, pulselength1);
  //pwm.setPWM(servonum_placehold, 0, pulselength2);
  //pwm.setPWM(servonum_placehold, 0, pulselength3);
  
  if(!read_mode && (loop_count % LOG_SKIP == 0) ){ //If in operation mode, only writing to EEPROM will occur
    if(curr_address <= (MAX_EEPROM_ADDR - data_size)){
      data_node to_log;
      to_log.roll = theta[2];
      to_log.yaw = theta[0];
      to_log.pitch = theta[1];
      to_log.h = alt;
      to_log.t = millis();
      EEPROM.put(curr_address, to_log);
      curr_address += data_size;
      data_info.num_logs++;
    }else{
      //EEPROM overflow whoops handle somehow
    }
  }else{ //If in collection mode, only reading from EEPROM will occur
    if(log_count > 0){
      data_node data_buffer;
      EEPROM.get(curr_address, data_buffer);
      float roll_data = data_buffer.roll;
      float yaw_data = data_buffer.yaw;
      float pitch_data = data_buffer.pitch;
      float h_data = data_buffer.h;
      unsigned long t_data = data_buffer.t;
      //Logging a line to the serial monitor
      print_data_line(t_data, yaw_data, pitch_data, roll_data);
      curr_address += data_info.data_size;
      log_count--;
    }else{
      //Done with data retrieval, do any closing remarks
      Serial.println("End of EEPROM data");
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
