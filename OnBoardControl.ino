#include <Adafruit_PWMServoDriver.h>
#include <Servo.h>
#include <MatrixMath.h>
#include <Adafruit_BMP3XX.h>
#include <bmp3.h>
#include <bmp3_defs.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS1.h>
#include <Wire.h>
//#include <WireIMXRT.h>
#include <WireKinetis.h>
//#include <EEPROM.h>
#include <Math.h>
#define PI 3.1415926535897932384626433832795
#define SDA0 22
//#define SDA1 17
#define SCL0 21
//#define SCL1 16
#define TVC1 13
#define TVC2 14
#define iris 15
#define reac 16
#define sealvl_P (69)     //Pa                                //**CHANGE ON DAY OF LAUNCH**
#define MAX_EEPROM_ADDR 65536
#define LOG_SKIP 100
#define MAX_LOGS 100
#define ITERATION_DELAY 5 //ms
Adafruit_PWMServoDriver pwm;// = Adafruit_PWMServoDriver(0x40, Wire);
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

Adafruit_LSM9DS1 lsm;// = Adafruit_LSM9DS1(); // i2c sensor
Adafruit_BMP3XX bmp; // I2C
const float g=9.80665;   //m/s^2
const float wndspd=0.0;    //m/s                              //**CHANGE ON DAY OF LAUNCH**
const float burn_engage_alt = 0;
const float iris_engage_alt = 0;
const float logging_engage_alt = 0;
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
double acc[3];        //Acceleration vector
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
int led = 13;
double f_real = 12.0; //Newtons
double L = 0.169; //meters
const int acc_buffer_size = 5;
double acc_buffer[acc_buffer_size];
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
bool logging_active = 0;
bool descent_mode = 0;
int curr_address = 0;
int num_logs = 0;
int data_size = 0;
//meta_data_node data_info;
int log_count = 0;
unsigned long loop_count = 0;
unsigned long prev_time = 0;
unsigned long curr_time = 0;
double w_x_bias = 0;
double w_y_bias = 0;
double w_z_bias = 0;
uint8_t phi_motor_num = 0; //CHECK THIS
uint8_t theta_motor_num = 1; //CHECK THIS
uint8_t reaction_wheel_num = 2; //CHECK THIS
double max_tvc_angle = 45; //deg
double min_tvc_angle = -45; //deg
uint8_t base_pwm = 127;
double c = 0.05;
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
  //Serial.println("Here1");
  Serial.begin(115200); 
  delay(2000);               // wait for a second     
  pwm = Adafruit_PWMServoDriver(0x40, Wire);
  lsm = Adafruit_LSM9DS1();
  //Serial.println("Here1");
  pinMode(SDA0,INPUT);    //SDA0    //altimeter
  pinMode(SDA0,INPUT);    //SDA1    //IMU
  pinMode(SCL0,INPUT);    //SCL0    //altimeter
  pinMode(SCL0,INPUT);    //SCL1    //IMU
  //pinMode(reac,OUTPUT);    //reaction wheel motor
  //pinMode(iris,OUTPUT);    //iris servo
  //pinMode(TVC1,OUTPUT);    //TVC servo 1
  //pinMode(TVC2,OUTPUT);    //TVC servo 2
  //Serial.println("Here1");
  /*
  if (!bmp.begin()) {
    Serial.println("Altimeter didnt' open up right. Infinite looping now.");
    while (1);
  }
  */
  //Serial.println("Shit");
  if(!lsm.begin(/*SDA0*/)){
    Serial.println("IMU didn't open up right. Infinite looping now.");
    while(1);
  }
  //Serial.println("Here1");
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);  //setup acceleration range --> 2g's
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);   //setup magnetometer range --> 4 Gauss
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);    //setup gryroscope range --> 245 degrees per second

  sensors_event_t accel, mag, gyro, temp;   //read IMU data

  double test_w_x = 0;
  double test_w_y = 0;
  double test_w_z = 0;
  
  
  for(int i = 0; i < 100; i++){
    lsm.getEvent(&accel, &mag, &gyro, &temp);   //takes snapshot at time t(i)
    test_w_x = gyro.gyro.x /*- w_xe*/;   //angular velocity around x-axis
    test_w_y = gyro.gyro.y /*- w_ye*/;   //angular velocity around y-axis
    test_w_z = gyro.gyro.z /*- w_ze*/;   //angular velocity around z-axis
   
  }

  double w_x_sum = 0;
  double w_y_sum = 0;
  double w_z_sum = 0;
  for(int i = 0; i < 1000; i++){
    lsm.getEvent(&accel, &mag, &gyro, &temp);   //takes snapshot at time t(i)
    test_w_x = gyro.gyro.x /*- w_xe*/;   //angular velocity around x-axis
    test_w_y = gyro.gyro.y /*- w_ye*/;   //angular velocity around y-axis
    test_w_z = gyro.gyro.z /*- w_ze*/;   //angular velocity around z-axis
    w_x_sum += test_w_x;
    w_y_sum += test_w_y;
    w_z_sum += test_w_z;
   
  }

  w_x_bias = w_x_sum / 1000.0;
  w_y_bias = w_y_sum / 1000.0;
  w_z_bias = w_z_sum / 1000.0;

  
/*
  if(!read_mode){
    data_size = sizeof(data_node);
    curr_address = sizeof(meta_data_node); //Start data logging after meta_data node
    data_info.data_size = data_size;
    data_info.num_logs = 0;
    EEPROM.put(0, data_info);
  }else{
    EEPROM.get(0,data_info);
    data_size = data_info.data_size;
    curr_address = sizeof(meta_data_node);
    log_count = data_info.num_logs;
    Serial.print("Time(ms), Yaw(deg), Pitch(deg), Roll(deg)");

  }
  */
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
  prev_time = millis();
  digitalWrite(led, LOW);   // turn the LED on (HIGH is the voltage level)
}

void loop() {
  // Serial.println("Here1");
  sensors_event_t accel, mag, gyro, temp;   //read IMU data
  lsm.getEvent(&accel, &mag, &gyro, &temp);   //takes snapshot at time t(i)
  w[0] = gyro.gyro.x - w_x_bias;/*- w_xe*/;   //angular velocity around x-axis
  w[1] = gyro.gyro.y - w_y_bias;/*- w_ye*/;   //angular velocity around y-axis
  w[2] = gyro.gyro.z - w_z_bias; /*- w_ze*/;   //angular velocity around z-axis
  acc[0] = accel.acceleration.x;
  acc[1] = accel.acceleration.y;
  acc[2] = accel.acceleration.z;
  mag_x = mag.magnetic.x;   //x-comp magnetic field
  mag_y = mag.magnetic.y;   //y-comp
  mag_z = mag.magnetic.z;   //z-comp
  P = bmp.pressure;         //Pressure in Pa
  alt = bmp.readAltitude(sealvl_P);   //altitude in meters
  //add in if statement to close iris.
  if(alt > logging_engage_alt){
    logging_active = 1;
  }
 
  float acc_mag = /*sqrt(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2])*/ acc[2];
  acc_buffer[loop_count % acc_buffer_size] = acc_mag;
  float acc_sum = 0;
  for(int i = 0; i < acc_buffer_size; i++){
    acc_sum += acc_buffer[i];
  }
  
  if(acc_sum < 5.0){ //Is this a reasonable buffer for free-fall phase acceleration?
    digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
    descent_mode = 1;
  }
  
  if(descent_mode && (alt < burn_engage_alt)){
    //IGNITE MOTOR
  }

  if(descent_mode && (alt < iris_engage_alt)){
    //ENGAGE IRIS
  }
  
  
  

  

  //calculate N here from previous angles
  //generate rotation matrices based on integrated angular velocities
  
  double R_z1[9] = {cosd(theta[0]), -sind(theta[0]), 0.0,
          sind(theta[0]), cosd(theta[0]), 0.0,
          0.0, 0.0, 1.0};
  double R_y2[9] = {cosd(theta[1]), 0, sind(theta[1]),
          0.0, 1.0, 0.0,
          -sind(theta[1]), 0.0, cosd(theta[1])};
  double R_x3[9] = {1.0, 0.0, 0.0,
          0.0, cosd(theta[2]), -sind(theta[2]),
          0.0, cosd(theta[2]), sind(theta[2])};
  /*
  Matrix.Multiply((mtx_type*)R_y2, (mtx_type*)R_x3, 3, 3, 3, (mtx_type*)N1);
  Matrix.Multiply((mtx_type*)N1, (mtx_type*)z_id, 3, 3, 1, (mtx_type*)N11);
  Matrix.Multiply((mtx_type*)R_x3, (mtx_type*)y_id, 3, 3, 1, (mtx_type*)N2);
  */

  double hy = theta[0];
  double hp = theta[1];
  double hr = theta[2];
  N[0] = 0;
  N[1] = sind(hr)/(cosd(hp)*cosd(hr)*cosd(hr) + cosd(hp)*sind(hr)*sind(hr));
  N[2] = cosd(hr)/(cosd(hp)*cosd(hr)*cosd(hr) + cosd(hp)*sind(hr)*sind(hr));
  N[3] = 0;
  N[4] = cosd(hr)/(cosd(hr)*cosd(hr) + sind(hr)*sind(hr));
  N[5] = -sind(hr)/(cosd(hr)*cosd(hr) + sind(hr)*sind(hr));
  N[6] = 1;
  N[7] = (sind(hp)*sind(hr))/(cosd(hp)*cosd(hr)*cosd(hr) + cosd(hp)*sind(hr)*sind(hr));
  N[8] = (cosd(hr)*sind(hp))/(cosd(hp)*cosd(hr)*cosd(hr) + cosd(hp)*sind(hr)*sind(hr));
  //Matrix.Invert(N, 3);
 // Matrix.Print(N, 3, 3, "Ass");
  Matrix.Multiply((mtx_type*)N, (mtx_type*)w, 3, 3, 1, (mtx_type*)theta_dot);
  
  
  curr_time = millis();
  if(descent_mode){
    theta[0] = ((curr_time - prev_time)/1000.0)*theta_dot[0] + theta[0];
    theta[1] = ((curr_time - prev_time)/1000.0)*theta_dot[1] + theta[1];
    theta[2] = ((curr_time - prev_time)/1000.0)*theta_dot[2] + theta[2];
  }
  prev_time = curr_time;

  double roll = theta[0];
  double yaw = theta[1];
  double pitch = theta[2];

  double R_zy[9];
  Matrix.Multiply((mtx_type*)R_z1, (mtx_type*)R_y2, 3, 3, 3, (mtx_type*)R_zy);
  Matrix.Multiply((mtx_type*)R_zy, (mtx_type*)R_x3, 3, 3, 3, (mtx_type*)R_zyx);      //Euler angle rotation matrix

  
  //calculate quaternions based on current orientation angles
  float q0 = 0.5*sqrt(R_zyx[0] + R_zyx[4] + R_zyx[8] + 1); //CHANGED FROM ZXZ to ZYX
  float q1 = 0.25*(1.0/q0)*(R_zyx[7] - R_zyx[5]);
  float q2 = 0.25*(1.0/q0)*(R_zyx[2] - R_zyx[6]);
  float q3 = 0.25*(1.0/q0)*(R_zyx[3] - R_zyx[1]);
  //find difference between current and desired orientation
  float q_0c = q0 - q_0e;
  float q_1c = q1 - q_1e;
  float q_2c = q2 - q_2e;
  float q_3c = q3 - q_3e;
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
  //theta[0] = (PI/180.0)*atan((2*(x_c[0]*x_c[1] + x_c[2]*x_c[3]))/(1 - 2*(sq(x_c[1]) + sq(x_c[2]))));
  //theta[1] = (PI/180.0)*asin(2*(x_c[0]*x_c[2] - x_c[3]*x_c[1]));
  //theta[2] = (PI/180.0)*atan((2*(x_c[0]*x_c[3] + x_c[1]*x_c[2]))/(1 - 2*(sq(x_c[2]) + sq(x_c[3]))));

  
  double F_x = -u[1] / L;
  double F_y = u[0] / L;

  double phi_tvc = asin(-F_y / f_real);
  double theta_tvc = asin(F_x / (f_real*cos(phi_tvc)));

  phi_tvc = -pitch;//phi_tvc * (180.0 / PI);
  theta_tvc = -yaw;//theta_tvc * (180.0 / PI);

  if(phi_tvc > max_tvc_angle){
    phi_tvc = max_tvc_angle;
  }
  if(phi_tvc < min_tvc_angle){
    phi_tvc = min_tvc_angle;
  }

  if(theta_tvc > max_tvc_angle){
    theta_tvc = max_tvc_angle;
  }
  if(theta_tvc < min_tvc_angle){
    theta_tvc = min_tvc_angle;
  }

  //Serial.println(phi_tvc);

  //Serial.print(phi_tvc);
  //Serial.print("->");
  

  uint16_t phi_pulse = (uint16_t)((map_d(phi_tvc, -90, 90, SERVOMIN, SERVOMAX)) + 0.5);
  //Serial.println(phi_pulse);
  uint16_t theta_pulse = (uint16_t)((map_d(theta_tvc, -90, 90, SERVOMIN, SERVOMAX)) + 0.5);

  if(descent_mode){
    pwm.setPWM(phi_motor_num, 0, phi_pulse);
    pwm.setPWM(theta_motor_num, 0, theta_pulse);
  }
  base_pwm = base_pwm - c*theta_dot[2];
  //Serial.println(base_pwm);
  //pwm.setPWM(reaction_wheel_num, 0, base_pwm); 
  analogWrite(reac, base_pwm);
  

  


 // print_data_line(curr_time, theta[0], theta[1], theta[2], bmp.readAltitude(sealvl_P));
  
 
  
  //pwm.setPWM(servonum_placehold, 0, pulselength1);
  //pwm.setPWM(servonum_placehold, 0, pulselength2);
  //pwm.setPWM(servonum_placehold, 0, pulselength3);
  
  /*
  if(logging_active && !read_mode && (loop_count % LOG_SKIP == 0)){ //If in operation mode, only writing to EEPROM will occur
    if((curr_address <= (MAX_EEPROM_ADDR - data_size)) && (data_info.num_logs < MAX_LOGS)){
      data_node to_log;
      to_log.roll = theta[2];
      to_log.yaw = theta[0];
      to_log.pitch = theta[1];
      to_log.h = alt;
      to_log.t = millis();
      EEPROM.put(curr_address, to_log);
      curr_address += data_size;
      data_info.num_logs++;
      EEPROM.put(0, data_info);
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
      print_data_line(t_data, yaw_data, pitch_data, roll_data, h_data);
      curr_address += data_info.data_size;
      log_count--;
    }else{
      //Done with data retrieval, do any closing remarks
      Serial.println("End of EEPROM data");
    }
    
  }
  */
  loop_count++;
  //probably need a delay in here
  //delay(ITERATION_DELAY);
}

double sind(double in){
  return sin(radians(in));
}
double cosd(double in){
  return cos(radians(in));
}

double map_d(double x, double in_min, double in_max, double out_min, double out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void print_data_line(unsigned long t, float yaw, float pitch, float roll, float h){
  Serial.print(t, DEC);
  Serial.print(',');
  Serial.print(yaw, DEC);
  Serial.print(',');
  Serial.print(pitch, DEC);
  Serial.print(',');
  Serial.print(roll, DEC);
  Serial.print(',');
  Serial.print(h, DEC);
  Serial.println();
}
