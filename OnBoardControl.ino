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
const float g=9.80665;   //m/s^2
const float wndspd=0.0;    //m/s


void setup() {
  Serial.begin(9600);
  SD.begin(BUILTIN_SDCARD);   //initialize SD card to save telemetry
  pinMode(SDA0,INPUT);    //SDA0
  pinMode(SDA1,INPUT);    //SDA1
  pinMode(SCL0,INPUT);    //SCL0
  pinMode(SCL1,INPUT);    //SCL1
  pinMode(reac,OUTPUT);    //reaction wheel motor
  pinMode(iris,OUTPUT);    //iris servo
  pinMode(TVC1,OUTPUT);    //TVC servo 1
  pinMode(TVC2,OUTPUT);    //TVC servo 2

}

void loop() {
  

}