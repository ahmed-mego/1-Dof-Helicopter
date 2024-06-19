#include "ESC.h"
#include "Wire.h"
#include "Simple_MPU6050.h"
#include <PID_v1.h>
#define MPU6050_DEFAULT_ADDRESS 0x68 
#define ESC_PIN 5 
Simple_MPU6050 mpu;
ESC motor (ESC_PIN, 1000, 2000, 1500); 
//---------Functions_Prototype-----
void Motor_Setup();
void MPU_Setup();
void Motor_Speed(int);
float MPU_Roll();
int PID_cmd(double, double, double, double, double);
//---------Motor_speed-----
int current_speed = 1500;
// --------Angle---------
double Inst_angle; 
double Desired_angle = 25;
//---------PID constants-----------  
double KP = 0.8;
// double KI = .2;
double KI = .4;
double KD = 0.09;
//---------Motor_command---
double cmd;

PID myPID(&Inst_angle, &cmd, &Desired_angle, KP, KI, KD, DIRECT);

void setup() {
  Serial.begin(9600);
  myPID.SetOutputLimits(1600, 2000);
  myPID.SetMode(AUTOMATIC);
  MPU_Setup();
  Motor_Setup();
}

void loop() {
  mpu.dmp_read_fifo(false);
myPID.Compute();
  motor.speed(cmd);
  Serial.print("Angle\t");
  Serial.print(Inst_angle);
  Serial.print("\t\t");
  Serial.print("Speed\t");
  Serial.print(cmd);
  Serial.println();
}

void print_Values (int16_t *gyro, int16_t *accel, int32_t *quat) {
  Quaternion q;
  VectorFloat gravity;
  float ypr[3] = { 0, 0, 0 };
  float xyz[3] = { 0, 0, 0 };
  mpu.GetQuaternion(&q, quat);
  mpu.GetGravity(&gravity, &q);
  mpu.GetYawPitchRoll(ypr, &q, &gravity);
  mpu.ConvertToDegrees(ypr, xyz);
  Inst_angle = xyz[1] *-1;
}

void Motor_Setup() {
  motor.arm();
  motor.speed(1500);
  delay(5000);
  Serial.println("\nGo");
}

void MPU_Setup() {
  int sdaPin = 0; 
  int sclPin = 1;
  mpu.begin(sdaPin, sclPin);
  mpu.Set_DMP_Output_Rate_Hz(10); 
  mpu.SetAddress(MPU6050_DEFAULT_ADDRESS); 
  mpu.CalibrateMPU();                      
  mpu.load_DMP_Image();                    
  mpu.on_FIFO(print_Values);               
}

