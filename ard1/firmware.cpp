#include <ros.h>
#include <std_msgs/String.h>
#include <auv_arduino/defs.h>
#include <auv_arduino/SetVertMotor.h>
#include <auv_arduino/SetVertMotorPWM.h>
#include <Arduino.h>
#include <Servo.h>


int MotorPWM[4] = {STOP_PWM, STOP_PWM, STOP_PWM, STOP_PWM};

Servo motor_FL;
Servo motor_FR;
Servo motor_BL;
Servo motor_BR;


ros::NodeHandle nh;
using auv_arduino::SetVertMotor;
using auv_arduino::SetVertMotorPWM;

int validateInputs(int motor, int speed){
  if (motor > 3 || speed > 100 || speed < -100){
    return 0;
  }
  return 1;
}

void SetVertMotorCallback(const SetVertMotor::Request & req, SetVertMotor::Response & res){
  int MotorNum = req.motor;
  int MotorSpeed = req.perc;
  //if given values are out of bounds return -1 for failure
  if (!validateInputs(MotorNum, MotorSpeed)){
    res.success = -1;
    return;
  }

  if (MotorSpeed == 0){
    MotorPWM[MotorNum] = STOP_PWM;
  } else if (MotorSpeed > 0){
    MotorPWM[MotorNum] = map(MotorSpeed, 0, 100, 1500, 1900);
  } else {
    MotorPWM[MotorNum] = map(MotorSpeed, -100, 0, 1100, 1500);
  }
  res.success = 1;
}
void SetVertMotorPWMCallback(const SetVertMotorPWM::Request & req, SetVertMotorPWM::Response & res){
  int MotorNum = req.motor;
  int MotorSpeed = req.usecs;

  if (!validateInputs(MotorNum, MotorSpeed)){
    res.success = -1;
    return;
  }

  MotorPWM[MotorNum] = MotorSpeed;
  res.success = 1;
}

ros::ServiceServer<SetVertMotor::Request, SetVertMotor::Response> server("setvertmotor_srv", &SetVertMotorCallback);
ros::ServiceServer<SetVertMotorPWM::Request, SetVertMotorPWM::Response> server1("setvertmotorpwm_srv", &SetVertMotorPWMCallback);


void setup()
{

  motor_FL.attach(MOTOR_VFL_PIN);
  motor_FR.attach(MOTOR_VFR_PIN);
  motor_BL.attach(MOTOR_VBL_PIN);
  motor_BR.attach(MOTOR_VBR_PIN);

  motor_FL.writeMicroseconds(STOP_PWM);
  motor_FR.writeMicroseconds(STOP_PWM);
  motor_BL.writeMicroseconds(STOP_PWM);
  motor_BR.writeMicroseconds(STOP_PWM);

  delay(1000);

  nh.initNode();
  nh.advertiseService(server);
  nh.advertiseService(server1);
}

void loop()
{

  motor_FL.writeMicroseconds(MotorPWM[MOTOR_FL]);
  motor_FR.writeMicroseconds(MotorPWM[MOTOR_FR]);
  motor_BL.writeMicroseconds(MotorPWM[MOTOR_BL]);
  motor_BR.writeMicroseconds(MotorPWM[MOTOR_BR]);

  nh.spinOnce();

}
