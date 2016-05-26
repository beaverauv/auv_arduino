#include <ros.h>
#include <std_msgs/String.h>
#include <auv_arduino/defs.h>
#include <auv_arduino/SetMotor.h>
#include <auv_arduino/SetMotorPWM.h>
#include <Arduino.h>
#include <Servo.h>


int MotorPWM[8] = {STOP_PWM, STOP_PWM, STOP_PWM, STOP_PWM, STOP_PWM, STOP_PWM, STOP_PWM, STOP_PWM};

Servo motor_VFL;
Servo motor_VFR;
Servo motor_VBL;
Servo motor_VBR;
Servo motor_HFL;
Servo motor_HFR;
Servo motor_HBL;
Servo motor_HBR;


ros::NodeHandle nh;
using auv_arduino::SetMotor;
using auv_arduino::SetMotorPWM;

int validateInputs(int motor, int speed){
  if (motor > 8 || speed > 100 || speed < -100){
    return 0;
  }
  return 1;
}

void SetMotorCallback(const SeMotor::Request & req, SetMotor::Response & res){
  int MotorNum = req.motor;
  int MotorSpeed = req.perc;
  //if given values are out of bounds return -1 for failure
  if (!validateInputs(MotorNum, MotorSpeed)){
    res.success = -1;
    return;
  }

  if (MotorSpeed == 0){
    MotorPWM[MotorNum-1] = STOP_PWM;
  } else if (MotorSpeed > 0){
    MotorPWM[MotorNum-1] = map(MotorSpeed, 0, 100, 1500, 1900);
  } else {
    MotorPWM[MotorNum-1] = map(MotorSpeed, -100, 0, 1100, 1500);
  }
  res.success = 1;
}
void SetMotorPWMCallback(const SetVertMotorPWM::Request & req, SetVertMotorPWM::Response & res){
  int MotorNum = req.motor;
  int MotorSpeed = req.usecs;

  if (!validateInputs(MotorNum, MotorSpeed)){
    res.success = -1;
    return;
  }

  MotorPWM[MotorNum-1] = MotorSpeed;
  res.success = 1;
}

ros::ServiceServer<SetMotor::Request, SetMotor::Response> server("setmotor_srv", &SetMotorCallback);
ros::ServiceServer<SetMotorPWM::Request, SetMotorPWM::Response> server1("setmotorpwm_srv", &SetMotorPWMCallback);


void setup()
{

  motor_VFL.attach(MOTOR_VFL_PIN);
  motor_VFR.attach(MOTOR_VFR_PIN);
  motor_VBL.attach(MOTOR_VBL_PIN);
  motor_VBR.attach(MOTOR_VBR_PIN);
  motor_HFL.attach(MOTOR_HFL_PIN);
  motor_HFR.attach(MOTOR_HFR_PIN);
  motor_HBL.attach(MOTOR_HBL_PIN);
  motor_HBR.attach(MOTOR_HBR_PIN);


  motor_VFL.writeMicroseconds(STOP_PWM);
  motor_VFR.writeMicroseconds(STOP_PWM);
  motor_VBL.writeMicroseconds(STOP_PWM);
  motor_VBR.writeMicroseconds(STOP_PWM);
  motor_HFL.writeMicroseconds(STOP_PWM);
  motor_HFR.writeMicroseconds(STOP_PWM);
  motor_HBL.writeMicroseconds(STOP_PWM);
  motor_HBR.writeMicroseconds(STOP_PWM);


  //delay(1000);

  nh.initNode();
  nh.advertiseService(server);
  nh.advertiseService(server1);
}

void loop()
{

  motor_VFL.writeMicroseconds(MotorPWM[MOTOR_VFL]);
  motor_VFR.writeMicroseconds(MotorPWM[MOTOR_VFR]);
  motor_VBL.writeMicroseconds(MotorPWM[MOTOR_VBL]);
  motor_VBR.writeMicroseconds(MotorPWM[MOTOR_VBR]);
  motor_HFL.writeMicroseconds(MotorPWM[MOTOR_HFL]);
  motor_HFR.writeMicroseconds(MotorPWM[MOTOR_HFR]);
  motor_HBL.writeMicroseconds(MotorPWM[MOTOR_HBL]);
  motor_HBR.writeMicroseconds(MotorPWM[MOTOR_HBR]);


  nh.spinOnce();

}
