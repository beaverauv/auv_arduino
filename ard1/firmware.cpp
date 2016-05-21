#include <ros.h>
#include <std_msgs/String.h>
#include <auv_arduino/defs.h>
#include <auv_arduino/SetVertMotor.h>
#include <auv_arduino/SetVertMotorPWM.h>
#include <Arduino.h>

int MotorPWM[4] = {STOP_PWM, STOP_PWM, STOP_PWM, STOP_PWM};

ros::NodeHandle nh;
using auv_arduino::SetVertMotor;
using auv_arduino::SetVertMotorPWM;

void SetMotorCallback(const SetVertMotor::Request & req, SetVertMotor::Response & res){
  int MotorNum = req.motor;
  int MotorSpeed = req.perc;
  //if given values are out of bounds return -1 for failure
  if (MotorNum > 3 || MotorSpeed > 100 || MotorSpeed < -100){
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
void SetMotorPWMCallback(const SetVertMotorPWM::Request & req, SetVertMotorPWM::Response & res){

}

ros::ServiceServer<SetVertMotor::Request, SetVertMotor::Response> server("setvertmotor_srv", &SetMotorCallback);
ros::ServiceServer<SetVertMotorPWM::Request, SetVertMotorPWM::Response> server1("setvertmotorpwm_srv", &SetMotorPWMCallback);


void setup()
{
  nh.initNode();
  nh.advertiseService(server);
  nh.advertiseService(server1);
}

void loop()
{
  nh.spinOnce();
  delay(1000);
}
