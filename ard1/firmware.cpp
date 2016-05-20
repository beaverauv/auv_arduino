#include <ros.h>
#include <std_msgs/String.h>
#include <auv_arduino/motor.h>
#include <Arduino.h>

ros::NodeHandle nh;
using auv_arduino::SetMotor;
using auv_arduino::SetMotorPWM;
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
void callback(const SetMotor::Request & req, SetMotor::Response & res){}
void callback1(const SetMotorPWM::Request & req, SetMotorPWM::Response & res){}

ros::ServiceServer<SetMotor::Request, SetMotor::Response> server("setmotor_srv", &callback);
ros::ServiceServer<SetMotorPWM::Request, SetMotorPWM::Response> server1("setmotorpwm_srv", &callback1);

char hello[13] = "hello world!";

void setup()
{
  nh.initNode();
  nh.advertiseService(server);
  nh.advertiseService(server1);
  nh.advertise(chatter);

}

void loop()
{
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(1000);
}
