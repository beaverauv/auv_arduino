#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <auv_arduino/defs.h>
#include <auv_arduino/InitESC.h>
#include <auv_motor_control/thruster_values.h>
#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <BlueRobotics_MS5837_Library/MS5837.h>

int MotorPWM[8] = {STOP_PWM, STOP_PWM, STOP_PWM, STOP_PWM, STOP_PWM, STOP_PWM, STOP_PWM, STOP_PWM};


Servo motor_HFL;
Servo motor_HFR;
Servo motor_HBL;
Servo motor_HBR;
Servo motor_VFL;
Servo motor_VFR;
Servo motor_VBL;
Servo motor_VBR;
std_msgs::Float64 fDepth;
MS5837 sDepth;
auv_motor_control::thruster_values thrust_val;
ros::Publisher pub("test", &thrust_val);
int PercentToPWM(int perc){
  if (perc == 0){
    return STOP_PWM;
  } else if (perc > 0){
    return map(perc, 0, 100, 1500, 1900);
  } else {
    return map(perc, -100, 0, 1100, 1500);
  }
}

void set_motorscb(const auv_motor_control::thruster_values& thruster_outputs){
  MotorPWM[MOTOR_HFL-1] = PercentToPWM(thruster_outputs.thruster_xy_frontLeft);
  MotorPWM[MOTOR_HFR-1] = PercentToPWM(thruster_outputs.thruster_xy_frontRight);
  MotorPWM[MOTOR_HBL-1] = PercentToPWM(thruster_outputs.thruster_xy_backLeft);
  MotorPWM[MOTOR_HBR-1] = PercentToPWM(thruster_outputs.thruster_xy_backRight);

  MotorPWM[MOTOR_VFL-1] = PercentToPWM(thruster_outputs.thruster_z_frontLeft);
  MotorPWM[MOTOR_VFR-1] = PercentToPWM(thruster_outputs.thruster_z_frontRight);
  MotorPWM[MOTOR_VBL-1] = PercentToPWM(thruster_outputs.thruster_z_backLeft);
  MotorPWM[MOTOR_VBR-1] = PercentToPWM(thruster_outputs.thruster_z_backRight);
  pub.publish(&thruster_outputs);
}

ros::Subscriber<auv_motor_control::thruster_values> sub("thruster_values", &set_motorscb);
using auv_arduino::InitESC;

void InitESCCallback(const InitESC::Request & req, InitESC::Response & res){
  motor_VFL.writeMicroseconds(STOP_PWM);
  motor_VFR.writeMicroseconds(STOP_PWM);
  motor_VBL.writeMicroseconds(STOP_PWM);
  motor_VBR.writeMicroseconds(STOP_PWM);
  motor_HFL.writeMicroseconds(STOP_PWM);
  motor_HFR.writeMicroseconds(STOP_PWM);
  motor_HBL.writeMicroseconds(STOP_PWM);
  motor_HBR.writeMicroseconds(STOP_PWM);
  delay(1000);
}
ros::ServiceServer<InitESC::Request, InitESC::Response> server2("initesc_srv", &InitESCCallback);
ros::Publisher pDepth("depth", &fDepth);

ros::NodeHandle nh;





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


  Wire.begin();
  sDepth.init();
  sDepth.setFluidDensity(997);

  //delay(1000);

  nh.initNode();
  nh.subscribe(sub);
  nh.advertiseService(server2);
  nh.advertise(pDepth);
  nh.advertise(pub);
}

void loop()
{

  motor_VFL.writeMicroseconds(MotorPWM[MOTOR_VFL-1]);
  motor_VFR.writeMicroseconds(MotorPWM[MOTOR_VFR-1]);
  motor_VBL.writeMicroseconds(MotorPWM[MOTOR_VBL-1]);
  motor_VBR.writeMicroseconds(MotorPWM[MOTOR_VBR-1]);
  motor_HFL.writeMicroseconds(MotorPWM[MOTOR_HFL-1]);
  motor_HFR.writeMicroseconds(MotorPWM[MOTOR_HFR-1]);
  motor_HBL.writeMicroseconds(MotorPWM[MOTOR_HBL-1]);
  motor_HBR.writeMicroseconds(MotorPWM[MOTOR_HBR-1]);
  sDepth.read();
  fDepth.data = sDepth.depth();
  pDepth.publish(&fDepth);
  nh.spinOnce();

}
