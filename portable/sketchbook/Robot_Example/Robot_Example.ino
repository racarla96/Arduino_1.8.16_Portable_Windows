#include "Robot_Mecanum.h"

Robot_Mecanum * robot;

void setup() {
  Serial.begin(115200);
  
  robot = Robot_Mecanum::GetInstance();
  robot->start();

  robot->set_M1_ref_rads(1);
  robot->set_M2_ref_rads(1);
//  robot->set_M3_ref_rads(1);
//  robot->set_M4_ref_rads(1);
}

void loop() {
//  delay(10000);
//  robot->stop();
//  delay(10000);
//  robot->start();
}
