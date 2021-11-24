#include "Robot_Mecanum.h"

Robot_Mecanum * robot;

float ref = 0;
float val;

int steps = 40;

unsigned long t_next;
unsigned long t_encoder = 50;

int cont = 0;

void setup() {
  Serial.begin(115200);

  robot = Robot_Mecanum::GetInstance();
  robot->start();

  t_next = millis();
}

void loop() {

  if (t_next <= millis()) {
    t_next = t_next + t_encoder;
    robot->set_M4_ref_rads(ref);
    val = robot->get_M4_rads();
    Serial.print(ref);
    Serial.print(",");
    Serial.print(val);
    Serial.println();
    cont++;
  }

  if (cont > steps) ref = 6;
  if (cont > steps*2) ref = 0;
  if (cont > steps*3) ref = -6;
  if (cont > steps*4) {ref = 0; cont = 0;}
}
