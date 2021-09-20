#include "ZBlock.h"

//float tau = 0.1164;
float tau = 1;
float K = 1.3818;
float period_s = 0.5;

ZBlock G = ZBlock(0.0, period_s, tau, ZBlockType::Z_TYPE_FIRSTORDER, ZBlockMethod::Z_METHOD_STD);
float input;
float output;

ZBlock I = ZBlock(0.0, 0.5, 1, ZBlockType::Z_TYPE_INT, ZBlockMethod::Z_METHOD_FE);
float output1;

unsigned long t_init;
unsigned long t_next;
unsigned long t_actual;
unsigned long t_period_ms = 50;

unsigned long t_step;
unsigned long t_low;

void setup() {
  Serial.begin(115200);

  t_init = millis();
  t_next = t_init;
  t_step = t_init + 1000; // One second, step
  t_low = t_step + 5000; // next, five second, low_step
}

void loop() {
  t_actual = millis();
  if(t_next > t_actual) return;
  t_next = t_next + t_period_ms;
  
  if(t_step > t_actual) input = 0;
  else input = 1*K;

  if(t_low < t_actual) input = 0;

  output = G.forwardStep(input);
  output1 = I.forwardStep(input);
  
  Serial.print(input);
  Serial.print(",");
  Serial.print(output);
  Serial.print(",");
  Serial.print(output1);
  Serial.println();
  
}
