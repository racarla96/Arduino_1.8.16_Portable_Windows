#include <Encoder.h>
#include <ZBlock.h>
#include <ZPID.h>

/* Period time */
const float PERIOD_ms = 10.0f;
const float PERIOD_s = PERIOD_ms / 1000.0f;
const float PERIOD_us = PERIOD_ms * 1000.0f;

/* Motor constants */
const float MOTOR_GEAR_RATIO = 70.0f;
const float ENCODER_CPR = 64.0f;
const float RADS_CONST = ((2.0f * PI) / (MOTOR_GEAR_RATIO * ENCODER_CPR * PERIOD_s));

const uint8_t  M2_Enc_A_Pin = 50;
const uint8_t  M2_Enc_B_Pin = 51;

        const float Driver_Resolution = 4096.0f;


Encoder * M2_Enc;
long M2_newPosition;
long M2_lastPosition;
long M2_increment;
float M2_ref_ang_vel;
float M2_ang_vel;
float M2_ang_vel_past;
float M2_control_action;
float M2_control_speed;
uint16_t M2_control_speed_u ;
ZPID * M2_PID_CONTROLLER;

const float control_Kp = 0.13;
const float control_Ki = 1;
const float control_L_lim = -12;
const float control_U_lim = 12;

/* GESTIÓN DE TIEMPO */
/* TAREAS */
volatile unsigned long t_clock;
// TAREA CONTROL
volatile unsigned long t_task_ctrl;
const unsigned long t_next_ctrl = 10;
// TAREA ENVIAR
volatile unsigned long t_task_message_send;
const unsigned long t_next_message_send = 2000;

#define MSG_SIZE_SEND 20
char ENVIAR_BUFF[MSG_SIZE_SEND];

void setup() {
  Serial.begin(115200);

  analogWriteResolution(12);
  
  M2_newPosition = 0;
  M2_lastPosition = 0;
  M2_increment = 0;
  M2_ref_ang_vel = 0;
  M2_ang_vel = 0;
  M2_control_action = 0;
  M2_control_speed = 0;
  M2_control_speed_u = 0;
  
  M2_Enc = new Encoder(M2_Enc_A_Pin, M2_Enc_B_Pin);
  M2_PID_CONTROLLER = new ZPID(control_Kp, control_Ki, 0, 1, 0, PERIOD_s, control_L_lim, control_U_lim, 0, 0, ZBlockMethod::Z_METHOD_TRAP, ZBlockMethod::Z_METHOD_BE);
  
  t_clock = millis();
  t_task_ctrl = t_clock;
}

void loop() {
  t_clock = millis();

  // TAREA CONTROL
  if (t_task_ctrl <= t_clock) { // TASK TO SEND
    M2_newPosition = M2_Enc->read();
  
    M2_increment = M2_newPosition - M2_lastPosition;
    M2_ang_vel = M2_increment * RADS_CONST;
  
    //  M2_ang_vel = M2_ang_vel * LPF_value + (1 - LPF_value) * M2_ang_vel_past;  //
    //  M2_ang_vel_past = M2_ang_vel;
  
    M2_lastPosition = M2_newPosition;
  
    M2_control_action =  M2_PID_CONTROLLER->update(M2_ref_ang_vel, M2_ang_vel);
    
    M2_control_speed = M2_control_action * (Driver_Resolution / 12);

    M2_control_speed_u = (uint16_t) abs(M2_control_speed);
    if (M2_control_speed == 0 || M2_ref_ang_vel == 0) {
      analogWrite(13, 0);
      analogWrite(12, 0);
    }
    else if (M2_control_speed > 0) {
      analogWrite(13, 0);
      analogWrite(12, M2_control_speed_u);     
    }
    else if (M2_control_speed < 0){
      analogWrite(13, M2_control_speed_u);
      analogWrite(12, 0);       
    }
      
    t_task_ctrl += t_next_ctrl;

    sprintf(ENVIAR_BUFF, "%06.2f,%06.2f\n", M2_ref_ang_vel, M2_ang_vel);
    Serial.write(ENVIAR_BUFF, MSG_SIZE_SEND);
  }

  // TAREA ENVIAR
  if (t_task_message_send <= t_clock) { // TASK TO SEND
    M2_ref_ang_vel = ((float) random(0, 2000))/100.0f - 10;
    t_task_message_send += t_next_message_send;
  }

}
