#define SERIAL_DEBUG Serial

#include <ZBlock.h>
#include <ZPID.h>
#include <Encoder.h>
#include <Adafruit_MotorShield.h>
#include <DueTimer.h>
#include <Wire.h>
#include <DFRobot_INA219.h>

DFRobot_INA219_IIC ina219(&Wire, INA219_I2C_ADDRESS4);
float DFRobot_INA219_Reading_mA = 1000;
float DFRobot_INA219_ExtMeter_Reading_mA = 1000;

const float Driver_Resolution = 4096.0f;
float Battery_Voltage;
float K_v;

#define PERIOD_ms 50.0f
#define PERIOD_s (PERIOD_ms / 1000.0f)
#define PERIOD_us 50.0f * 1000.0f


#define MOTOR_GEAR_RATIO 70.0f
#define ENCODER_CPR 64.0f
#define RADS_CONST ((2.0f * PI) / (MOTOR_GEAR_RATIO * ENCODER_CPR * PERIOD_s))

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *M4_Motor = AFMS.getMotor(4);
#define M4_Enc_A_Pin 24
#define M4_Enc_B_Pin 25
Encoder M4_Enc(M4_Enc_A_Pin, M4_Enc_B_Pin);
long M4_newPosition, M4_lastPosition, M4_increment = 0;
float M4_ref_ang_vel = 0;
float M4_ang_vel = 0; // Angular Velocity
float M4_control_action = 0;
float M4_control_speed = 0;
uint16_t M4_control_speed_u = 0;

long control_period_us = (long) PERIOD_us;
void control_Handler();
float control_Kp = 1.28/2;
float control_Ki = 9.35/2;
float control_L_lim = -12;
float control_U_lim = 12;

ZPID * M4_PID_CONTROLLER;

void setup()
{
  SERIAL_DEBUG.begin(115200);
  
  while (ina219.begin() != true) {}
  ina219.linearCalibrate(DFRobot_INA219_Reading_mA, DFRobot_INA219_ExtMeter_Reading_mA);

  AFMS.begin();
  M4_Motor->setSpeed(0);

  M4_PID_CONTROLLER = new ZPID(control_Kp, control_Ki, 0, 10, 0, PERIOD_s, control_L_lim, control_U_lim, 0, 0, ZBlockMethod::Z_METHOD_TRAP, ZBlockMethod::Z_METHOD_BE);

  Timer1.attachInterrupt(control_Handler).setPeriod(control_period_us).start();
}

void loop()
{
}

int counter = 0;
void control_Handler()
{
  M4_newPosition = M4_Enc.read();
  Battery_Voltage = ina219.getBusVoltage_V();

  if(Battery_Voltage > 12){
    M4_increment = M4_newPosition - M4_lastPosition;
    M4_ang_vel = M4_increment * RADS_CONST;
    M4_control_action =  M4_PID_CONTROLLER->update(M4_ref_ang_vel, M4_ang_vel);
    M4_control_action = -1 * M4_control_action;
  
    K_v = Driver_Resolution / Battery_Voltage;
    M4_control_speed = M4_control_action * K_v;
  
    if(M4_control_speed == 0)     M4_Motor->run(RELEASE);
    else if(M4_control_speed > 0) M4_Motor->run(FORWARD);          
    else if(M4_control_speed < 0) M4_Motor->run(BACKWARD);
    M4_control_speed_u = (uint16_t) abs(M4_control_speed);
    M4_Motor->setSpeed(M4_control_speed_u);
    
    M4_lastPosition = M4_newPosition;
  } 
  
  SERIAL_DEBUG.print(M4_ref_ang_vel);
  SERIAL_DEBUG.print(",");
  SERIAL_DEBUG.print(M4_ang_vel);
  SERIAL_DEBUG.print(",");
  SERIAL_DEBUG.print(M4_control_action);
  SERIAL_DEBUG.println();
}
