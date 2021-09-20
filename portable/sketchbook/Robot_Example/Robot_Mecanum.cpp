#include "Robot_Mecanum.h"

Robot_Mecanum* Robot_Mecanum::robot_= nullptr;;

/**
 * Static methods should be defined outside the class.
 */
Robot_Mecanum *Robot_Mecanum::GetInstance()
{
    /**
     * This is a safer way to create an instance. instance = new Singleton is
     * dangeruous in case two instance threads wants to access at the same time
     */
    if(robot_ == nullptr){
        robot_ = new Robot_Mecanum();
    }
    return robot_;
}



Robot_Mecanum::Robot_Mecanum() {
    AFMS = new Adafruit_MotorShield(); 
    AFMS->begin();

    ina219 = new DFRobot_INA219_IIC(&Wire, INA219_I2C_ADDRESS4);
    while (ina219->begin() != true) {}
    ina219->linearCalibrate(DFRobot_INA219_Reading_mA, DFRobot_INA219_ExtMeter_Reading_mA);

    M1_Motor = AFMS->getMotor(1);
    M2_Motor = AFMS->getMotor(2);
    M3_Motor = AFMS->getMotor(3);
    M4_Motor = AFMS->getMotor(4);

    M1_Motor->setSpeed(0);
    M2_Motor->setSpeed(0);
    M3_Motor->setSpeed(0);
    M4_Motor->setSpeed(0);

    M1_Enc = new Encoder(M1_Enc_A_Pin, M1_Enc_B_Pin);
    M2_Enc = new Encoder(M2_Enc_A_Pin, M2_Enc_B_Pin);
    M3_Enc = new Encoder(M3_Enc_A_Pin, M3_Enc_B_Pin);
    M4_Enc = new Encoder(M4_Enc_A_Pin, M4_Enc_B_Pin);

    M1_PID_CONTROLLER = new ZPID(control_Kp, control_Ki, 0, 1, 0, PERIOD_s, control_L_lim, control_U_lim, 0, 0, ZBlockMethod::Z_METHOD_TRAP, ZBlockMethod::Z_METHOD_BE);
    M2_PID_CONTROLLER = new ZPID(control_Kp, control_Ki, 0, 1, 0, PERIOD_s, control_L_lim, control_U_lim, 0, 0, ZBlockMethod::Z_METHOD_TRAP, ZBlockMethod::Z_METHOD_BE);
    M3_PID_CONTROLLER = new ZPID(control_Kp, control_Ki, 0, 1, 0, PERIOD_s, control_L_lim, control_U_lim, 0, 0, ZBlockMethod::Z_METHOD_TRAP, ZBlockMethod::Z_METHOD_BE);
    M4_PID_CONTROLLER = new ZPID(control_Kp, control_Ki, 0, 1, 0, PERIOD_s, control_L_lim, control_U_lim, 0, 0, ZBlockMethod::Z_METHOD_TRAP, ZBlockMethod::Z_METHOD_BE);

    M1_newPosition = 0;
    M1_lastPosition = 0;
    M1_increment = 0;
    M1_ref_ang_vel = 0;
    M1_ang_vel = 0;
    M1_control_action = 0;
    M1_control_speed = 0;
    M1_control_speed_u = 0;

    M2_newPosition = 0;
    M2_lastPosition = 0;
    M2_increment = 0;
    M2_ref_ang_vel = 0;
    M2_ang_vel = 0;
    M2_control_action = 0;
    M2_control_speed = 0;
    M2_control_speed_u = 0;

    M3_newPosition = 0;
    M3_lastPosition = 0;
    M3_increment = 0;
    M3_ref_ang_vel = 0;
    M3_ang_vel = 0;
    M3_control_action = 0;
    M3_control_speed = 0;
    M3_control_speed_u = 0;

    M4_newPosition = 0;
    M4_lastPosition = 0;
    M4_increment = 0;
    M4_ref_ang_vel = 0;
    M4_ang_vel = 0;
    M4_control_action = 0;
    M4_control_speed = 0;
    M4_control_speed_u = 0;
}

void  Robot_Mecanum::controlHandler(){
    if(robot_ != nullptr){
        robot_->control();
    }  
}

void Robot_Mecanum::start(){
    Timer1.attachInterrupt(Robot_Mecanum::controlHandler);
    Timer1.setPeriod(control_period_us);

    M1_Motor->setSpeed(0);
    M2_Motor->setSpeed(0);
    M3_Motor->setSpeed(0);
    M4_Motor->setSpeed(0);

    M1_ref_ang_vel = 0;
    M2_ref_ang_vel = 0;
    M3_ref_ang_vel = 0;
    M4_ref_ang_vel = 0;

    Timer1.start();
}

void Robot_Mecanum::stop(){
    Timer1.stop();

    M1_Motor->setSpeed(0);
    M2_Motor->setSpeed(0);
    M3_Motor->setSpeed(0);
    M4_Motor->setSpeed(0);
}

void Robot_Mecanum::control()
{
  M1_newPosition = M1_Enc->read();
//  Serial.print(M1_control_speed_u);
//  Serial.print(",");
//  Serial.println(M1_newPosition);
  M2_newPosition = M2_Enc->read();
  M3_newPosition = M3_Enc->read();
  M4_newPosition = M4_Enc->read();

  Battery_Voltage = ina219->getBusVoltage_V();
  control_K_v = Driver_Resolution / Battery_Voltage;

  if(Battery_Voltage > 12){
    M1_increment = M1_newPosition - M1_lastPosition;
    M1_ang_vel = M1_increment * RADS_CONST;

    M2_increment = M2_newPosition - M2_lastPosition;
    M2_ang_vel = M2_increment * RADS_CONST;

    M3_increment = M3_newPosition - M3_lastPosition;
    M3_ang_vel = M3_increment * RADS_CONST;

    M4_increment = M4_newPosition - M4_lastPosition;
    M4_ang_vel = M4_increment * RADS_CONST;

    M1_control_action =  M1_PID_CONTROLLER->update(M1_ref_ang_vel, M1_ang_vel);
    M2_control_action =  M2_PID_CONTROLLER->update(M2_ref_ang_vel, M2_ang_vel);
    M3_control_action =  M3_PID_CONTROLLER->update(M3_ref_ang_vel, M3_ang_vel);
    M4_control_action =  M4_PID_CONTROLLER->update(M4_ref_ang_vel, M4_ang_vel);

    M1_control_action = -1 * M1_control_action;
    M2_control_action = -1 * M2_control_action;
    M3_control_action = -1 * M3_control_action;
    M4_control_action = -1 * M4_control_action;
  
    M1_control_speed = M1_control_action * control_K_v;
    M2_control_speed = M2_control_action * control_K_v;
    M3_control_speed = M3_control_action * control_K_v;
    M4_control_speed = M4_control_action * control_K_v;

    if(M1_control_speed == 0)     M1_Motor->run(RELEASE);
    else if(M1_control_speed > 0) M1_Motor->run(FORWARD);          
    else if(M1_control_speed < 0) M1_Motor->run(BACKWARD);
    M1_control_speed_u = (uint16_t) abs(M1_control_speed);
    
    if(M2_control_speed == 0)     M2_Motor->run(RELEASE);
    else if(M2_control_speed > 0) M2_Motor->run(FORWARD);          
    else if(M2_control_speed < 0) M2_Motor->run(BACKWARD);
    M2_control_speed_u = (uint16_t) abs(M2_control_speed);

    if(M3_control_speed == 0)     M3_Motor->run(RELEASE);
    else if(M3_control_speed > 0) M3_Motor->run(FORWARD);          
    else if(M3_control_speed < 0) M3_Motor->run(BACKWARD);
    M3_control_speed_u = (uint16_t) abs(M3_control_speed);
  
    if(M4_control_speed == 0)     M4_Motor->run(RELEASE);
    else if(M4_control_speed > 0) M4_Motor->run(FORWARD);          
    else if(M4_control_speed < 0) M4_Motor->run(BACKWARD);
    M4_control_speed_u = (uint16_t) abs(M4_control_speed);
    
    M1_lastPosition = M1_newPosition;
    M2_lastPosition = M2_newPosition;
    M3_lastPosition = M3_newPosition;
    M4_lastPosition = M4_newPosition;

    M1_Motor->setSpeed(M1_control_speed_u);
    M2_Motor->setSpeed(M2_control_speed_u);
    M3_Motor->setSpeed(M3_control_speed_u);
    M4_Motor->setSpeed(M4_control_speed_u);
  } 
}

float Robot_Mecanum::get_battery_voltage_level() {
    return ina219->getBusVoltage_V();
}


float Robot_Mecanum::get_M1_rads() {
    return M1_ang_vel;
}

float Robot_Mecanum::get_M2_rads() {
    return M2_ang_vel;
}

float Robot_Mecanum::get_M3_rads() {
    return M3_ang_vel;
}

float Robot_Mecanum::get_M4_rads() {
    return M4_ang_vel;
}

void Robot_Mecanum::set_M1_ref_rads(float M1_ref_ang_vel) {
    this->M1_ref_ang_vel = -1 * M1_ref_ang_vel;
}

void Robot_Mecanum::set_M2_ref_rads(float M2_ref_ang_vel) {
    this->M2_ref_ang_vel = -1 * M2_ref_ang_vel;
}

void Robot_Mecanum::set_M3_ref_rads(float M3_ref_ang_vel) {
    this->M3_ref_ang_vel = -1 * M3_ref_ang_vel;
}

void Robot_Mecanum::set_M4_ref_rads(float M4_ref_ang_vel) {
    this->M4_ref_ang_vel = -1 * M4_ref_ang_vel;
}
