#ifndef ROBOT_MECANUM_H
#define ROBOT_MECANUM_H

/* Singleton design pattern */

/**
 * The Robot_Mecanum class defines the `GetInstance` method that serves as an
 * alternative to constructor and lets clients access the same instance of this
 * class over and over.
 */

#include "Arduino.h"

#include <ZBlock.h>
#include <ZPID.h>
#include <Encoder.h>
#include <Adafruit_MotorShield.h>
#include <DueTimer.h>
#include <Wire.h>
#include <DFRobot_INA219.h>

class Robot_Mecanum {
    private:
        /* Period time */
        const float PERIOD_ms = 50.0f;
        const float PERIOD_s = PERIOD_ms / 1000.0f;
        const float PERIOD_us = PERIOD_ms * 1000.0f;

        /* Motor constants */
        const float MOTOR_GEAR_RATIO = 70.0f;
        const float ENCODER_CPR = 64.0f;
        const float RADS_CONST = ((2.0f * PI) / (MOTOR_GEAR_RATIO * ENCODER_CPR * PERIOD_s));

        /* Pinout */
        const uint8_t  M1_Enc_A_Pin = 49;
        const uint8_t  M1_Enc_B_Pin = 48;

        const uint8_t  M2_Enc_A_Pin = 50;
        const uint8_t  M2_Enc_B_Pin = 51;

        const uint8_t  M3_Enc_A_Pin = 27;
        const uint8_t  M3_Enc_B_Pin = 26;

        const uint8_t  M4_Enc_A_Pin = 24;
        const uint8_t  M4_Enc_B_Pin = 25;

        /* Battery Sensor */
        DFRobot_INA219_IIC * ina219;
        float DFRobot_INA219_Reading_mA = 1000;
        float DFRobot_INA219_ExtMeter_Reading_mA = 1000;

        /* Driver resolution */
        const float Driver_Resolution = 4096.0f;

        /* */
        const long control_period_us = (long) PERIOD_us;
        const float control_Kp = 0.13567;
        const float control_Ki = 4.66;
        const float control_L_lim = -12;
        const float control_U_lim = 12;
        float control_K_v;

        float Battery_Voltage;

        Adafruit_MotorShield * AFMS; 

        Adafruit_DCMotor * M1_Motor;
        Encoder * M1_Enc;
        long M1_newPosition;
        long M1_lastPosition;
        long M1_increment;
        float M1_ref_ang_vel;
        float M1_ang_vel; // Angular Velocity
        float M1_control_action;
        float M1_control_speed;
        uint16_t M1_control_speed_u;
        ZPID * M1_PID_CONTROLLER;

        Adafruit_DCMotor * M2_Motor;
        Encoder * M2_Enc;
        long M2_newPosition;
        long M2_lastPosition;
        long M2_increment;
        float M2_ref_ang_vel;
        float M2_ang_vel; // Angular Velocity
        float M2_control_action;
        float M2_control_speed;
        uint16_t M2_control_speed_u ;
        ZPID * M2_PID_CONTROLLER;

        Adafruit_DCMotor * M3_Motor;
        Encoder * M3_Enc;
        long M3_newPosition;
        long M3_lastPosition;
        long M3_increment;
        float M3_ref_ang_vel;
        float M3_ang_vel; // Angular Velocity
        float M3_control_action;
        float M3_control_speed;
        uint16_t M3_control_speed_u;
        ZPID * M3_PID_CONTROLLER;

        Adafruit_DCMotor * M4_Motor;
        Encoder * M4_Enc;
        long M4_newPosition;
        long M4_lastPosition;
        long M4_increment;
        float M4_ref_ang_vel;
        float M4_ang_vel; // Angular Velocity
        float M4_control_action;
        float M4_control_speed;
        uint16_t M4_control_speed_u;
        ZPID * M4_PID_CONTROLLER;
    protected:
        Robot_Mecanum();

        static Robot_Mecanum* robot_;

        static void controlHandler();
    public:
        /**
         * Robot_Mecanum should not be cloneable.
         */
        Robot_Mecanum(Robot_Mecanum &other) = delete;
        /**
         * Singletons should not be assignable.
         */
        void operator=(const Robot_Mecanum &) = delete;
        /**
         * This is the static method that controls the access to the singleton
         * instance. On the first run, it creates a singleton object and places it
         * into the static field. On subsequent runs, it returns the client existing
         * object stored in the static field.
         */

        static Robot_Mecanum *GetInstance();

        void start();
        void stop();

        void control();

        float get_battery_voltage_level();

        float get_M1_rads();
        float get_M2_rads();
        float get_M3_rads();
        float get_M4_rads();

        void set_M1_ref_rads(float M1_ref_ang_vel);
        void set_M2_ref_rads(float M2_ref_ang_vel);
        void set_M3_ref_rads(float M3_ref_ang_vel);
        void set_M4_ref_rads(float M4_ref_ang_vel);
};

#endif /* ROBOT_MECANUM_H */
