/**
 * @file defines_constants.h
 * @author Edgar W (mail@techniccontroller.com)
 * @brief In this file all constants like parameters and pin mappings are defined.
 * @version 0.1
 * @date 2023-05-29
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once


#define I2C_PORT0 i2c0
#define I2C_SDA0 12
#define I2C_SCL0 13

#define I2C_PORT1 i2c1
#define I2C_SDA1 14
#define I2C_SCL1 15


#define TCAADDR 0x70
#define ASADDR 0x36


// Pin defines
// Motor 1 is the motor for Joint 1
#define MOTOR1_STEP_PIN 20
#define MOTOR1_DIR_PIN 19

// Motor 2 is the motor for Joint 2
#define MOTOR2_STEP_PIN 22
#define MOTOR2_DIR_PIN 21

// Motor 3 is the motor for Joint 3
#define MOTOR3_STEP_PIN 27
#define MOTOR3_DIR_PIN 26

// Motor 4 and Motor 5 working together for Joint 4 and Joint 5
#define MOTOR4_ENABLE_PIN 6
#define MOTOR4_IN1_PIN 7
#define MOTOR4_IN2_PIN 8
#define MOTOR4_ENC_A_PIN 2
#define MOTOR4_ENC_B_PIN 3

#define MOTOR5_ENABLE_PIN 11
#define MOTOR5_IN1_PIN 10
#define MOTOR5_IN2_PIN 9
#define MOTOR5_ENC_A_PIN 4
#define MOTOR5_ENC_B_PIN 5

#define MS1_PIN 18
#define MS2_PIN 17
#define MS3_PIN 16
#define ENABLE_PIN 28

#define EEPROM_SIZE 512
#define EEPROM_ADDR_ENCJ2 0
#define EEPROM_ADDR_ENCJ3 4
#define EEPROM_ADDR_ENCJ4 8
#define EEPROM_ADDR_ENCJ5 12

#define MAX_VEL_STEPPER 96.0f       // 3000 steps/s = 96 deg/s
#define MAX_VEL_DCMOTOR 200.0f      // PWM

#define INIT_VEL_STEPPER 3.2f       // 100 steps/s = 3.2 deg/s
#define INIT_VEL_DCMOTOR 50.0f      // PWM

/**
 * @brief Enum for the different motors
 * 
 */
enum Motor
{
	M1=1,
    M2,
    M3,
    M4,
    M5,
    MNONE = 99
};

/**
 * @brief Enum for the different joints
 * 
 */
enum Joint
{
    J1=1,
    J2,
    J3,
    J4,
    J5,
    JNONE = 99
};
