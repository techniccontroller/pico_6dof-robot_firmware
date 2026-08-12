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


#define MICROSTEP_RESOLUTION 8

/**
 * @brief I2C addresses
 * 
 */
#define TCAADDR 0x70
#define ASADDR 0x36

/**
 * @brief PIN MAPPING
 * 
 */
// I2C
#define I2C_PORT0 i2c0
#define I2C_SDA0_PIN 12
#define I2C_SCL0_PIN 13

// Stepper Driver control pins
#define MS1_PIN 18
#define MS2_PIN 17
#define MS3_PIN 16

// J1 analog Hall-effect endstop input (ADC2). Endstop behavior is not yet active.
#define J1_HALL_SENSOR_PIN 28
#define J1_HALL_SENSOR_ADC_INPUT 2
#define J1_HALL_SENSOR_ACTIVE_THRESHOLD 1700
#define J1_HALL_SENSOR_DEBOUNCE_SAMPLES 3
#define J1_HOMING_SPEED_DEG_S 10.0f
#define J1_HOMING_DIRECTION -1.0f
#define J1_HOMING_TIMEOUT_MS 30000
#define J1_ENDSTOP_POSITION_DEG -91.0f

// Motor 1 is the motor for Joint 1 (stepper motor)
#define MOTOR1_STEP_PIN 20
#define MOTOR1_DIR_PIN 19

// Motor 2 is the motor for Joint 2 (stepper motor)
#define MOTOR2_STEP_PIN 22
#define MOTOR2_DIR_PIN 21

// Motor 3 is the motor for Joint 3 (stepper motor)
#define MOTOR3_STEP_PIN 27
#define MOTOR3_DIR_PIN 26

// Motor 5 and Motor 6 work together for Joint 5 and Joint 6 (DC motors)
#define MOTOR5_ENABLE_PIN 6
#define MOTOR5_IN1_PIN 7
#define MOTOR5_IN2_PIN 8

#define MOTOR6_ENABLE_PIN 11
#define MOTOR6_IN1_PIN 10
#define MOTOR6_IN2_PIN 9

// Servo
#define SERVO_CONTINUE_J4_PIN 14
#define SERVO_GRIPPER_PIN 15

// Continuous-rotation servo commands for Joint 4. Tune the neutral command if
// the servo creeps while stopped; set direction to -1.0f to reverse it.
#define CONTINUOUS_SERVO_J4_NEUTRAL_COMMAND 100.0f
#define CONTINUOUS_SERVO_J4_MIN_COMMAND 0.0f
#define CONTINUOUS_SERVO_J4_MAX_COMMAND 180.0f
#define CONTINUOUS_SERVO_J4_DIRECTION -1.0f


/**
 * @brief EEPROM addresses
 * 
 */
#define EEPROM_SIZE 512
#define EEPROM_ADDR_ENCJ2 0
#define EEPROM_ADDR_ENCJ3 4
#define EEPROM_ADDR_ENCJ5 12 // J5/J6 logical axes swapped; retain each physical encoder's saved zero
#define EEPROM_ADDR_ENCJ6 8
#define EEPROM_ADDR_ENCJ4 16


/**
 * @brief constants for the robot
 * 
 */
#define DEFAULT_VEL_STEPPER 30.0f       // 3000 steps/s = 96 deg/s
#define MAX_VEL_DCMOTOR 200.0f      // PWM
#define DEFAULT_ACC_STEPPER 5000.0f 

#define INIT_VEL_STEPPER 3.2f       // 100 steps/s = 3.2 deg/s
#define INIT_VEL_DCMOTOR 50.0f      // PWM
#define STEPPER_ENCODER_POSITION_TOLERANCE_DEG 0.10f
#define J2_ENCODER_DRIVE_DIRECTION -1.0f
#define J3_ENCODER_DRIVE_DIRECTION -1.0f
#define J2_J3_PARALLEL_COUPLING_RATIO 1.0f
#define DCMOTOR_POSITION_TOLERANCE_DEG 0.50f
#define DCMOTOR_POSITION_REENGAGE_DEG 1.50f
#define DCMOTOR_MAX_PWM 255.0f
#define DCMOTOR_INTEGRAL_OUTPUT_LIMIT 50.0f
#define DCMOTOR_DERIVATIVE_FILTER_ALPHA 0.15f

// Calibratable differential-wrist plant inverse after swapping logical J5/J6.
// [M5]   [ J5_TO_M5  J6_TO_M5 ] [J5 command]
// [M6] = [ J5_TO_M6  J6_TO_M6 ] [J6 command]
#define J5_TO_M5_COEFFICIENT 1.0f
#define J5_TO_M6_COEFFICIENT -1.0f
#define J6_TO_M5_COEFFICIENT -1.0f
#define J6_TO_M6_COEFFICIENT -1.0f

// These can be calibrated independently for unequal motors and directions.
#define MOTOR5_MIN_PWM_POSITIVE 20.0f
#define MOTOR5_MIN_PWM_NEGATIVE 20.0f
#define MOTOR6_MIN_PWM_POSITIVE 20.0f
#define MOTOR6_MIN_PWM_NEGATIVE 20.0f
#define DEFAULT_VEL_CONTINUOUS_SERVO 50.0f // Offset from neutral servo command
#define MAX_VEL_CONTINUOUS_SERVO 80.0f     // Offset from neutral servo command
#define J4_POSITION_TOLERANCE_DEG 0.18f    // About two AS5600 counts; suppresses encoder chatter
#define J4_MIN_DRIVE_COMMAND 3.0f          // Measured servo command needed to overcome stiction
#define J4_HOLD_MAX_COMMAND 12.0f          // Limit corrections after a zero-speed hold request
#define J4_VELOCITY_FILTER_ALPHA 0.25f     // Low-pass filter for the differentiated encoder angle
#define J4_STOPPED_VELOCITY_DEG_S 0.6f
#define J4_INTEGRAL_LIMIT 12.0f            // Maximum integral contribution in servo-command units
#define J4_NEUTRAL_LEARNING_ERROR_DEG 0.75f
#define J4_NEUTRAL_ERROR_GAIN 2.0f         // Neutral-command units per degree per second
#define J4_NEUTRAL_VELOCITY_GAIN 0.15f     // Neutral-command units per measured degree
#define J4_NEUTRAL_MAX_RATE 1.0f           // Maximum neutral-command change per second
#define J4_NEUTRAL_ESTIMATOR_MAX_OFFSET 10.0f
#define J4_PID_P_DEFAULT 0.5f
#define J4_PID_I_DEFAULT 0.0f            // Neutral adaptation provides the bias correction
#define J4_PID_D_DEFAULT 0.10f

#define LIMIT_J2_MIN -31.0f
#define LIMIT_J2_MAX 110.0f
#define LIMIT_J3_MIN -120.0f
#define LIMIT_J3_MAX 10.0f
#define LIMIT_J5_MIN -60.0f
#define LIMIT_J5_MAX 160.0f
#define LIMIT_J6_MIN -160.0f
#define LIMIT_J6_MAX 60.0f
#define LIMIT_J2_J3_DIFF_MIN -5.0f
#define LIMIT_J2_J3_DIFF_MAX 108.0f

#define LIMIT_GRIP_OPEN 10.0f
#define LIMIT_GRIP_CLOSE 170.0f


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
    M6,
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
    J6,
    JNONE = 99
};
