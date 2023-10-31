/**
 * @file MotorController.h
 * @author Edgar W (mail@techniccontroller.com)
 * @brief 
 * @version 0.1
 * @date 2023-06-04
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef RPI_PICO_CONTROLLER_H
#define RPI_PICO_CONTROLLER_H

#include "AccelStepper.h"
#include "DCMotor.h"
#include "AS5600.h"
#include <defines_constants.h>
#include "StepperConfiguration.h"


class MotorController {

    // enum for setpoint type (velocity, position)
    enum SetpointType {
        VELOCITY,
        POSITION
    };

    enum MotorControlState {
        DISABLED,
        INITIALIZATION,
        POSITION_CONTROL,
        VELOCITY_CONTROL
    };
    // struct to hold setpoint (value, type, motor)
    struct Setpoint {
        float value;
        SetpointType type;
        AccelStepper *motor;
    };

public:
    MotorController(StepperConfiguration *stepperConfiguration);

    void addM1(AccelStepper *stepper1 = NULL, AS5600 *encoder1 = NULL);
    void addM2(AccelStepper *stepper2 = NULL, AS5600 *encoder2 = NULL);
    void addM3(AccelStepper *stepper3 = NULL, AS5600 *encoder3 = NULL);
    void addM4(DCMotor *motor4 = NULL);
    void addM5(DCMotor *motor5 = NULL);

    void stepStepper(AccelStepper * stepper, AS5600 * encoder, MotorControlState * state, float * setpoint_pos, float * setpoint_vel);
    void step();
    void run();
    void reset();

    void initializeM1();
    void initializeM2();
    void initializeM3();
    void initializeM4();
    void initializeM5();

    void setM1Position(float position);
    void setM2Position(float position);
    void setM3Position(float position);
    void setM4Position(float position);
    void setM5Position(float position);

    void setM1Velocity(float velocity);
    void setM2Velocity(float velocity);
    void setM3Velocity(float velocity);
    void setM4Velocity(float velocity);
    void setM5Velocity(float velocity);

    void setM1PositionVelocity(float position, float velocity);
    void setM2PositionVelocity(float position, float velocity);
    void setM3PositionVelocity(float position, float velocity);
    void setM4PositionVelocity(float position, float velocity);
    void setM5PositionVelocity(float position, float velocity);

    AccelStepper *getM1();
    AccelStepper *getM2();
    AccelStepper *getM3();
    DCMotor *getM4();
    DCMotor *getM5();

    AS5600 *getE1();
    AS5600 *getE2();
    AS5600 *getE3();

private:
    float g_m1_setpoint_pos = 0;
    float g_m2_setpoint_pos = 0;
    float g_m3_setpoint_pos = 0;
    float g_m4_setpoint_pos = 0;
    float g_m5_setpoint_pos = 0;

    float g_m1_setpoint_vel = 0;
    float g_m2_setpoint_vel = 0;
    float g_m3_setpoint_vel = 0;
    float g_m4_setpoint_vel = 0;
    float g_m5_setpoint_vel = 0;

    MotorControlState g_m1_state = MotorControlState::DISABLED;
    MotorControlState g_m2_state = MotorControlState::DISABLED;
    MotorControlState g_m3_state = MotorControlState::DISABLED;
    MotorControlState g_m4_state = MotorControlState::DISABLED;
    MotorControlState g_m5_state = MotorControlState::DISABLED;

    StepperConfiguration *m_stepperConfiguration; /**< Stepper configuration. */

    AccelStepper *g_stepper1;
    AccelStepper *g_stepper2;
    AccelStepper *g_stepper3;
    DCMotor *g_motor4;
    DCMotor *g_motor5;
    AS5600 *g_encoder1;
    AS5600 *g_encoder2;
    AS5600 *g_encoder3;
};



#endif //RPI_PICO_CONTROLLER_H