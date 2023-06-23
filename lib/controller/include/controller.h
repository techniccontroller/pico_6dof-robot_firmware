/**
 * @file controller.h
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
#include "AS5600.h"


class Controller {

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

    Controller(AccelStepper *stepper1=NULL, AccelStepper *stepper2=NULL, AS5600 *encoder1=NULL, AS5600 *encoder2=NULL);

    void step();
    void run();

    void initializeM1();
    void initializeM2();

    void setM1Position(float position);
    void setM2Position(float position);
    void setM1Velocity(float velocity);
    void setM2Velocity(float velocity);
    void setM1PositionVelocity(float position, float velocity);
    void setM2PositionVelocity(float position, float velocity);

    private:

    float g_m1_setpoint_pos = 0;
    float g_m2_setpoint_pos = 0;
    float g_m1_setpoint_vel = 0;
    float g_m2_setpoint_vel = 0;
    MotorControlState g_m1_state = MotorControlState::DISABLED;
    MotorControlState g_m2_state = MotorControlState::DISABLED;

    AccelStepper *g_stepper1;
    AccelStepper *g_stepper2;
    AS5600 *g_encoder1;
    AS5600 *g_encoder2;

};



#endif //RPI_PICO_CONTROLLER_H