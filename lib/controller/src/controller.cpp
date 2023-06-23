#include "controller.h"

#include <stdlib.h>
#include <stdio.h>

Controller::Controller(AccelStepper *stepper1, AccelStepper *stepper2, AS5600 *encoder1, AS5600 *encoder2)
{
    g_stepper1 = stepper1;
    g_stepper2 = stepper2;
    g_encoder1 = encoder1;
    g_encoder2 = encoder2;
}


void Controller::step()
{
    float angle1;
    float angle2;
    float speed1;
    float speed2;

    switch (g_m1_state)
    {
    case DISABLED:
        break;
    case INITIALIZATION:
        angle1 = (g_encoder1->getCorrectedAngle()* 360.0) / 0xFFF;
        if(angle1 > 180){
            angle1 = angle1 - 360;
        }
        if(abs(angle1) < 0.1) {
            g_m1_state = MotorControlState::POSITION_CONTROL;
            g_stepper1->setCurrentPosition(0);
            g_m1_setpoint_pos = 0;
            g_stepper1->setMaxSpeed(100);
            g_stepper1->setAcceleration(500.0);
            g_stepper1->moveTo(g_m1_setpoint_pos);
        }
        speed1 = 200 * angle1;
        if (speed1 > 100) {
            speed1 = 100;
        } else if (speed1 < -100) {
            speed1 = -100;
        }
        g_stepper1->setSpeed(speed1);
        printf("M1 - angle: %f, speed: %f\n\r", angle1, speed1);
        break;
    case POSITION_CONTROL:
        g_stepper1->setMaxSpeed(g_m1_setpoint_vel);
        g_stepper1->setAcceleration(500.0);
        g_stepper1->moveTo(g_m1_setpoint_pos);
        break;
    case VELOCITY_CONTROL:
        g_stepper1->setSpeed(g_m1_setpoint_vel);
        break;
    default:
        break;
    }

    switch (g_m2_state)
    {
    case DISABLED:
        break;
    case INITIALIZATION:
        angle2 = (g_encoder2->getCorrectedAngle() * 360.0) / 0xFFF;
        if(angle2 > 180){
            angle2 = angle2 - 360;
        }
        if(abs(angle2) < 0.1) {
            g_m2_state = MotorControlState::POSITION_CONTROL;
            g_stepper2->setCurrentPosition(0);
            g_m2_setpoint_pos = 0;
            g_stepper2->setMaxSpeed(100);
            g_stepper2->setAcceleration(500.0);
            g_stepper2->moveTo(g_m2_setpoint_pos);
        }
        speed2 = 200 * angle2;
        if (speed2 > 100) {
            speed2 = 100;
        } else if (speed2 < -100) {
            speed2 = -100;
        }
        g_stepper2->setSpeed(speed2);
        printf("M2 - angle: %f, speed: %f\n\r", angle2, speed2);
        break;
    case POSITION_CONTROL:
        g_stepper2->setMaxSpeed(g_m2_setpoint_vel);
        g_stepper2->setAcceleration(500.0);
        g_stepper2->moveTo(g_m2_setpoint_pos);
        break;
    case VELOCITY_CONTROL:
        g_stepper2->setSpeed(g_m2_setpoint_vel);
        break;
    default:
        break;
    }
}

void Controller::run()
{
    switch (g_m1_state)
    {
    case DISABLED:
        g_stepper1->run();
        break;
    case INITIALIZATION:
        g_stepper1->runSpeed();
        break;
    case POSITION_CONTROL:
        g_stepper1->run();
        break;
    case VELOCITY_CONTROL:
        g_stepper1->runSpeed();
        break;
    default:
        break;
    }

    switch (g_m2_state)
    {
    case DISABLED:
        g_stepper1->run();
        break;
    case INITIALIZATION:
        g_stepper2->runSpeed();
        break;
    case POSITION_CONTROL:
        g_stepper2->run();
        break;
    case VELOCITY_CONTROL:
        g_stepper2->runSpeed();
        break;
    default:
        break;
    }
}

void Controller::initializeM1()
{
    g_m1_state = MotorControlState::INITIALIZATION;
}

void Controller::initializeM2()
{
    g_m2_state = MotorControlState::INITIALIZATION;
}

void Controller::setM1Position(float position)
{
    g_m1_state = MotorControlState::POSITION_CONTROL;
    g_m1_setpoint_pos = position;
    g_m1_setpoint_vel = 100;
    printf("M1 - setpoint: %f\n\r", g_m1_setpoint_pos);
}

void Controller::setM2Position(float position)
{
    g_m2_state = MotorControlState::POSITION_CONTROL;
    g_m2_setpoint_pos = position;
    g_m2_setpoint_vel = 100;
    printf("M2 - setpoint: %f\n\r", g_m2_setpoint_pos);
}

void Controller::setM1Velocity(float velocity)
{
    g_m1_state = MotorControlState::VELOCITY_CONTROL;
    g_m1_setpoint_vel = velocity;
}

void Controller::setM2Velocity(float velocity)
{
    g_m2_state = MotorControlState::VELOCITY_CONTROL;
    g_m2_setpoint_vel = velocity;
}

void Controller::setM1PositionVelocity(float position, float velocity)
{
    g_m1_state = MotorControlState::POSITION_CONTROL;
    g_m1_setpoint_pos = position;
    g_m1_setpoint_vel = velocity;
    printf("M1 - setpoint pos: %f, vel: %f\n\r", g_m1_setpoint_pos, g_m1_setpoint_vel);
}

void Controller::setM2PositionVelocity(float position, float velocity)
{
    g_m2_state = MotorControlState::POSITION_CONTROL;
    g_m2_setpoint_pos = position;
    g_m2_setpoint_vel = velocity;
    printf("M2 - setpoint pos: %f, vel: %f\n\r", g_m2_setpoint_pos, g_m2_setpoint_vel);
}