#include "Controller.h"

#include <stdlib.h>
#include <stdio.h>

Controller::Controller()
{
}

void Controller::addM1(AccelStepper *stepper1, AS5600 *encoder1)
{
    g_stepper1 = stepper1;
    g_encoder1 = encoder1;
}

void Controller::addM2(AccelStepper *stepper2, AS5600 *encoder2)
{
    g_stepper2 = stepper2;
    g_encoder2 = encoder2;
}

void Controller::addM3(AccelStepper *stepper3, AS5600 *encoder3)
{
    g_stepper3 = stepper3;
    g_encoder3 = encoder3;
}

void Controller::addM4(DCMotor *motor4)
{
    g_motor4 = motor4;
}

void Controller::addM5(DCMotor *motor5)
{
    g_motor5 = motor5;
}

AccelStepper *Controller::getM1()
{
    return g_stepper1;
}

AccelStepper *Controller::getM2()
{
    return g_stepper2;
}

AccelStepper *Controller::getM3()
{
    return g_stepper3;
}

DCMotor *Controller::getM4()
{
    return g_motor4;
}

DCMotor *Controller::getM5()
{
    return g_motor5;
}

AS5600 *Controller::getE1()
{
    return g_encoder1;
}

AS5600 *Controller::getE2()
{
    return g_encoder2;
}

AS5600 *Controller::getE3()
{
    return g_encoder3;
}


void Controller::step()
{
    float angle1;
    float angle2;
    float angle3;
    float speed1;
    float speed2;
    float speed3;

    switch (g_m1_state)
    {
    case DISABLED:
        break;
    case INITIALIZATION:
        if(g_encoder1 != NULL){
            angle1 = (g_encoder1->getCorrectedAngle()* 360.0) / 0xFFF;
        } else {
            angle1 = 0;
        }
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
        g_stepper1->setMaxSpeed(speed1);
        g_stepper1->setSpeed(speed1);
        printf("M1 - angle: %f, speed: %f\n\r", angle1, speed1);
        break;
    case POSITION_CONTROL:
        g_stepper1->setMaxSpeed(g_m1_setpoint_vel);
        g_stepper1->setAcceleration(500.0);
        g_stepper1->moveTo(g_m1_setpoint_pos);
        break;
    case VELOCITY_CONTROL:
        g_stepper1->setMaxSpeed(g_m1_setpoint_vel);
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
        if(g_encoder2 != NULL){
            angle2 = (g_encoder2->getCorrectedAngle() * 360.0) / 0xFFF;
        } else {
            angle2 = 0;
        }
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
        g_stepper2->setMaxSpeed(speed2);
        g_stepper2->setSpeed(speed2);
        printf("M2 - angle: %f, speed: %f\n\r", angle2, speed2);
        break;
    case POSITION_CONTROL:
        g_stepper2->setMaxSpeed(g_m2_setpoint_vel);
        g_stepper2->setAcceleration(500.0);
        g_stepper2->moveTo(g_m2_setpoint_pos);
        break;
    case VELOCITY_CONTROL:
        g_stepper2->setMaxSpeed(g_m2_setpoint_vel);
        g_stepper2->setSpeed(g_m2_setpoint_vel);
        break;
    default:
        break;
    }

    switch (g_m3_state)
    {
    case DISABLED:
        break;
    case INITIALIZATION:
        if(g_encoder3 != NULL){
            angle3 = (g_encoder3->getCorrectedAngle() * 360.0) / 0xFFF;
        } else {
            angle3 = 0;
        }
        if(angle3 > 180){
            angle3 = angle3 - 360;
        }
        if(abs(angle3) < 0.1) {
            g_m3_state = MotorControlState::POSITION_CONTROL;
            g_stepper3->setCurrentPosition(0);
            g_m3_setpoint_pos = 0;
            g_stepper3->setMaxSpeed(100);
            g_stepper3->setAcceleration(500.0);
            g_stepper3->moveTo(g_m3_setpoint_pos);
        }
        speed3 = 200 * angle3;
        if (speed3 > 100) {
            speed3 = 100;
        } else if (speed3 < -100) {
            speed3 = -100;
        }
        g_stepper3->setMaxSpeed(speed3);
        g_stepper3->setSpeed(speed3);
        printf("M3 - angle: %f, speed: %f\n\r", angle3, speed3);
        break;
    case POSITION_CONTROL:
        g_stepper3->setMaxSpeed(g_m3_setpoint_vel);
        g_stepper3->setAcceleration(500.0);
        g_stepper3->moveTo(g_m3_setpoint_pos);
        break;
    case VELOCITY_CONTROL:
        g_stepper3->setMaxSpeed(g_m3_setpoint_vel);
        g_stepper3->setSpeed(g_m3_setpoint_vel);
        break;
    default:
        break;
    }

    switch (g_m4_state)
    {
    case DISABLED:
        break;
    case INITIALIZATION:
        break;
    case POSITION_CONTROL:
        break;
    case VELOCITY_CONTROL:
        g_motor4->setSpeed(g_m4_setpoint_vel);
        break;
    default:
        break;
    }

    switch (g_m5_state)
    {   
    case DISABLED:
        break;
    case INITIALIZATION:
        break;
    case POSITION_CONTROL:
        break;
    case VELOCITY_CONTROL:
        g_motor5->setSpeed(g_m5_setpoint_vel);
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
        g_stepper2->run();
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

    switch (g_m3_state)
    {
    case DISABLED:
        g_stepper3->run();
        break;
    case INITIALIZATION:
        g_stepper3->runSpeed();
        break;
    case POSITION_CONTROL:
        g_stepper3->run();
        break;
    case VELOCITY_CONTROL:
        g_stepper3->runSpeed();
        break;
    default:
        break;
    }

    switch (g_m4_state)
    {
    case DISABLED:
        g_motor4->run();
        break;
    case INITIALIZATION:
        g_motor4->runSpeed();
        break;
    case POSITION_CONTROL:
        g_motor4->run();
        break;
    case VELOCITY_CONTROL:
        g_motor4->runSpeed();
        break;      
    default:
        break;
    }

    switch (g_m5_state)
    {   
    case DISABLED:
        g_motor5->run();
        break;
    case INITIALIZATION:    
        g_motor5->runSpeed();
        break;
    case POSITION_CONTROL:
        g_motor5->run();
        break;
    case VELOCITY_CONTROL:
        g_motor5->runSpeed();
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

void Controller::initializeM3()
{
    g_m3_state = MotorControlState::INITIALIZATION;
}

void Controller::initializeM4()
{
    //g_m4_state = MotorControlState::INITIALIZATION;
}

void Controller::initializeM5()
{
    //g_m5_state = MotorControlState::INITIALIZATION;
}

void Controller::setM1Position(float position)
{
    g_m1_state = MotorControlState::POSITION_CONTROL;
    g_m1_setpoint_pos = position;
    g_m1_setpoint_vel = 3000;
    printf("M1 - setpoint: %f\n\r", g_m1_setpoint_pos);
}

void Controller::setM2Position(float position)
{
    g_m2_state = MotorControlState::POSITION_CONTROL;
    g_m2_setpoint_pos = position;
    g_m2_setpoint_vel = 3000;
    printf("M2 - setpoint: %f\n\r", g_m2_setpoint_pos);
}

void Controller::setM3Position(float position)
{
    g_m3_state = MotorControlState::POSITION_CONTROL;
    g_m3_setpoint_pos = position;
    g_m3_setpoint_vel = 3000;
    printf("M3 - setpoint: %f\n\r", g_m3_setpoint_pos);
}

void Controller::setM4Position(float position)
{
    g_m4_state = MotorControlState::POSITION_CONTROL;
    g_m4_setpoint_pos = position;
    g_m4_setpoint_vel = 3000;
    printf("M4 - setpoint: %f\n\r", g_m4_setpoint_pos);
}

void Controller::setM5Position(float position)
{
    g_m5_state = MotorControlState::POSITION_CONTROL;
    g_m5_setpoint_pos = position;
    g_m5_setpoint_vel = 3000;
    printf("M5 - setpoint: %f\n\r", g_m5_setpoint_pos);
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

void Controller::setM3Velocity(float velocity)
{
    g_m3_state = MotorControlState::VELOCITY_CONTROL;
    g_m3_setpoint_vel = velocity;
}

void Controller::setM4Velocity(float velocity)
{
    g_m4_state = MotorControlState::VELOCITY_CONTROL;
    g_m4_setpoint_vel = velocity;
}

void Controller::setM5Velocity(float velocity)
{
    g_m5_state = MotorControlState::VELOCITY_CONTROL;
    g_m5_setpoint_vel = velocity;
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

void Controller::setM3PositionVelocity(float position, float velocity)
{
    g_m3_state = MotorControlState::POSITION_CONTROL;
    g_m3_setpoint_pos = position;
    g_m3_setpoint_vel = velocity;
    printf("M3 - setpoint pos: %f, vel: %f\n\r", g_m3_setpoint_pos, g_m3_setpoint_vel);
}

void Controller::setM4PositionVelocity(float position, float velocity)
{
    g_m4_state = MotorControlState::POSITION_CONTROL;
    g_m4_setpoint_pos = position;
    g_m4_setpoint_vel = velocity;
    printf("M4 - setpoint pos: %f, vel: %f\n\r", g_m4_setpoint_pos, g_m4_setpoint_vel);
}

void Controller::setM5PositionVelocity(float position, float velocity)
{
    g_m5_state = MotorControlState::POSITION_CONTROL;
    g_m5_setpoint_pos = position;
    g_m5_setpoint_vel = velocity;
    printf("M5 - setpoint pos: %f, vel: %f\n\r", g_m5_setpoint_pos, g_m5_setpoint_vel);
}