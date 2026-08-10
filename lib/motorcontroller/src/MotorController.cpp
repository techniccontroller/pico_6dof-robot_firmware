#include "MotorController.h"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <algorithm>

MotorController::MotorController(StepperConfiguration *stepperConfiguration)
{
    m_stepperConfiguration = stepperConfiguration;
}

void MotorController::addM1(AccelStepper *stepper1, AS5600 *encoder1)
{
    g_stepper1 = stepper1;
    g_encoder1 = encoder1;
}

void MotorController::addM2(AccelStepper *stepper2, AS5600 *encoder2)
{
    g_stepper2 = stepper2;
    g_encoder2 = encoder2;
}

void MotorController::addM3(AccelStepper *stepper3, AS5600 *encoder3)
{
    g_stepper3 = stepper3;
    g_encoder3 = encoder3;
}

void MotorController::addM4(ContinuousServo *motor4)
{
    g_motor4 = motor4;
}

void MotorController::addM5(DCMotor *motor5)
{
    g_motor5 = motor5;
}

void MotorController::addM6(DCMotor *motor6)
{
    g_motor6 = motor6;
}

AccelStepper *MotorController::getM1()
{
    return g_stepper1;
}

AccelStepper *MotorController::getM2()
{
    return g_stepper2;
}

AccelStepper *MotorController::getM3()
{
    return g_stepper3;
}

ContinuousServo *MotorController::getM4()
{
    return g_motor4;
}

DCMotor *MotorController::getM5()
{
    return g_motor5;
}

DCMotor *MotorController::getM6()
{
    return g_motor6;
}

AS5600 *MotorController::getE1()
{
    return g_encoder1;
}

AS5600 *MotorController::getE2()
{
    return g_encoder2;
}

AS5600 *MotorController::getE3()
{
    return g_encoder3;
}

/**
 * @brief Performs a single step of the motor controller for a single stepper motor (either stepper1, stepper2 or stepper3)
 * 
 * @param stepper   The stepper motor to control
 * @param encoder   The encoder to read the current position from
 * @param state     The current state of the motor to control
 * @param setpoint_pos  The current position setpoint of the motor to control [deg]
 * @param setpoint_vel  The current velocity setpoint of the motor to control [deg/s]
 */
void MotorController::stepStepper(AccelStepper * stepper, AS5600 * encoder, MotorControlState * state, float * setpoint_pos, float * setpoint_vel)
{
    long speed_steps = 0;
    long setpoint_steps = 0;
    float angle = 0;
    float speed = 0;

    switch (*state)
    {
    case DISABLED:
        break;
    case INITIALIZATION:
        if(encoder != NULL){
            angle = (encoder->getCorrectedAngle()* 360.0) / 0xFFF;
        } else {
            angle = 0;
        }
        if(angle > 180){
            angle = angle - 360;
        }
        if(abs(angle) < 0.1) {
            *state = MotorControlState::POSITION_CONTROL;
            stepper->setCurrentPosition(0);
            *setpoint_pos = 0;
            *setpoint_vel = INIT_VEL_STEPPER;
            printf("M1/2/3 - initialized\n\r");
        }
        speed = std::clamp(5 * angle, -INIT_VEL_STEPPER, INIT_VEL_STEPPER);
        speed_steps = m_stepperConfiguration->angleDegToSteps(speed);
        stepper->setMaxSpeed(speed_steps);
        stepper->setSpeed(speed_steps);
        printf("M1/2/3 - angle: %f, speed: %f\n\r", angle, speed);
        break;
    case POSITION_CONTROL:
        speed_steps = m_stepperConfiguration->angleDegToSteps(*setpoint_vel);
        stepper->setMaxSpeed(speed_steps);
        stepper->setAcceleration(500.0);
        setpoint_steps = m_stepperConfiguration->angleDegToSteps(*setpoint_pos);
        stepper->moveTo(setpoint_steps);
        break;
    case VELOCITY_CONTROL:
        speed_steps = m_stepperConfiguration->angleDegToSteps(*setpoint_vel);
        stepper->setMaxSpeed(speed_steps);
        stepper->setSpeed(speed_steps);
        break;
    }
}

void MotorController::step()
{

    stepStepper(g_stepper1, g_encoder1, &g_m1_state, &g_m1_setpoint_pos, &g_m1_setpoint_vel);
    stepStepper(g_stepper2, g_encoder2, &g_m2_state, &g_m2_setpoint_pos, &g_m2_setpoint_vel);
    stepStepper(g_stepper3, g_encoder3, &g_m3_state, &g_m3_setpoint_pos, &g_m3_setpoint_vel);

    if(g_motor4 != NULL) switch (g_m4_state)
    {
    case VELOCITY_CONTROL:
        g_motor4->setSpeed(std::clamp(
            g_m4_setpoint_vel,
            -MAX_VEL_CONTINUOUS_SERVO,
            MAX_VEL_CONTINUOUS_SERVO
        ));
        break;
    case DISABLED:
    case INITIALIZATION:
    case POSITION_CONTROL:
    default:
        g_motor4->stop();
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

    switch (g_m6_state)
    {   
    case DISABLED:
        break;
    case INITIALIZATION:
        break;
    case POSITION_CONTROL:
        break;
    case VELOCITY_CONTROL:
        g_motor6->setSpeed(g_m6_setpoint_vel);
        break;
    default:
        break;
    }


}

void MotorController::run()
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

    switch (g_m6_state)
    {   
    case DISABLED:
        g_motor6->run();
        break;
    case INITIALIZATION:    
        g_motor6->runSpeed();
        break;
    case POSITION_CONTROL:
        g_motor6->run();
        break;
    case VELOCITY_CONTROL:
        g_motor6->runSpeed();
        break;
    default:
        break;
    }

}

void MotorController::reset()
{
    g_m1_state = MotorControlState::DISABLED;
    g_m2_state = MotorControlState::DISABLED;
    g_m3_state = MotorControlState::DISABLED;
    g_m4_state = MotorControlState::DISABLED;
    g_m5_state = MotorControlState::DISABLED;
    g_m6_state = MotorControlState::DISABLED;
    if(g_motor4 != NULL){
        g_motor4->stop();
    }
}

void MotorController::initializeM1()
{
    g_m1_state = MotorControlState::INITIALIZATION;
}

void MotorController::initializeM2()
{
    g_m2_state = MotorControlState::INITIALIZATION;
}

void MotorController::initializeM3()
{
    g_m3_state = MotorControlState::INITIALIZATION;
}

void MotorController::initializeM4()
{
    g_m4_state = MotorControlState::DISABLED;
    if(g_motor4 != NULL){
        g_motor4->stop();
    }
}

void MotorController::initializeM5()
{
    //g_m5_state = MotorControlState::INITIALIZATION;
}

void MotorController::initializeM6()
{
    //g_m6_state = MotorControlState::INITIALIZATION;
}

void MotorController::setM1Position(float position)
{
    g_m1_state = MotorControlState::POSITION_CONTROL;
    g_m1_setpoint_pos = position;
    g_m1_setpoint_vel = 3000;
    printf("M1 - setpoint: %f\n\r", g_m1_setpoint_pos);
}

void MotorController::setM2Position(float position)
{
    g_m2_state = MotorControlState::POSITION_CONTROL;
    g_m2_setpoint_pos = position;
    g_m2_setpoint_vel = 3000;
    printf("M2 - setpoint: %f\n\r", g_m2_setpoint_pos);
}

void MotorController::setM3Position(float position)
{
    g_m3_state = MotorControlState::POSITION_CONTROL;
    g_m3_setpoint_pos = position;
    g_m3_setpoint_vel = 3000;
    printf("M3 - setpoint: %f\n\r", g_m3_setpoint_pos);
}

void MotorController::setM4Position(float position)
{
    // A continuous servo has no intrinsic position mode in the raw motor controller.
    g_m4_state = MotorControlState::DISABLED;
    g_m4_setpoint_pos = position;
    if(g_motor4 != NULL){
        g_motor4->stop();
    }
}

void MotorController::setM5Position(float position)
{
    g_m5_state = MotorControlState::POSITION_CONTROL;
    g_m5_setpoint_pos = position;
    g_m5_setpoint_vel = 3000;
    printf("M5 - setpoint: %f\n\r", g_m5_setpoint_pos);
}

void MotorController::setM6Position(float position)
{
    g_m6_state = MotorControlState::POSITION_CONTROL;
    g_m6_setpoint_pos = position;
    g_m6_setpoint_vel = 3000;
    printf("M6 - setpoint: %f\n\r", g_m6_setpoint_pos);
}

void MotorController::setM1Velocity(float velocity)
{
    g_m1_state = MotorControlState::VELOCITY_CONTROL;
    g_m1_setpoint_vel = velocity;
}

void MotorController::setM2Velocity(float velocity)
{
    g_m2_state = MotorControlState::VELOCITY_CONTROL;
    g_m2_setpoint_vel = velocity;
}

void MotorController::setM3Velocity(float velocity)
{
    g_m3_state = MotorControlState::VELOCITY_CONTROL;
    g_m3_setpoint_vel = velocity;
}

void MotorController::setM4Velocity(float velocity)
{
    g_m4_state = MotorControlState::VELOCITY_CONTROL;
    g_m4_setpoint_vel = velocity;
}

void MotorController::setM5Velocity(float velocity)
{
    g_m5_state = MotorControlState::VELOCITY_CONTROL;
    g_m5_setpoint_vel = velocity;
}

void MotorController::setM6Velocity(float velocity)
{
    g_m6_state = MotorControlState::VELOCITY_CONTROL;
    g_m6_setpoint_vel = velocity;
}

void MotorController::setM1PositionVelocity(float position, float velocity)
{
    g_m1_state = MotorControlState::POSITION_CONTROL;
    g_m1_setpoint_pos = position;
    g_m1_setpoint_vel = velocity;
    printf("M1 - setpoint pos: %f, vel: %f\n\r", g_m1_setpoint_pos, g_m1_setpoint_vel);
}

void MotorController::setM2PositionVelocity(float position, float velocity)
{
    g_m2_state = MotorControlState::POSITION_CONTROL;
    g_m2_setpoint_pos = position;
    g_m2_setpoint_vel = velocity;
    printf("M2 - setpoint pos: %f, vel: %f\n\r", g_m2_setpoint_pos, g_m2_setpoint_vel);
}

void MotorController::setM3PositionVelocity(float position, float velocity)
{
    g_m3_state = MotorControlState::POSITION_CONTROL;
    g_m3_setpoint_pos = position;
    g_m3_setpoint_vel = velocity;
    printf("M3 - setpoint pos: %f, vel: %f\n\r", g_m3_setpoint_pos, g_m3_setpoint_vel);
}

void MotorController::setM4PositionVelocity(float position, float velocity)
{
    // Position control for J4 belongs to JointController, where encoder feedback is available.
    g_m4_setpoint_pos = position;
    setM4Velocity(velocity);
}

void MotorController::setM5PositionVelocity(float position, float velocity)
{
    g_m5_state = MotorControlState::POSITION_CONTROL;
    g_m5_setpoint_pos = position;
    g_m5_setpoint_vel = velocity;
    printf("M5 - setpoint pos: %f, vel: %f\n\r", g_m5_setpoint_pos, g_m5_setpoint_vel);
}

void MotorController::setM6PositionVelocity(float position, float velocity)
{
    g_m6_state = MotorControlState::POSITION_CONTROL;
    g_m6_setpoint_pos = position;
    g_m6_setpoint_vel = velocity;
    printf("M6 - setpoint pos: %f, vel: %f\n\r", g_m6_setpoint_pos, g_m6_setpoint_vel);
}
