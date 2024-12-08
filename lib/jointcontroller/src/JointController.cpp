#include "JointController.h"

#include <stdlib.h>
#include <stdio.h>
#include <algorithm>

JointController::JointController(StepperConfiguration *stepperConfiguration)
{
    m_stepperConfiguration = stepperConfiguration;
}

void JointController::addM1(AccelStepper *stepper1)
{
    m_stepper1 = stepper1;
}

void JointController::addM2(AccelStepper *stepper2)
{
    m_stepper2 = stepper2;
}

void JointController::addM3(AccelStepper *stepper3)
{
    m_stepper3 = stepper3;
}

void JointController::addM4(DCMotor *motor4)
{
    m_motor4 = motor4;
}

void JointController::addM5(DCMotor *motor5)
{
    m_motor5 = motor5;
}

void JointController::addE1(AS5600 *encoder1)
{
    m_encoder1 = encoder1;
}

void JointController::addE2(AS5600 *encoder2)
{
    m_encoder2 = encoder2;
}

void JointController::addE3(AS5600 *encoder3)
{
    m_encoder3 = encoder3;
}

void JointController::addE4(AS5600 *encoder4)
{
    m_encoder4 = encoder4;
}

void JointController::addE5(AS5600 *encoder5)
{
    m_encoder5 = encoder5;
}

void JointController::initialize()
{
    initializeJ1();
    initializeJ2();
    initializeJ3();
    initializeJ4();
    initializeJ5();
}

/**
 * @brief Performs a single step of the joint controller for a single stepper motor (either stepper1, stepper2 or stepper3)
 * 
 * @param stepper   The stepper motor of the joint to control
 * @param encoder   The encoder to read the current position from
 * @param state     The current state of the joint to control
 * @param setpoint_pos  The current position setpoint of the joint to control [deg]
 * @param setpoint_vel  The current velocity setpoint of the joint to control [deg/s]
 */
void JointController::stepStepper(AccelStepper * stepper, AS5600 * encoder, JointControlState * state, float * setpoint_pos, float * setpoint_vel)
{
    long speed_steps = 0;
    long setpoint_steps = 0;
    float angle = 0;
    float speed = 0;
    long currentPos = 0;

    switch (*state)
    {
    case DISABLED:
        currentPos = stepper->currentPosition();
        stepper->setAcceleration(10000.0);
        stepper->moveTo(currentPos);
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
            *state = JointControlState::POSITION_CONTROL;
            stepper->setCurrentPosition(0);
            *setpoint_pos = 0;
            *setpoint_vel = INIT_VEL_STEPPER;
        }
        speed = std::clamp(5 * angle, -INIT_VEL_STEPPER, INIT_VEL_STEPPER);
        speed_steps = m_stepperConfiguration->angleDegToSteps(speed);
        stepper->setMaxSpeed(speed_steps);
        stepper->setSpeed(speed_steps);
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

/**
 * @brief Get the current configuration of the robot
 * 
 * @return std::vector<float>   The current configuration of the robot [deg]
 */
std::vector<float> JointController::getConfiguration()
{
    std::vector<float> config;
    config.push_back(m_stepperConfiguration->stepsToAngleDeg(m_stepper1->currentPosition()));
    config.push_back(m_stepperConfiguration->stepsToAngleDeg(m_stepper2->currentPosition()));
    config.push_back(m_stepperConfiguration->stepsToAngleDeg(m_stepper3->currentPosition()));
    if(m_encoder4 != NULL)
        config.push_back(m_encoder4->getCorrectedAngleDeg());
    else
        config.push_back(0);
    if(m_encoder5 != NULL)
        config.push_back(m_encoder5->getCorrectedAngleDeg());
    else
        config.push_back(0);
    return config;
}

/**
 * @brief Check the limits of joint J2 and J3 and disable them if they are reached
 * 
 * In case a limit is reached, the corresponding joint is disabled and the motor needs to be manually moved back to a valid position.
 * 
 */
void JointController::checkJointLimitsJ2J3(){
    float angle_J2 = m_encoder2->readAngleDeg();
    float angle_J3 = m_encoder3->readAngleDeg();
    float angleDiff = getDiffAngleJ2J3();
    if(angleDiff > LIMIT_J2_J3_DIFF_MAX){
        // warning - robot reached the max limit between J2 and J3

        if(m_stepper2->speed() > 0){
            m_state_j2 = JointControlState::DISABLED;
        }
        if(m_stepper3->speed() > 0){
            m_state_j3 = JointControlState::DISABLED;
        }
    }
    else if(angleDiff < LIMIT_J2_J3_DIFF_MIN){
        // warning - robot reached the min limit between J2 and J3
        if(m_stepper2->speed() < 0){
            m_state_j2 = JointControlState::DISABLED;
        }
        if(m_stepper3->speed() < 0){
            m_state_j3 = JointControlState::DISABLED;
        }
    }
    if(angle_J3 < LIMIT_J3_MIN){
        // warning - robot reached a the limit of J3
        if(m_stepper3->speed() > 0){
            m_state_j3 = JointControlState::DISABLED;
        }
    }
    if(angle_J2 < LIMIT_J2_MIN){
        // warning - robot reached the min limit of J2
        if(m_stepper2->speed() > 0){
            m_state_j2 = JointControlState::DISABLED;
        }
    } else if(angle_J2 > LIMIT_J2_MAX){
        // warning - robot reached the max limit of J2
        if(m_stepper2->speed() < 0){
            m_state_j2 = JointControlState::DISABLED;
        }
    }
}

float JointController::getDiffAngleJ2J3(){
    float angle_J2 = m_encoder2->readAngleDeg();
    float angle_J3 = m_encoder3->readAngleDeg();
    float inv_angle_J2 = (360 - angle_J2);
    if(angle_J2 > 180){
        angle_J2 = angle_J2 - 360;
    }
    if(angle_J3 > 180){
        angle_J3 = angle_J3 - 360;
    }
    if(inv_angle_J2 > 180){
        inv_angle_J2 = inv_angle_J2 - 360;
    }
    return inv_angle_J2 - angle_J3;
}

/**
 * @brief Performs a single step of the joint controller for a single DC motor (either motor4 or motor5)
 * 
 * @param motor     The DC motor of the joint to control
 * @param encoder   The encoder to read the current position from
 * @param state     The current state of the joint to control
 * @param pid_p     The proportional gain of the PID controller
 * @param pid_i     The integral gain of the PID controller
 * @param pid_d     The derivative gain of the PID controller
 * @param setpoint_pos  The current position setpoint of the joint to control [deg]
 * @param setpoint_vel  The current velocity setpoint of the joint to control [PWM]
 * @param speed_m4  The output speed of the motor4 [PWM] (as return value)
 * @param speed_m5  The output speed of the motor5 [PWM] (as return value)
 */
void JointController::stepDCMotor(DCMotor * motor, AS5600 * encoder, JointControlState * state, float pid_p, float pid_i, float pid_d, float * setpoint_pos, float * setpoint_vel, float * speed_m4, float * speed_m5){
    
    float angle = 0;
    float speed = 0;

    switch (*state)
    {
    case DISABLED:
        *speed_m4 = 0;
        *speed_m5 = 0;
        break;
    case INITIALIZATION:
        *state = JointControlState::POSITION_CONTROL;
        *setpoint_pos = 0;
        *setpoint_vel = INIT_VEL_DCMOTOR;
        break;
    case POSITION_CONTROL:

        angle = encoder->getCorrectedAngleDeg();

        if(angle > 180){
            angle = angle - 360;
        }

        if(encoder != NULL){
            // PID controller
            float diff = *setpoint_pos - angle;
            float p = pid_p;
            float i = pid_i;
            float d = pid_d;
            float dt = 0.01;
            static float integral = 0;
            static float derivative = 0;
            static float prev_diff = 0;
            integral = integral + diff * dt;
            derivative = (diff - prev_diff) / dt;
            prev_diff = diff;
            speed = p * diff + i * integral + d * derivative;
            speed = std::clamp(speed, -*setpoint_vel, *setpoint_vel);
            speed = std::clamp(speed, -MAX_VEL_DCMOTOR, MAX_VEL_DCMOTOR);
            *speed_m4 = -speed;
            *speed_m5 = -speed;
        }
        else {
            *state = JointControlState::DISABLED;
        }
        break;
    case VELOCITY_CONTROL:
        *speed_m4 = *setpoint_vel;
        *speed_m5 = *setpoint_vel;
        break;
    }
}

/**
 * @brief Check if the joint limit of J4 is reached
 * 
 * @return true     If the joint limit is reached
 * @return false    If the joint limit is not reached
 */
bool JointController::isJointLimitReachedJ4(float speed_m4_j4){
    float angle = m_encoder4->readAngleDeg();
    if(angle > 180){
        angle = angle - 360;
    }
    if(angle < LIMIT_J4_MIN && speed_m4_j4 > 0){
        return true;
    }
    else if(angle > LIMIT_J4_MAX && speed_m4_j4 < 0){
        return true;
    }
    return false;
}

/**
 * @brief Check if the joint limit of J5 is reached
 * 
 * @return true     If the joint limit is reached
 * @return false    If the joint limit is not reached
 */
bool JointController::isJointLimitReachedJ5(float speed_m5_j5){
    float angle = m_encoder5->readAngleDeg();
    if(angle > 180){
        angle = angle - 360;
    }
    if(angle < LIMIT_J5_MIN && speed_m5_j5 > 0){
        return true;
    }
    else if(angle > LIMIT_J5_MAX && speed_m5_j5 < 0){
        return true;
    }
    return false;
}

bool JointController::isAllEncoderStatusValid(){
    if(m_encoder1 != NULL && m_encoder1->getStatus() != 32){
        return false;
    }
    if(m_encoder2 != NULL && m_encoder2->getStatus() != 32){
        return false;
    }
    if(m_encoder3 != NULL && m_encoder3->getStatus() != 32){
        return false;
    }
    if(m_encoder4 != NULL && m_encoder4->getStatus() != 32){
        return false;
    }
    if(m_encoder5 != NULL && m_encoder5->getStatus() != 32){
        return false;
    }
    return true;
}
/**
 * @brief Execute on control step of the joint controller 
 * 
 * Should be called with the control frequency (e.g. 100Hz)
 * 
 */
void JointController::step()
{
    bool encodersValid = isAllEncoderStatusValid();
    if(!encodersValid){
        m_state_j1 = JointControlState::DISABLED;
        m_state_j2 = JointControlState::DISABLED;
        m_state_j3 = JointControlState::DISABLED;
        m_state_j4 = JointControlState::DISABLED;
        m_state_j5 = JointControlState::DISABLED;
    }
    if(encodersValid) checkJointLimitsJ2J3();
    stepStepper(m_stepper1, m_encoder1, &m_state_j1, &m_setpoint_pos_j1, &m_setpoint_vel_j1);
    stepStepper(m_stepper2, m_encoder2, &m_state_j2, &m_setpoint_pos_j2, &m_setpoint_vel_j2);
    stepStepper(m_stepper3, m_encoder3, &m_state_j3, &m_setpoint_pos_j3, &m_setpoint_vel_j3);

    float speed_m4_j4 = 0;
    float speed_m4_j5 = 0;
    float speed_m5_j4 = 0;
    float speed_m5_j5 = 0;

    stepDCMotor(m_motor4, m_encoder4, &m_state_j4, pid_p_j4, pid_i_j4, pid_d_j4, &m_setpoint_pos_j4, &m_setpoint_vel_j4, &speed_m4_j4, &speed_m5_j4);
    if(encodersValid && isJointLimitReachedJ4(speed_m4_j4)){
        speed_m4_j4 = 0;
        speed_m5_j4 = 0;
    }
    stepDCMotor(m_motor5, m_encoder5, &m_state_j5, pid_p_j5, pid_i_j5, pid_d_j5, &m_setpoint_pos_j5, &m_setpoint_vel_j5, &speed_m4_j5, &speed_m5_j5);
    if(encodersValid && isJointLimitReachedJ5(speed_m5_j5)){
        speed_m4_j5 = 0;
        speed_m5_j5 = 0;
    }
    m_motor4->setSpeed(speed_m4_j4 + speed_m4_j5);
    m_motor5->setSpeed(speed_m5_j4 - speed_m5_j5);
}

/**
 * @brief This function triggers the step signals for the stepper motors
 * 
 * It need to be called as often as possible. 
 * 
 */
void JointController::run()
{
    switch (m_state_j1)
    {
    case DISABLED:
        m_stepper1->run();
        break;
    case INITIALIZATION:
        m_stepper1->runSpeed();
        break;
    case POSITION_CONTROL:
        m_stepper1->run();
        break;
    case VELOCITY_CONTROL:
        m_stepper1->runSpeed();
        break;
    default:
        break;
    }

    switch (m_state_j2)
    {
    case DISABLED:
        m_stepper2->run();
        break;
    case INITIALIZATION:
        m_stepper2->runSpeed();
        break;
    case POSITION_CONTROL:
        m_stepper2->run();
        break;
    case VELOCITY_CONTROL:
        m_stepper2->runSpeed();
        break;
    default:
        break;
    }

    switch (m_state_j3)
    {
    case DISABLED:
        m_stepper3->run();
        break;
    case INITIALIZATION:
        m_stepper3->runSpeed();
        break;
    case POSITION_CONTROL:
        m_stepper3->run();
        break;
    case VELOCITY_CONTROL:
        m_stepper3->runSpeed();
        break;
    default:
        break;
    }

    m_motor4->runSpeed();
    m_motor5->runSpeed();
}

void JointController::reset()
{
    m_state_j1 = JointControlState::DISABLED;
    m_state_j2 = JointControlState::DISABLED;
    m_state_j3 = JointControlState::DISABLED;
    m_state_j4 = JointControlState::DISABLED;
    m_state_j5 = JointControlState::DISABLED;
}

void JointController::initializeJ1()
{
    m_state_j1 = JointControlState::INITIALIZATION;
}

void JointController::initializeJ2()
{
    m_state_j2 = JointControlState::INITIALIZATION;
}

void JointController::initializeJ3()
{
    m_state_j3 = JointControlState::INITIALIZATION;
}

void JointController::initializeJ4()
{
    m_state_j4 = JointControlState::INITIALIZATION;
}

void JointController::initializeJ5()
{
    m_state_j5 = JointControlState::INITIALIZATION;
}

void JointController::zeroJ1()
{
    if(m_encoder1 != NULL){
        m_encoder1->setZero();
    }
    m_stepper1->setCurrentPosition(0);
}

void JointController::zeroJ2()
{
    if(m_encoder2 != NULL){
        m_encoder2->setZero();
    }
    m_stepper2->setCurrentPosition(0);
}

void JointController::zeroJ3()
{
    if(m_encoder3 != NULL){
        m_encoder3->setZero();
    }
    m_stepper3->setCurrentPosition(0);
}

void JointController::zeroJ4()
{
    if(m_encoder4 != NULL){
        m_encoder4->setZero();
    }
}

void JointController::zeroJ5()
{
    if(m_encoder5 != NULL){
        m_encoder5->setZero();
    }
}

/**
 * @brief Set the J1 Position
 * 
 * @param position  The position to move to [deg]
 */
void JointController::setJ1Position(float position)
{
    m_state_j1 = JointControlState::POSITION_CONTROL;
    m_setpoint_pos_j1 = position;
    m_setpoint_vel_j1 = MAX_VEL_STEPPER;
}

/**
 * @brief Set the J2 Position
 * 
 * @param position  The position to move to [deg]
 */
void JointController::setJ2Position(float position)
{
    m_state_j2 = JointControlState::POSITION_CONTROL;
    m_setpoint_pos_j2 = position;
    m_setpoint_vel_j2 = MAX_VEL_STEPPER;
}

/**
 * @brief Set the J3 Position
 * 
 * @param position  The position to move to [deg]
 */
void JointController::setJ3Position(float position)
{
    m_state_j3 = JointControlState::POSITION_CONTROL;
    m_setpoint_pos_j3 = position;
    m_setpoint_vel_j3 = MAX_VEL_STEPPER;
}

/**
 * @brief Set the J4 Position
 * 
 * @param position  The position to move to [deg]
 */
void JointController::setJ4Position(float position)
{
    m_state_j4 = JointControlState::POSITION_CONTROL;
    m_setpoint_pos_j4 = position;
    m_setpoint_vel_j4 = MAX_VEL_DCMOTOR;
}

/**
 * @brief Set the J5 Position
 * 
 * @param position  The position to move to [deg]
 */
void JointController::setJ5Position(float position)
{
    m_state_j5 = JointControlState::POSITION_CONTROL;
    m_setpoint_pos_j5 = position;
    m_setpoint_vel_j5 = MAX_VEL_DCMOTOR;
}

/**
 * @brief Set the J1 Velocity
 * 
 * @param velocity  The velocity to move at [deg/s]
 */
void JointController::setJ1Velocity(float velocity)
{
    m_state_j1 = JointControlState::VELOCITY_CONTROL;
    m_setpoint_vel_j1 = velocity;
}

/**
 * @brief Set the J2 Velocity
 * 
 * @param velocity  The velocity to move at [deg/s]
 */
void JointController::setJ2Velocity(float velocity)
{
    m_state_j2 = JointControlState::VELOCITY_CONTROL;
    m_setpoint_vel_j2 = velocity;
}

/**
 * @brief Set the J3 Velocity
 * 
 * @param velocity  The velocity to move at [deg/s]
 */
void JointController::setJ3Velocity(float velocity)
{
    m_state_j3 = JointControlState::VELOCITY_CONTROL;
    m_setpoint_vel_j3 = velocity;
}

/**
 * @brief Set the J4 Velocity
 * 
 * @param velocity  The velocity to move at [PWM]
 */
void JointController::setJ4Velocity(float velocity)
{
    m_state_j4 = JointControlState::VELOCITY_CONTROL;
    m_setpoint_vel_j4 = velocity;
}

/**
 * @brief Set the J5 Velocity
 * 
 * @param velocity  The velocity to move at [PWM]
 */
void JointController::setJ5Velocity(float velocity)
{
    m_state_j5 = JointControlState::VELOCITY_CONTROL;
    m_setpoint_vel_j5 = velocity;
}

/**
 * @brief Set the J1 Position and Velocity
 * 
 * @param position  The position to move to [deg]
 * @param velocity  The velocity to move at [deg/s]
 */
void JointController::setJ1PositionVelocity(float position, float velocity)
{
    m_state_j1 = JointControlState::POSITION_CONTROL;
    m_setpoint_pos_j1 = position;
    m_setpoint_vel_j1 = velocity;
}

/**
 * @brief Set the J2 Position and Velocity
 * 
 * @param position  The position to move to [deg]
 * @param velocity  The velocity to move at [deg/s]
 */
void JointController::setJ2PositionVelocity(float position, float velocity)
{
    m_state_j2 = JointControlState::POSITION_CONTROL;
    m_setpoint_pos_j2 = position;
    m_setpoint_vel_j2 = velocity;
}

/**
 * @brief Set the J3 Position and Velocity
 * 
 * @param position  The position to move to [deg]
 * @param velocity  The velocity to move at [deg/s]
 */
void JointController::setJ3PositionVelocity(float position, float velocity)
{
    m_state_j3 = JointControlState::POSITION_CONTROL;
    m_setpoint_pos_j3 = position;
    m_setpoint_vel_j3 = velocity;
}

/**
 * @brief Set the J4 Position and Velocity
 * 
 * @param position  The position to move to [deg]
 * @param velocity  The velocity to move at [PWM]
 */
void JointController::setJ4PositionVelocity(float position, float velocity)
{
    m_state_j4 = JointControlState::POSITION_CONTROL;
    m_setpoint_pos_j4 = position;
    m_setpoint_vel_j4 = velocity;
}

/**
 * @brief Set the J5 Position and Velocity
 * 
 * @param position  The position to move to [deg]
 * @param velocity  The velocity to move at [PWM]
 */
void JointController::setJ5PositionVelocity(float position, float velocity)
{
    m_state_j5 = JointControlState::POSITION_CONTROL;
    m_setpoint_pos_j5 = position;
    m_setpoint_vel_j5 = velocity;
}

/**
 * @brief Set the J4 PID parameters
 * 
 * @param p     Proportional gain
 * @param i     Integral gain
 * @param d     Derivative gain
 */
void JointController::setJ4PID(float p, float i, float d)
{
    pid_p_j4 = p;
    pid_i_j4 = i;
    pid_d_j4 = d;
}

/**
 * @brief Set the J5 PID parameters
 * 
 * @param p     Proportional gain
 * @param i     Integral gain
 * @param d     Derivative gain
 */
void JointController::setJ5PID(float p, float i, float d)
{
    pid_p_j5 = p;
    pid_i_j5 = i;
    pid_d_j5 = d;
}

/**
 * @brief Move Robot to a given configuration
 * 
 * @param config    The configuration to move to [deg]
 * @param velocity  The velocity to move at [deg/s]
 */
void JointController::moveToConfiguration(std::vector<float> config, float velocity)
{
    if(config.size() < 3){
        return;
    }
    std::vector<float> currentConfig = getConfiguration();

    long config_steps[3];
    config_steps[0] = m_stepperConfiguration->angleDegToSteps(config[0]);
    config_steps[1] = m_stepperConfiguration->angleDegToSteps(config[1]);
    config_steps[2] = m_stepperConfiguration->angleDegToSteps(config[2]);

    long currentConfig_steps[3];
    currentConfig_steps[0] = m_stepperConfiguration->angleDegToSteps(currentConfig[0]);
    currentConfig_steps[1] = m_stepperConfiguration->angleDegToSteps(currentConfig[1]);
    currentConfig_steps[2] = m_stepperConfiguration->angleDegToSteps(currentConfig[2]);

    float velocity_steps = m_stepperConfiguration->angleDegToSteps(velocity);

    long stepsToGo1 = config_steps[0] - currentConfig_steps[0];
    float time1 = abs(stepsToGo1) / velocity_steps;
    
    long stepsToGo2 = config_steps[1] - currentConfig_steps[1];
    float time2 = abs(stepsToGo2) / velocity_steps;
    
    long stepsToGo3 = config_steps[2] - currentConfig_steps[2];
    float time3 = abs(stepsToGo3) / velocity_steps;

    float longestTime = time1;
    if (time2 > longestTime){
        longestTime = time2;
    }
    if (time3 > longestTime){
        longestTime = time3;
    }

    if (longestTime > 0.0) {
        // Now work out a new max speed for each stepper so they will all 
        // arrived at the same time of longestTime
        float speed1_steps = stepsToGo1 / longestTime;
        float speed1 = m_stepperConfiguration->stepsToAngleDeg(speed1_steps);
        setJ1PositionVelocity(config[0], speed1);

        float speed2_steps = stepsToGo2 / longestTime;
        float speed2 = m_stepperConfiguration->stepsToAngleDeg(speed2_steps);
        setJ2PositionVelocity(config[1], speed2);

        float speed3_steps = stepsToGo3 / longestTime;
        float speed3 = m_stepperConfiguration->stepsToAngleDeg(speed3_steps);
        setJ3PositionVelocity(config[2], speed3);
    }

    // Check length of config vector if it also contains J4 and J5
    if(config.size() > 3){
        setJ4Position(config[3]);
        setJ5Position(config[4]);
    }
}
