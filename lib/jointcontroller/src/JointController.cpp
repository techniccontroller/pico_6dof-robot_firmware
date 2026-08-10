#include "JointController.h"

#include <stdlib.h>
#include <stdio.h>
#include <algorithm>
#include <cmath>

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

void JointController::addM4(ContinuousServo *motor4)
{
    m_motor4 = motor4;
}

void JointController::addM5(DCMotor *motor5)
{
    m_motor5 = motor5;
}

void JointController::addM6(DCMotor *motor6)
{
    m_motor6 = motor6;
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

void JointController::addE6(AS5600 *encoder6)
{
    m_encoder6 = encoder6;
}

void JointController::initialize()
{
    initializeJ1();
    initializeJ2();
    initializeJ3();
    initializeJ4();
    initializeJ5();
    initializeJ6();
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
        stepper->setAcceleration(DEFAULT_ACC_STEPPER);
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
    if(m_encoder6 != NULL)
        config.push_back(m_encoder6->getCorrectedAngleDeg());
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
 * @brief Calculate one logical joint command for the coupled J5/J6 drive.
 *
 * J5 and J6 have separate controller histories. Their logical commands are
 * mixed into physical M5/M6 outputs only after both controllers have run.
 */
float JointController::stepDCJoint(
    AS5600 *encoder, JointControlState *state,
    float pid_p, float pid_i, float pid_d,
    float *setpoint_pos, float *setpoint_vel,
    float *integral, float *previous_error, float *filtered_derivative,
    bool *previous_error_valid, bool *position_complete)
{
    if(*state == JointControlState::DISABLED){
        *integral = 0.0f;
        *previous_error = 0.0f;
        *filtered_derivative = 0.0f;
        *previous_error_valid = false;
        return 0.0f;
    }

    if(*state == JointControlState::INITIALIZATION){
        *state = JointControlState::POSITION_CONTROL;
        *setpoint_pos = 0.0f;
        *setpoint_vel = INIT_VEL_DCMOTOR;
        *integral = 0.0f;
        *previous_error = 0.0f;
        *filtered_derivative = 0.0f;
        *previous_error_valid = false;
        *position_complete = false;
        return 0.0f;
    }

    if(*state == JointControlState::VELOCITY_CONTROL){
        *position_complete = false;
        *filtered_derivative = 0.0f;
        *previous_error_valid = false;
        return std::clamp(
            *setpoint_vel, -MAX_VEL_DCMOTOR, MAX_VEL_DCMOTOR);
    }

    if(encoder == NULL){
        *state = JointControlState::DISABLED;
        *previous_error_valid = false;
        return 0.0f;
    }

    float angle = encoder->getCorrectedAngleDeg();
    if(angle > 180.0f){
        angle -= 360.0f;
    }

    float error = *setpoint_pos - angle;
    while(error > 180.0f) error -= 360.0f;
    while(error < -180.0f) error += 360.0f;

    constexpr float dt = 0.01f;
    if(*position_complete){
        if(std::fabs(error) < DCMOTOR_POSITION_REENGAGE_DEG){
            *integral = 0.0f;
            *filtered_derivative = 0.0f;
            *previous_error = error;
            *previous_error_valid = true;
            return 0.0f;
        }

        // Only a real displacement can re-arm a completed worm-drive move.
        *position_complete = false;
        *integral = 0.0f;
        *filtered_derivative = 0.0f;
        *previous_error_valid = false;
    }

    bool crossedTarget = *previous_error_valid
                      && ((*previous_error > 0.0f && error < 0.0f)
                       || (*previous_error < 0.0f && error > 0.0f));
    if(std::fabs(error) <= DCMOTOR_POSITION_TOLERANCE_DEG || crossedTarget){
        *position_complete = true;
        *integral = 0.0f;
        *previous_error = error;
        *filtered_derivative = 0.0f;
        *previous_error_valid = true;
        return 0.0f;
    }

    *integral += error * dt;
    if(pid_i > 0.0f){
        float integralLimit = DCMOTOR_INTEGRAL_OUTPUT_LIMIT / pid_i;
        *integral = std::clamp(*integral, -integralLimit, integralLimit);
    }
    else{
        *integral = 0.0f;
    }

    if(!*previous_error_valid){
        *filtered_derivative = 0.0f;
    }
    float rawDerivative = *previous_error_valid
                        ? (error - *previous_error) / dt
                        : 0.0f;
    *previous_error = error;
    *previous_error_valid = true;
    *filtered_derivative += DCMOTOR_DERIVATIVE_FILTER_ALPHA
                          * (rawDerivative - *filtered_derivative);
    float command = pid_p * error
                  + pid_i * *integral
                  + pid_d * *filtered_derivative;
    float commandLimit = std::clamp(
        std::fabs(*setpoint_vel), 0.0f, MAX_VEL_DCMOTOR);
    return std::clamp(command, -commandLimit, commandLimit);
}

float JointController::compensateDCMotorDeadZone(
    float command, float positiveMinimum, float negativeMinimum)
{
    float magnitude = std::fabs(command);
    if(magnitude < 0.001f){
        return 0.0f;
    }
    float minimum = command > 0.0f ? positiveMinimum : negativeMinimum;
    if(magnitude < minimum){
        return std::copysign(minimum, command);
    }
    return command;
}

/**
 * @brief Check if the joint limit of J5 is reached
 * 
 * @return true     If the joint limit is reached
 * @return false    If the joint limit is not reached
 */
bool JointController::isJointLimitReachedJ5(float jointCommand){
    float angle = m_encoder5->getCorrectedAngleDeg();
    if(angle > 180){
        angle = angle - 360;
    }
    if(angle < LIMIT_J5_MIN && jointCommand < 0){
        return true;
    }
    else if(angle > LIMIT_J5_MAX && jointCommand > 0){
        return true;
    }
    return false;
}

/**
 * @brief Check if the joint limit of J6 is reached
 * 
 * @return true     If the joint limit is reached
 * @return false    If the joint limit is not reached
 */
bool JointController::isJointLimitReachedJ6(float jointCommand){
    float angle = m_encoder6->getCorrectedAngleDeg();
    if(angle > 180){
        angle = angle - 360;
    }
    if(angle < LIMIT_J6_MIN && jointCommand < 0){
        return true;
    }
    else if(angle > LIMIT_J6_MAX && jointCommand > 0){
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
    if(m_encoder6 != NULL && m_encoder6->getStatus() != 32){
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
        m_state_j6 = JointControlState::DISABLED;
    }
    if(encodersValid) checkJointLimitsJ2J3();
    stepStepper(m_stepper1, m_encoder1, &m_state_j1, &m_setpoint_pos_j1, &m_setpoint_vel_j1);
    stepStepper(m_stepper2, m_encoder2, &m_state_j2, &m_setpoint_pos_j2, &m_setpoint_vel_j2);
    stepStepper(m_stepper3, m_encoder3, &m_state_j3, &m_setpoint_pos_j3, &m_setpoint_vel_j3);
    stepContinuousServo();

    float jointCommandJ5 = stepDCJoint(
        m_encoder5, &m_state_j5,
        pid_p_j5, pid_i_j5, pid_d_j5,
        &m_setpoint_pos_j5, &m_setpoint_vel_j5,
        &pid_integral_j5, &pid_previous_error_j5,
        &pid_filtered_derivative_j5,
        &pid_previous_error_valid_j5,
        &m_j5_position_complete);
    float jointCommandJ6 = stepDCJoint(
        m_encoder6, &m_state_j6,
        pid_p_j6, pid_i_j6, pid_d_j6,
        &m_setpoint_pos_j6, &m_setpoint_vel_j6,
        &pid_integral_j6, &pid_previous_error_j6,
        &pid_filtered_derivative_j6,
        &pid_previous_error_valid_j6,
        &m_j6_position_complete);

    if(encodersValid && isJointLimitReachedJ5(jointCommandJ5)){
        jointCommandJ5 = 0.0f;
        pid_integral_j5 = 0.0f;
        pid_previous_error_valid_j5 = false;
    }
    if(encodersValid && isJointLimitReachedJ6(jointCommandJ6)){
        jointCommandJ6 = 0.0f;
        pid_integral_j6 = 0.0f;
        pid_previous_error_valid_j6 = false;
    }

    // Calibratable 2x2 inverse plant model. Unlike two ad-hoc motor sums, this
    // expresses how each logical joint must drive both physical motors.
    float motorCommandM5 = m_j5_to_m5 * jointCommandJ5
                         + m_j6_to_m5 * jointCommandJ6;
    float motorCommandM6 = m_j5_to_m6 * jointCommandJ5
                         + m_j6_to_m6 * jointCommandJ6;
    motorCommandM5 = compensateDCMotorDeadZone(
        motorCommandM5,
        MOTOR5_MIN_PWM_POSITIVE,
        MOTOR5_MIN_PWM_NEGATIVE);
    motorCommandM6 = compensateDCMotorDeadZone(
        motorCommandM6,
        MOTOR6_MIN_PWM_POSITIVE,
        MOTOR6_MIN_PWM_NEGATIVE);

    float largestMotorCommand = std::max(
        std::fabs(motorCommandM5), std::fabs(motorCommandM6));
    if(largestMotorCommand > DCMOTOR_MAX_PWM){
        float scale = DCMOTOR_MAX_PWM / largestMotorCommand;
        motorCommandM5 *= scale;
        motorCommandM6 *= scale;
        // Common anti-windup: neither joint integrator may continue growing
        // while the coupled actuator pair cannot deliver the requested vector.
        pid_integral_j5 *= scale;
        pid_integral_j6 *= scale;
    }

    m_motor5->setSpeed(motorCommandM5);
    m_motor6->setSpeed(motorCommandM6);
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

    m_motor5->runSpeed();
    m_motor6->runSpeed();
}

void JointController::reset()
{
    m_state_j1 = JointControlState::DISABLED;
    m_state_j2 = JointControlState::DISABLED;
    m_state_j3 = JointControlState::DISABLED;
    m_state_j4 = JointControlState::DISABLED;
    m_state_j5 = JointControlState::DISABLED;
    m_state_j6 = JointControlState::DISABLED;
    pid_integral_j5 = 0.0f;
    pid_previous_error_valid_j5 = false;
    m_j5_position_complete = false;
    pid_integral_j6 = 0.0f;
    pid_previous_error_valid_j6 = false;
    m_j6_position_complete = false;
    m_j4_latched_hold_mode = false;
    resetJ4ControlHistory();
    if(m_motor4 != NULL){
        m_motor4->stop();
    }
    if(m_motor5 != NULL){
        m_motor5->setSpeed(0.0f);
    }
    if(m_motor6 != NULL){
        m_motor6->setSpeed(0.0f);
    }
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
    pid_integral_j5 = 0.0f;
    pid_previous_error_valid_j5 = false;
    m_j5_position_complete = false;
}

void JointController::initializeJ6()
{
    m_state_j6 = JointControlState::INITIALIZATION;
    pid_integral_j6 = 0.0f;
    pid_previous_error_valid_j6 = false;
    m_j6_position_complete = false;
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

void JointController::resetJ4ControlHistory()
{
    pid_integral_j4 = 0.0f;
    m_j4_position_sample_valid = false;
    m_j4_previous_position = 0.0f;
    m_j4_filtered_velocity = 0.0f;
    m_j4_drive_accumulator = 0.0f;
    m_j4_previous_drive_sign = 0.0f;
    m_j4_servo_frame_phase = false;
}

void JointController::updateJ4NeutralEstimator(float positionError, float velocity)
{
    // Near the target, the persistent position error and residual velocity are
    // estimates of neutral-command bias. Adapt continuously, but slowly enough
    // that this feed-forward term cannot fight the fast PD position loop.
    if(std::fabs(positionError) > J4_NEUTRAL_LEARNING_ERROR_DEG){
        return;
    }

    constexpr float dt = 0.01f;
    float adjustmentRate = m_motor4->getDirection() *
        (J4_NEUTRAL_ERROR_GAIN * positionError
         - J4_NEUTRAL_VELOCITY_GAIN * velocity);
    adjustmentRate = std::clamp(
        adjustmentRate,
        -J4_NEUTRAL_MAX_RATE,
        J4_NEUTRAL_MAX_RATE
    );

    float minimumNeutral = CONTINUOUS_SERVO_J4_NEUTRAL_COMMAND
                         - J4_NEUTRAL_ESTIMATOR_MAX_OFFSET;
    float maximumNeutral = CONTINUOUS_SERVO_J4_NEUTRAL_COMMAND
                         + J4_NEUTRAL_ESTIMATOR_MAX_OFFSET;
    m_motor4->setNeutralCommand(std::clamp(
        m_motor4->getNeutralCommand() + adjustmentRate * dt,
        minimumNeutral,
        maximumNeutral
    ));
}

/**
 * @brief Update the closed-loop controller for the J4 continuous servo.
 *
 * The servo accepts a signed speed offset while the AS5600 provides position
 * feedback. Position errors use the shortest path across the 0/360-degree
 * boundary. This method is called by the existing 100 Hz joint-control loop.
 */
void JointController::stepContinuousServo()
{
    if(m_motor4 == NULL){
        return;
    }

    if(m_state_j4 == JointControlState::DISABLED){
        m_motor4->stop();
        m_j4_latched_hold_mode = false;
        resetJ4ControlHistory();
        return;
    }

    if(m_state_j4 == JointControlState::INITIALIZATION){
        m_state_j4 = JointControlState::POSITION_CONTROL;
        m_setpoint_pos_j4 = 0.0f;
        m_setpoint_vel_j4 = DEFAULT_VEL_CONTINUOUS_SERVO;
        m_j4_latched_hold_mode = false;
        resetJ4ControlHistory();
    }

    if(m_state_j4 == JointControlState::VELOCITY_CONTROL){
        m_motor4->setSpeed(std::clamp(
            m_setpoint_vel_j4,
            -MAX_VEL_CONTINUOUS_SERVO,
            MAX_VEL_CONTINUOUS_SERVO
        ));
        return;
    }

    if(m_encoder4 == NULL){
        m_state_j4 = JointControlState::DISABLED;
        m_motor4->stop();
        return;
    }

    float currentPosition = m_encoder4->getCorrectedAngleDeg();

    constexpr float dt = 0.01f;
    if(!m_j4_position_sample_valid){
        m_j4_previous_position = currentPosition;
        m_j4_position_sample_valid = true;
    }
    float positionDelta = currentPosition - m_j4_previous_position;
    while(positionDelta > 180.0f) positionDelta -= 360.0f;
    while(positionDelta < -180.0f) positionDelta += 360.0f;
    m_j4_previous_position = currentPosition;
    float rawVelocity = positionDelta / dt;
    m_j4_filtered_velocity += J4_VELOCITY_FILTER_ALPHA
                            * (rawVelocity - m_j4_filtered_velocity);

    float error = m_setpoint_pos_j4 - currentPosition;
    while(error > 180.0f) error -= 360.0f;
    while(error < -180.0f) error += 360.0f;
    float absoluteError = std::fabs(error);

    // Integrate only while position control is active and prevent wind-up.
    pid_integral_j4 += error * dt;
    if(pid_i_j4 > 0.0f){
        float integralLimit = J4_INTEGRAL_LIMIT / pid_i_j4;
        pid_integral_j4 = std::clamp(
            pid_integral_j4, -integralLimit, integralLimit);
    }
    else{
        pid_integral_j4 = 0.0f;
    }

    float speedLimit = std::clamp(
        std::fabs(m_setpoint_vel_j4),
        0.0f,
        MAX_VEL_CONTINUOUS_SERVO
    );
    if(m_j4_latched_hold_mode){
        speedLimit = std::min(speedLimit, J4_HOLD_MAX_COMMAND);
    }
    // Derivative-on-measurement avoids a kick when a new target is commanded.
    float speed = pid_p_j4 * error
                + pid_i_j4 * pid_integral_j4
                - pid_d_j4 * m_j4_filtered_velocity;
    speed = std::clamp(speed, -speedLimit, speedLimit);

    bool positionSettled = absoluteError <= J4_POSITION_TOLERANCE_DEG;
    bool motionSettled = std::fabs(m_j4_filtered_velocity)
                      <= J4_STOPPED_VELOCITY_DEG_S;
    if(positionSettled && motionSettled){
        speed = 0.0f;
        pid_integral_j4 *= 0.98f;
    }
    updateJ4NeutralEstimator(error, m_j4_filtered_velocity);

    // A hobby servo consumes one command per 20 ms frame. Send position-loop
    // output at that rate and use pulse-density modulation below the physical
    // dead zone. A requested command of 1 with a dead zone of 5 therefore
    // becomes one gentle 5-unit frame out of five, not a permanent 5-unit kick.
    m_j4_servo_frame_phase = !m_j4_servo_frame_phase;
    if(!m_j4_servo_frame_phase){
        return;
    }

    float outputSpeed = speed;
    float absoluteSpeed = std::fabs(speed);
    if(absoluteSpeed > 0.0f && absoluteSpeed < J4_MIN_DRIVE_COMMAND){
        float driveSign = std::copysign(1.0f, speed);
        if(driveSign != m_j4_previous_drive_sign){
            m_j4_drive_accumulator = 0.0f;
        }
        m_j4_previous_drive_sign = driveSign;
        m_j4_drive_accumulator += absoluteSpeed / J4_MIN_DRIVE_COMMAND;
        if(m_j4_drive_accumulator >= 1.0f){
            outputSpeed = driveSign * J4_MIN_DRIVE_COMMAND;
            m_j4_drive_accumulator -= 1.0f;
        }
        else{
            outputSpeed = 0.0f;
        }
    }
    else{
        m_j4_drive_accumulator = 0.0f;
        m_j4_previous_drive_sign = absoluteSpeed > 0.0f
                                 ? std::copysign(1.0f, speed)
                                 : 0.0f;
    }

    m_motor4->setSpeed(outputSpeed);
}

void JointController::zeroJ4()
{
    if(m_encoder4 != NULL){
        m_encoder4->setZero();
    }
    m_setpoint_pos_j4 = 0.0f;
    resetJ4ControlHistory();
}

void JointController::zeroJ5()
{
    if(m_encoder5 != NULL){
        m_encoder5->setZero();
    }
    m_setpoint_pos_j5 = 0.0f;
    pid_integral_j5 = 0.0f;
    pid_previous_error_valid_j5 = false;
    m_j5_position_complete = true;
}

void JointController::zeroJ6()
{
    if(m_encoder6 != NULL){
        m_encoder6->setZero();
    }
    m_setpoint_pos_j6 = 0.0f;
    pid_integral_j6 = 0.0f;
    pid_previous_error_valid_j6 = false;
    m_j6_position_complete = true;
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
    m_setpoint_vel_j1 = DEFAULT_VEL_STEPPER;
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
    m_setpoint_vel_j2 = DEFAULT_VEL_STEPPER;
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
    m_setpoint_vel_j3 = DEFAULT_VEL_STEPPER;
}

/**
 * @brief Set the J4 position using encoder feedback and the continuous servo.
 */
void JointController::setJ4Position(float position)
{
    m_state_j4 = JointControlState::POSITION_CONTROL;
    m_setpoint_pos_j4 = position;
    m_setpoint_vel_j4 = DEFAULT_VEL_CONTINUOUS_SERVO;
    m_j4_latched_hold_mode = false;
    resetJ4ControlHistory();
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
    pid_integral_j5 = 0.0f;
    pid_previous_error_valid_j5 = false;
    m_j5_position_complete = false;
}

/**
 * @brief Set the J6 Position
 * 
 * @param position  The position to move to [deg]
 */
void JointController::setJ6Position(float position)
{
    m_state_j6 = JointControlState::POSITION_CONTROL;
    m_setpoint_pos_j6 = position;
    m_setpoint_vel_j6 = MAX_VEL_DCMOTOR;
    pid_integral_j6 = 0.0f;
    pid_previous_error_valid_j6 = false;
    m_j6_position_complete = false;
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
 * @brief Set J4 signed speed as an offset from the neutral servo command.
 */
void JointController::setJ4Velocity(float velocity)
{
    if(std::fabs(velocity) < 0.001f){
        if(m_motor4 != NULL){
            m_motor4->stop();
        }
        if(m_encoder4 != NULL){
            m_setpoint_pos_j4 = m_encoder4->getCorrectedAngleDeg();
            m_setpoint_vel_j4 = DEFAULT_VEL_CONTINUOUS_SERVO;
            m_state_j4 = JointControlState::POSITION_CONTROL;
            m_j4_latched_hold_mode = true;
            resetJ4ControlHistory();
        }
        else{
            m_state_j4 = JointControlState::DISABLED;
        }
        return;
    }

    m_state_j4 = JointControlState::VELOCITY_CONTROL;
    m_setpoint_vel_j4 = velocity;
    m_j4_latched_hold_mode = false;
    resetJ4ControlHistory();
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
    pid_integral_j5 = 0.0f;
    pid_previous_error_valid_j5 = false;
    m_j5_position_complete = false;
}

/**
 * @brief Set the J6 Velocity
 * 
 * @param velocity  The velocity to move at [PWM]
 */
void JointController::setJ6Velocity(float velocity)
{
    m_state_j6 = JointControlState::VELOCITY_CONTROL;
    m_setpoint_vel_j6 = velocity;
    pid_integral_j6 = 0.0f;
    pid_previous_error_valid_j6 = false;
    m_j6_position_complete = false;
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
 * @brief Set the J4 position and maximum continuous-servo speed.
 */
void JointController::setJ4PositionVelocity(float position, float velocity)
{
    m_state_j4 = JointControlState::POSITION_CONTROL;
    m_setpoint_pos_j4 = position;
    m_setpoint_vel_j4 = std::fabs(velocity);
    m_j4_latched_hold_mode = false;
    resetJ4ControlHistory();
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
    m_setpoint_vel_j5 = std::fabs(velocity);
    pid_integral_j5 = 0.0f;
    pid_previous_error_valid_j5 = false;
    m_j5_position_complete = false;
}

/**
 * @brief Set the J6 Position and Velocity
 * 
 * @param position  The position to move to [deg]
 * @param velocity  The velocity to move at [PWM]
 */
void JointController::setJ6PositionVelocity(float position, float velocity)
{
    m_state_j6 = JointControlState::POSITION_CONTROL;
    m_setpoint_pos_j6 = position;
    m_setpoint_vel_j6 = std::fabs(velocity);
    pid_integral_j6 = 0.0f;
    pid_previous_error_valid_j6 = false;
    m_j6_position_complete = false;
}

/**
 * @brief Set the J4 PID parameters
 */
void JointController::setJ4PID(float p, float i, float d)
{
    pid_p_j4 = p;
    pid_i_j4 = i;
    pid_d_j4 = d;
    resetJ4ControlHistory();
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
    pid_integral_j5 = 0.0f;
    pid_previous_error_valid_j5 = false;
}

/**
 * @brief Set the J6 PID parameters
 * 
 * @param p     Proportional gain
 * @param i     Integral gain
 * @param d     Derivative gain
 */
void JointController::setJ6PID(float p, float i, float d)
{
    pid_p_j6 = p;
    pid_i_j6 = i;
    pid_d_j6 = d;
    pid_integral_j6 = 0.0f;
    pid_previous_error_valid_j6 = false;
}

void JointController::setJ5J6Mixing(
    float j5ToM5, float j5ToM6, float j6ToM5, float j6ToM6)
{
    m_j5_to_m5 = j5ToM5;
    m_j5_to_m6 = j5ToM6;
    m_j6_to_m5 = j6ToM5;
    m_j6_to_m6 = j6ToM6;
    pid_integral_j5 = 0.0f;
    pid_integral_j6 = 0.0f;
    pid_previous_error_valid_j5 = false;
    pid_previous_error_valid_j6 = false;
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

    if(config.size() >= 6){
        setJ4Position(config[3]);
        setJ5Position(config[4]);
        setJ6Position(config[5]);
    }
}
