/**
 * @file jointcontroller.h
 * @author Edgar W (mail@techniccontroller.com)
 * @brief 
 * @version 0.1
 * @date 2023-10-28
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef RPI_PICO_JOINTCONTROLLER_H
#define RPI_PICO_JOINTCONTROLLER_H

#include <vector>
#include <AccelStepper.h>
#include <DCMotor.h>
#include <AS5600.h>
#include <defines_constants.h>
#include <StepperConfiguration.h>


class JointController {

    enum JointControlState {
        DISABLED,
        INITIALIZATION,
        POSITION_CONTROL,
        VELOCITY_CONTROL
    };

public:
    JointController(StepperConfiguration *stepperConfiguration);

    void addM1(AccelStepper *stepper = NULL);
    void addM2(AccelStepper *stepper = NULL);
    void addM3(AccelStepper *stepper = NULL);
    void addM4(DCMotor *motor = NULL);
    void addM5(DCMotor *motor = NULL);

    void addE1(AS5600 *encoder = NULL);
    void addE2(AS5600 *encoder = NULL);
    void addE3(AS5600 *encoder = NULL);
    void addE4(AS5600 *encoder = NULL);
    void addE5(AS5600 *encoder = NULL);

    void initialize();

    void moveToConfiguration(std::vector<float> config, float velocity = 0.1f);

    void stepStepper(AccelStepper * stepper, AS5600 * encoder, JointControlState * state, float * setpoint_pos, float * setpoint_vel);
    std::vector<float> getConfiguration();
    void checkJointLimitsJ2J3();
    void stepDCMotor(DCMotor * motor, AS5600 * encoder, JointControlState * state, float pid_p, float pid_i, float pid_d, float * setpoint_pos, float * setpoint_vel, float * speed_m4, float * speed_m5);
    bool isJointLimitReachedJ4(float speed_m4_j4);
    bool isAllEncoderStatusValid();

    void step();
    void run();
    void reset();

    void initializeJ1();
    void initializeJ2();
    void initializeJ3();
    void initializeJ4();
    void initializeJ5();

    void zeroJ1();
    void zeroJ2();
    void zeroJ3();
    void zeroJ4();
    void zeroJ5();

    void setJ1Position(float position);
    void setJ2Position(float position);
    void setJ3Position(float position);
    void setJ4Position(float position);
    void setJ5Position(float position);

    void setJ1Velocity(float velocity);
    void setJ2Velocity(float velocity);
    void setJ3Velocity(float velocity);
    void setJ4Velocity(float velocity);
    void setJ5Velocity(float velocity);

    void setJ1PositionVelocity(float position, float velocity);
    void setJ2PositionVelocity(float position, float velocity);
    void setJ3PositionVelocity(float position, float velocity);
    void setJ4PositionVelocity(float position, float velocity);
    void setJ5PositionVelocity(float position, float velocity);

    void setJ4PID(float p, float i, float d);
    void setJ5PID(float p, float i, float d);

private:
    AccelStepper *m_stepper1;
    AccelStepper *m_stepper2;
    AccelStepper *m_stepper3;
    DCMotor *m_motor4;
    DCMotor *m_motor5;

    AS5600 *m_encoder1;
    AS5600 *m_encoder2;
    AS5600 *m_encoder3;
    AS5600 *m_encoder4;
    AS5600 *m_encoder5;

    JointControlState m_state_j1 = JointControlState::DISABLED;
    JointControlState m_state_j2 = JointControlState::DISABLED;
    JointControlState m_state_j3 = JointControlState::DISABLED;
    JointControlState m_state_j4 = JointControlState::DISABLED;
    JointControlState m_state_j5 = JointControlState::DISABLED;

    float pid_p_j4 = 10;
    float pid_i_j4 = 0;
    float pid_d_j4 = 0;
    float pid_p_j5 = 10;
    float pid_i_j5 = 0;
    float pid_d_j5 = 0;

    StepperConfiguration *m_stepperConfiguration; /**< Stepper configuration. */

    float m_setpoint_pos_j1 = 0;    /**< The current position setpoint of the joint 1 [deg]*/
    float m_setpoint_pos_j2 = 0;    /**< The current position setpoint of the joint 2 [deg]*/
    float m_setpoint_pos_j3 = 0;    /**< The current position setpoint of the joint 3 [deg]*/
    float m_setpoint_pos_j4 = 0;    /**< The current position setpoint of the joint 4 [deg]*/
    float m_setpoint_pos_j5 = 0;    /**< The current position setpoint of the joint 5 [deg]*/

    float m_setpoint_vel_j1 = 0;    /**< The current velocity setpoint of the joint 1 [deg/s]*/
    float m_setpoint_vel_j2 = 0;    /**< The current velocity setpoint of the joint 2 [deg/s]*/
    float m_setpoint_vel_j3 = 0;    /**< The current velocity setpoint of the joint 3 [deg/s]*/
    float m_setpoint_vel_j4 = 0;    /**< The current velocity setpoint of the joint 4 [PWM]*/
    float m_setpoint_vel_j5 = 0;    /**< The current velocity setpoint of the joint 5 [PWM]*/
};



#endif //RPI_PICO_JOINTCONTROLLER_H