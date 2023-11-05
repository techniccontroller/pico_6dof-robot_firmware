#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include <AS5600.h>
#include <AccelStepper.h>
#include <StepperConfiguration.h>
#include <DCMotor.h>
#include <MotorController.h>
#include <JointController.h>
#include <Gripper.h>

#pragma once

class Robot
{


    

public:

    enum RobotMode
    {
        JOINTCONTROL,
        MOTORCONTROL,
        AUTO
    };

    Robot(void);

    void setMode(RobotMode mode);
    RobotMode getMode();

    void step();
    void run();

    void saveEEPROMDataPersistent();
    void loadAllSensorCalibrationData();
    void writeAllSensorCalibrationData();

    void initJoint(int joint);
    void zeroJoint(int joint);
    void setMotorVelocity(int motor, float velocity);

    void setJointPosition(int joint, float position);
    void setJointVelocity(int joint, float velocity);
    void setJointPositionVelocity(int joint, float position, float velocity);

    void moveToConfiguration(std::vector<float> config, float velocity = 10);
    void moveToPose(std::vector<float> pose, float velocity = 10);

    void openGripper();
    void closeGripper();
    void setGripperPosition(float position);

    std::vector<float> getConfiguration();
    std::vector<float> getPose();

    float getJointPosition(int joint);

    void printEncoderPositions();

    void setPID(int joint, float p, float i, float d);

private:
    RobotMode mode = RobotMode::AUTO;     /**< Current mode of the robot. */
    AS5600 m_encoderJ2; /**< Encoder J2. */
    AS5600 m_encoderJ3; /**< Encoder J3. */
    AS5600 m_encoderJ4; /**< Encoder J4. */
    AS5600 m_encoderJ5; /**< Encoder J5. */

    AccelStepper m_stepperM1; /**< Stepper M1. */
    AccelStepper m_stepperM2; /**< Stepper M2. */
    AccelStepper m_stepperM3; /**< Stepper M3. */

    DCMotor m_motorM4; /**< Motor M4. */
    DCMotor m_motorM5; /**< Motor M5. */

    JointController m_jointController; /**< Joint controller. */
    MotorController m_motorController;           /**< MotorController. */

    StepperConfiguration m_stepperConfiguration; /**< Stepper configuration. */

    Gripper m_gripper; /**< Gripper. */

    void loadSensorCalibrationData(AS5600 *encoder, int addr);
    void writeSensorCalibrationData(AS5600 *encoder, int addr);
    void initEEPROM();

    std::vector<float> inverseKinematics(float x, float y, float z);
    std::vector<float> forwardKinematics(std::vector<float> config);
};