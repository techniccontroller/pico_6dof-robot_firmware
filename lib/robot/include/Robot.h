#include <stdio.h>
#include <string>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include <AS5600.h>
#include <AccelStepper.h>
#include <StepperConfiguration.h>
#include <DCMotor.h>
#include <ContinuousServo.h>
#include <MotorController.h>
#include <JointController.h>
#include <Gripper.h>
#include <json.hpp>

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

    enum GripperPose
    {
        NONE,
        DOWN,
        LEVEL,
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

    void moveToConfiguration(std::vector<float> config, float velocity);
    void moveToPose(std::vector<float> pose, float velocity);

    void openGripper();
    void closeGripper();
    void setGripperPosition(float position);

    std::vector<float> getConfiguration();
    std::vector<float> getPose();

    float getJointPosition(int joint);

    std::string getRobotDataAsJson();

    void setPID(int joint, float p, float i, float d);
    void setJ5J6Mixing(float j5ToM5, float j5ToM6,
                       float j6ToM5, float j6ToM6);

private:
    RobotMode mode = RobotMode::AUTO;     /**< Current mode of the robot. */
    AS5600 m_encoderJ2; /**< Encoder J2. */
    AS5600 m_encoderJ3; /**< Encoder J3. */
    AS5600 m_encoderJ4; /**< Encoder J4. */
    AS5600 m_encoderJ5; /**< Encoder J5. */
    AS5600 m_encoderJ6; /**< Encoder J6. */

    AccelStepper m_stepperM1; /**< Stepper M1. */
    AccelStepper m_stepperM2; /**< Stepper M2. */
    AccelStepper m_stepperM3; /**< Stepper M3. */

    ContinuousServo m_motorM4; /**< Continuous-rotation servo M4. */
    DCMotor m_motorM5; /**< Motor M5. */
    DCMotor m_motorM6; /**< Motor M6. */

    JointController m_jointController; /**< Joint controller. */
    MotorController m_motorController;           /**< MotorController. */

    StepperConfiguration m_stepperConfiguration; /**< Stepper configuration. */

    Gripper m_gripper; /**< Gripper. */

    void loadSensorCalibrationData(AS5600 *encoder, int addr);
    void writeSensorCalibrationData(AS5600 *encoder, int addr);
    void initEEPROM();

    std::vector<float> inverseKinematics(float x, float y, float z, GripperPose gripperPose = GripperPose::DOWN);
    std::vector<float> forwardKinematics(std::vector<float> config);
};
