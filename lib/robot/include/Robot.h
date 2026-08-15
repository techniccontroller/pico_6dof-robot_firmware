#include <stdio.h>
#include <string>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include <as5600.h>
#include <AccelStepper.h>
#include <StepperConfiguration.h>
#include <DCMotor.h>
#include <ContinuousServo.h>
#include <MotorController.h>
#include <JointController.h>
#include <Gripper.h>
#include <json.hpp>

#pragma once

/** POD telemetry message copied from the robot core to the communication core. */
struct RobotTelemetry
{
    float config[6] = {};
    uint16_t j1HallSensorRaw = 4095;
    bool j1Homed = false;
    float encoderPositions[5] = {};
    float encoderPositionsRaw[5] = {};
    uint8_t encoderStatus[5] = {};
    int encoderZeros[5] = {};
    float j4NeutralCommand = 0.0f;
    float j4OutputCommand = 0.0f;
    float j4Speed = 0.0f;
    float diffAngleJ2J3 = 0.0f;
    float stepperSpeeds[3] = {};
};

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

    void moveToConfiguration(std::vector<float> config, float velocity);
    void moveToConfiguration(const float config[6], float velocity);

    void openGripper();
    void closeGripper();
    void setGripperPosition(float position);

    std::vector<float> getConfiguration();
    void copyConfiguration(float config[6]);

    float getJointPosition(int joint);

    std::string getRobotDataAsJson();
    RobotTelemetry getTelemetrySnapshot();
    static std::string telemetryToJson(const RobotTelemetry& telemetry);

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
    uint16_t m_j1HallSensorRaw = 4095;

    void loadSensorCalibrationData(AS5600 *encoder, int addr);
    void writeSensorCalibrationData(AS5600 *encoder, int addr);
    void initEEPROM();

};
