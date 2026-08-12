#include "Robot.h"
#include "defines_constants.h"
#include "EEPROM.h"
#include "hardware/adc.h"
#include <math.h>
#include <string>
#include <cstdio>

namespace
{
void roundJsonFloatsToThreeDecimals(nlohmann::json& value)
{
    if (value.is_number_float())
    {
        value = round(value.get<double>() * 1000.0) / 1000.0;
        return;
    }

    if (value.is_array() || value.is_object())
    {
        for (auto& child : value)
        {
            roundJsonFloatsToThreeDecimals(child);
        }
    }
}
}


Robot::Robot(void): 
    m_encoderJ2(I2C_PORT0, I2C_SCL0_PIN, I2C_SDA0_PIN, ASADDR, 7, TCAADDR), 
    m_encoderJ3(I2C_PORT0, I2C_SCL0_PIN, I2C_SDA0_PIN, ASADDR, 6, TCAADDR),
    m_encoderJ4(I2C_PORT0, I2C_SCL0_PIN, I2C_SDA0_PIN, ASADDR, 5, TCAADDR),
    m_encoderJ5(I2C_PORT0, I2C_SCL0_PIN, I2C_SDA0_PIN, ASADDR, 4, TCAADDR),
    m_encoderJ6(I2C_PORT0, I2C_SCL0_PIN, I2C_SDA0_PIN, ASADDR, 3, TCAADDR),
    m_stepperM1(AccelStepper::DRIVER, MOTOR1_STEP_PIN, MOTOR1_DIR_PIN),
    m_stepperM2(AccelStepper::DRIVER, MOTOR2_STEP_PIN, MOTOR2_DIR_PIN),
    m_stepperM3(AccelStepper::DRIVER, MOTOR3_STEP_PIN, MOTOR3_DIR_PIN),
    m_stepperConfiguration(MS1_PIN, MS2_PIN, MS3_PIN, MICROSTEP_RESOLUTION, 60.0/16.0 * 60.0/16.0),
    m_motorM4(SERVO_CONTINUE_J4_PIN, CONTINUOUS_SERVO_J4_NEUTRAL_COMMAND,
              CONTINUOUS_SERVO_J4_MIN_COMMAND, CONTINUOUS_SERVO_J4_MAX_COMMAND,
              CONTINUOUS_SERVO_J4_DIRECTION),
    m_motorM5(MOTOR5_ENABLE_PIN, MOTOR5_IN1_PIN, MOTOR5_IN2_PIN),
    m_motorM6(MOTOR6_ENABLE_PIN, MOTOR6_IN1_PIN, MOTOR6_IN2_PIN),
    m_jointController(&m_stepperConfiguration),
    m_motorController(&m_stepperConfiguration),
    m_gripper(SERVO_GRIPPER_PIN)
{
    adc_init();
    adc_gpio_init(J1_HALL_SENSOR_PIN);
    adc_select_input(J1_HALL_SENSOR_ADC_INPUT);
    m_j1HallSensorRaw = adc_read();

    m_stepperM1.setMaxSpeed(2000);
    m_stepperM1.setAcceleration(500);
    m_stepperM2.setMaxSpeed(2000);
    m_stepperM2.setAcceleration(500);
    m_stepperM3.setMaxSpeed(2000);
    m_stepperM3.setAcceleration(500);

    loadAllSensorCalibrationData();

    m_motorController.addM1(&m_stepperM1, NULL);
    m_motorController.addM2(&m_stepperM2, &m_encoderJ2);
    m_motorController.addM3(&m_stepperM3, &m_encoderJ3);
    m_motorController.addM4(&m_motorM4);
    m_motorController.addM5(&m_motorM5);
    m_motorController.addM6(&m_motorM6);

    m_jointController.addM1(&m_stepperM1);
    m_jointController.addM2(&m_stepperM2);
    m_jointController.addM3(&m_stepperM3);
    m_jointController.addM4(&m_motorM4);
    m_jointController.addM5(&m_motorM5);
    m_jointController.addM6(&m_motorM6);

    m_jointController.addE1(NULL);
    m_jointController.addE2(&m_encoderJ2);
    m_jointController.addE3(&m_encoderJ3);
    m_jointController.addE4(&m_encoderJ4);
    m_jointController.addE5(&m_encoderJ5);
    m_jointController.addE6(&m_encoderJ6);

    initEEPROM();
}

void Robot::setMode(RobotMode mode)
{
    this->mode = mode;
    m_jointController.reset();
    m_motorController.reset();
}

Robot::RobotMode Robot::getMode()
{
    return mode;
}

void Robot::step()
{
    adc_select_input(J1_HALL_SENSOR_ADC_INPUT);
    m_j1HallSensorRaw = adc_read();
    m_jointController.setJ1HallSensorRaw(m_j1HallSensorRaw);

    switch (mode)
    {
    case RobotMode::AUTO:
    case RobotMode::JOINTCONTROL:
        m_jointController.step();
        break;
    
    case RobotMode::MOTORCONTROL:
        m_motorController.step();
        break;
    
    default:
        break;
    }
}

void Robot::run()
{
    switch (mode)
    {
    case RobotMode::AUTO:
    case RobotMode::JOINTCONTROL:
        m_jointController.run();
        break;
    
    case RobotMode::MOTORCONTROL:
        m_motorController.run();
        break;
    
    default:
        break;
    }
}

void Robot::loadAllSensorCalibrationData()
{
    loadSensorCalibrationData(&m_encoderJ2, EEPROM_ADDR_ENCJ2);
    loadSensorCalibrationData(&m_encoderJ3, EEPROM_ADDR_ENCJ3);
    loadSensorCalibrationData(&m_encoderJ4, EEPROM_ADDR_ENCJ4);
    loadSensorCalibrationData(&m_encoderJ5, EEPROM_ADDR_ENCJ5);
    loadSensorCalibrationData(&m_encoderJ6, EEPROM_ADDR_ENCJ6);
}

void Robot::loadSensorCalibrationData(AS5600 *encoder, int addr)
{
    int zero = 0;
    zero = EEPROM.get(addr, zero);
    if(zero >= AS5601_ANGLE_MAX)
    {
        encoder->setZero();
        printf("Set current pos as zero: %d\n", encoder->getZero());
    }
    else
    {
        encoder->setZero(zero);
        printf("Loaded zero: %d\n", zero);
    }
}

void Robot::writeAllSensorCalibrationData()
{
    writeSensorCalibrationData(&m_encoderJ2, EEPROM_ADDR_ENCJ2);
    writeSensorCalibrationData(&m_encoderJ3, EEPROM_ADDR_ENCJ3);
    writeSensorCalibrationData(&m_encoderJ4, EEPROM_ADDR_ENCJ4);
    writeSensorCalibrationData(&m_encoderJ5, EEPROM_ADDR_ENCJ5);
    writeSensorCalibrationData(&m_encoderJ6, EEPROM_ADDR_ENCJ6);
}

void Robot::writeSensorCalibrationData(AS5600 *encoder, int addr)
{
    int zero = encoder->getZero();
    EEPROM.put(addr, zero);
    printf("Wrote zero: %d\n", zero);
}

void Robot::saveEEPROMDataPersistent()
{
    EEPROM.commit();
}

void Robot::initEEPROM()
{
    EEPROM.begin(EEPROM_SIZE);
}

void Robot::moveToPose(std::vector<float> pose, float velocity)
{
    std::vector<float> config = inverseKinematics(pose[0], pose[1], pose[2]);
    printf("config: %f, %f, %f\n", config[0], config[1], config[2]);
    moveToConfiguration(config, velocity);
}

void Robot::openGripper()
{
    m_gripper.open();
}

void Robot::closeGripper()
{
    m_gripper.close();
}

void Robot::setGripperPosition(float position)
{
    m_gripper.moveToPosition(position);
}

std::vector<float> Robot::getConfiguration()
{
    return m_jointController.getConfiguration();
}

std::vector<float> Robot::getPose()
{
    return forwardKinematics(getConfiguration());
}

float Robot::getJointPosition(int joint)
{
    std::vector<float> config = getConfiguration();
    switch(joint){
        case Joint::J1:
        return config[0];
        break;
        case Joint::J2:
        return config[1];
        break;
        case Joint::J3:
        return config[2];
        break;
        case Joint::J4:
        return config[3];
        break;
        case Joint::J5:
        return config[4];
        break;
        case Joint::J6:
        return config[5];
        break;
    }
    return 0.0f;
}

std::string Robot::getRobotDataAsJson()
{
    nlohmann::json jsonObj;
    
    std::vector<float> config = getConfiguration();
    std::vector<float> pose = getPose();

    jsonObj["robot_data"]["config"] = config;

    jsonObj["robot_data"]["j1_hall_sensor_raw"] = m_j1HallSensorRaw;
    jsonObj["robot_data"]["j1_homed"] = m_jointController.isJ1Homed();

    jsonObj["robot_data"]["pose"] = pose;

    jsonObj["robot_data"]["encoder_positions"] = {m_encoderJ2.getCorrectedAngleDeg(), m_encoderJ3.getCorrectedAngleDeg(),
                                     m_encoderJ4.getCorrectedAngleDeg(), m_encoderJ5.getCorrectedAngleDeg(), m_encoderJ6.getCorrectedAngleDeg()};
    jsonObj["robot_data"]["encoder_positions_raw"] = {m_encoderJ2.readAngleDeg(), m_encoderJ3.readAngleDeg(),
                                         m_encoderJ4.readAngleDeg(), m_encoderJ5.readAngleDeg(), m_encoderJ6.readAngleDeg()};
    jsonObj["robot_data"]["encoder_status"] = {m_encoderJ2.getStatus(), m_encoderJ3.getStatus(),
                                  m_encoderJ4.getStatus(), m_encoderJ5.getStatus(), m_encoderJ6.getStatus()};
    jsonObj["robot_data"]["j4_continuous_servo"] = {
        {"neutral_command", m_motorM4.getNeutralCommand()},
        {"output_command", m_motorM4.getCommand()},
        {"speed", m_motorM4.getSpeed()}
    };
    jsonObj["robot_data"]["diffAngleJ2J3"] = m_jointController.getDiffAngleJ2J3();
    jsonObj["robot_data"]["speeds"] = {m_stepperM1.speed(), m_stepperM2.speed(), m_stepperM3.speed()};
    roundJsonFloatsToThreeDecimals(jsonObj);
    return jsonObj.dump();
}

void Robot::setPID(int joint, float p, float i, float d)
{
    switch(joint){
        case Joint::J4:
        m_jointController.setJ4PID(p, i, d);
        break;
        case Joint::J5:
        m_jointController.setJ5PID(p, i, d);
        break;
        case Joint::J6:
        m_jointController.setJ6PID(p, i, d);
        break;
    }
}

void Robot::setJ5J6Mixing(
    float j5ToM5, float j5ToM6, float j6ToM5, float j6ToM6)
{
    m_jointController.setJ5J6Mixing(
        j5ToM5, j5ToM6, j6ToM5, j6ToM6);
}

/**
 * @brief Move Robot to a specific configuration
 * 
 * @param config    Configuration to move to [deg]
 * @param velocity  Velocity to move with [deg/s]
 */
void Robot::moveToConfiguration(std::vector<float> config, float velocity){
    m_jointController.moveToConfiguration(config, velocity);
}

/**
 * @brief Calculate the configuration of the robot from a given pose
 * 
 * @param x     X position of the robot [m]
 * @param y     Y position of the robot [m]
 * @param z     Z position of the robot [m]
 * @param gripperPose    Pose of the gripper (NONE, DOWN, LEVEL)
 * @return std::vector<float>   Configuration of the robot [deg]
 */
std::vector<float> Robot::inverseKinematics(float x, float y, float z, GripperPose gripperPose){
    float l1 = 0.21;
    float l2 = 0.23;

    float b = sqrt(x*x+y*y);
    float a = sqrt(b*b+z*z);
    float alpha = atan2(z, b);
    float gamma = acos((l1*l1+l2*l2-a*a)/(2*l1*l2));
    float beta = acos((l1*l1+a*a-l2*l2)/(2*l1*a));
    float q1 = M_PI/2 - alpha - beta;
    float q2 = M_PI - gamma;
    float theta = M_PI/2 - q1;


    float m0 = atan2(x, y);
    float m1 = q1;
    float m2 = q2-theta;

    std::vector<float> result;
    result.push_back(m0 * 180.0 / M_PI);
    result.push_back(m1 * 180.0 / M_PI);
    result.push_back(-m2 * 180.0 / M_PI);
    result.push_back(0.0f); // The current Cartesian IK keeps J4 at its zero position.

    if(gripperPose == GripperPose::DOWN){
        float j5 = -(M_PI/2 - m2);
        float j6 = m0;
        result.push_back(j5 * 180.0 / M_PI);
        result.push_back(j6 * 180.0 / M_PI);
    }
    else if(gripperPose == GripperPose::LEVEL){
        float j5 = m2;
        float j6 = 0;
        result.push_back(j5 * 180.0 / M_PI);
        result.push_back(j6 * 180.0 / M_PI);
    }
    return result;
}

/**
 * @brief Calculate the pose of the robot from a given configuration
 * 
 * @param config    Configuration of the robot [deg]
 * @return std::vector<float>   Pose of the robot [m]
 */
std::vector<float> Robot::forwardKinematics(std::vector<float> config)
{
    float l1 = 0.21;
    float l2 = 0.23;

    float m0 = config[0] / 180.0 * M_PI;
    float m1 = config[1] / 180.0 * M_PI;
    float m2 = -config[2] / 180.0 * M_PI;
    float q0 = m0;
    float q1 = m1;
    float theta = M_PI/2 - q1;
    float q2 = m2 + theta;

    float gamma = M_PI - q2;
    float a = sqrt(l1*l1 + l2*l2 - 2 * l1 * l2 * cos(gamma));
    float beta = acos((l1*l1 + a*a - l2*l2) / (2 * l1 * a));
    float alpha = M_PI/2 - beta - q1;

    float z = a * sin(alpha);
    float b = a * cos(alpha);

    float x = b * sin(q0);
    float y = b * cos(q0);

    std::vector<float> result;
    result.push_back(x);
    result.push_back(y);
    result.push_back(z);
    return result;
}

void Robot::initJoint(int joint){
    switch(joint){
        case Joint::J1:
        m_jointController.initializeJ1();
        break;
        case Joint::J2:
        m_jointController.initializeJ2();
        break;
        case Joint::J3:
        m_jointController.initializeJ3();
        break;
        case Joint::J4:
        m_jointController.initializeJ4();
        break;
        case Joint::J5:
        m_jointController.initializeJ5();
        break;
        case Joint::J6:
        m_jointController.initializeJ6();
        break;
    }
}

void Robot::zeroJoint(int joint){
    switch(joint){
        case Joint::J1:
        m_jointController.zeroJ1();
        break;
        case Joint::J2:
        m_jointController.zeroJ2();
        break;
        case Joint::J3:
        m_jointController.zeroJ3();
        break;
        case Joint::J4:
        m_jointController.zeroJ4();
        break;
        case Joint::J5:
        m_jointController.zeroJ5();
        break;
        case Joint::J6:
        m_jointController.zeroJ6();
        break;
    }
}

/**
 * @brief 
 * 
 * @param motor 
 * @param velocity 
 */
void Robot::setMotorVelocity(int motor, float velocity){
    switch(motor){
        case Motor::M1:
        m_motorController.setM1Velocity(velocity);
        break;
        case Motor::M2:
        m_motorController.setM2Velocity(velocity);
        break;
        case Motor::M3:
        m_motorController.setM3Velocity(velocity);
        break;
        case Motor::M4:
        m_motorController.setM4Velocity(velocity);
        break;
        case Motor::M5:
        m_motorController.setM5Velocity(velocity);
        break;
        case Motor::M6:
        m_motorController.setM6Velocity(velocity);
        break;
    }
}

/**
 * @brief Set the velocity of a joint
 * 
 * @param joint     Joint to set the velocity of
 * @param velocity  Velocity to set [deg/s]
 */
void Robot::setJointVelocity(int joint, float velocity){
    switch(joint){
        case Joint::J1:
        m_jointController.setJ1Velocity(velocity);
        break;
        case Joint::J2:
        m_jointController.setJ2Velocity(velocity);
        break;
        case Joint::J3:
        m_jointController.setJ3Velocity(velocity);
        break;
        case Joint::J4:
        m_jointController.setJ4Velocity(velocity);
        break;
        case Joint::J5:
        m_jointController.setJ5Velocity(velocity);
        break;
        case Joint::J6:
        m_jointController.setJ6Velocity(velocity);
        break;
    }
}

void Robot::setJointPositionVelocity(int joint, float position, float velocity){
    switch(joint){
        case Joint::J1:
        m_jointController.setJ1PositionVelocity(position, velocity);
        break;
        case Joint::J2:
        m_jointController.setJ2PositionVelocity(position, velocity);
        break;
        case Joint::J3:
        m_jointController.setJ3PositionVelocity(position, velocity);
        break;
        case Joint::J4:
        m_jointController.setJ4PositionVelocity(position, velocity);
        break;
        case Joint::J5:
        m_jointController.setJ5PositionVelocity(position, velocity);
        break;
        case Joint::J6:
        m_jointController.setJ6PositionVelocity(position, velocity);
        break;
    }
}

void Robot::setJointPosition(int joint, float position){
    switch(joint){
        case Joint::J1:
        m_jointController.setJ1Position(position);
        break;
        case Joint::J2:
        m_jointController.setJ2Position(position);
        break;
        case Joint::J3:
        m_jointController.setJ3Position(position);
        break;
        case Joint::J4:
        m_jointController.setJ4Position(position);
        break;
        case Joint::J5:
        m_jointController.setJ5Position(position);
        break;
        case Joint::J6:
        m_jointController.setJ6Position(position);
        break;
    }
}

