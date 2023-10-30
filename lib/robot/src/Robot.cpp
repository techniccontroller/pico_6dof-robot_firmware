#include "Robot.h"
#include "defines_constants.h"
#include "EEPROM.h"
#include <math.h>


Robot::Robot(void): 
    m_encoderJ2(I2C_PORT0, I2C_SCL0, I2C_SDA0, ASADDR, 7, TCAADDR), 
    m_encoderJ3(I2C_PORT0, I2C_SCL0, I2C_SDA0, ASADDR, 6, TCAADDR),
    m_encoderJ4(I2C_PORT0, I2C_SCL0, I2C_SDA0, ASADDR, 5, TCAADDR),
    m_encoderJ5(I2C_PORT0, I2C_SCL0, I2C_SDA0, ASADDR, 4, TCAADDR),
    m_stepperM1(AccelStepper::DRIVER, MOTOR1_STEP_PIN, MOTOR1_DIR_PIN),
    m_stepperM2(AccelStepper::DRIVER, MOTOR2_STEP_PIN, MOTOR2_DIR_PIN),
    m_stepperM3(AccelStepper::DRIVER, MOTOR3_STEP_PIN, MOTOR3_DIR_PIN),
    m_stepperConfiguration(MS1_PIN, MS2_PIN, MS3_PIN, ENABLE_PIN, 4, 60.0/16.0 * 60.0/16.0),
    m_motorM4(MOTOR4_ENABLE_PIN, MOTOR4_IN1_PIN, MOTOR4_IN2_PIN),
    m_motorM5(MOTOR5_ENABLE_PIN, MOTOR5_IN1_PIN, MOTOR5_IN2_PIN),
    m_jointController(&m_stepperConfiguration)
{
    m_stepperM1.setMaxSpeed(2000);
    m_stepperM1.setAcceleration(500);
    m_stepperM2.setMaxSpeed(2000);
    m_stepperM2.setAcceleration(500);
    m_stepperM3.setMaxSpeed(2000);
    m_stepperM3.setAcceleration(500);

    loadAllSensorCalibrationData();

    m_controller.addM1(&m_stepperM1, NULL);
    m_controller.addM2(&m_stepperM2, &m_encoderJ2);
    m_controller.addM3(&m_stepperM3, &m_encoderJ3);
    m_controller.addM4(&m_motorM4);
    m_controller.addM5(&m_motorM5);

    m_jointController.addM1(&m_stepperM1);
    m_jointController.addM2(&m_stepperM2);
    m_jointController.addM3(&m_stepperM3);
    m_jointController.addM4(&m_motorM4);
    m_jointController.addM5(&m_motorM5);

    m_jointController.addE1(NULL);
    m_jointController.addE2(&m_encoderJ2);
    m_jointController.addE3(&m_encoderJ3);
    m_jointController.addE4(&m_encoderJ4);
    m_jointController.addE5(&m_encoderJ5);

    initEEPROM();
}

void Robot::step()
{
    //m_controller.step();
    m_jointController.step();
}

void Robot::run()
{
    //m_controller.run();
    m_jointController.run();
}

void Robot::loadAllSensorCalibrationData()
{
    loadSensorCalibrationData(&m_encoderJ2, EEPROM_ADDR_ENCJ2);
    loadSensorCalibrationData(&m_encoderJ3, EEPROM_ADDR_ENCJ3);
    loadSensorCalibrationData(&m_encoderJ4, EEPROM_ADDR_ENCJ4);
    loadSensorCalibrationData(&m_encoderJ5, EEPROM_ADDR_ENCJ5);
}

void Robot::loadSensorCalibrationData(AS5600 *encoder, int addr)
{
    int zero = 0;
    zero = EEPROM.get(addr, zero);
    if(zero >= AS5601_ANGLE_MAX)
    {
        encoder->setZero();
    }
    else
    {
        encoder->setZero(zero);
    }
}

void Robot::writeAllSensorCalibrationData()
{
    writeSensorCalibrationData(&m_encoderJ2, EEPROM_ADDR_ENCJ2);
    writeSensorCalibrationData(&m_encoderJ3, EEPROM_ADDR_ENCJ3);
    writeSensorCalibrationData(&m_encoderJ4, EEPROM_ADDR_ENCJ4);
    writeSensorCalibrationData(&m_encoderJ5, EEPROM_ADDR_ENCJ5);
}

void Robot::writeSensorCalibrationData(AS5600 *encoder, int addr)
{
    EEPROM.put(addr, encoder->getZero());
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
    }
    return 0.0f;
}

void Robot::printEncoderPositions()
{
    printf("Encoder positions: %f, %f, %f, %f\n", m_encoderJ2.getCorrectedAngleDeg(), m_encoderJ3.getCorrectedAngleDeg(), m_encoderJ4.getCorrectedAngleDeg(), m_encoderJ5.getCorrectedAngleDeg());
    printf("Encoder positions(raw): %d, %d, %d, %d\n", m_encoderJ2.readAngle(), m_encoderJ3.readAngle(), m_encoderJ4.readAngle(), m_encoderJ5.readAngle());
    printf("Encoder status: %d, %d, %d, %d\n", m_encoderJ2.getStatus(), m_encoderJ3.getStatus(), m_encoderJ4.getStatus(), m_encoderJ5.getStatus());
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
    }
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
 * @return std::vector<float>   Configuration of the robot [deg]
 */
std::vector<float> Robot::inverseKinematics(float x, float y, float z){
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
    }
}


void Robot::setMotorVelocity(int motor, float velocity){
    switch(motor){
        case Motor::M1:
        m_controller.setM1Velocity(velocity);
        break;
        case Motor::M2:
        m_controller.setM2Velocity(velocity);
        break;
        case Motor::M3:
        m_controller.setM3Velocity(velocity);
        break;
        case Motor::M4:
        m_controller.setM4Velocity(velocity);
        break;
        case Motor::M5:
        m_controller.setM5Velocity(velocity);
        break;
    }
}

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
    }
}

