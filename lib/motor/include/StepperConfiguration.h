/**
 * @file StepperConfiguration.h
 * @author Edgar W (mail@techniccontroller.com)
 * @brief 
 * @version 0.1
 * @date 2023-06-02
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include <stdint.h>
#include <vector>

class StepperConfiguration{
public:

    StepperConfiguration(uint8_t pinMS1, uint8_t pinMS2, uint8_t pinMS3, uint8_t pinEnable, uint8_t microsteppingResolution = 1, float gearRatio = 1.0);

    void setMicrostepping(uint8_t res);
    void setGearRatio(float ratio);
    int angleDegToSteps(float angle);
    int angleRadToSteps(float angle);
    float stepsToAngleDeg(int steps);
    float stepsToAngleRad(int steps);

    std::vector<float> inverseKinematics(float x, float y, float z);


private:

    void setupMicrosteppingPins();

    uint8_t _pinMS1;
    uint8_t _pinMS2;
    uint8_t _pinMS3;
    uint8_t _pinEnable;
    uint8_t _microsteppingResolution;
    float _gearRatio;

};