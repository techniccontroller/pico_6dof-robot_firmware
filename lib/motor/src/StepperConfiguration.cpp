#include "StepperConfiguration.h"
#include "hardware/gpio.h"
#include <math.h>

StepperConfiguration::StepperConfiguration(uint8_t pinMS1, uint8_t pinMS2, uint8_t pinMS3, uint8_t pinEnable, uint8_t microsteppingResolution, float gearRatio)
{
    _pinMS1 = pinMS1;
    _pinMS2 = pinMS2;
    _pinMS3 = pinMS3;
    _pinEnable = pinEnable;
    _microsteppingResolution = microsteppingResolution;
    _gearRatio = gearRatio;
    setupMicrosteppingPins();
    setMicrostepping(_microsteppingResolution);
}

void StepperConfiguration::setupMicrosteppingPins()
{
    gpio_init(_pinMS1);
    gpio_init(_pinMS2);
    gpio_init(_pinMS3);
    gpio_set_dir(_pinMS1, GPIO_OUT);
    gpio_set_dir(_pinMS2, GPIO_OUT);
    gpio_set_dir(_pinMS3, GPIO_OUT);
}

void StepperConfiguration::setMicrostepping(uint8_t res)
{
    switch (res)
    {
    case 1:
        gpio_put(_pinMS1, 0);
        gpio_put(_pinMS2, 0);
        gpio_put(_pinMS3, 0);
        break;
    case 2:
        gpio_put(_pinMS1, 1);
        gpio_put(_pinMS2, 0);
        gpio_put(_pinMS3, 0);
        break;
    case 4:
        gpio_put(_pinMS1, 0);
        gpio_put(_pinMS2, 1);
        gpio_put(_pinMS3, 0);
        break;
    case 8:
        gpio_put(_pinMS1, 1);
        gpio_put(_pinMS2, 1);
        gpio_put(_pinMS3, 0);
        break;
    case 16:
        gpio_put(_pinMS1, 1);
        gpio_put(_pinMS2, 1);
        gpio_put(_pinMS3, 1);
        break;

    default:
        // no valid microstep resolution
        break;
    }
}

void StepperConfiguration::setGearRatio(float ratio)
{
    _gearRatio = ratio;
}

int StepperConfiguration::angleDegToSteps(float angle){
    float motorRotations = angle * _gearRatio / 360.0;
    int steps = motorRotations * 200.0 * _microsteppingResolution;
    return steps;
}

int StepperConfiguration::angleRadToSteps(float angle){
    float motorRotations = angle * _gearRatio / (2 * M_PI);
    int steps = motorRotations * 200.0 * _microsteppingResolution;
    return steps;
}


float StepperConfiguration::stepsToAngleDeg(int steps){
    float motorRotations = steps / 200.0 / _microsteppingResolution;
    float angle = motorRotations * 360.0 / _gearRatio;
    return angle;
}

float StepperConfiguration::stepsToAngleRad(int steps){
    float motorRotations = steps / 200.0 / _microsteppingResolution;
    float angle = motorRotations * 2 * M_PI / _gearRatio;
    return angle;
}

std::vector<float> inverseKinematics(float x, float y, float z){
    std::vector<float> result;

    float l1 = 0.21;
    float l2 = 0.23;
    float l3 = sqrt(x*x+y*y);

    float gamma = acos((l1*l1+l2*l2-l3*l3)/(2*l1*l2));
    float alpha = atan2(y,x);
    float beta = acos((l1*l1+l3*l3-l2*l2)/(2*l1*l3));

    float q1 = M_PI/2 - alpha - beta;
    float q2 = M_PI - (M_PI/2 - q1) - gamma;

    result.push_back(q1);
    result.push_back(q2);
    return result;
}


