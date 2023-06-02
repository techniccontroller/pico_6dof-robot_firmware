#include "StepperConfiguration.h"
#include "hardware/gpio.h"

StepperConfiguration::StepperConfiguration(uint8_t pinMS1, uint8_t pinMS2, uint8_t pinMS3, uint8_t pinEnable, uint8_t microsteppingResolution)
{
    _pinMS1 = pinMS1;
    _pinMS2 = pinMS2;
    _pinMS3 = pinMS3;
    _pinEnable = pinEnable;
    _microsteppingResolution = microsteppingResolution;
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
