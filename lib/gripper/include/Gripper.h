#include <stdio.h>
#include "pico/stdlib.h"
#include "pico_servo.h"

#pragma once

class Gripper
{
public:
    Gripper(uint8_t pin);

    void open();

    void moveToPosition(float position);

    void close();

private:
    uint8_t m_pin;
};