
#pragma once

#include <stdint.h>


class DCMotor {

    public:

    DCMotor(uint8_t enable_pin, uint8_t dir1_pin, uint8_t dir2_pin);
    DCMotor(uint8_t enable_pin, uint8_t dir1_pin, uint8_t dir2_pin, volatile long *current_pos, volatile int *direction);

    void ctrlOutput(int output);
    void setSpeed(float speed);
    void setCurrentPosition(long pos);
    void moveTo(long absolute);
    void move(long relative);
    void run();
    void runSpeed();
    long distanceToGo();
    long currentPosition();
    long targetPosition();
    void stop();

    private:

    uint8_t m_enable_pin;
    uint8_t m_dir1_pin;
    uint8_t m_dir2_pin;
    long local_current_pos = 0;
    int local_direction = 0;

    float m_speed = 0.0f;
    volatile long * m_current_pos = 0;
    volatile int * m_direction = 0;
    long m_target_pos = 0;

};