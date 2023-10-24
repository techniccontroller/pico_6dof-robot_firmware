
#pragma once

#include <stdint.h>


class DCMotor {

    public:

    DCMotor(uint8_t enable_pin, uint8_t dir1_pin, uint8_t dir2_pin, volatile long *current_pos);

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

    float m_speed;
    volatile long * m_current_pos;
    long m_target_pos;

};