#include "DCMotor.h"
#include "gpio_functions.h"
#include "pwm_functions.h"
#include <stdlib.h>

DCMotor::DCMotor(uint8_t enable_pin, uint8_t dir1_pin, uint8_t dir2_pin, volatile long *current_pos, volatile int *direction)
{
    m_enable_pin = enable_pin;
    m_dir1_pin = dir1_pin;
    m_dir2_pin = dir2_pin;
    m_current_pos = current_pos;
    m_direction = direction;
    
    pwm_func_init(m_enable_pin);
    gpio_func_set_mode(m_dir1_pin, GPIO_FUNC_OUTPUT);
    gpio_func_set_mode(m_dir2_pin, GPIO_FUNC_OUTPUT);
    ctrlOutput(0);
}

void DCMotor::ctrlOutput(int output)
{
    if(output > 0){
        gpio_func_set_state(m_dir1_pin, HIGH);
        gpio_func_set_state(m_dir2_pin, LOW);
        *m_direction = 1;
    }
    else if(output < 0){
        gpio_func_set_state(m_dir1_pin, LOW);
        gpio_func_set_state(m_dir2_pin, HIGH);
        *m_direction = -1;
    }
    else{
        gpio_func_set_state(m_dir1_pin, LOW);
        gpio_func_set_state(m_dir2_pin, LOW);
        *m_direction = 0;
    }
    pwm_func_set_level(m_enable_pin, std::abs(output));
    
}

void DCMotor::setSpeed(float speed)
{
    m_speed = speed;
}

void DCMotor::setCurrentPosition(long pos)
{
    *m_current_pos = pos;
}

void DCMotor::moveTo(long absolute)
{
    m_target_pos = absolute;
}

void DCMotor::move(long relative)
{
    m_target_pos = *m_current_pos + relative;
}

void DCMotor::run()
{
    if(*m_current_pos < m_target_pos){
        ctrlOutput(abs(m_speed));
    }
    else if(*m_current_pos > m_target_pos){
        ctrlOutput(-abs(m_speed));
    }
    else{
        ctrlOutput(0);
    }
}

void DCMotor::runSpeed()
{
    ctrlOutput(m_speed);
}

long DCMotor::distanceToGo()
{
    return m_target_pos - *m_current_pos;
}

long DCMotor::currentPosition()
{
    return *m_current_pos;
}

long DCMotor::targetPosition()
{
    return m_target_pos;
}

void DCMotor::stop()
{
    ctrlOutput(0);
}
