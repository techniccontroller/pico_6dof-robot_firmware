#include <Gripper.h>
#include "pico_servo.h"
#include "defines_constants.h"

Gripper::Gripper(uint8_t pin)
{
    m_pin = pin;
    servo_init();
    servo_clock_auto();
    servo_attach(m_pin);
}

/**
 * @brief Opens the gripper to full open position.
 * 
 */
void Gripper::open()
{
    servo_move_to(m_pin, LIMIT_GRIP_OPEN);
}

/**
 * @brief Move the gripper to a specific position.
 * 
 * @param position  Position to move to.
 */
void Gripper::moveToPosition(float position)
{
    if(position < LIMIT_GRIP_OPEN)
    {
        position = LIMIT_GRIP_OPEN;
    }
    else if(position > LIMIT_GRIP_CLOSE)
    {
        position = LIMIT_GRIP_CLOSE;
    }
    servo_move_to(m_pin, position);
}

/**
 * @brief Closes the gripper to full close position.
 * 
 */
void Gripper::close()
{
    servo_move_to(m_pin, LIMIT_GRIP_CLOSE);
}
