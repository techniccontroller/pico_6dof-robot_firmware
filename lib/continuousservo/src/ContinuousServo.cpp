#include "ContinuousServo.h"

#include <algorithm>
#include <cmath>
#include "pico_servo.h"

ContinuousServo::ContinuousServo(uint8_t pin, float neutralCommand,
                                 float minimumCommand, float maximumCommand,
                                 float direction)
    : m_pin(pin),
      m_neutralCommand(neutralCommand),
      m_minimumCommand(minimumCommand),
      m_maximumCommand(maximumCommand),
      m_direction(direction < 0.0f ? -1.0f : 1.0f),
      m_command(neutralCommand)
{
    servo_init();
    servo_clock_auto();
    servo_attach(m_pin);
    stop();
}

void ContinuousServo::setSpeed(float speed)
{
    m_command = std::clamp(
        m_neutralCommand + m_direction * speed,
        m_minimumCommand,
        m_maximumCommand
    );
    m_speed = (m_command - m_neutralCommand) * m_direction;
    servo_move_to_float(m_pin, m_command);
}

void ContinuousServo::stop()
{
    setSpeed(0.0f);
}

void ContinuousServo::setNeutralCommand(float neutralCommand)
{
    m_neutralCommand = std::clamp(
        neutralCommand,
        m_minimumCommand,
        m_maximumCommand
    );
}

float ContinuousServo::getSpeed() const
{
    return m_speed;
}

float ContinuousServo::getCommand() const
{
    return m_command;
}

float ContinuousServo::getNeutralCommand() const
{
    return m_neutralCommand;
}

float ContinuousServo::getDirection() const
{
    return m_direction;
}
