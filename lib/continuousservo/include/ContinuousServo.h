#pragma once

#include <stdint.h>

/**
 * @brief Continuous-rotation servo controlled by a signed speed command.
 *
 * A speed of zero sends the configured neutral servo command. Positive and
 * negative speeds move on opposite sides of the neutral command.
 */
class ContinuousServo
{
public:
    ContinuousServo(uint8_t pin, float neutralCommand, float minimumCommand,
                    float maximumCommand, float direction = 1.0f);

    void setSpeed(float speed);
    void stop();
    void setNeutralCommand(float neutralCommand);

    float getSpeed() const;
    float getCommand() const;
    float getNeutralCommand() const;
    float getDirection() const;

private:
    uint8_t m_pin;
    float m_neutralCommand;
    float m_minimumCommand;
    float m_maximumCommand;
    float m_direction;
    float m_speed = 0.0f;
    float m_command = 0.0f;
};
