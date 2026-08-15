#pragma once

#include <stdint.h>

enum class RobotCommandType : uint8_t
{
    INIT_JOINT,
    ZERO_JOINT,
    SET_JOINT_POSITION,
    SET_JOINT_VELOCITY,
    SET_JOINT_POSITION_VELOCITY,
    SET_MOTOR_VELOCITY,
    SET_MODE,
    MOVE_TO_CONFIGURATION,
    SET_PID,
    SET_J5_J6_MIXING,
    SAVE_ZEROS,
    LOAD_ZEROS,
    OPEN_GRIPPER,
    CLOSE_GRIPPER,
    SET_GRIPPER_POSITION
};

/** Fixed-size message that can safely be copied through a Pico SDK queue. */
struct RobotCommand
{
    RobotCommandType type = RobotCommandType::SET_MODE;
    uint8_t target = 0;
    uint8_t value_count = 0;
    float values[7] = {};
};
