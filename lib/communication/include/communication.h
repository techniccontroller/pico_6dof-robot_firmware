/**
 * @file communication.h
 * @author Edgar W. (mail@techniccontrollercom)
 * @brief This library handles all communication related functionality. It acts as input layer to the external client.
 * @version 0.2
 * @date 2023-05-29
 *
 * @copyright Copyright (c) 2022
 *
 */
#pragma once

#include <string.h>
#include "defines_constants.h"
#include "comm_functions.h"
#include "AccelStepper.h"
#include "MultiStepper.h"
#include "AS5600.h"
#include "controller.h"
#include <vector>

/**
 * @brief Set this flag to 1 to receive additional debug output via the communication interface
 *
 */
#define DEBUG_IS_ENABLED 1

class Communication
{
public:
    Communication(Controller *controller, AccelStepper *stepper1=NULL, AccelStepper *stepper2=NULL, AccelStepper *stepper3=NULL, AccelStepper *stepper4=NULL, AS5600 *encoder1=NULL, AS5600 *encoder2=NULL);

    /**
     * @brief Check if new commands have been received and process them if so
     *
     */
    void check_incoming_cmds();

private:
    static const int BUFFER_SIZE = 30; /**< Size of the buffer for incoming commands. */

    static bool starts_with(const char *pre, const char *str);
    static bool contains(const char *substring, const char *str);

    /**
     * @brief Extract the motor number from a command
     *
     * @param cmd given command, e.g. 'M1'
     * @return uint8_t motor number
     */
    static uint8_t extract_related_motor(char *cmd);

    /**
     * @brief extract control value from command if available
     *
     * @param cmd given command, with a value in brackets 'MY_COMMAND(value)'
     * @return int
     */
    std::vector<float> extract_cmd_values(const char *cmd);
    void process_cmd(char *cmd);

    char buffer[BUFFER_SIZE];        /**< Buffer for incoming commands. */
    bool _manual_drive = false;      /**< Flag wheter the manual driving mode is active. */
    AccelStepper *_stepper1;          /**< Stepper 1. */
    AccelStepper *_stepper2;          /**< Stepper 2. */
    AccelStepper *_stepper3;          /**< Stepper 3. */
    AccelStepper *_stepper4;          /**< Stepper 4. */
    AS5600 *_encoder1;               /**< Encoder 1. */
    AS5600 *_encoder2;               /**< Encoder 2. */
    Controller *_controller;         /**< Controller. */
};
