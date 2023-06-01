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
#include "motor.h"

/**
 * @brief Set this flag to 1 to receive additional debug output via the communication interface
 * 
 */
#define DEBUG_IS_ENABLED 1

extern bool g_manual_drive; /**< Flag wheter the manual driving mode is active. */
extern int g_event_queue[]; /**< Event queue from the state machine to be filled by incoming commands. */

/**
 * @brief Check if new commands have been received and process them if so
 * 
 */
void check_incoming_cmds();
