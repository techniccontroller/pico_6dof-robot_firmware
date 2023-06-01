/**
 * @file comm_functions.h
 * @author Edgar W. (mail@techniccontroller.com)
 * @brief HAL for communication functions specific for Raspberry Pi Pico
 * @version 0.1
 * @date 2022-07-07
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include "stdbool.h"
#include "stdint.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"

/**
 * @brief Initialize the communication. 
 * 
 * Call it once at setup time.
 * 
 */
void comm_func_init();

/**
 * @brief Read all bytes which are currently in the input buffer up to a given len.
 * 
 * @param dst Buffer into which the bytes are to be written.
 * @param len What is the maximum number of bytes to be read.
 * @return uint16_t Returns the number of read bytes. If 0 then no bytes have been received.
 */
uint16_t comm_func_read_all_bytes(uint8_t *dst, uint8_t len);

/**
 * @brief Write a string to the output buffer.
 * 
 * Use sprintf() to prepare the string, if you want to include variables in the output.
 * 
 * @param s The string to be sent
 */
void comm_func_write(const char *s);
