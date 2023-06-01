/**
 * @file pwm_functions.h
 * @author Edgar W. (mail@techniccontroller.com)
 * @brief HAL for pwm functions specific for Raspberry Pi Pico
 * @version 0.1
 * @date 2022-07-07
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include "pico/stdlib.h"
#include "hardware/pwm.h"

/**
 * @brief Initialize the PWM functionality on the given pin.
 * 
 * When using multiple PWM pins in parallel, 
 * make sure the pin numbers are not 16 units apart, 
 * as those pin pairs using the same PWM timer and channel.
 * 
 * @param gpio GPIO pin that should be used for PWM output
 */
void pwm_func_init(uint8_t gpio);

/**
 * @brief Deinitialize the PWM functionality from given pin, so that it can be used for other functions
 * 
 * @param gpio GPIO pin that is used for PWM output
 */
void pwm_func_deinit(uint8_t gpio);

/**
 * @brief Set the PWM output level of the given pin
 * 
 * @param gpio GPIO to set level of
 * @param level PWM Level for this GPIO (0-255)
 */
void pwm_func_set_level(uint gpio, uint16_t level);