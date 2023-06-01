/**
 * @file gpio_functions.h
 * @author Edgar W. (mail@techniccontroller.com)
 * @brief HAL for gpio functions specific for Raspberry Pi Pico
 * @version 0.1
 * @date 2022-07-07
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#define GPIO_FUNC_INPUT 0
#define GPIO_FUNC_INPUT_PULLUP 1
#define GPIO_FUNC_INPUT_PULLDOWN 2
#define GPIO_FUNC_OUTPUT 3

#define GPIO_FUNC_IRQ_LEVEL_LOW GPIO_IRQ_LEVEL_LOW
#define GPIO_FUNC_IRQ_LEVEL_HIGH GPIO_IRQ_LEVEL_HIGH
#define GPIO_FUNC_IRQ_EDGE_FALL GPIO_IRQ_EDGE_FALL
#define GPIO_FUNC_IRQ_EDGE_RISE GPIO_IRQ_EDGE_RISE

typedef void (*gpio_func_irq_callback_t)(uint gpio, uint32_t event_mask);

/**
 * @brief Set the pin mode of a given GPIO
 * 
 * @param gpio GPIO number
 * @param mode Pin mode of the GPIO (INPUT, INPUT_PULLUP, INPUT_PULLDOWN, OUTPUT)
 */
void gpio_func_set_mode(uint8_t gpio, uint8_t mode);

/**
 * @brief Get state of a given GPIO
 * 
 * @param gpio GPIO number
 * @return Current state of GPIO. 0 for low, non-zero for high 
 */
bool gpio_func_get_state(uint8_t gpio);

/**
 * @brief Set state of a given GPIO
 * 
 * @param gpio GPIO number
 * @param state If false clear the GPIO, otherwise set it. 
 */
void gpio_func_set_state(uint8_t gpio, bool state);

/**
 * @brief Add a pin interrupt to a given GPIO
 * 
 * @param gpio GPIO number
 * @param events Which events will cause an interrupt. (LEVEL_LOW, LEVEL_HIGH, EDGE_FALL, EDGE_RISE)
 * @param enabled Enable or disable flag
 * @param callback user function to call on GPIO irq. if NULL, the callback is removed
 */
void gpio_func_add_irq(uint gpio, uint32_t events, bool enabled, gpio_func_irq_callback_t callback);
