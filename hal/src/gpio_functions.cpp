#include "gpio_functions.h"


void gpio_func_set_mode(uint8_t gpio, uint8_t mode)
{
    gpio_init(gpio);
    gpio_disable_pulls(gpio);

    switch (mode)
    {
    case GPIO_FUNC_INPUT:
        gpio_set_dir(gpio, GPIO_IN);
        break;
    
    case GPIO_FUNC_INPUT_PULLDOWN:
        gpio_set_dir(gpio, GPIO_IN);
        gpio_pull_down(gpio);
        break;
    
    case GPIO_FUNC_INPUT_PULLUP:
        gpio_set_dir(gpio, GPIO_IN);
        gpio_pull_up(gpio);
        break;
    
    case GPIO_FUNC_OUTPUT:
        gpio_set_dir(gpio, GPIO_OUT);
        break;
    
    default:
        break;
    }
}

bool gpio_func_get_state(uint8_t gpio)
{
    return gpio_get(gpio);
}

void gpio_func_set_state(uint8_t gpio, bool state)
{
    gpio_put(gpio, state);
}

void gpio_func_add_irq(uint gpio, uint32_t events, bool enabled, gpio_func_irq_callback_t callback)
{
    gpio_set_irq_enabled_with_callback(gpio, events, enabled, callback);
}