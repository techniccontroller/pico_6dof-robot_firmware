#include "pwm_functions.h"

#define PWM_WRAP 255 // TOP of PWM timer -> 0...255

void pwm_func_init(uint8_t gpio)
{
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_set_wrap(slice_num, PWM_WRAP);
    pwm_set_enabled(slice_num, true);
}

void pwm_func_deinit(uint8_t gpio)
{
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_set_enabled(slice_num, false);
}

void pwm_func_set_level(uint gpio, uint16_t level)
{
    if(level > PWM_WRAP)
    {
        level = PWM_WRAP;
    }
    pwm_set_gpio_level(gpio, level);
}