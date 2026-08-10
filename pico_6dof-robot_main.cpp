#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"

#include "gpio_functions.h"
#include "defines_constants.h"
#include "Robot.h"
#include "Communication.h"

#define PICO_W 1
#ifdef PICO_W
#include "pico/cyw43_arch.h"
#endif

#ifdef PICO_DEFAULT_LED_PIN
#define LED_PIN PICO_DEFAULT_LED_PIN
#else
#define LED_PIN 25
#endif

constexpr uint32_t TELEMETRY_RATE_HZ = 30;
constexpr uint64_t TELEMETRY_INTERVAL_US = 1000000 / TELEMETRY_RATE_HZ;


void setLEDState(bool state)
{
    if(PICO_W){
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, state);
    }
    else{
        gpio_put(LED_PIN, state);
    }
}


int main()
{
    stdio_init_all();

    if(PICO_W){
        if (cyw43_arch_init()) {
            printf("Wi-Fi init failed");
            return -1;
        }
    }
    else {
        gpio_init(LED_PIN);
        gpio_set_dir(LED_PIN, GPIO_OUT);
    }

    Robot robot;

    setup_default_uart();

    Communication comm(&robot); 

    puts("Start programm...");

    printf("System Clock Frequency is %d Hz\n", clock_get_hz(clk_sys));
    printf("USB Clock Frequency is %d Hz\n", clock_get_hz(clk_usb));
    
    uint64_t last_telemetry_time_us = time_us_64();
    uint32_t last_step_time = time_us_64() / 1000;

    while (true) {
        
        comm.check_incoming_cmds();

        uint64_t current_time_us = time_us_64();
        if((current_time_us - last_telemetry_time_us) >= TELEMETRY_INTERVAL_US)
        {
            std::string robotDataJson = robot.getRobotDataAsJson();
            puts(robotDataJson.c_str());
            last_telemetry_time_us = current_time_us;
        }

        uint32_t current_time = current_time_us / 1000;
        if((current_time - last_step_time > 10))
        {
            robot.step();
            last_step_time = current_time;
        }

        robot.run();
    }

    return 0;
}
