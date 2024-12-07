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

volatile long g_encoder_pos_motor4 = 0; /**< Current encoder position of motor4 */
volatile long g_encoder_pos_motor5 = 0; /**< Current encoder position of motor5 */
volatile int g_direction_motor4 = 0; /**< Current direction of motor4 */
volatile int g_direction_motor5 = 0; /**< Current direction of motor5 */

void callback_encoder(uint gpio, uint32_t events)
{
    if(gpio == MOTOR4_ENC_A_PIN || gpio == MOTOR4_ENC_B_PIN){
        if(g_direction_motor4 == 1)
        {
            g_encoder_pos_motor4++;
        }
        else if(g_direction_motor4 == -1)
        {
            g_encoder_pos_motor4--;
        }
    }

    if(gpio == MOTOR5_ENC_A_PIN || gpio == MOTOR5_ENC_B_PIN){
        if(g_direction_motor5 == 1)
        {
            g_encoder_pos_motor5++;
        }
        else if(g_direction_motor5 == -1)
        {
            g_encoder_pos_motor5--;
        }
    }
}

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
    
    gpio_func_set_mode(MOTOR4_ENC_A_PIN, GPIO_FUNC_INPUT);
    gpio_func_add_irq(MOTOR4_ENC_A_PIN, GPIO_FUNC_IRQ_EDGE_FALL, true, &callback_encoder);
    gpio_func_set_mode(MOTOR4_ENC_B_PIN, GPIO_FUNC_INPUT);
    //gpio_func_add_irq(MOTOR4_ENC_B_PIN, GPIO_FUNC_IRQ_EDGE_RISE | GPIO_FUNC_IRQ_EDGE_FALL, true, &callback_encoder);
    gpio_func_set_mode(MOTOR5_ENC_A_PIN, GPIO_FUNC_INPUT);
    gpio_func_add_irq(MOTOR5_ENC_A_PIN, GPIO_FUNC_IRQ_EDGE_FALL, true, &callback_encoder);
    gpio_func_set_mode(MOTOR5_ENC_B_PIN, GPIO_FUNC_INPUT);
    //gpio_func_add_irq(MOTOR5_ENC_B_PIN, GPIO_FUNC_IRQ_EDGE_RISE | GPIO_FUNC_IRQ_EDGE_FALL, true, &callback_encoder);

    Communication comm(&robot); 

    puts("Start programm...");

    printf("System Clock Frequency is %d Hz\n", clock_get_hz(clk_sys));
    printf("USB Clock Frequency is %d Hz\n", clock_get_hz(clk_usb));
    
    uint32_t last_print_time = time_us_64() / 1000;
    uint32_t last_step_time = time_us_64() / 1000;

    while (true) {
        
        comm.check_incoming_cmds();

        // measure and print config every 500ms
        uint32_t current_time = time_us_64() / 1000;
        if((current_time - last_print_time > 2000))
        {
            std::string robotDataJson = robot.getRobotDataAsJson();
            std::string topLevelJson = "{ \"robot_data\": " + robotDataJson + " }";
            puts(topLevelJson.c_str());
            last_print_time = current_time;
        }

        if((current_time - last_step_time > 10))
        {
            robot.step();
            last_step_time = current_time;
        }

        robot.run();
    }

    return 0;
}
