#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "motor.h"
#include "communication.h"
#include "as5600.h"

// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT i2c1
#define I2C_SDA 26
#define I2C_SCL 27


// Pin defines
#define MOTOR1_STEP_PIN 20
#define MOTOR2_STEP_PIN 22
#define MOTOR3_STEP_PIN 13
#define MOTOR4_STEP_PIN 11
#define MOTOR1_DIR_PIN 19
#define MOTOR2_DIR_PIN 21
#define MOTOR3_DIR_PIN 12
#define MOTOR4_DIR_PIN 10

#define MS1_PIN 18
#define MS2_PIN 17
#define MS3_PIN 16

void pio0_interrupt_handler() {
    pio_interrupt_clear(pio0, 0) ;
    puts("rotation1 done");
    //moveStepsMotor1(0xFFFFFFFF) ;
}

void pio0_interrupt_handler1() {
    pio_interrupt_clear(pio0, 1) ;
    puts("rotation2 done");
    //moveStepsMotor2(0xFFFFFFFF) ;
}

void pio1_interrupt_handler() {
    pio_interrupt_clear(pio1, 0) ;
    puts("rotation3 done");
    //moveStepsMotor3(0xFFFFFFFF) ;
}

void pio1_interrupt_handler1() {
    pio_interrupt_clear(pio1, 1) ;
    puts("rotation4 done");
    //moveStepsMotor4(0xFFFFFFFF) ;
}


int main()
{
    stdio_init_all();

    as560x_init(I2C_PORT);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    // Make the I2C pins available to picotool
    //bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    setup_default_uart();

    setupMotor1(MOTOR1_STEP_PIN, MOTOR1_DIR_PIN, pio0_interrupt_handler);
    setupMotor2(MOTOR2_STEP_PIN, MOTOR2_DIR_PIN, pio0_interrupt_handler1);
    setupMotor3(MOTOR3_STEP_PIN, MOTOR3_DIR_PIN, pio1_interrupt_handler);
    setupMotor4(MOTOR4_STEP_PIN, MOTOR4_DIR_PIN, pio1_interrupt_handler1);
    setupMicrostepping(MS1_PIN, MS2_PIN, MS3_PIN, 4);
    
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    puts("Start programm... (wait for 10 seconds)");
    sleep_ms(10000);

    puts("rotation1 start");
    setDirectionMotor1(1);
    setSpeedMotor1(300);
    moveStepsMotor1(100, true);




    /*puts("rotation2 start");
    setDirectionMotor2(1);
    setSpeedMotor2(100);
    moveStepsMotor2(10);

    puts("rotation3 start");
    setDirectionMotor3(1);
    setSpeedMotor3(1000);
    moveStepsMotor3(10);

    puts("rotation4 start");
    setDirectionMotor4(1);
    setSpeedMotor4(600);
    moveStepsMotor4(10);*/

    uint32_t last_print_time = time_us_64() / 1000;

    while (true) {
        /*puts("clockwise");
        setDirectionMotor1(1);
        setSpeedMotor1(1000);
        gpio_put(LED_PIN, 1);
        sleep_ms(5000);
        
        puts("counter clockwise");
        setDirectionMotor1(0);
        setSpeedMotor1(2000);
        gpio_put(LED_PIN, 0);
        sleep_ms(10000);*/
        check_incoming_cmds();

        // measure and print angle every 200ms
        uint32_t current_time = time_us_64() / 1000;
        if((current_time - last_print_time > 500))
        {
            uint8_t status = (uint8_t) as560xGetStatus();
            if (!(status & AS560x_STATUS_MAGNET_DETECTED)) {
                printf("ERROR\n\r");
            }

            int angle = as560xReadAngle() * 360 / 0xFFF;
            printf("Current angle: %d\n\r",angle);
            
            last_print_time = current_time;
            
        }

        sleep_ms(20);
    }

    return 0;
}