#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <StepperConfiguration.h>
#include "communication.h"
#include "as5600.h"


#define PICO_W 1
#ifdef PICO_W
#include "pico/cyw43_arch.h"
#endif

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
#define ENABLE_PIN 15

#ifdef PICO_DEFAULT_LED_PIN
#define LED_PIN PICO_DEFAULT_LED_PIN
#else
#define LED_PIN 25
#endif

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

    as560x_init(I2C_PORT);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    // Make the I2C pins available to picotool
    //bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    setup_default_uart();

    AccelStepper stepper1(AccelStepper::DRIVER, MOTOR1_STEP_PIN, MOTOR1_DIR_PIN);
    AccelStepper stepper2(AccelStepper::DRIVER, MOTOR2_STEP_PIN, MOTOR2_DIR_PIN);
    MultiStepper steppers;
    StepperConfiguration stepper_config(MS1_PIN, MS2_PIN, MS3_PIN, ENABLE_PIN, 4);

    // Configure each stepper
    stepper1.setMaxSpeed(2000.0);
    stepper1.setAcceleration(500.0);
    stepper1.moveTo(1000);
    
    stepper2.setMaxSpeed(2000.0);
    stepper2.setAcceleration(500.0);
    stepper2.moveTo(1000);

    // Then give them to MultiStepper to manage
    steppers.addStepper(stepper1);
    steppers.addStepper(stepper2);    

    puts("Start programm... (wait for 10 seconds)");
    //sleep_ms(10000);

    uint32_t last_print_time = time_us_64() / 1000;

    while (true) {
        /*long positions[2]; // Array of desired stepper positions
  
        positions[0] = 1000;
        positions[1] = 50;
        steppers.moveTo(positions);
        steppers.runSpeedToPosition(); // Blocks until all are in position
        sleep_ms(1000);
        
        // Move to a different coordinate
        positions[0] = -100;
        positions[1] = 100;
        steppers.moveTo(positions);
        steppers.runSpeedToPosition(); // Blocks until all are in position
        sleep_ms(1000);*/

        // Change direction at the limits
        if (stepper1.distanceToGo() == 0)
            stepper1.moveTo(-stepper1.currentPosition());
        if (stepper2.distanceToGo() == 0)
            stepper2.moveTo(-stepper2.currentPosition());
        stepper1.run();
        stepper2.run();
        

        //check_incoming_cmds();

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

        //sleep_ms(20);
    }

    return 0;
}