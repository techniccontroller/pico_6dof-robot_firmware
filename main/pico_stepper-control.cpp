#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include <AccelStepper.h>
#include <MultiStepper.h>
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

void setMicrostepping(unsigned int res){
    switch (res) {
        case 1:
            gpio_put(MS1_PIN, 0); gpio_put(MS2_PIN, 0); gpio_put(MS3_PIN, 0);
            break;
        case 2:
            gpio_put(MS1_PIN, 1); gpio_put(MS2_PIN, 0); gpio_put(MS3_PIN, 0);
            break;
        case 4:
            gpio_put(MS1_PIN, 0); gpio_put(MS2_PIN, 1); gpio_put(MS3_PIN, 0);
            break;
        case 8:
            gpio_put(MS1_PIN, 1); gpio_put(MS2_PIN, 1); gpio_put(MS3_PIN, 0);
            break;
        case 16:
            gpio_put(MS1_PIN, 1); gpio_put(MS2_PIN, 1); gpio_put(MS3_PIN, 1);
            break;

        default:
            // no valid microstep resolution
            break;
    }
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

    gpio_init(MS1_PIN);
    gpio_init(MS2_PIN);
    gpio_init(MS3_PIN);
    gpio_set_dir(MS1_PIN, GPIO_OUT);
    gpio_set_dir(MS2_PIN, GPIO_OUT);
    gpio_set_dir(MS3_PIN, GPIO_OUT);
    setMicrostepping(4);


    AccelStepper stepper1(AccelStepper::DRIVER, MOTOR1_STEP_PIN, MOTOR1_DIR_PIN);
    AccelStepper stepper2(AccelStepper::DRIVER, MOTOR2_STEP_PIN, MOTOR2_DIR_PIN);

    // Up to 10 steppers can be handled as a group by MultiStepper
    MultiStepper steppers;

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

    
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

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

        /*// measure and print angle every 200ms
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
            
        }*/

        //sleep_ms(20);
    }

    return 0;
}