#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include <AccelStepper.h>
#include <MultiStepper.h>
#include "communication.h"
#include <StepperConfiguration.h>
#include "controller.h"
#include "as5600.h"
#include "DCMotor.h"
#include "gpio_functions.h"


#define PICO_W 1
#ifdef PICO_W
#include "pico/cyw43_arch.h"
#endif

// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT0 i2c0
#define I2C_PORT1 i2c1
#define I2C_SDA0 14
#define I2C_SCL0 15
#define I2C_SDA1 12
#define I2C_SCL1 13


// Pin defines
#define MOTOR1_STEP_PIN 20
#define MOTOR2_STEP_PIN 22
#define MOTOR3_STEP_PIN 27
#define MOTOR1_DIR_PIN 19
#define MOTOR2_DIR_PIN 21
#define MOTOR3_DIR_PIN 26

#define MOTOR4_ENABLE_PIN 6
#define MOTOR4_IN1_PIN 7
#define MOTOR4_IN2_PIN 8
#define MOTOR4_ENC_A_PIN 2
#define MOTOR4_ENC_B_PIN 3
#define MOTOR5_ENABLE_PIN 11
#define MOTOR5_IN1_PIN 10
#define MOTOR5_IN2_PIN 9
#define MOTOR5_ENC_A_PIN 4
#define MOTOR5_ENC_B_PIN 5

#define MS1_PIN 18
#define MS2_PIN 17
#define MS3_PIN 16
#define ENABLE_PIN 28



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

    AS5600 encoder2(I2C_PORT0, I2C_SCL0, I2C_SDA0, 0x36);
    AS5600 encoder1(I2C_PORT1, I2C_SCL1, I2C_SDA1, 0x36);
    // Make the I2C pins available to picotool
    //bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA1_PIN, PICO_DEFAULT_I2C_SCL1_PIN, GPIO_FUNC_I2C));

    setup_default_uart();

    AccelStepper stepper1(AccelStepper::DRIVER, MOTOR1_STEP_PIN, MOTOR1_DIR_PIN);
    AccelStepper stepper2(AccelStepper::DRIVER, MOTOR2_STEP_PIN, MOTOR2_DIR_PIN);
    AccelStepper stepper3(AccelStepper::DRIVER, MOTOR3_STEP_PIN, MOTOR3_DIR_PIN);
    MultiStepper steppers;
    StepperConfiguration stepper_config(MS1_PIN, MS2_PIN, MS3_PIN, ENABLE_PIN, 4, 60.0/16.0 * 60.0/16.0);

    gpio_func_set_mode(MOTOR4_ENC_A_PIN, GPIO_FUNC_INPUT);
    gpio_func_add_irq(MOTOR4_ENC_A_PIN, GPIO_FUNC_IRQ_EDGE_FALL, true, &callback_encoder);
    gpio_func_set_mode(MOTOR4_ENC_B_PIN, GPIO_FUNC_INPUT);
    //gpio_func_add_irq(MOTOR4_ENC_B_PIN, GPIO_FUNC_IRQ_EDGE_RISE | GPIO_FUNC_IRQ_EDGE_FALL, true, &callback_encoder);
    gpio_func_set_mode(MOTOR5_ENC_A_PIN, GPIO_FUNC_INPUT);
    gpio_func_add_irq(MOTOR5_ENC_A_PIN, GPIO_FUNC_IRQ_EDGE_FALL, true, &callback_encoder);
    gpio_func_set_mode(MOTOR5_ENC_B_PIN, GPIO_FUNC_INPUT);
    //gpio_func_add_irq(MOTOR5_ENC_B_PIN, GPIO_FUNC_IRQ_EDGE_RISE | GPIO_FUNC_IRQ_EDGE_FALL, true, &callback_encoder);
    
    DCMotor motor4(MOTOR4_ENABLE_PIN, MOTOR4_IN1_PIN, MOTOR4_IN2_PIN, &g_encoder_pos_motor4, &g_direction_motor4);
    DCMotor motor5(MOTOR5_ENABLE_PIN, MOTOR5_IN1_PIN, MOTOR5_IN2_PIN, &g_encoder_pos_motor5, &g_direction_motor5);
    
    Controller controller;
    controller.addM1(&stepper1, &encoder1);
    controller.addM2(&stepper2, &encoder2);
    controller.addM3(&stepper3, NULL);
    controller.addM4(&motor4);
    controller.addM5(&motor5);
    Communication comm(&controller, &stepper_config);

    // Configure each stepper
    stepper1.setMaxSpeed(2000.0);
    stepper1.setAcceleration(500.0);
    //stepper1.moveTo(1000);
    
    stepper2.setMaxSpeed(2000.0);
    stepper2.setAcceleration(500.0);
    //stepper2.moveTo(1000);

    stepper3.setMaxSpeed(2000.0);
    stepper3.setAcceleration(500.0);
    //stepper3.moveTo(1000);

    // Then give them to MultiStepper to manage
    steppers.addStepper(stepper1);
    steppers.addStepper(stepper2); 
    steppers.addStepper(stepper3);   

    puts("Start programm... (wait for 10 seconds)");
    //sleep_ms(10000);

    uint32_t last_print_time = time_us_64() / 1000;
    uint32_t last_step_time = time_us_64() / 1000;

    while (true) {
        /*long positions[3]; // Array of desired stepper positions
  
        positions[0] = 1000;
        positions[1] = 50;
        positions[2] = 100;
        steppers.moveTo(positions);
        steppers.runSpeedToPosition(); // Blocks until all are in position
        sleep_ms(1000);
        
        // Move to a different coordinate
        positions[0] = -100;
        positions[1] = 100;
        positions[2] = 50;
        steppers.moveTo(positions);
        steppers.runSpeedToPosition(); // Blocks until all are in position
        sleep_ms(1000);*/
        

        comm.check_incoming_cmds();

        // measure and print angle every 200ms
        uint32_t current_time = time_us_64() / 1000;
        if((current_time - last_print_time > 500))
        {
            uint8_t status1 = (uint8_t) encoder1.getStatus();
            if (!(status1 & AS560x_STATUS_MAGNET_DETECTED)) {
                printf("ERROR with angle 1\n\r");
            } 

            uint8_t status2 = (uint8_t) encoder2.getStatus();
            if (!(status2 & AS560x_STATUS_MAGNET_DETECTED)) {
                printf("ERROR with angle 2\n\r");
            } 
            
            float angleEnc1 = (encoder1.getCorrectedAngle() * 360.0) / 0xFFF;
            float angleEnc2 = (encoder2.getCorrectedAngle() * 360.0) / 0xFFF;
            float angleEnc3 = 0.0;
            float angleEnc4 = 0.0;
            float angleEnc5 = 0.0;
            float angleMotor1 = stepper_config.stepsToAngleDeg(stepper1.currentPosition());
            float angleMotor2 = stepper_config.stepsToAngleDeg(stepper2.currentPosition());
            float angleMotor3 = stepper_config.stepsToAngleDeg(stepper3.currentPosition());
            float angleMotor4 = g_encoder_pos_motor4;
            float angleMotor5 = g_encoder_pos_motor5;
            printf("Current angle(motor ): [%6.1f] [%6.1f] [%6.1f] [%6.1f] [%6.1f]\n\r", angleMotor1, angleMotor2, angleMotor3, angleMotor4, angleMotor5);
            printf("Current angle(sensor): [%6.1f] [%6.1f] [%6.1f] [%6.1f] [%6.1f]\n\r\n\r", angleEnc1, angleEnc2, angleEnc3, angleEnc4, angleEnc5);
            last_print_time = current_time;
            
        }

        if((current_time - last_step_time > 50))
        {
            controller.step();
            last_step_time = current_time;
        }

        controller.run();

        //sleep_ms(20);
    }

    return 0;
}