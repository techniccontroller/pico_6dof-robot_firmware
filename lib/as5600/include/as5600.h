/**
 * @file as5600.h
 * @author Edgar W (mail@techniccontroller.com)
 * @brief Library for AS5600 Absolute Magnetic Encoder
 * @version 0.1
 * @date 2023-05-29
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef RPI_PICO_AS5600_H
#define RPI_PICO_AS5600_H

#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include <stdint.h>


#define I2C_TIMEOUT_US (100000)

#define AS5601_ANGLE_MAX (0xFFFL)
#define AS560x_STATUS_MAGNET_DETECTED (0x20)
#define AS560x_STATUS_MAGNET_HIGH (0x08)
#define AS560x_STATUS_MAGNET_LOW (0x10)
#define AS560x_STATUS_REG (0x0B)
#define AS560x_RAW_ANGLE_REG (0x0C)


class AS5600 {


public:
    AS5600(i2c_inst_t * i2c_port, uint8_t scl_pin, uint8_t sda_pin, uint8_t addr, uint8_t mux_channel, uint8_t mux_addr);
    int readAngle();
    int getCorrectedAngle();
    uint8_t getStatus();
    void sensorData();
    void setZero();
    void setZero(int zero);
    int getZero();

private:

    void muxselect(uint8_t i);
    uint16_t readReg(int addr, bool wide, uint16_t mask);
    void printReg16(const char *formatStr, int addr, uint16_t mask);
    void printReg8(const char *formatStr, int addr, uint8_t mask);

    uint8_t g_addr;
    i2c_inst_t * g_i2c_port;
    int g_zero = 0;
    uint8_t g_mux_channel;
    uint8_t g_mux_addr;
};


#endif //RPI_PICO_AS5600_H
