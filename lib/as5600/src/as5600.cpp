#include "AS5600.h"
#include <stdlib.h>
#include <stdio.h>
#include <machine/endian.h>


AS5600::AS5600(i2c_inst_t * i2c_port, uint8_t scl_pin, uint8_t sda_pin, uint8_t addr, uint8_t mux_channel, uint8_t mux_addr) {
    g_mux_channel = mux_channel;
    g_mux_addr = mux_addr;
    g_i2c_port = i2c_port;
    g_addr = addr;
    i2c_init(g_i2c_port, 100 * 1000);
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
}

void AS5600::setZero() {
    g_zero = readAngle();
    printf("Zero set to %d\n\r", g_zero);
}

void AS5600::setZero(int zero) {
    g_zero = zero;
}

int AS5600::getZero() {
    return g_zero;
}

int AS5600::readAngle() {
    return readReg(AS560x_RAW_ANGLE_REG, true, 0xFFF);
}

int AS5600::getCorrectedAngle() {
    int angle = readAngle();
    int corrected = angle - g_zero;
    if (corrected < 0) {
        corrected += AS5601_ANGLE_MAX;
    }
    return corrected;
}

float AS5600::getCorrectedAngleDeg() {
    return (float) getCorrectedAngle() / AS5601_ANGLE_MAX * 360.0;
}

uint8_t AS5600::getStatus() {
    return (uint8_t) readReg(AS560x_STATUS_REG, false, 0x38);
}

/**
 * @brief Select channel on TCA9548A I2C multiplexer
 * 
 * @param i Channel to select (0-7)
 */
void AS5600::muxselect(uint8_t i) {
  if (i > 7) return;
  int value = 1 << i;
  int ret = i2c_write_blocking(g_i2c_port, g_mux_addr, (uint8_t *) &value, 1, false);
}

uint16_t AS5600::readReg(int addr, bool wide, uint16_t mask) {

    muxselect(g_mux_channel);

    uint16_t buf;
    int result = i2c_write_timeout_us(g_i2c_port, g_addr, (uint8_t *) &addr, 1, true, I2C_TIMEOUT_US);
    if (result <= 0) {
        // error
    }
    result = i2c_read_timeout_us(g_i2c_port, g_addr, (uint8_t *) &buf, (wide ? 2 : 1), false, I2C_TIMEOUT_US);
    if (result <= 0) {
        // error
    }
    if (wide) {
        return __bswap16(buf) & mask;
    } else {
        return buf & mask;
    }
}

void AS5600::printReg16(const char *formatStr, int addr, uint16_t mask) {
    uint16_t result = readReg(addr, true, mask);
    printf(formatStr, result & mask);
}

void AS5600::printReg8(const char *formatStr, int addr, uint8_t mask) {
    uint8_t result = (uint8_t) readReg(addr, false, mask);
    printf(formatStr, result & mask);
}

void AS5600::sensorData() {
    printReg8("Status: %02x; ", 0xb, 0x38);
    printReg8("AGC: %3x; ", 0x1a, 0xff);
    printReg16("Angle: %04x\n\r", 0x0c, 0xFFF);
}
