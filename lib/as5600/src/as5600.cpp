#include "hardware/i2c.h"
#include <stdio.h>
#include <machine/endian.h>

#include "as5600.h"

#define I2C_PORT i2c1
#define AS560x_ADDR 0x36
#define AS560x_STATUS_REG (0x0B)
#define AS560x_RAW_ANGLE_REG (0x0C)

static i2c_inst_t * g_i2c_port = NULL; 

uint16_t as560xReadReg(int addr, bool wide, uint16_t mask) {
    uint16_t buf;
    int result = i2c_write_timeout_us(I2C_PORT, AS560x_ADDR, (uint8_t *) &addr, 1, true, I2C_TIMEOUT_US);
    if (result <= 0) {
        // error
    }
    result = i2c_read_timeout_us(I2C_PORT, AS560x_ADDR, (uint8_t *) &buf, (wide ? 2 : 1), false, I2C_TIMEOUT_US);
    if (result <= 0) {
        // error
    }
    if (wide) {
        return __bswap16(buf) & mask;
    } else {
        return buf & mask;
    }
}

void AS560x_print_reg16(const char *formatStr, int addr, uint16_t mask) {
    uint16_t result = as560xReadReg(addr, true, mask);
    printf(formatStr, result & mask);
}

void AS560x_print_reg8(const char *formatStr, int addr, uint8_t mask) {
    uint8_t result = (uint8_t) as560xReadReg(addr, false, mask);
    printf(formatStr, result & mask);
}

void as560x_init(i2c_inst_t * i2c_port) {
    i2c_init(I2C_PORT, 100 * 1000);
}

int as560xReadAngle() {
    return as560xReadReg(AS560x_RAW_ANGLE_REG, true, 0xFFF);
}

uint8_t as560xGetStatus() {
    return (uint8_t) as560xReadReg(AS560x_STATUS_REG, false, 0x38);
}

__unused void sensorData() {
    AS560x_print_reg8("Status: %02x; ", 0xb, 0x38);
    AS560x_print_reg8("AGC: %3x; ", 0x1a, 0xff);
    AS560x_print_reg16("Angle: %04x\n\r", 0x0c, 0xFFF);
}
