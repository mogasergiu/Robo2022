#include "pico/stdlib.h"
#include "hardware/i2c.h"

#ifndef I2C_H
#define I2C_H

typedef struct {
    int (*probe)();
    int (*read_byte)(uint8_t saddr, uint8_t reg, uint8_t *value);
    int (*write_byte)(uint8_t saddr, uint8_t reg, uint8_t value);
    int (*read_bytes)(uint8_t saddr, uint8_t reg, uint8_t *buf, size_t len);
    int (*write_bytes)(uint8_t saddr, uint8_t reg, uint8_t *buf, size_t len);
} i2c_slave_ops_t;

int i2c_read_byte(uint8_t saddr, uint8_t reg, uint8_t *value);

int i2c_write_byte(uint8_t saddr, uint8_t reg, uint8_t value);

int i2c_read_bytes(uint8_t saddr, uint8_t reg, uint8_t *buf, size_t len);

int i2c_write_bytes(uint8_t saddr, uint8_t reg, uint8_t *buf, size_t len);

#endif  /* I2C_H */
