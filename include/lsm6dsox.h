#include <i2c.h>
#include <position.h>
#include <gpio.h>
#include "hardware/i2c.h"

#ifndef LSM6DSOX_H

#define LSM6DSOX_I2C_BUS_ADDR 0x6a
#define CTRL1_XL 0x10
#define CTRL2_G 0x11
#define OUTX_L_G 0x22
#define OUTX_H_G 0x23
#define OUTY_L_G 0x24
#define OUTY_H_G 0x25
#define OUTZ_L_G 0x26
#define OUTZ_H_G 0x27
#define OUTX_L_A 0x28
#define OUTX_H_A 0x29
#define OUTY_L_A 0x2a
#define OUTY_H_A 0x2b
#define OUTZ_L_A 0x2c
#define OUTZ_H_A 0x2d

typedef struct {
    const position_ops_t * const acc;
    const position_ops_t * const gyro;
    const i2c_slave_ops_t * const ops;
    const gpio_t * const gpios;
} lsm6dsox_t;

extern lsm6dsox_t lsm6dsox;

#endif  /* LSM6DSOX_H */
