#include <lsm6dsox.h>
#include <debug.h>

static const gpio_t gpios[] = {
    {
        .num = D18_GPIO12_A4_SDA,
        .func = GPIO_FUNC_I2C,
    },
    {
        .num = D19_GPIO13_A5_SCL,
        .func = GPIO_FUNC_I2C,
    },
};

static inline void init_gpios()
{
    uint8_t i, ngpios;

    ngpios = sizeof(gpios) / sizeof(*gpios);
    pr_debug("%s:%u ngpios %u\n", __func__, __LINE__, ngpios);
    for (i = 0; i < ngpios; i++) {
        gpio_init(gpios[i].num);
        gpio_set_function(gpios[i].num, gpios[i].func);
        gpio_pull_up(gpios[i].num);
        pr_debug("%s:%u Initialized GPIO%u with function %u\n", gpios[i].num, gpios[i].func);
    }
}

static inline int lsm6dsox_accelerometer_probe()
{
    return i2c_write_byte(LSM6DSOX_I2C_BUS_ADDR, CTRL1_XL, 0b1010000);
}

static int lsm6dsox_accelerometer_get_x()
{
    uint8_t reg, value;
    uint16_t x;

    reg = OUTX_L_A;
    i2c_read_byte(LSM6DSOX_I2C_BUS_ADDR, reg, &value);
    pr_debug("%s:%u OUTX_L_A %hhu\n", __func__, __LINE__, value);
    x = (uint16_t) value << 8;

    reg = OUTX_H_A;
    i2c_read_byte(LSM6DSOX_I2C_BUS_ADDR, reg, &value);
    pr_debug("%s:%u OUTX_H_A %hhu\n", __func__, __LINE__, value);
    x += value;

    return x;
}

static int lsm6dsox_accelerometer_get_y()
{
    uint8_t reg, value;
    uint16_t y;

    reg = OUTY_L_A;
    i2c_read_byte(LSM6DSOX_I2C_BUS_ADDR, reg, &value);
    pr_debug("%s:%u OUTY_L_A %hhu\n", __func__, __LINE__, value);
    y = (uint16_t) value << 8;

    reg = OUTY_H_A;
    i2c_read_byte(LSM6DSOX_I2C_BUS_ADDR, reg, &value);
    pr_debug("%s:%u OUTY_H_A %hhu\n", __func__, __LINE__, value);
    y += value;

    return y;
}

static int lsm6dsox_accelerometer_get_z()
{
    uint8_t reg, value;
    uint8_t z;

    reg = OUTZ_L_A;
    i2c_read_byte(LSM6DSOX_I2C_BUS_ADDR, reg, &value);
    pr_debug("%s:%u OUTZ_L_A %hhu\n", __func__, __LINE__, value);
    z = (uint16_t) value << 8;

    reg = OUTZ_H_A;
    i2c_read_byte(LSM6DSOX_I2C_BUS_ADDR, reg, &value);
    pr_debug("%s:%u OUTZ_H_A %hhu\n", __func__, __LINE__, value);
    z += value;

    return z;
}

static const position_ops_t lsm6dsox_accelerometer_position_ops = {
    .get_x = lsm6dsox_accelerometer_get_x,
    .get_y = lsm6dsox_accelerometer_get_y,
    .get_z = lsm6dsox_accelerometer_get_z,
};

static inline int lsm6dsox_gyroscope_probe()
{
    return i2c_write_byte(LSM6DSOX_I2C_BUS_ADDR, CTRL2_G, 0b1010000);
}

static int lsm6dsox_gyroscope_get_x()
{
    uint8_t reg, value;
    uint16_t x;

    reg = OUTX_H_G;
    i2c_read_byte(LSM6DSOX_I2C_BUS_ADDR, reg, &value);
    pr_debug("%s:%u OUTX_H_A %hhu\n", __func__, __LINE__, value);
    x = (uint16_t) value << 8;

    reg = OUTX_L_G;
    i2c_read_byte(LSM6DSOX_I2C_BUS_ADDR, reg, &value);
    pr_debug("%s:%u OUTX_L_A %hhu\n", __func__, __LINE__, value);
    x += value;

    return x;
}

static int lsm6dsox_gyroscope_get_y()
{
    uint8_t reg, value;
    uint16_t y;

    reg = OUTY_H_G;
    i2c_read_byte(LSM6DSOX_I2C_BUS_ADDR, reg, &value);
    pr_debug("%s:%u OUTY_H_A %hhu\n", __func__, __LINE__, value);
    y = (u_int16_t) value << 8;

    reg = OUTY_L_G;
    i2c_read_byte(LSM6DSOX_I2C_BUS_ADDR, reg, &value);
    pr_debug("%s:%u OUTY_L_A %hhu\n", __func__, __LINE__, value);
    y+= value;

    return y;
}

static int lsm6dsox_gyroscope_get_z()
{
    uint8_t reg, value;
    uint16_t z;

    reg = OUTZ_H_G;
    i2c_read_byte(LSM6DSOX_I2C_BUS_ADDR, reg, &value);
    pr_debug("%s:%u OUTZ_H_A %hhu\n", __func__, __LINE__, value);
    z = (u_int16_t) value << 8;

    reg = OUTZ_L_G;
    i2c_read_byte(LSM6DSOX_I2C_BUS_ADDR, reg, &value);
    pr_debug("%s:%u OUTZ_L_A %hhu\n", __func__, __LINE__, value);
    z+= value;

    return z;
}

static const position_ops_t lsm6dsox_gyroscope_position_ops = {
    .get_x = lsm6dsox_gyroscope_get_x,
    .get_y = lsm6dsox_gyroscope_get_y,
    .get_z = lsm6dsox_gyroscope_get_z,
};

static int lsm6dsox_probe()
{
    int ret;

    init_gpios();

    ret = lsm6dsox_accelerometer_probe();
    if (ret < 0) {
        pr_debug("%s:%u Accelerometer returned %d\n", __func__, __LINE__, ret);
        return ret;
    }

    if (lsm6dsox_gyroscope_probe() < 0) {
        pr_debug("%s:%u Gyroscope returned %d\n", __func__, __LINE__, ret);
        return ret;
    }

    return 0;
}

static const i2c_slave_ops_t lsm6dsox_i2c_ops = {
    .probe = lsm6dsox_probe,
    .write_byte = i2c_write_byte,
    .read_byte = i2c_read_byte,
};

lsm6dsox_t lsm6dsox = {
    .acc = &lsm6dsox_accelerometer_position_ops,
    .gyro = &lsm6dsox_gyroscope_position_ops,
    .ops = &lsm6dsox_i2c_ops,
    .gpios = gpios,
};
