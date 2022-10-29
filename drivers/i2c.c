#include <i2c.h>
#include <debug.h>

int i2c_write_byte(uint8_t saddr, uint8_t reg, uint8_t value)
{
    uint8_t msg[2];
    msg[0] = reg;
    msg[1] = value;

    pr_debug("%s:%u Writing to saddr %x reg %x value %x", saddr, reg, value);

    return i2c_write_blocking(i2c_default, saddr, msg, sizeof(msg), false);
}

int i2c_read_byte(uint8_t saddr, uint8_t reg, uint8_t *value)
{
    int ret;

    pr_debug("%s:%u Writing to saddr %x reg %x", saddr, reg);
    ret = i2c_write_blocking(i2c_default, saddr, &reg, sizeof(reg), false);
    if (ret <= 0)
        return -1;

    pr_debug("%s:%u Reading from saddr %x reg %x", saddr, reg);
    ret = i2c_read_blocking(i2c_default, saddr, value, sizeof(*value), false);
    if (ret <= 0)
        return -1;
}
