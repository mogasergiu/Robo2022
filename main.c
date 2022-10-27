#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include <stdio.h>

#define DEFAULUT_PWM_WRAP 255

#define TX_GPIO0 0
#define RX_GPIO1 1
#define D12_GPIO4_CIPO 4  // laser interrupt
#define D10_GPIO5 5  // laser interrupt
#define D13_GPIO6_SCK 6  // start-stop interrupt
#define D11_GPIO7_COPI 7  // laser interrupt
#define D18_GPIO12_A4_SDA 12
#define D19_GPIO13_A5_SCL 13
#define D3_GPIO15 15  // laser interrupt
#define D4_GPIO16 16  // pwm
#define D5_GPIO17 17  // high/low
#define D6_GPIO18 18  // high/low
#define D7_GPIO19 19  // pwm
#define D8_GPIO20 20  // high/low
#define D9_GPIO21 21  // high/low
#define D2_GPIO25 25  // laser interrupt
#define D14_GPIO26_A0 26  // m1 encoder out_a
#define D15_GPIO27_A1 27  // m1 encoder out_b
#define D16_GPIO28_A2 28  // m2 encoder out_a
#define D17_GPIO29_A3 29  // m2 encoder out_b

#define LSM6DSOX_I2C_BUS_ADDR 0x6a
#define LSM6DSOX_WRITE (LSM6DSOX_I2C_BUS_ADDR << 1) + 0
#define LSM6DSOX_READ (LSM6DSOX_I2C_BUS_ADDR << 1) + 1
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

#define DEBUG 1

typedef struct {
    uint num;
    enum gpio_irq_level irq_type;
    enum gpio_function func;
    bool is_irq;
    bool is_out;
} gpio_t;

gpio_t gpios[] = {
    {
        .num = D18_GPIO12_A4_SDA,
        .func = GPIO_FUNC_I2C,
    },
    {
        .num = D19_GPIO13_A5_SCL,
        .func = GPIO_FUNC_I2C,
    },
    {
        .num = D2_GPIO25,
        .func = GPIO_FUNC_PWM,
    },
    {
        .num = D14_GPIO26_A0,
        .is_irq = true,
        .irq_type = GPIO_IRQ_EDGE_RISE,
        .func = GPIO_FUNC_PIO0,
    },
    {
        .num = D15_GPIO27_A1,
        .is_irq = true,
        .irq_type = GPIO_IRQ_EDGE_RISE,
        .func = GPIO_FUNC_PIO0,
    },
    {
        .num = D16_GPIO28_A2,
        .is_irq = true,
        .irq_type = GPIO_IRQ_EDGE_RISE,
        .func = GPIO_FUNC_PIO0,
    },
    {
        .num = D17_GPIO29_A3,
        .is_irq = true,
        .irq_type = GPIO_IRQ_EDGE_RISE,
        .func = GPIO_FUNC_PIO0,
    },
    {
        .num = D3_GPIO15,
        .is_irq = true,
        .irq_type = GPIO_IRQ_EDGE_RISE,
        .func = GPIO_FUNC_PIO0,
    },
};

#if DEBUG
#define pr_debug() printf("[DEBUG] %s:%u\n", __func__, __LINE__);
#else
#define pr_debug();
#endif

static inline uint map_get(uint val, uint low_from, uint high_from,
                           uint low_to, uint high_to)
{
    return (val - low_from) *
           (high_to - low_to) / (high_from - low_from) + low_to;
}

static void pwm_gpio_enable(gpio_t *gpio)
{
    uint slice_num, channel;

    slice_num = pwm_gpio_to_slice_num(gpio->num);
    channel = pwm_gpio_to_channel(gpio->num);

    pwm_set_wrap(slice_num, DEFAULUT_PWM_WRAP);
    pwm_set_chan_level(slice_num, channel, 169);
    pwm_set_enabled(slice_num, true);
}

static void pwm_gpio_set_duty_cycle(gpio_t *gpio, uint percent)
{
    uint slice_num, channel, percent_to_wrap;

    slice_num = pwm_gpio_to_slice_num(gpio->num);
    channel = pwm_gpio_to_channel(gpio->num);
    percent_to_wrap = map_get(percent, 0, 100, 0, DEFAULUT_PWM_WRAP);
    
    pwm_set_chan_level(slice_num, channel, percent_to_wrap);
}

static uint counter = 0;

void gpio_irq_handler(uint gpio_num, uint32_t event_mask)
{
    printf("dude...");
    switch (gpio_num) {
    case D14_GPIO26_A0:
        printf("%u\n", counter++);
        break;
    case D15_GPIO27_A1:
        printf("M1 B\n");
        break;
    case D16_GPIO28_A2:
        printf("M2 A\n");
        break;
    case D17_GPIO29_A3:
        printf("M2 B\n");
        break;
    case D3_GPIO15:
        printf("D3 %u\n", time_us_32());
        break;
    case D2_GPIO25:
        printf("D2 %u\n", time_us_32());
        break;
    }
}

static void i2c_write_byte(uint8_t slave_addr, uint8_t reg, uint8_t value)
{
    uint8_t buf[2];
    buf[0] = reg;
    buf[1] = value;

    i2c_write_blocking(i2c_default, slave_addr, buf, sizeof(buf), false);
}

static void i2c_read_byte(uint8_t slave_addr, uint8_t reg, uint8_t *value)
{
    i2c_write_blocking(i2c_default, slave_addr, &reg, sizeof(reg), false);

    i2c_read_blocking(i2c_default, slave_addr, value, sizeof(*value), false);
}

static inline void init_accelerometer()
{
    i2c_write_byte(LSM6DSOX_I2C_BUS_ADDR, CTRL1_XL, 0b1010000);
}

static inline void init_gyroscope()
{
    i2c_write_byte(LSM6DSOX_I2C_BUS_ADDR, CTRL2_G, 0b1010000);
}

static void init_all()
{
    size_t ngpios, i;

    /*
     * Enable Serial Interface
     */
    stdio_init_all();

#if DEBUG
    sleep_ms(5000);
#endif

    i2c_init(i2c_default, 400 * 1000);

    /*
     * Enable each GPIO individually
     */
gpio_set_irq_enabled_with_callback(26, GPIO_IRQ_EDGE_RISE, true, &gpio_irq_handler);
    ngpios = sizeof(gpios) / sizeof(*gpios);
    printf("ngpios %u\n", ngpios);
    for (i = 0; i < ngpios; i++) {
        gpio_init(gpios[i].num);

        gpio_set_function(gpios[i].num, gpios[i].func);

        if (gpios[i].is_irq) {
            pr_debug();

            gpio_set_irq_enabled(gpios[i].num, gpios[i].irq_type, true);
            

            pr_debug();

            continue;
        }

        pr_debug();

        switch (gpios[i].func) {
        case GPIO_FUNC_PWM:
            pr_debug();

            pwm_gpio_enable(&gpios[i]);

            pr_debug();

            break;

        case GPIO_FUNC_I2C:
            pr_debug();

            gpio_pull_up(gpios[i].num);

            pr_debug();

            break;

        default:
            pr_debug();

            gpio_set_dir(gpios[i].num, gpios[i].is_out);

            pr_debug()
            break;
        }
    }

    init_accelerometer();
    init_gyroscope();
}

int main() {
    init_all();

    while (true) {
        printf("hi\n");
        sleep_ms(1000);
        uint8_t reg, value;

        reg = CTRL1_XL;
        i2c_read_byte(LSM6DSOX_I2C_BUS_ADDR, reg, &value);
        printf("CTRL1_XL %hhu\n", value);

        reg = CTRL2_G;
        i2c_read_byte(LSM6DSOX_I2C_BUS_ADDR, reg, &value);
        printf("CTRL2_G %hhu\n", value);

        reg = OUTX_L_G;
        i2c_read_byte(LSM6DSOX_I2C_BUS_ADDR, reg, &value);
        printf("OUTX_L_G %hhu\n", value);

        reg = OUTX_H_G;
        i2c_read_byte(LSM6DSOX_I2C_BUS_ADDR, reg, &value);
        printf("OUTX_H_G %hhu\n", value);

        reg = OUTY_L_G;
        i2c_read_byte(LSM6DSOX_I2C_BUS_ADDR, reg, &value);
        printf("OUTY_L_G %hhu\n", value);

        reg = OUTY_H_G;
        i2c_read_byte(LSM6DSOX_I2C_BUS_ADDR, reg, &value);
        printf("OUTY_H_G %hhu\n", value);

        reg = OUTZ_L_G;
        i2c_read_byte(LSM6DSOX_I2C_BUS_ADDR, reg, &value);
        printf("OUTZ_L_G %hhu\n", value);

        reg = OUTZ_H_G;
        i2c_read_byte(LSM6DSOX_I2C_BUS_ADDR, reg, &value);
        printf("OUTZ_H_G %hhu\n", value);

        reg = OUTX_L_A;
        i2c_read_byte(LSM6DSOX_I2C_BUS_ADDR, reg, &value);
        printf("OUTX_L_A %hhu\n", value);

        reg = OUTX_H_A;
        i2c_read_byte(LSM6DSOX_I2C_BUS_ADDR, reg, &value);
        printf("OUTX_H_A %hhu\n", value);

        reg = OUTY_L_A;
        i2c_read_byte(LSM6DSOX_I2C_BUS_ADDR, reg, &value);
        printf("OUTY_L_A %hhu\n", value);

        reg = OUTY_H_A;
        i2c_read_byte(LSM6DSOX_I2C_BUS_ADDR, reg, &value);
        printf("OUTY_H_A %hhu\n", value);

        reg = OUTZ_L_A;
        i2c_read_byte(LSM6DSOX_I2C_BUS_ADDR, reg, &value);
        printf("OUTZ_L_A %hhu\n", value);

        reg = OUTZ_H_A;
        i2c_read_byte(LSM6DSOX_I2C_BUS_ADDR, reg, &value);
        printf("OUTZ_H_A %hhu\n", value);


    }
    return 0;
}