#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include <lsm6dsox.h>
#include <stdio.h>
#include <debug.h>

#define DEFAULUT_PWM_WRAP 255

gpio_t gpios[] = {
    {
        .num = D2_GPIO25,
        .func = GPIO_FUNC_PWM,
    },
    {
        .num = D3_GPIO15,
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
    pwm_set_chan_level(slice_num, channel, 130);
    pwm_set_clkdiv(slice_num, 46.f);
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

void gpio_irq_handler(uint gpio_num, uint32_t event_mask)
{
    switch (gpio_num) {
    case D12_GPIO4_CIPO:
        printf("D12 %u\n", time_us_32());
        gpio_put(D5_GPIO17, 1);
        break;
    case D11_GPIO7_COPI:
        printf("D11 %u\n", time_us_32());
        gpio_put(D5_GPIO17, 0);
        break;
    }
}

static int init_all()
{
    int ret;

    /*
     * Enable Serial Interface
     */
    stdio_init_all();

#if DEBUG
    sleep_ms(2000);
#endif

    i2c_init(i2c_default, 400 * 1000);

    ret = lsm6dsox.ops->probe();
    if (ret < 0) {
        pr_debug("%s:%u Failed to probe LSM6DSOX\n", __func__, __LINE__);
        return ret;
    }

    gpio_init(D11_GPIO7_COPI);
    gpio_set_function(D11_GPIO7_COPI, GPIO_FUNC_PIO0);
    gpio_set_irq_enabled_with_callback(D11_GPIO7_COPI, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    gpio_init(D12_GPIO4_CIPO);
    gpio_set_function(D12_GPIO4_CIPO, GPIO_FUNC_PIO0);
    gpio_set_irq_enabled(D12_GPIO4_CIPO, GPIO_IRQ_EDGE_RISE, true);

    set_sys_clock_khz(220000, true);

    return 0;
}

int main() {
    init_all();

    gpio_init(D3_GPIO15);
    gpio_set_dir(D3_GPIO15, GPIO_OUT);
    gpio_set_function(D3_GPIO15, GPIO_FUNC_PWM);
    pwm_gpio_enable(&gpios[1]);

    gpio_init(D5_GPIO17);
    gpio_set_dir(D5_GPIO17, GPIO_OUT);
    gpio_put(D5_GPIO17, 0);

    pwm_config cfg = pwm_get_default_config();
    printf("div: %u\n", cfg.div);

    while (true) {
        //printf("hi\n");
        pr_debug("g.x %d\ng.y %d\ng.z %d\na.x %d\na.y %d\na.z %d\n",
                lsm6dsox.gyro->get_x(), lsm6dsox.gyro->get_y(), lsm6dsox.gyro->get_z(),
                lsm6dsox.acc->get_x(), lsm6dsox.acc->get_y(), lsm6dsox.acc->get_z());
        sleep_ms(1000);
    }
    return 0;
}