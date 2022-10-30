#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "pico/binary_info.h"
#include "pico/multicore.h"
#include <lsm6dsox.h>
#include <stdio.h>
#include <debug.h>
#include <gpio.h>

static gpio_t gpios[] = {
    {
        .num = D14_GPIO26_A0,   // m1 INT
        .is_irq = true,
        .irq_type = GPIO_IRQ_EDGE_RISE,
        .func = GPIO_FUNC_PIO0,
    },
    {
        .num = D16_GPIO28_A2,   // m2 INT
        .is_irq = true,
        .irq_type = GPIO_IRQ_EDGE_RISE,
        .func = GPIO_FUNC_PIO0,
    },
    {
        .num = D6_GPIO18,   // start-stop INT
        .is_irq = true,
        .irq_type = GPIO_IRQ_EDGE_RISE,
        .func = GPIO_FUNC_PIO0,
    },
    {
        .num = D7_GPIO19,   // button INT
        .is_irq = true,
        .irq_type = GPIO_IRQ_EDGE_FALL,
        .func = GPIO_FUNC_PIO0,
    },
    {
        .num = D13_GPIO6_SCK,   // buton INT
        .is_irq = true,
        .irq_type = GPIO_IRQ_EDGE_FALL,
        .func = GPIO_FUNC_PIO0,
    },
};

static void gpio_irq_handler(uint gpio_num, uint32_t event_mask)
{
    switch (gpio_num) {
    case D6_GPIO18:
        pr_debug("D6_GPIO18 start-stop interrupt\n");
        break;
    case D7_GPIO19:
        pr_debug("D7_GPIO19 button INT\n");
        break;
    case D13_GPIO6_SCK:
        pr_debug("D13_GPIO6_SCK button INT\n");
        break;
    case D14_GPIO26_A0:
        pr_debug("D14_GPIO26_A0 button INT\n");
        break;
    case D16_GPIO28_A2:
        pr_debug("D16_GPIO28_A2 button INT\n");
        break;
    }
}

static int init_all()
{
    int i, ret;
    size_t ngpios;

    /*
     * Enable Serial Interface
     */
    stdio_init_all();

#if DEBUG
    sleep_ms(2000);
#endif

    i2c_init(i2c_default, 400 * 1000);

    set_sys_clock_khz(220000, true);

    ret = lsm6dsox.ops->probe();
    if (ret < 0) {
        pr_debug("%s:%u Failed to probe LSM6DSOX\n", __func__, __LINE__);
        return ret;
    }

    gpio_set_irq_callback(&gpio_irq_handler);
    irq_set_enabled(IO_IRQ_BANK0, true);

    ngpios = sizeof(gpios) / sizeof(*gpios);
    for (i = 0; i < ngpios; i++) {
        gpio_init(gpios[i].num);
        gpio_set_function(gpios[i].num, gpios[i].func);

        if (gpios[i].is_out)
            gpio_set_dir(gpios[i].num, GPIO_OUT);

        if (gpios[i].is_irq)
            gpio_set_irq_enabled(gpios[i].num, gpios[i].irq_type, true);
    }

    return 0;
}

extern void core1_main();

int main() {
    multicore_launch_core1(core1_main);
    init_all();

    while (true) {
        //printf("hi\n");
        // pr_debug("g.x %d\ng.y %d\ng.z %d\na.x %d\na.y %d\na.z %d\n",
        //         lsm6dsox.gyro->get_x(), lsm6dsox.gyro->get_y(), lsm6dsox.gyro->get_z(),
        //         lsm6dsox.acc->get_x(), lsm6dsox.acc->get_y(), lsm6dsox.acc->get_z());
        sleep_ms(1000);
    }
    return 0;
}