#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "pico/binary_info.h"
#include "pico/multicore.h"
#include <lsm6dsox.h>
#include <stdio.h>
#include <debug.h>
#include <gpio.h>
#include <math.h>
#include <stdlib.h>

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

    set_sys_clock_khz(200000, true);

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

float acc_Rx, acc_Ry, acc_Rz, acc_R;
float gyro_Rx, gyro_Ry, gyro_Rz, gyro_R;
const float acc_Ry_error = .3f;

static inline void update_position()
{
    float volts_accRx, volts_accRy, volts_accRz;
    float volts_gyroRx, volts_gyroRy, volts_gyroRz;
    const float acc_volts_0g = 1.45f;
    const float gyro_volts_0g = 1.23f;
    const float acc_sensitivity = 0.488f;
    const float gyro_sensitivity = 0.002f;
    float cos_accRx, cos_accRy, cos_accRz;
    float cos_gyroRx, cos_gyroRy, cos_gyroRz;
    float wGyro = 10.f;
    volts_accRx = (float) lsm6dsox.acc->get_x() * 3.3f / 65535.f;
    volts_accRy = (float) lsm6dsox.acc->get_y() * 3.3f / 65535.f;
    volts_accRz = (float) lsm6dsox.acc->get_z() * 3.3f / 65535.f;

    acc_Rx = (volts_accRx - acc_volts_0g) / acc_sensitivity;
    acc_Ry = (volts_accRy - acc_volts_0g) / acc_sensitivity;
    acc_Rz = (volts_accRz - acc_volts_0g) / acc_sensitivity;

    // acc_R = sqrt(acc_Rx * acc_Rx + acc_Ry * acc_Ry + acc_Rz * acc_Rz);

    // cos_accRx = acc_Rx / acc_R;
    // cos_accRy = acc_Ry / acc_R;
    // cos_accRz = acc_Rz / acc_R;

    // printf("acc_Rx %f\nacc_Ry %f\nacc_Rz %f\n", acc_Rx, acc_Ry, acc_Rz);

    volts_gyroRx = (float) lsm6dsox.gyro->get_x() * 3.3f / 65535.f;
    volts_gyroRy = (float) lsm6dsox.gyro->get_y() * 3.3f / 65535.f;
    volts_gyroRz = (float) lsm6dsox.gyro->get_z() * 3.3f / 65535.f;

    gyro_Rx = (volts_gyroRx - gyro_volts_0g) / gyro_sensitivity;
    gyro_Ry = (volts_gyroRy - gyro_volts_0g) / gyro_sensitivity;
    gyro_Rz = (volts_gyroRz - gyro_volts_0g) / gyro_sensitivity;

    // gyro_R = sqrt(gyro_Rx * gyro_Rx + gyro_Ry * gyro_Ry + gyro_Rz * gyro_Rz);

    // cos_gyroRx = gyro_Rx / gyro_R;
    // cos_gyroRy = gyro_Ry / gyro_R;
    // cos_gyroRz = gyro_Rz / gyro_R;

    // printf("gyro_Rx %f\ngyro_Ry %f\ngyro_Rz %f\n", gyro_Rx, gyro_Ry, gyro_Rz);

    // Rx = (acc_Rx + gyro_Rx * wGyro) / (1 + wGyro);
    // Ry = (acc_Ry + gyro_Ry * wGyro) / (1 + wGyro);
    // Rz = (acc_Rz + gyro_Rz * wGyro) / (1 + wGyro);

    // printf("Rx %f\nRy %f\nRz %f\n", Rx, Ry, Rz);
}

int main() {
    multicore_launch_core1(core1_main);
    init_all();

    while (true) {
        //printf("hi\n");
        // printf("g.x %d\ng.y %d\ng.z %d\na.x %d\na.y %d\na.z %d\n",
        //         lsm6dsox.gyro->get_x(), lsm6dsox.gyro->get_y(), lsm6dsox.gyro->get_z(),
        //         lsm6dsox.acc->get_x(), lsm6dsox.acc->get_y(), lsm6dsox.acc->get_z());
        update_position();

        if ((int)acc_Rx != -2)
            printf("Ox - stanga/dreapta %d\n", (int)acc_Rx);

        if ((int)(acc_Ry - acc_Ry_error) != -3)
            printf("Oy - fata/spate %d\n", (int)(acc_Ry - acc_Ry_error));

        if ((int)gyro_Rx / 100 != -6)
            printf("Ox - sus/jos %d\n", (int)gyro_Rx / 100);

        if ((int)gyro_Rz / 100 > -6)
            printf("Oz - rotatie spre dreapta %d\n", (int)gyro_Rz / 100);
        else if ((int)gyro_Rz / 100 < -6)
            printf("Oz - rotatie spre stanga %d\n", (int)gyro_Rz / 100);

        sleep_ms(100);
    }
    return 0;
}