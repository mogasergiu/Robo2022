#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/sync.h"
#include "pico/binary_info.h"
#include "pico/multicore.h"
#include <lsm6dsox.h>
#include <stdio.h>
#include <debug.h>
#include <gpio.h>
#include <pwm.h>

#define MOTOR_LEFT_BACK 0
#define MOTOR_LEFT_FRONT 1
#define MOTOR_RIGHT_BACK 2
#define MOTOR_RIGHT_FRONT 3

#define SENSOR_LOWER_LEFT (1 << 0)  // D9_GPIO21
#define SENSOR_LOWER_RIGHT (1 << 1)  // D12_GPIO4_CIPO
#define SENSOR_MIDDLE (1 << 2)  // D8_GPIO20
#define SENSOR_UPPER_LEFT (1 << 3)  // D10_GPIO5
#define SENSOR_UPPER_RIGHT (1 << 4)  // D11_GPIO7_COPI

// Only sensor IRQ's
static gpio_t gpios[] = {
    {
        .num = D3_GPIO15,
        .func = GPIO_FUNC_PWM,
        .is_out = true,
    },
    {
        .num = D5_GPIO17,
        .func = GPIO_FUNC_PWM,
        .is_out = true,
    },
    {
        .num = D2_GPIO25,
        .func = GPIO_FUNC_PWM,
        .is_out = true,
    },
    {
        .num = D4_GPIO16,
        .func = GPIO_FUNC_PWM,
        .is_out = true,
    },
    {
        .num = D10_GPIO5,  // 0 to 3v3
        // .is_irq = true,
        .irq_type = GPIO_IRQ_EDGE_RISE,
        // .func = GPIO_FUNC_PIO1,
        // .is_out = true,
    },
    {
        .num = D8_GPIO20,  // 0 to 3v3
        // .is_irq = true,
        .irq_type = GPIO_IRQ_EDGE_RISE,
        // .func = GPIO_FUNC_PIO1,
        // .is_out = true,
    },
    {
        .num = D11_GPIO7_COPI,  // 0 to 3v3
        // .is_irq = true,
        .irq_type = GPIO_IRQ_EDGE_RISE,
        // .func = GPIO_FUNC_PIO1,
        // .is_out = true,
    },
    {
        .num = D9_GPIO21,  // 3v3 to 0
        // .is_irq = true,
        .irq_type = GPIO_IRQ_EDGE_FALL,
        // .func = GPIO_FUNC_PIO1,
        // .is_out = true,
    },
    {
        .num = D12_GPIO4_CIPO,  // 3v3 to 0
        // .is_irq = true,
        .irq_type = GPIO_IRQ_EDGE_FALL,
        // .func = GPIO_FUNC_PIO1,
        // .is_out = true,
    },
};

static uint8_t sensor_mask;

static inline void invert_irq_and_set_sensor_bit(uint gpio_num, uint8_t sensor_bit,
                                                 enum gpio_irq_level irq_type)
{
    if (irq_type == GPIO_IRQ_EDGE_RISE)
        if (sensor_mask & sensor_bit) {
            sensor_mask &= ~sensor_bit;
            gpio_set_irq_enabled(gpio_num, GPIO_IRQ_EDGE_RISE, true);
            pr_debug("Setting %u GPIO to GPIO_IRQ_EDGE_RISE\n", gpio_num);
        } else {
            sensor_mask |= sensor_bit;
            gpio_set_irq_enabled(gpio_num, GPIO_IRQ_EDGE_FALL, true);
            pr_debug("Setting %u GPIO to GPIO_IRQ_EDGE_FALL\n", gpio_num);
        }
    else
        if (sensor_mask & sensor_bit) {
            sensor_mask &= ~sensor_bit;
            gpio_set_irq_enabled(gpio_num, GPIO_IRQ_EDGE_FALL, true);
            pr_debug("Setting %u GPIO to GPIO_IRQ_EDGE_FALL\n", gpio_num);
        } else {
            sensor_mask |= sensor_bit;
            gpio_set_irq_enabled(gpio_num, GPIO_IRQ_EDGE_RISE, true);
            pr_debug("Setting %u GPIO to GPIO_IRQ_EDGE_RISE\n", gpio_num);
        }
}

static void gpio_irq_handler(uint gpio_num, uint32_t event_mask)
{
    uint32_t irq_ctx;
    
    irq_ctx = save_and_disable_interrupts();

    switch (gpio_num) {
    case D10_GPIO5:
        invert_irq_and_set_sensor_bit(gpio_num, SENSOR_LOWER_LEFT, GPIO_IRQ_EDGE_RISE);
        pr_debug("D10_GPIO5 SENSOR_LOWER_LEFT INT\n");
        break;
    case D8_GPIO20:
        invert_irq_and_set_sensor_bit(gpio_num, SENSOR_LOWER_RIGHT, GPIO_IRQ_EDGE_RISE);
        pr_debug("D8_GPIO20 SENSOR_LOWER_RIGHT INT\n");
        break;
    case D11_GPIO7_COPI:
        invert_irq_and_set_sensor_bit(gpio_num, SENSOR_MIDDLE, GPIO_IRQ_EDGE_RISE);
        pr_debug("D11_GPIO7_COPI SENSOR_MIDDLE INT\n");
        break;
    case D9_GPIO21:
        invert_irq_and_set_sensor_bit(gpio_num, SENSOR_UPPER_LEFT, GPIO_IRQ_EDGE_FALL);
        pr_debug("D9_GPIO21 SENSOR_UPPER_LEFT INT\n");
        break;
    case D12_GPIO4_CIPO:
        invert_irq_and_set_sensor_bit(gpio_num, SENSOR_UPPER_RIGHT, GPIO_IRQ_EDGE_FALL);
        pr_debug("D12_GPIO4_CIPO SENSOR_UPPER_RIGHT INT\n");
        break;
    }
    
    restore_interrupts(irq_ctx);
}

static int init_all()
{
    int i, ret;
    size_t ngpios;

#if DEBUG
    sleep_ms(2000);
#endif

    set_sys_clock_khz(200000, true);

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

        if (gpios[i].func == GPIO_FUNC_PWM)
            pwm_gpio_enable(&gpios[i]);
    }

    return 0;
}

void core1_main() {
    int i;

    init_all();

    while (true) {
        for (i = 4; i < 9; i++)
            switch(gpios[i].num) {
            case D10_GPIO5:
                if (gpio_get(D10_GPIO5))
                    sensor_mask |= (1 << (i - 4));
                else
                    sensor_mask &= ~(1 << (i - 4));
                break;
            case D8_GPIO20:
                if (gpio_get(D8_GPIO20))
                    sensor_mask |= (1 << (i - 4));
                else
                    sensor_mask &= ~(1 << (i - 4));
                break;
            case D11_GPIO7_COPI:
                if (gpio_get(D11_GPIO7_COPI))
                    sensor_mask |= (1 << (i - 4));
                else
                    sensor_mask &= ~(1 << (i - 4));
                break;
            case D9_GPIO21:
                if (gpio_get(D9_GPIO21))
                    sensor_mask &= ~(1 << (i - 4));
                else
                    sensor_mask |= (1 << (i - 4));
                break;
            case D12_GPIO4_CIPO:
                if (gpio_get(D12_GPIO4_CIPO))
                    sensor_mask &= ~(1 << (i - 4));
                else
                    sensor_mask |= (1 << (i - 4));
                break;
            }

        switch (sensor_mask) {
        case SENSOR_LOWER_LEFT:
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_BACK], 0);
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_FRONT], 0);
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_FRONT], 10);
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_BACK], 10);

            break;
        case SENSOR_LOWER_RIGHT:
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_BACK], 0);
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_FRONT], 0);
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_FRONT], 10);
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_BACK], 10);

            break;
        case SENSOR_MIDDLE:
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_BACK], 0);
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_BACK], 0);
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_FRONT], 10);
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_FRONT], 10);

            break;
        case SENSOR_UPPER_LEFT:
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_BACK], 0);
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_BACK], 0);
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_FRONT], 20);
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_FRONT], 30);

            break;
        case SENSOR_UPPER_RIGHT:
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_BACK], 0);
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_BACK], 0);
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_FRONT], 30);
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_FRONT], 20);

            break;
        case (SENSOR_LOWER_LEFT | SENSOR_UPPER_LEFT):
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_BACK], 0);
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_BACK], 0);
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_FRONT], 20);
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_FRONT], 40);

            break;
        case (SENSOR_LOWER_RIGHT | SENSOR_UPPER_RIGHT):
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_BACK], 0);
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_BACK], 0);
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_FRONT], 40);
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_FRONT], 20);

            break;
        case (SENSOR_UPPER_LEFT | SENSOR_MIDDLE):
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_BACK], 0);
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_BACK], 0);
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_FRONT], 40);
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_FRONT], 50);

            break;
        case (SENSOR_UPPER_RIGHT | SENSOR_MIDDLE):
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_BACK], 0);
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_BACK], 0);
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_FRONT], 50);
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_FRONT], 40);

            break;
        case 0:
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_BACK], 0);
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_BACK], 0);
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_FRONT], 0);
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_FRONT], 0);
        }
    }
}