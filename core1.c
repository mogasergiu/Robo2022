#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "pico/binary_info.h"
#include "pico/multicore.h"
#include <lsm6dsox.h>
#include <stdio.h>
#include <debug.h>
#include <gpio.h>
#include <pwm.h>

#define MOTOR_RIGHT_BACK 0
#define MOTOR_RIGHT_FRONT 1
#define MOTOR_LEFT_BACK 2
#define MOTOR_LEFT_FRONT 3

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
        .num = D9_GPIO21,
        .is_irq = true,
        .irq_type = GPIO_IRQ_LEVEL_HIGH,
        .func = GPIO_FUNC_PIO1,
    },
    {
        .num = D12_GPIO4_CIPO,
        .is_irq = true,
        .irq_type = GPIO_IRQ_LEVEL_HIGH,
        .func = GPIO_FUNC_PIO1,
    },
    {
        .num = D8_GPIO20,
        .is_irq = true,
        .irq_type = GPIO_IRQ_LEVEL_LOW,
        .func = GPIO_FUNC_PIO1,
    },
    {
        .num = D10_GPIO5,
        .is_irq = true,
        .irq_type = GPIO_IRQ_LEVEL_LOW,
        .func = GPIO_FUNC_PIO1,
    },
    {
        .num = D11_GPIO7_COPI,
        .is_irq = true,
        .irq_type = GPIO_IRQ_LEVEL_LOW,
        .func = GPIO_FUNC_PIO1,
    },
};

static uint8_t sensor_mask;

static void gpio_irq_handler(uint gpio_num, uint32_t event_mask)
{
    switch (gpio_num) {
    case D9_GPIO21:
        sensor_mask |= SENSOR_LOWER_LEFT;  // lower-left
        pr_debug("D9_GPIO21 start-stop interrupt\n");
        break;
    case D12_GPIO4_CIPO:
        sensor_mask |= SENSOR_LOWER_RIGHT;  // lower-right
        pr_debug("D12_GPIO4_CIPO button INT\n");
        break;
    case D8_GPIO20:
        sensor_mask |= SENSOR_MIDDLE;  // middle
        pr_debug("D8_GPIO20 button INT\n");
        break;
    case D10_GPIO5:
        sensor_mask |= SENSOR_UPPER_LEFT;  // upper-left
        pr_debug("D10_GPIO5 button INT\n");
        break;
    case D11_GPIO7_COPI:
        sensor_mask |= SENSOR_UPPER_RIGHT;  // upper-right
        pr_debug("D11_GPIO7_COPI button INT\n");
        break;
    }
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
    init_all();

    while (true) {
        pr_debug("hi from core1\n");
        
        switch (sensor_mask) {
        case SENSOR_LOWER_LEFT:  // m1 back and m0 front
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_BACK0], 0);
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_FRONT1], 0);
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_FRONT0], 100);
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_BACK1], 100);

            break;
        case SENSOR_LOWER_RIGHT:  // m1 front and m0 back
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_BACK1], 0);
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_BACK0], 0);
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_FRONT1], 100);
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_FRONT0], 100);

            break;
        case SENSOR_MIDDLE:  // m1 front and m0 front
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_BACK0], 0);
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_BACK1], 0);
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_FRONT1], 100);
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_FRONT0], 100);

            break;
        default:
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_BACK0], 0);
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_BACK1], 0);
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_FRONT0], 0);
            pwm_gpio_set_duty_cycle(&gpios[MOTOR_FRONT1], 0);
        }

        sensor_mask = 0;
        //sleep_ms(100);
    }
}

// #include "pico/stdlib.h"
// #include "hardware/adc.h"
// #include "pico/binary_info.h"
// #include "pico/multicore.h"
// #include <lsm6dsox.h>
// #include <stdio.h>
// #include <debug.h>
// #include <gpio.h>
// #include <pwm.h>

// #define MOTOR_BACK0 0
// #define MOTOR_FRONT0 1
// #define MOTOR_BACK1 2
// #define MOTOR_FRONT1 3

// #define SENSOR_LOWER_LEFT (1 << 0)  // D9_GPIO21
// #define SENSOR_LOWER_RIGHT (1 << 1)  // D12_GPIO4_CIPO
// #define SENSOR_MIDDLE (1 << 2)  // D8_GPIO20
// #define SENSOR_UPPER_LEFT (1 << 3)  // D10_GPIO5
// #define SENSOR_UPPER_RIGHT (1 << 4)  // D11_GPIO7_COPI

// // Only sensor IRQ's
// static gpio_t gpios[] = {
//     {
//         .num = D3_GPIO15,
//         .func = GPIO_FUNC_PWM,
//         .is_out = true,
//     },
//     {
//         .num = D5_GPIO17,
//         .func = GPIO_FUNC_PWM,
//         .is_out = true,
//     },
//     {
//         .num = D2_GPIO25,
//         .func = GPIO_FUNC_PWM,
//         .is_out = true,
//     },
//     {
//         .num = D4_GPIO16,
//         .func = GPIO_FUNC_PWM,
//         .is_out = true,
//     },
//     {
//         .num = D9_GPIO21,
//         // .is_irq = true,
//         // .irq_type = GPIO_IRQ_LEVEL_HIGH,
//         // .func = GPIO_FUNC_PIO1,
//         // .is_out = true,
//     },
//     {
//         .num = D12_GPIO4_CIPO,
//         // .is_irq = true,
//         // .irq_type = GPIO_IRQ_LEVEL_HIGH,
//         // .func = GPIO_FUNC_PIO1,
//         // .is_out = true,
//     },
//     {
//         .num = D8_GPIO20,
//         // .is_irq = true,
//         // .irq_type = GPIO_IRQ_LEVEL_LOW,
//         // .func = GPIO_FUNC_PIO1,
//         // .is_out = true,
//     },
//     {
//         .num = D10_GPIO5,
//         // .is_irq = true,
//         // .irq_type = GPIO_IRQ_LEVEL_LOW,
//         // .func = GPIO_FUNC_PIO1,
//         // .is_out = true,
//     },
//     {
//         .num = D11_GPIO7_COPI,
//         // .is_irq = true,
//         // .irq_type = GPIO_IRQ_LEVEL_LOW,
//         // .func = GPIO_FUNC_PIO1,
//         // .is_out = true,
//     },
// };

// static uint8_t sensor_mask;

// static void gpio_irq_handler(uint gpio_num, uint32_t event_mask)
// {
//     switch (gpio_num) {
//     case D9_GPIO21:
//         sensor_mask |= SENSOR_LOWER_LEFT;  // lower-left
//                     pwm_gpio_set_duty_cycle(&gpios[MOTOR_BACK0], 0);
//             pwm_gpio_set_duty_cycle(&gpios[MOTOR_FRONT1], 0);
//             pwm_gpio_set_duty_cycle(&gpios[MOTOR_FRONT0], 100);
//             pwm_gpio_set_duty_cycle(&gpios[MOTOR_BACK1], 100);
//         pr_debug("D9_GPIO21 start-stop interrupt\n");
//         break;
//     case D12_GPIO4_CIPO:
//         sensor_mask |= SENSOR_LOWER_RIGHT;  // lower-right
//         pr_debug("D12_GPIO4_CIPO button INT\n");
//         break;
//     case D8_GPIO20:
//         sensor_mask |= SENSOR_MIDDLE;  // middle
//         pr_debug("D8_GPIO20 button INT\n");
//         break;
//     case D10_GPIO5:
//         sensor_mask |= SENSOR_UPPER_LEFT;  // upper-left
//         pr_debug("D10_GPIO5 button INT\n");
//         break;
//     case D11_GPIO7_COPI:
//         sensor_mask |= SENSOR_UPPER_RIGHT;  // upper-right
//         pr_debug("D11_GPIO7_COPI button INT\n");
//         break;
//     }
// }

// static int init_all()
// {
//     int i, ret;
//     size_t ngpios;

// #if DEBUG
//     sleep_ms(2000);
// #endif

//     set_sys_clock_khz(220000, true);

//     // gpio_set_irq_callback(&gpio_irq_handler);
//     // irq_set_enabled(IO_IRQ_BANK0, true);

//     ngpios = sizeof(gpios) / sizeof(*gpios);
//     for (i = 0; i < ngpios; i++) {
//         gpio_init(gpios[i].num);

//         if (gpios[i].func)
//             gpio_set_function(gpios[i].num, gpios[i].func);

//         if (gpios[i].is_out)
//             gpio_set_dir(gpios[i].num, GPIO_OUT);
//         else
//             gpio_set_dir(gpios[i].num, GPIO_IN);

//         if (gpios[i].is_irq)
//             gpio_set_irq_enabled(gpios[i].num, gpios[i].irq_type, true);

//         if (gpios[i].func == GPIO_FUNC_PWM)
//             pwm_gpio_enable(&gpios[i]);
//     }

//     return 0;
// }

// void core1_main() {
//     init_all();

//     while (true) {
//         pr_debug("hi from core1\n");
        

//         int i;
//         for(i = 4; i <= 8; i++)
//             if (gpio_get(gpios[i].num))
//                 sensor_mask |= (1 << i);

//         printf("%hu\n", sensor_mask);

//         switch (sensor_mask) {
//         case SENSOR_LOWER_LEFT:  // m1 back and m0 front
//             pwm_gpio_set_duty_cycle(&gpios[MOTOR_BACK0], 0);
//             pwm_gpio_set_duty_cycle(&gpios[MOTOR_FRONT1], 0);
//             pwm_gpio_set_duty_cycle(&gpios[MOTOR_FRONT0], 100);
//             pwm_gpio_set_duty_cycle(&gpios[MOTOR_BACK1], 100);

//             break;
//         case SENSOR_LOWER_RIGHT:  // m1 front and m0 back
//             pwm_gpio_set_duty_cycle(&gpios[MOTOR_BACK1], 0);
//             pwm_gpio_set_duty_cycle(&gpios[MOTOR_BACK0], 0);
//             pwm_gpio_set_duty_cycle(&gpios[MOTOR_FRONT1], 100);
//             pwm_gpio_set_duty_cycle(&gpios[MOTOR_FRONT0], 100);

//             break;
//         case SENSOR_MIDDLE:  // m1 front and m0 front
//             pwm_gpio_set_duty_cycle(&gpios[MOTOR_BACK0], 0);
//             pwm_gpio_set_duty_cycle(&gpios[MOTOR_BACK1], 0);
//             pwm_gpio_set_duty_cycle(&gpios[MOTOR_FRONT1], 100);
//             pwm_gpio_set_duty_cycle(&gpios[MOTOR_FRONT0], 100);

//             break;
//         default:
//             pwm_gpio_set_duty_cycle(&gpios[MOTOR_BACK0], 0);
//             pwm_gpio_set_duty_cycle(&gpios[MOTOR_BACK1], 0);
//             pwm_gpio_set_duty_cycle(&gpios[MOTOR_FRONT0], 0);
//             pwm_gpio_set_duty_cycle(&gpios[MOTOR_FRONT1], 0);
//         }

//         sensor_mask = 0;
//         //sleep_ms(100);
//     }
// }