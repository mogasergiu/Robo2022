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

#define MOTOR_LEFT_FRONT 0
#define MOTOR_LEFT_BACK 1
#define MOTOR_RIGHT_FRONT 2
#define MOTOR_RIGHT_BACK 3

#define SENSOR_LOWER_LEFT (1 << 0)  // D9_GPIO21
#define SENSOR_LOWER_RIGHT (1 << 1)  // D12_GPIO4_CIPO
#define SENSOR_MIDDLE (1 << 2)  // D8_GPIO20
#define SENSOR_UPPER_LEFT (1 << 3)  // D10_GPIO5
#define SENSOR_UPPER_RIGHT (1 << 4)  // D11_GPIO7_COPI

// Only sensor IRQ's
static gpio_t gpios[] = {
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
};

#define ATTACK_WAIT (1 << 0)
#define ATTACK_GRADUAL (1 << 1)
static uint8_t strat_mask;

static uint8_t sensor_mask;
extern uint8_t dir_mask;
extern bool mleft_moving_fw, mright_moving_fw, start_stop;
extern uint8_t b1, b2;

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

static void move_forward(uint8_t mleft_dc, uint8_t mright_dc)
{
    pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_BACK], 0);
    pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_BACK], 0);
    pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_FRONT], mleft_dc);
    pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_FRONT], mright_dc);
}

static void move_left(uint8_t mleft_dc, uint8_t mright_dc)
{
    pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_BACK], 0);
    pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_FRONT], 0);
    pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_FRONT], mright_dc);
    pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_BACK], mleft_dc);
}

static void move_right(uint8_t mleft_dc, uint8_t mright_dc)
{
    pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_BACK], 0);
    pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_FRONT], 0);
    pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_FRONT], mleft_dc);
    pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_BACK], mright_dc);
}

static void move_angle(uint8_t mleft_dc, uint8_t mright_dc)
{
    pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_BACK], 0);
    pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_BACK], 0);
    pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_FRONT], mleft_dc);
    pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_FRONT], mright_dc);
}

static inline void stop()
{
    move_angle(0, 0);
}

static void update_by_sensor()
{
       switch (sensor_mask) {
        case SENSOR_LOWER_LEFT:
            move_left(70, 70);

            break;
        case SENSOR_LOWER_RIGHT:
            move_right(70, 70);

            break;
        case SENSOR_MIDDLE:
            move_forward(100, 100);

            break;
        case SENSOR_UPPER_LEFT:
            move_angle(50, 70);

            break;
        case SENSOR_UPPER_RIGHT:
            move_angle(70, 50);

            break;
        case (SENSOR_LOWER_LEFT | SENSOR_UPPER_LEFT):
            move_angle(60, 90);

            break;
        case (SENSOR_LOWER_RIGHT | SENSOR_UPPER_RIGHT):
            move_angle(90, 60);

            break;
        case (SENSOR_UPPER_LEFT | SENSOR_MIDDLE):
            move_angle(50, 60);

            break;
        case (SENSOR_UPPER_RIGHT | SENSOR_MIDDLE):
            move_angle(60, 50);

            break;
        default:
            stop();
        }
}

int64_t attack_wait_handler(alarm_id_t id, void *user_data)
{
    update_by_sensor();

    return 0;
}

#define ACC_OX_LEFT_RIGHT_AXIS (1 << 0)
#define ACC_OY_FRONT_BACK_AXIS (1 << 1)
#define GYRO_OZ_LEFT_ORIENTATION (1 << 3)
#define GYRO_OZ_RIGHT_ORIENTATION (1 << 4)
#define GYRO_OX_UP_DOWN_ORIENTATION (1 << 5)

void core1_main()
{
    int i;

    init_all();

    start_stop = 1;

    while (!start_stop);

    switch (b1) {
    case 0:
        switch (b2) {
            case 1:
                move_angle(60, 60);
                sleep_ms(500);

                break;
            case 2:
                move_angle(60, 80);
                sleep_ms(300);

                break;
            case 3:
                move_angle(80, 60);
                sleep_ms(300);

                break;
            case 4:
                move_left(50, 50);
                sleep_ms(300);
                move_angle(80, 60);

                break;
            case 5:
                move_right(50, 50);
                sleep_ms(300);
                move_angle(60, 80);

                break;
        }

        break;

    case 1:
        switch (b2) {
            case 1:
                move_forward(50, 50);

                sleep_ms(300);

                for (i = 0; i < 5; i++)
                switch(gpios[i].num) {
                case D10_GPIO5:
                case D8_GPIO20:
                case D11_GPIO7_COPI:
                    if (gpio_get(gpios[i].num))
                        sensor_mask |= (1 << (i));
                    else
                        sensor_mask &= ~(1 << (i));
                
                    break;

                case D9_GPIO21:
                case D12_GPIO4_CIPO:
                    if (gpio_get(gpios[i].num))
                        sensor_mask &= ~(1 << (i));
                    else
                        sensor_mask |= (1 << (i));
                
                    break;
                }

                if (sensor_mask == SENSOR_MIDDLE &&
                    dir_mask & ACC_OY_FRONT_BACK_AXIS &&
                    !mleft_moving_fw && !mright_moving_fw) {
                    move_forward(100, 100);
                }

                break;

            case 2:
                move_angle(60, 80);

                sleep_ms(300);

                for (i = 0; i < 5; i++)
                switch(gpios[i].num) {
                case D10_GPIO5:
                case D8_GPIO20:
                case D11_GPIO7_COPI:
                    if (gpio_get(gpios[i].num))
                        sensor_mask |= (1 << (i));
                    else
                        sensor_mask &= ~(1 << (i));
                
                    break;

                case D9_GPIO21:
                case D12_GPIO4_CIPO:
                    if (gpio_get(gpios[i].num))
                        sensor_mask &= ~(1 << (i));
                    else
                        sensor_mask |= (1 << (i));
                
                    break;
                }

                if (sensor_mask == SENSOR_MIDDLE &&
                    dir_mask & ACC_OY_FRONT_BACK_AXIS &&
                    !mleft_moving_fw && !mright_moving_fw) {
                    move_forward(100, 100);
                }

                break;
            case 3:
                move_angle(80, 60);

                sleep_ms(300);

                for (i = 0; i < 5; i++)
                switch(gpios[i].num) {
                case D10_GPIO5:
                case D8_GPIO20:
                case D11_GPIO7_COPI:
                    if (gpio_get(gpios[i].num))
                        sensor_mask |= (1 << (i));
                    else
                        sensor_mask &= ~(1 << (i));
                
                    break;

                case D9_GPIO21:
                case D12_GPIO4_CIPO:
                    if (gpio_get(gpios[i].num))
                        sensor_mask &= ~(1 << (i));
                    else
                        sensor_mask |= (1 << (i));
                
                    break;
                }
                
                if (sensor_mask == SENSOR_MIDDLE &&
                    dir_mask & ACC_OY_FRONT_BACK_AXIS &&
                    !mleft_moving_fw && !mright_moving_fw) {
                    move_forward(100, 100);
                }
    
                break;

            case 4:
                move_left(50, 50);
                sleep_ms(300);
                move_angle(80, 60);

                strat_mask |= ATTACK_WAIT;

                break;

            case 5:
                move_right(50, 50);
                sleep_ms(300);
                move_angle(60, 80);

                strat_mask |= ATTACK_WAIT;

                break;
        }

        break;
    case 2:
        switch (b2) {
            case 1:
                move_forward(60, 60);
                sleep_ms(500);

                strat_mask |= ATTACK_GRADUAL;

                break;

            case 2:
                move_angle(60, 80);
                sleep_ms(300);

                strat_mask |= ATTACK_GRADUAL;

                break;

            case 3:
                move_angle(80, 60);
                sleep_ms(300);

                strat_mask |= ATTACK_GRADUAL;
    
                break;

            case 4:
                move_right(50, 50);
                sleep_ms(300);
                move_angle(80, 60);

                strat_mask |= ATTACK_GRADUAL;

                break;

            case 5:
                move_right(50, 50);
                sleep_ms(300);
                move_angle(60, 80);

                strat_mask |= ATTACK_GRADUAL;
    
                break;
        }

        break;
    }

    while (start_stop) {
        for (i = 0; i < 5; i++)
            switch(gpios[i].num) {
            case D10_GPIO5:
            case D8_GPIO20:
            case D11_GPIO7_COPI:
                if (gpio_get(gpios[i].num))
                    sensor_mask |= (1 << (i));
                else
                    sensor_mask &= ~(1 << (i));

                break;
            case D9_GPIO21:
            case D12_GPIO4_CIPO:
                if (gpio_get(gpios[i].num))
                    sensor_mask &= ~(1 << (i));
                else
                    sensor_mask |= (1 << (i));

                break;
            }

        if (strat_mask & ATTACK_WAIT)
            add_alarm_in_ms(300, attack_wait_handler, NULL, false);
        else
            switch (sensor_mask) {
            case SENSOR_LOWER_LEFT:
                pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_BACK], 0);
                pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_FRONT], 0);

                if (strat_mask & ATTACK_GRADUAL) {
                    for (i = 60; i <= 100; i++) {
                        pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_FRONT], i);
                        pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_BACK], i);
                    }
                } else {
                    pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_FRONT], 60);
                    pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_BACK], 60);
                }

                break;
                
            case SENSOR_LOWER_RIGHT:
                pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_BACK], 0);
                pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_FRONT], 0);

                if (strat_mask & ATTACK_GRADUAL) {
                    for (i = 60; i <= 100; i++) {
                        pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_FRONT], i);
                        pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_BACK], i);
                    }
                } else {
                    pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_FRONT], 60);
                    pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_BACK], 60);                    
                }

                break;

            case SENSOR_MIDDLE:
                pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_BACK], 0);
                pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_BACK], 0);

                if (strat_mask & ATTACK_GRADUAL) {
                    for (i = 60; i <= 100; i++) {
                        pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_FRONT], i);
                        pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_FRONT], i);
                    }
                } else {
                    pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_FRONT], 60);
                    pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_FRONT], 60);
                }

                break;

            case SENSOR_UPPER_LEFT:
                move_angle(20, 30);

                if (strat_mask & ATTACK_GRADUAL) {
                    for (i = 60; i <= 100; i++) {                        
                        pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_FRONT], i);
                        pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_FRONT], i);
                    }
                } else {
                    pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_FRONT], 60);
                    pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_FRONT], 60);                    
                }

                break;

            case SENSOR_UPPER_RIGHT:
                move_angle(30, 20);

                if (strat_mask & ATTACK_GRADUAL) {
                    for (i = 60; i <= 100; i++) {                        
                        pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_FRONT], i);
                        pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_FRONT], i);
                    }
                } else {
                    pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_FRONT], 60);
                    pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_FRONT], 60);                    
                }                

                break;

            case (SENSOR_LOWER_LEFT | SENSOR_UPPER_LEFT):
                move_angle(20, 40);

                if (strat_mask & ATTACK_GRADUAL) {
                    for (i = 60; i <= 100; i++) {                        
                        pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_FRONT], i);
                        pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_FRONT], i);
                    }
                } else {
                    pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_FRONT], 60);
                    pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_FRONT], 60);                    
                }

                break;

            case (SENSOR_LOWER_RIGHT | SENSOR_UPPER_RIGHT):
                pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_BACK], 0);
                pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_BACK], 0);

                if (strat_mask & ATTACK_GRADUAL) {
                    for (i = 60; i <= 100; i++) {                        
                        pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_FRONT], i);
                        pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_FRONT], i);
                    }
                } else {
                    pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_FRONT], 60);
                    pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_FRONT], 60);                    
                }
 
                break;

            case (SENSOR_UPPER_LEFT | SENSOR_MIDDLE):
                pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_BACK], 0);
                pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_BACK], 0);

                if (strat_mask & ATTACK_GRADUAL) {
                    for (i = 60; i <= 100; i++) {                        
                        pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_FRONT], i);
                        pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_FRONT], i);
                    }
                } else {
                    pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_FRONT], 60);
                    pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_FRONT], 60);                    
                }

                break;

            case (SENSOR_UPPER_RIGHT | SENSOR_MIDDLE):
                pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_BACK], 0);
                pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_BACK], 0);

                if (strat_mask & ATTACK_GRADUAL) {
                    for (i = 60; i <= 100; i++) {                        
                        pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_FRONT], i);
                        pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_FRONT], i);
                    }
                } else {
                    pwm_gpio_set_duty_cycle(&gpios[MOTOR_LEFT_FRONT], 60);
                    pwm_gpio_set_duty_cycle(&gpios[MOTOR_RIGHT_FRONT], 60);                    
                }

                break;

            default:
                stop();
            }
    }

    stop();
}