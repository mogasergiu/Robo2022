#include "hardware/pwm.h"
#include <gpio.h>

#ifndef PWM_H
#define PWM_H

#define DEFAULUT_PWM_WRAP 255

void pwm_gpio_enable(gpio_t *gpio);

void pwm_gpio_set_duty_cycle(gpio_t *gpio, uint percent);

#endif  /* PWM_H */