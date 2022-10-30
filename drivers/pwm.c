#include <pwm.h>

static inline uint map_get(uint val, uint low_from, uint high_from,
                           uint low_to, uint high_to)
{
    return (val - low_from) *
           (high_to - low_to) / (high_from - low_from) + low_to;
}

void pwm_gpio_enable(gpio_t *gpio)
{
    uint slice_num, channel;

    slice_num = pwm_gpio_to_slice_num(gpio->num);
    channel = pwm_gpio_to_channel(gpio->num);

    pwm_set_wrap(slice_num, DEFAULUT_PWM_WRAP);
    pwm_set_chan_level(slice_num, channel, 0);
    pwm_set_clkdiv(slice_num, 46.f);
    pwm_set_enabled(slice_num, true);
}

void pwm_gpio_set_duty_cycle(gpio_t *gpio, uint percent)
{
    uint slice_num, channel, percent_to_wrap;

    slice_num = pwm_gpio_to_slice_num(gpio->num);
    channel = pwm_gpio_to_channel(gpio->num);
    percent_to_wrap = map_get(percent, 0, 100, 0, DEFAULUT_PWM_WRAP);
    
    pwm_set_chan_level(slice_num, channel, percent_to_wrap);
}
