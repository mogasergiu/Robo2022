#include "hardware/gpio.h"

#ifndef GPIO_H
#define GPIO_H

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

typedef struct {
    uint num;
    enum gpio_irq_level irq_type;
    enum gpio_function func;
    bool is_irq;
    bool is_out;
} gpio_t;

#endif  /* GPIO_H */