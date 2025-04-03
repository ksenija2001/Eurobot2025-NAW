#ifndef GPIO_H
#define GPIO_H

#include "driver/gpio.h"

void init_gpio(gpio_mode_t mode, gpio_int_type_t intr, gpio_num_t pin, gpio_pulldown_t pulldown, gpio_pullup_t pullup);

void gpio_set(gpio_num_t pin);
void gpio_reset(gpio_num_t pin);

#endif // GPIO_H