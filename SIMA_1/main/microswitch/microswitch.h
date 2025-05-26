#ifndef MICROSWITCH_H
#define MICROSWITCH_H

#include <stdio.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

void microswitch_init(gpio_config_t *conf, gpio_int_type_t intr, gpio_mode_t mode, uint64_t gpio, gpio_pullup_t pu_en, gpio_pulldown_t pd_en);
void microswitch_interrupt(int intr, gpio_num_t gpio, gpio_isr_t isr_handler);

#endif //MICROSWITCH_H