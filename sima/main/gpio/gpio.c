#include "gpio.h"

void init_gpio(gpio_mode_t mode, gpio_int_type_t intr, gpio_num_t pin, gpio_pulldown_t pulldown, gpio_pullup_t pullup){
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};

    io_conf.mode = mode;
    io_conf.pin_bit_mask = 0 | (1 << pin);

    io_conf.intr_type = intr;

    io_conf.pull_down_en = pulldown;
    io_conf.pull_up_en = pullup;
    
    //configure GPIO with the given settings
    gpio_config(&io_conf);
}

void gpio_set(gpio_num_t pin){
    gpio_set_level(pin, 1);
}
void gpio_reset(gpio_num_t pin){
    gpio_set_level(pin, 0);
}