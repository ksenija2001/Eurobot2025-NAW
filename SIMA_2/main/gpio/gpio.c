#include "gpio.h"

void init_gpio(gpio_mode_t mode, gpio_int_type_t intr, gpio_num_t pin, gpio_pulldown_t pulldown, gpio_pullup_t pullup){
    
    #if defined(DEBUG_GPIO) && DEBUG_GPIO_LEVEL == HIGH_DEBUG_GPIO_LEVEL
        ESP_LOGI(GPIO_TAG, "Initializing GPIO pin %d", pin);
        // print param for gpio ?
    #endif
    
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
    #if defined(DEBUG_GPIO) && DEBUG_GPIO_LEVEL == LOW_DEBUG_GPIO_LEVEL
        ESP_LOGI(GPIO_TAG, "Setting GPIO pin %d to HIGH", pin);
    #endif
    gpio_set_level(pin, 1);
}
void gpio_reset(gpio_num_t pin){
    #if defined(DEBUG_GPIO) && DEBUG_GPIO_LEVEL == LOW_DEBUG_GPIO_LEVEL
        ESP_LOGI(GPIO_TAG, "Setting GPIO pin %d to LOW", pin);
    #endif
    gpio_set_level(pin, 0);
}