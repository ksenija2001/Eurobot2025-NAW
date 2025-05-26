#include "microswitch.h"

void microswitch_init(gpio_config_t *conf, gpio_int_type_t intr, gpio_mode_t mode, uint64_t gpio, gpio_pullup_t pu_en, gpio_pulldown_t pd_en)
{
    conf->intr_type = intr;
    conf->mode = mode;
    conf->pin_bit_mask = 1<<gpio;
    conf->pull_down_en = pd_en;
    conf->pull_up_en = pu_en;

    ESP_ERROR_CHECK(gpio_config(conf));
}

void microswitch_interrupt(int intr, gpio_num_t gpio, gpio_isr_t isr_handler)
{
    //ESP_ERROR_CHECK(gpio_install_isr_service(intr));
    ESP_ERROR_CHECK(gpio_isr_handler_add(gpio, isr_handler, (void *)gpio));
}