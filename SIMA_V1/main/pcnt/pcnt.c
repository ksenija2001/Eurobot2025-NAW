#include "pcnt.h"

void pcnt_unit_init(pcnt_unit_handle_t *pcnt, pcnt_unit_config_t *conf, int high_limit, int low_limit, int intr_priority)
{
    conf->high_limit = high_limit;
    conf->low_limit = low_limit;
    conf->intr_priority = intr_priority;

    ESP_ERROR_CHECK(pcnt_new_unit(conf, pcnt));
}

void pcnt_channel_init(pcnt_unit_handle_t pcnt, pcnt_chan_config_t *conf, pcnt_channel_handle_t *chan, int edge_gpio, int level_gpio)
{
    conf->edge_gpio_num = edge_gpio;
    conf->level_gpio_num = level_gpio;

    ESP_ERROR_CHECK(pcnt_new_channel(pcnt, conf, chan));
}

void pcnt_set_pin_actions(pcnt_channel_handle_t chan)
{
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
}

void pcnt_enable_clear_start(pcnt_unit_handle_t pcnt)
{
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt));
}


void disable_all_pcnts(pcnt_unit_handle_t pcnt1, pcnt_unit_handle_t pcnt2)
{
    pcnt_unit_disable(pcnt1);
    pcnt_unit_disable(pcnt2);
}