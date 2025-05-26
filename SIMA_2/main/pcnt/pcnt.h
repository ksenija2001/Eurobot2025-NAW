#ifndef PCNT_H
#define PCNT_H

#include <stdio.h>
#include <stdint.h>
#include "driver/pulse_cnt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

typedef struct {
    pcnt_unit_handle_t pcnt_handle;
    pcnt_unit_config_t pcnt_conf;
    pcnt_channel_handle_t pcnt_chan;
    pcnt_chan_config_t pcnt_chan_conf;
} pcnt_s;

extern pcnt_s pulse_counter1, pulse_counter2;

void pcnt_unit_init(pcnt_unit_handle_t *pcnt, pcnt_unit_config_t *conf, int high_limit, int low_limit, int int_priority);
void pcnt_channel_init(pcnt_unit_handle_t pcnt, pcnt_chan_config_t *conf, pcnt_channel_handle_t *chan, int edge_gpio, int level_gpio);
void pcnt_set_pin_actions(pcnt_channel_handle_t chan);
void pcnt_enable_clear_start(pcnt_unit_handle_t pcnt);
void disable_all_pcnts(pcnt_unit_handle_t pcnt1, pcnt_unit_handle_t pcnt2);

#endif //PCNT_H