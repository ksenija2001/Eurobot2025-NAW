#include "gptimer.h"
#include "driver/gptimer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

void gptimer_init(gptimer_handle_t *gptimer, gptimer_config_t *conf, gptimer_clock_source_t clk_src, gptimer_count_direction_t dir, uint32_t res, int intr_priority)
{
    conf->clk_src = clk_src;
    conf->direction = dir;
    conf->resolution_hz = res;
    conf->intr_priority = intr_priority;

    ESP_ERROR_CHECK(gptimer_new_timer(conf, gptimer));
}

void gptimer_alarm_init(gptimer_handle_t gptimer, gptimer_alarm_config_t *alarm, uint64_t count, uint64_t reload, bool auto_reload)
{
    alarm->alarm_count = count;
    alarm->reload_count = reload;
    alarm->flags.auto_reload_on_alarm = auto_reload;

    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, alarm));
}

void gptimer_cbs_init(gptimer_handle_t gptimer, gptimer_event_callbacks_t *cbs, gptimer_alarm_cb_t callback_function, void *user_data)
{
    cbs->on_alarm = callback_function;

    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, cbs, user_data));
}

void gptimer_enable_and_start(gptimer_handle_t gptimer)
{
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    ESP_ERROR_CHECK(gptimer_start(gptimer));
}
