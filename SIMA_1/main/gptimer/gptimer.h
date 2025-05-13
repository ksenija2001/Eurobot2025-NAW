#ifndef GPTIMER_H
#define GPTIMER_H

#include <stdbool.h>
#include <stdint.h>
#include "driver/gptimer.h"

/*  gptimer_init - Function that initiates the general purpose timer with provided configuration
    
    Parameters: 
                1. gptimer_handle_t *gptimer    -> a pointer to the gptimer instance that is being initialized
                2. gptimer_config_t *conf   -> a pointer to the gptimer configuration object
                3. gptimer_clock_source_t clk_src   -> choosing a clock source
                                                    -> there are different options, but always choose GPTIMER_CLK_SRC_DEFAULT (uses the default source, usually APB - 80MHz)
                4. gptimer_count_direction_t dir    -> sets the direction of the clock counter
                                                    0: GPTIMER_COUNT_DOWN
                                                    1: GPTIMER_COUNT_UP
                5. uint32_t res -> sets the resolution (working frequency) in Hz, meaning every step will be 1/frequency seconds
                6. int intr_priority    -> sets the priority of the interrupt
                                        -> setting it to 0 means that the compiler will try to give it a low priority (1, 2, 3)
*/
void gptimer_init(gptimer_handle_t *gptimer, gptimer_config_t *conf, gptimer_clock_source_t clk_src, gptimer_count_direction_t dir, uint32_t res, int intr_priority);

/*  gptimer_alarm_init - Function that sets an alarm action. That is an action that happens when gptimer counter reaches a certain value

    Parameters:
                1. gptimer_handle_t gptimer -> a general purpose timer that the alarm action is being mapped to
                2. gptimer_alarm_config_t *alarm    -> a pointer to the configuration of the alarm
                3. uint64_t count   -> an alarm action happens when the counter count reaches this value
                4. uint64_t reload  -> after alarm activation, the counter resets to this value
                5. bool auto_reload -> This value needs to be set to 1 in order for the reload to happen after the alarm event
*/
void gptimer_alarm_init(gptimer_handle_t gptimer, gptimer_alarm_config_t *alarm, uint64_t count, uint64_t reload, bool auto_reload);

/*  gptimer_cbs_init - Function that connects the desired callback function with the gp timer

    Parameters:
                1. gptimer_handle_t gptimer -> a general purpose timer that is used
                2. gptimer_event_callbacks_t *cbs   -> a pointer to the callbacks type that defines that the callback is happening on the alarm
                3. gptimer_alarm_cb_t   -> a callback function that activates when the alarm event happens
                4. void *user_data  -> added user context that could for example define types that the data is being stored as or queues in which something acquired in the callback is placed
*/
void gptimer_cbs_init(gptimer_handle_t gptimer, gptimer_event_callbacks_t *cbs, gptimer_alarm_cb_t callback_function, void *user_data);

/*  gptimer_enable_and_start - Function that enables and starts the desired gptimer

    Parameters:
                1. gptimer_handle_t gptimer -> the timer that is being used
*/
void gptimer_enable_and_start(gptimer_handle_t gptimer);

#endif //GPTIMER_H