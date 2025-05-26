#ifndef MCPWM_H
#define MCPWM_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/mcpwm.h"
#include "driver/mcpwm_types.h"
#include "driver/mcpwm_types_legacy.h"

typedef struct {
    mcpwm_unit_t unit;
    mcpwm_timer_t timer;
} motor_t;


/*  mcpwm unit setup - Function that sets up a mcpwm unit

    Parameters: 
                1. mcpwm_config_t *conf -> a pointer to the configuration object of mcpwm
                2. mcpwm_counter_type_t count   -> defines if the timer counts up or down
                                                0: MCPWM_FREEZE_COUNTER
                                                1: MCPWM_UP_COUNTER
                                                2: MCPWM_DOWN_COUNTER
                                                3: MCPWM_UP_DOWN_COUNTER
                                                4: MCPWM_COUNTER_MAX
                3. uint16_t freq    -> frequency of the module in Hz
*/
void mcpwm_unit_setup(mcpwm_config_t *conf, mcpwm_counter_type_t count, uint16_t freq);

/*  motor_init - Function that initiates a motor on a ceratin mcpwm unit

    Parameters:
                1. mcpwm_config_t *conf -> a pointer to a configuration object
                2. mcpwm_unit_t unit    -> a mcpwm unit that is being used
                                        0: MCPWM_UNIT_0
                                        1: MCPWM_UNIT_1 (or MCPWM_UNIT_MAX)
                3. mcpwm_timer_t timer  -> each mcpwm unit has 3 timers
                                        0: MCPWM_TIMER_0
                                        1: MCPWM_TIMER_1
                                        2: MCPWM_TIMER_2
                4. uint8_t gpio1    -> pin 1 of the motor
                5. uint8_t gpio2    -> pin 2 of the motor
*/
void motor_init(motor_t *motor,mcpwm_config_t *conf, mcpwm_unit_t unit, mcpwm_timer_t timer, uint8_t gpio1, uint8_t gpio2);

#endif  //MCPWM_H