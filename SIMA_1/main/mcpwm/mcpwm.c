#include "mcpwm.h"

void mcpwm_unit_setup(mcpwm_config_t *conf, mcpwm_counter_type_t count, uint16_t freq)
{
    conf->cmpr_a = 0;
    conf->cmpr_b = 0;
    conf->counter_mode = count;
    conf->duty_mode = MCPWM_DUTY_MODE_0;
    conf->frequency = freq;
}

void motor_init(motor_t *motor,mcpwm_config_t *conf, mcpwm_unit_t unit, mcpwm_timer_t timer, uint8_t gpio1, uint8_t gpio2)
{
	if (timer == MCPWM_TIMER_0)
	{
		mcpwm_gpio_init(unit, MCPWM0A, gpio1);
		mcpwm_gpio_init(unit, MCPWM0B, gpio2);
		printf("Timer 0\n");
	}
	else if (timer == MCPWM_TIMER_1)
	{
		mcpwm_gpio_init(unit, MCPWM1A, gpio1);
		mcpwm_gpio_init(unit, MCPWM1B, gpio2);	
		printf("Timer 1\n");
	}
	else
	{
		mcpwm_gpio_init(unit, MCPWM2A, gpio1);
		mcpwm_gpio_init(unit, MCPWM2B, gpio2);
		printf("Timer 2\n");
	}
	
	mcpwm_init(unit, timer, conf);

	motor->timer = timer;
	motor->unit = unit;
	
	mcpwm_set_signal_low(unit, timer, MCPWM_OPR_A);
	mcpwm_set_signal_low(unit, timer, MCPWM_OPR_B);
}