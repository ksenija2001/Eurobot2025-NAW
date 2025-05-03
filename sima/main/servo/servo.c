#include "servo.h"

void init_servo(Servo* servo, servo_pin gpio, servo_timer timer, servo_channel channel, uint64_t freq, float max_duty, float min_duty){
   ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = timer,
        .freq_hz          = freq,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = channel,
        .timer_sel      = timer,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = gpio,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);

    servo->pin = gpio;
    servo->freq = freq;
    servo->timer = timer;
    servo->channel = channel;
    servo->max_duty = max_duty;
    servo->min_duty = min_duty;
}

void servo_move(Servo* servo, float angle){
    if(angle > 90) return;
    if(angle < -90) return;

    servo->angle = angle;

    angle += 90;
    servo->duty = servo->min_duty + angle / 180.0 * (servo->max_duty - servo->min_duty);

    if(servo->duty > servo->max_duty) servo->duty = servo->max_duty;
    if(servo->duty < servo->min_duty) servo->duty = servo->min_duty;

    servo->duty_percent = servo->duty / 100.0;
    
    ledc_set_duty(LEDC_MODE, servo->channel, servo->duty_percent * SERVO_DUTY_MASK);
    ledc_update_duty(LEDC_MODE, servo->channel);
}

void servo_wave(Servo* servo){
    if(servo->angle == servo->angle_closed) servo_move(servo, servo->angle_open);
    else servo_move(servo, servo->angle_closed);
}