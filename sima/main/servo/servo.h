#ifndef SERVO_H
#define SERVO_H

#define DEBUG_SERVO
#define LOW_DEBUG_SERVO_LEVEL 0
#define HIGH_DEBUG_SERVO_LEVEL 1

#if defined(DEBUG_SERVO)
    #define SERVO_TAG "Servo"

    #include "esp_log.h"
#endif

#if !defined(DEBUG_SERVO_LEVEL)

    #define DEBUG_SERVO_LEVEL HIGH_SERVO_DEBUG_LEVEL

#endif

#include "credentials.h"
#include "driver/ledc.h"

// #define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
// #define LEDC_OUTPUT_IO          (17) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
// #define LEDC_FREQUENCY          (50) // Frequency in Hertz. Set frequency at 4 kHz

#if LEDC_DUTY_RES == LEDC_TIMER_13_BIT
    #define SERVO_DUTY_MASK 0b1111111111111
#endif

#define SERVO_MIN_DUTY_PERCENT 2.5
#define SERVO_MAX_DUTY_PERCENT 12.5

#if SIMA_ID == 1
    #define SERVO_ANGLE_ORT 90
    #define SERVO_ANGLE_ORT_INV -87

    #define SERVO_ANGLE_ZERO -14
    #define SERVO_ANGLE_ZERO_INV 14

#elif SIMA_ID == 2
    #define SERVO_ANGLE_ORT 90
    #define SERVO_ANGLE_ORT_INV -90

    #define SERVO_ANGLE_ZERO -8
    #define SERVO_ANGLE_ZERO_INV 8

#elif SIMA_ID == 3
    #define SERVO_ANGLE_ORT 80
    #define SERVO_ANGLE_ORT_INV -90

    #define SERVO_ANGLE_ZERO -25
    #define SERVO_ANGLE_ZERO_INV 8

#elif SIMA_ID == 4
    #define SERVO_ANGLE_ORT 82
    #define SERVO_ANGLE_ORT_INV -88

    #define SERVO_ANGLE_ZERO -25
    #define SERVO_ANGLE_ZERO_INV 18

#else
    #define SERVO_ANGLE_ORT 90
    #define SERVO_ANGLE_ORT_INV -90

    #define SERVO_ANGLE_ZERO 0
    #define SERVO_ANGLE_ZERO_INV 0
#endif

typedef ledc_channel_t servo_channel;
typedef ledc_timer_t servo_timer;
typedef gpio_num_t servo_pin;

typedef struct {
    servo_channel channel;
    servo_timer timer;

    servo_pin pin;
    uint64_t freq;

    float max_duty;
    float min_duty;
    float zero;

    float angle_open;
    float angle_closed;

    float angle;

    float duty;
    float duty_percent;
} Servo;

// 0.075
// 0.125 - moja krajnja je 0.117
// 0.025

void init_servo(Servo* servo, servo_pin gpio, servo_timer timer, servo_channel channel, uint64_t freq, float max_duty, float min_duty);
void servo_move(Servo* servo, float angle);

void servo_wave(Servo* servo);

#endif