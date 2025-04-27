#ifndef MAIN_H
#define MAIN_H

#include <stdio.h>
#include <stdint.h>
#include "gptimer/gptimer.h"
#include "mcpwm/mcpwm.h"
#include "motor_control/motor_control.h"
#include "pcnt/pcnt.h"
#include "microswitch/microswitch.h"
#include "control/control.h"
#include "wifi/wifi/wifi.h"
#include "wifi/socket/socket.h"
#include "wifi/credentials/credentials.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "esp_log.h"

#define WHEEL_CIRC 43.0
#define WHEEL_DIFF 83.0 //83.0 na malom
#define CALIBRATION 680.0  //350.0, obicna, 692 velika/mzd 680

#define MR_A 7
#define MR_B 6
#define ML_A 4
#define ML_B 5
#define RIGHT_ENC_EDGE 41
#define RIGHT_ENC_LEVEL 42
#define LEFT_ENC_EDGE 38
#define LEFT_ENC_LEVEL 40
#define MICROSWITCH 21
#define SERVO_1 18
#define SERVO_2 17

#define UNIT_0 MCPWM_UNIT_0
#define UNIT_1 MCPWM_UNIT_1
#define TIMER_0 MCPWM_TIMER_0
#define TIMER_1 MCPWM_TIMER_1
#define TIMER_2 MCPWM_TIMER_2
#define GPIO_IN GPIO_MODE_INPUT
#define GPIO_OUT GPIO_MODE_OUTPUT
#define GPIO_PU_EN GPIO_PULLUP_ENABLE
#define GPIO_PU_DIS GPIO_PULLUP_DISABLE
#define GPIO_PD_EN GPIO_PULLDOWN_ENABLE
#define GPIO_PD_DIS GPIO_PULLDOWN_DISABLE

typedef struct {
    float x;
    float y;
    float teta;
} coord_t;

typedef enum {
    ROBOT_WAIT_FOR_SIGNAL,
    ROBOT_MOVE,
    ROBOT_CHECK,
    ROBOT_SERVO,
    ROBOT_AVOID
} robot_states_e;

#endif  //MAIN_H