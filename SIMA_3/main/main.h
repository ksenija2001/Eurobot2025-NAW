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
#include "i2c/i2c.h"
#include "tof/tof.h"
#include "tof/point_cloud.h"
#include "utils/utils.h"
#include "spline/spline.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "esp_log.h"

#define I2C_TOF_ADDRESS 0x52
#define TOF_NUMBER_OF_ZONES VL53LMZ_RESOLUTION_4X4

#define WHEEL_CIRC 43.0
#define WHEEL_DIFF 84.0 //83.0 na malom
#define CALIBRATION 356.0  //350.0, obicna, 692 velika/mzd 680

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

#define WIFI_USED 1
#define DETECTION 1
#define AVOIDANCE 1

typedef struct {
    float x_target[3];
    float y_target[3];
    float theta_target[3];
    uint8_t num_of_bezier;
} spline_targets_t;

typedef struct {
    float x;
    float y;
    float teta;
    float speed;
} coord_t;

typedef enum {
    ROBOT_WAIT_FOR_SIGNAL,
    ROBOT_SETUP_TARGET,
    ROBOT_MOVING,
    ROBOT_CHECK,
    ROBOT_SERVO,
    ROBOT_STOPPED,
    ROBOT_AVOID,
    ROBOT_TEMP
} robot_states_e;

typedef struct {
    robot_states_e robot_states;
    bool tof_initialized;
    bool in_position;
    bool avoiding;
    uint64_t timer_count;
    uint64_t stop_time;
    motor_t right_motor;
    motor_t left_motor;
    target_zone_t target_zone;
} sima_t;

void app_main_2();
void setup_new_spline_targets(float *x, float *y, float *theta, uint8_t *bezier_num, position_t robot, coord_t *coordinates);

#endif  //MAIN_H