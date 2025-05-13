#ifndef CONTROL_H
#define CONTROL_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "math.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "../motor_control/motor_control.h"
#include "../pcnt/pcnt.h"

#define PI 3.14159265358979323846
#define MAX_SPEED 20.0
#define MIN_SPEED 4.0   //7.0 za male

extern const char* TAG_Control;

extern float v_right;
extern float v_left;
extern bool finished_movement;

typedef struct {
    float d;
    float x_pos;
    float y_pos;
    float ang;
    float angle_in_rad;
    float angle_in_deg;
} position_t;

typedef struct {
	int right;
	int left;
    int prev_right;
    int prev_left;
} encoder_info_queue_t;

typedef enum {
    ROBOT_CALCS,
    ROBOT_STOP,
    ROBOT_MOVE_STRAIGHT,
    ROBOT_ROTATE
} robot_move_states_e;

extern robot_move_states_e state;

void odom_init(float r, float d, float calib, position_t *robot);
void calc_pos(encoder_info_queue_t *enc, position_t *robot);
float calc_angle(float target_x, float target_y, position_t *robot);
void speed_control(float target_x, float target_y, float target_speed, position_t *robot);
void rotate_control(float current_angle, float target_angle, float target_speed);
float pid_r(motor_t motor, float target);   //Bilo void
float pid_l(motor_t motor, float target);
void rotate_right(float target_angle, encoder_info_queue_t *enc, position_t *robot, motor_t right_motor, motor_t left_motor);
void rotate_left(float target_angle, encoder_info_queue_t *enc, position_t *robot, motor_t right_motor, motor_t left_motor);
void go_to_xy(float target_x, float target_y, float speed, encoder_info_queue_t *enc, position_t *robot, motor_t right_motor, motor_t left_motor, pcnt_unit_handle_t pcnt1, pcnt_unit_handle_t pcnt2);
void move_straight(float target_x, float target_y, float speed, encoder_info_queue_t *enc, position_t *robot, motor_t right_motor, motor_t left_motor);
void encoder_reset(encoder_info_queue_t *enc);
void set_start_position(float x, float y, float teta, position_t *robot);
void reset_values();

#endif //CONTROL_H