#ifndef CONTROL_H
#define CONTROL_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "math.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "../motor_control/motor_control.h"
#include "../pcnt/pcnt.h"
#include "../wifi/credentials/credentials.h"
#include "../tof/tof.h"
#include "../wifi/socket/socket.h"
#include "../utils/utils.h"

#define PI 3.14159265358979323846
#define MAX_SPEED 20.0
#define MIN_SPEED 4.0   //7.0 za male

extern const char* TAG_Control;

extern float v_right;
extern float v_left;
extern bool finished_movement;
extern bool avoidance_complete;
extern bool robot_rotated;

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

typedef struct {
    float x_min;
    float x_max;
    float y_min;
    float y_max;
} target_zone_t;

typedef enum {
    AVOID_SET_TARGETS,
    AVOID_ROTATE,
    AVOID_MOVE_TO_POS,
    AVOID_CHECK
} avoid_states_e;

extern const target_zone_t zone1;
extern const target_zone_t zone2;
extern const target_zone_t zone3;
extern const target_zone_t forbidden_zone;

extern robot_move_states_e state;
extern avoid_states_e a_state;

extern uint32_t counter;

void odom_init(float r, float d, float calib, position_t *robot);
void calc_pos(encoder_info_queue_t *enc, position_t *robot);
float calc_angle(float target_x, float target_y, position_t *robot);
void speed_control(float target_x, float target_y, float target_speed, position_t *robot);
void rotate_control(float current_angle, float target_angle, float target_speed);
float pid_r(motor_t motor, float target);   //Bilo void
float pid_l(motor_t motor, float target);
void rotate_right(float target_angle, encoder_info_queue_t *enc, position_t *robot, motor_t right_motor, motor_t left_motor);
void rotate_left(float target_angle, encoder_info_queue_t *enc, position_t *robot, motor_t right_motor, motor_t left_motor);
void go_to_xy(float target_x, float target_y, float speed, encoder_info_queue_t *enc, position_t *robot, motor_t right_motor, motor_t left_motor, pcnt_unit_handle_t pcnt1, pcnt_unit_handle_t pcnt2, Socket rpi_socket);
void move_straight(float target_x, float target_y, float speed, encoder_info_queue_t *enc, position_t *robot, motor_t right_motor, motor_t left_motor);
void encoder_reset(encoder_info_queue_t *enc);
void set_start_position(float x, float y, float teta, position_t *robot);
void reset_values();
target_zone_t set_target_zone(int target_zone);
bool check_zone(position_t robot, target_zone_t zone);
void avoid_obstacle(position_t *robot, encoder_info_queue_t *enc, motor_t right_motor, motor_t left_motor, pcnt_unit_handle_t pcnt1, pcnt_unit_handle_t pcnt2, VL53LMZ_Result_t data, VL53LMZ_Interrupt_Zone zone, Socket socket);
void store_position(position_t *robot, position_t *temp_robot);
void restore_position(position_t *robot, position_t *temp_robot);

#endif //CONTROL_H