#include "control.h"

static float vr_inc;
static float vl_inc;
static float mm2inc;
static float ref_new_r, ref_new_l;
float v_left, v_right;
bool stop_condition;
float vr, vl;
robot_move_states_e state;
bool robot_rotated = false;
bool robot_moved = false;
bool finished_movement = false;
float angle_count = 0.0;
float angle_count_deg = 0.0;
float angle_of_rot = 0.0;
float target_dist = 0.0;
float dist = 0.0;
float start_x, start_y;
float last_u_left, last_u_right;
float last_target_speed = 0;
void odom_init(float r, float d, float calib, position_t *robot)
{
    robot->d = d;
    float obim = r*PI;
    mm2inc = obim/calib;
}

void calc_pos(encoder_info_queue_t *enc, position_t *robot)
{
    vr_inc = vr_inc*0.7 + 0.3*(enc->right - enc->prev_right);
    vl_inc = vl_inc*0.7 + 0.3*(enc->left - enc->prev_left);
    //ESP_LOGI("Enk", "VRINC: %d VINC: %d", vr_inc, vl_inc);
    enc->prev_right = enc->right;
    enc->prev_left = enc->left;
    vr = vr_inc*mm2inc;
    vl = vl_inc*mm2inc;
    robot->ang = (vl - vr)/robot->d;
    float lin_speed = (vr+vl)/2.0;
    robot->angle_in_rad += robot->ang;
    robot->angle_in_deg = robot->angle_in_rad * 180.0/PI;
    if (robot->angle_in_deg > 180.0) robot->angle_in_deg -= 360.0;
    else if (robot->angle_in_deg < -180.0) robot->angle_in_deg += 360.0;
    robot->x_pos += lin_speed*cos(robot->angle_in_rad);
    robot->y_pos += lin_speed*sin(robot->angle_in_rad);
    ESP_LOGI("Robot", "X: %f Y: %f Rad: %f Deg: %f ENKR: %d, ENKL: %d", robot->x_pos, robot->y_pos, robot->angle_in_rad, robot->angle_in_deg, enc->right, enc->left);
}

float calc_angle(float target_x, float target_y, position_t *robot)
{
    float x = target_x - robot->x_pos;
    float y = target_y - robot->y_pos;
    float angle_rotate = atan2(y, x);
    float angle_in_deg = angle_rotate * 180.0/PI;
    float angle_of_rotation = angle_in_deg - robot->angle_in_deg;
    if (angle_of_rotation > 180.0) angle_of_rotation -= 360.0;
    else if (angle_of_rotation < -180.0) angle_of_rotation += 360.0;
    return angle_of_rotation;
}

void speed_control(float target_x, float target_y, float target_speed, position_t *robot)
{
    // static float e_lin, e_ang, v_lin, v_ang;
    // float Kp_lin = target_speed/(0.20*target_dist);
    // static float Kp_ang = 0.1;
    // float target_angle = atan2(target_y, target_x);
    // e_lin = sqrt(pow(target_x - robot->x_pos, 2) + pow(target_y - robot->y_pos, 2));
    // e_ang = target_angle - robot->angle_in_rad;
    //v_lin = Kp_lin*(target_dist-dist);
    float distance_error = ((target_y - start_y)*robot->x_pos-(target_x - start_x)*robot->y_pos + target_x*start_y - target_y*start_x)/target_dist;
    ESP_LOGW("distance error", "%f", distance_error);

    if(target_speed>last_target_speed + 1) last_target_speed += 1;
    
    v_left = last_target_speed + distance_error * 0.05;
    v_right = last_target_speed - distance_error * 0.05;

    if(v_left > 50.0) v_left = 50.0;
    if(v_left < 0.0) v_left = 0.0;
    if(v_right > 50.0) v_right = 50.0;
    if(v_right < 0.0) v_right = 0.0;
    ESP_LOGW("V", "%f, %f", v_left, v_right);


    // if (v_left > target_speed) v_left = target_speed;
    // else if (v_left < MIN_SPEED) v_left = MIN_SPEED;
    // if (v_right > target_speed) v_right = target_speed;
    // else if (v_right < MIN_SPEED) v_right = MIN_SPEED;
    //ESP_LOGI("Pos", "EL: %f, EA: %f VR: %f, VL: %f", e_lin, e_ang, v_right, v_left);
}

float pid_l(motor_t motor, float target)
{
    static float kp = 2.0;    //7.75 za mali (8.2), 7.7 za veliki (~8.05)
    static float ki = 0.001; //1.9 za mali(1.75), 1.6 za veliki (~1.54)
    static float kd = 0.001; //0.52 za mali, 0.32 za veliki
    // if (target > ref_new_l) ref_new_l += 0.1;
    // else if (target < ref_new_l) ref_new_l -= 0.1;
    static float ep, ei = 0, ed, u, ep_prev;
    ep = (target - vl_inc) * kp;
    ei += ep * ki;
    // ed = ep - ep_prev;
    // ep_prev = ep;
    // u = kp*ep + ki*ei + kd*ed;
    u = ep + ei;
    // if(u > last_u_left + 0.1) u = last_u_left + 0.1;
    // if(u < last_u_left - 0.1) u = last_u_left - 0.1;
    // last_u_left = u;
    ESP_LOGI("Left", "%f   %f", u, target);
    if (u < 0.0) u = 0.0;
    else if (u > 95.0) u = 95.0;
    //motor_forward(motor, u);
    return u;
}

float pid_r(motor_t motor, float target)
{
    static float kp = 2.0;  //8.0 za veliki (~7.94), 7.9 za mali (7.6)
    static float ki = 0.001; //1.75 (1.4)
    static float kd = 0.001;
    // if (target > ref_new_r) ref_new_r += 0.1;
    // else if (target < ref_new_r) ref_new_r -= 0.1;
    static float ep, ei = 0, ed, u, ep_prev;
    ep = (target - vr_inc) * kp;
    ei += ep * ki;
    // ed = ep - ep_prev;
    // ep_prev = ep;
    // u = kp*ep + ki*ei + kd*ed;
    u = ep + ei;
    // if(u > last_u_right + 0.1) u = last_u_right + 0.1;
    // if(u < last_u_right - 0.1) u = last_u_right - 0.1;
    // last_u_right = u;
    ESP_LOGI("Right", "%f   %f", u, target);
    if (u < 0.0) u = 0.0;
    else if (u > 95.0) u = 95.0;
    //motor_forward(motor, u);
    return u;
}

void rotate_right(float target_angle, encoder_info_queue_t *enc, position_t *robot, motor_t right_motor, motor_t left_motor)
{
    //ESP_LOGI("Funk", "Rotate right");
    motor_back(right_motor, 24.0);
    motor_forward(left_motor, 27.0);
    if (angle_count_deg <= target_angle)
    {
        calc_pos(enc, robot);
        angle_count += robot->ang;
        angle_count_deg = angle_count * 180/PI;
        //vTaskDelay(10/portTICK_PERIOD_MS);
        robot_rotated = false;
    }
    else
    {
        robot_rotated = true;
    }
}

void rotate_left(float target_angle, encoder_info_queue_t *enc, position_t *robot, motor_t right_motor, motor_t left_motor)
{
    //ESP_LOGI("Funk", "Rotate Left");
    motor_back(left_motor, 27.0);
    motor_forward(right_motor, 24.0);
    if (angle_count_deg <= target_angle)
    {
        calc_pos(enc, robot);
        angle_count += fabs(robot->ang);
        angle_count_deg = angle_count * 180/PI;
        ESP_LOGI("Ang", "Ang: %f Target: %f", angle_count_deg, target_angle);
        vTaskDelay(10/portTICK_PERIOD_MS);
        robot_rotated = false;
    }
    else
    {
        robot_rotated = true;
    }
}

void go_to_xy(float target_x, float target_y, float speed, encoder_info_queue_t *enc, position_t *robot, motor_t right_motor, motor_t left_motor, pcnt_unit_handle_t pcnt1, pcnt_unit_handle_t pcnt2)
{
    switch (state)
    {
        case ROBOT_CALCS:
            //ESP_LOGI("State", "U robot calcs stejtu");
            reset_values();
            start_x = robot->x_pos;
            start_y = robot->y_pos;
            angle_of_rot = calc_angle(target_x, target_y, robot);
            target_dist = sqrt(pow(target_x-robot->x_pos, 2) + pow(target_y-robot->y_pos, 2));
            state = ROBOT_ROTATE;
        break;
        case ROBOT_ROTATE:
            //ESP_LOGI("Calcs", "Ang: %f Dist: %f", angle_of_rot, target_dist);
            if (angle_of_rot > 0.0) rotate_right(angle_of_rot, enc, robot, right_motor, left_motor);
            else if (angle_of_rot < 0.0) rotate_left(fabs(angle_of_rot), enc, robot, right_motor, left_motor);
            if (robot_rotated)
            {
                state = ROBOT_STOP;
            }
        break;
        case ROBOT_MOVE_STRAIGHT:
            //ESP_LOGI("Straight", "Robot se krece pravo");
            move_straight(target_x, target_y, speed, enc, robot, right_motor, left_motor);
            if (robot_moved)
            {
                state = ROBOT_STOP;
            }
        break;
        case ROBOT_STOP:
            motors_stop(right_motor, left_motor);
            disable_all_pcnts(pcnt1, pcnt2);
            encoder_reset(enc);
            ESP_LOGI("Stop", "Robot stoji");
            vTaskDelay(200/portTICK_PERIOD_MS);
            pcnt_enable_clear_start(pcnt1);
            pcnt_enable_clear_start(pcnt2);
            if (!robot_moved) state = ROBOT_MOVE_STRAIGHT;
            else if (robot_rotated && robot_moved)
            {
                //motors_stop(right_motor, left_motor);
                finished_movement = true;
                state = ROBOT_CALCS;
            }
        break;
    }
}

void move_straight(float target_x, float target_y, float speed, encoder_info_queue_t *enc, position_t *robot, motor_t right_motor, motor_t left_motor)
{
    if (dist < target_dist)
    {
        //ESP_LOGI("Move_staright", "U while si");
        calc_pos(enc, robot);
        dist += (vr+vl)/2.0;
        speed_control(target_x, target_y, speed, robot);
        float speed_l = pid_l(left_motor, v_left);
        float speed_r = pid_r(right_motor, v_right);
        motor_forward(right_motor, speed_r);
        motor_forward(left_motor, speed_l);
        //ESP_LOGI("Dist", "%f", dist);
        //vTaskDelay(10/portTICK_PERIOD_MS);
    }
    else
    {
        robot_moved = true;
    }
}

void encoder_reset(encoder_info_queue_t *enc)
{
    enc->left = 0;
    enc->right = 0;
    enc->prev_left = 0;
    enc->prev_right = 0;
}

void set_start_position(float x, float y, float teta, position_t *robot)
{
    robot->x_pos = x;
    robot->y_pos = y;
    robot->angle_in_deg = teta;
}

void reset_values()
{
    finished_movement = false;
    robot_rotated = false;
    robot_moved = false;
    angle_count = 0.0;
    dist = 0.0;
    ref_new_l = 0.0;
    ref_new_r = 0.0;
}