#include "control.h"

const char* TAG_Control = "Control";

static float vr_inc;
static float vl_inc;
float mm2inc;
static float ref_new_r, ref_new_l;
float v_left, v_right;
float v_ang_left, v_ang_right;
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
float prev_distance_error = 0.0;
float ei_left = 0.0;
float ei_right = 0.0;
float ep_prev_r = 0.0;
float ep_prev_l = 0.0;
float distance_error_i = 0.0;
float rotate_error_i = 0.0;
float prev_rotate_error = 0.0;
float last_rotate_speed = 0.0;
float distance_error;
float traj_kp = 0.025;  //0.025
float traj_kd = 1.0;    //1.0
float last_distance_error_traj;

void odom_init(float r, float d, float calib, position_t *robot)
{
    robot->d = d;
    float obim = r*PI;
    mm2inc = obim/calib;
    ESP_LOGI(TAG_Control, "D: %f R: %f Obim: %f, Cal: %f, MM2INC: %f", robot->d, r, obim, calib, mm2inc);
}

void calc_pos(encoder_info_queue_t *enc, position_t *robot)
{
    vr_inc = vr_inc*0.7 + 0.3*(enc->right - enc->prev_right);
    vl_inc = vl_inc*0.7 + 0.3*(enc->left - enc->prev_left);
    // vr_inc = enc->right - enc->prev_right;
    // vl_inc = enc->left - enc->prev_left;
    ESP_LOGI(TAG_Control, "ER: %d EL: %d VRINC: %f VINC: %f", enc->right, enc->left ,vr_inc, vl_inc);
    enc->prev_right = enc->right;
    enc->prev_left = enc->left;
    vr = vr_inc*mm2inc;
    vl = vl_inc*mm2inc;
    robot->ang = (vl - vr)/robot->d;
    float lin_speed = (vr + vl)/2.0;
    robot->angle_in_rad += robot->ang;
    robot->angle_in_deg = robot->angle_in_rad * 180.0/PI;
    if (robot->angle_in_deg > 180.0) robot->angle_in_deg -= 360.0;
    else if (robot->angle_in_deg < -180.0) robot->angle_in_deg += 360.0;
    robot->x_pos += lin_speed*cos(robot->angle_in_rad);
    robot->y_pos += lin_speed*sin(robot->angle_in_rad);
    ESP_LOGW(TAG_Control, "X: %f Y: %f Rad: %f Deg: %f ENKR: %d, ENKL: %d", robot->x_pos, robot->y_pos, robot->angle_in_rad, robot->angle_in_deg, enc->right, enc->left);
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
    float Kp_lin = 1.5; //2.1   //1.5 za jednu bateriju 1.35
    float Ki_lin = 0.00001;
    float Kd_lin = 150.0;   //250.1 120.0

    float distance_error = ((target_y - start_y)*robot->x_pos-(target_x - start_x)*robot->y_pos + target_x*start_y - target_y*start_x)/target_dist;
    float distance_error_pd = Kp_lin*distance_error + Ki_lin*(distance_error_i+distance_error) +Kd_lin*(distance_error - prev_distance_error);
    prev_distance_error = distance_error;
    //ESP_LOGW(TAG_Control, "Distance error: %f", distance_error);

    if(target_speed>last_target_speed + 0.5) last_target_speed += 0.5;
    
    v_left = last_target_speed + distance_error_pd * 0.1;
    v_right = last_target_speed - distance_error_pd * 0.1;

    if(v_left > (target_speed + 10.0)) v_left = target_speed + 10.0;
    if(v_left < 0.0) v_left = 0.0;
    if(v_right > (target_speed + 10.0)) v_right = target_speed + 10.0;
    if(v_right < 0.0) v_right = 0.0;
    //ESP_LOGW(TAG_Control, "VL: %f, VR: %f", v_left, v_right);
}

void rotate_control(float current_angle, float target_angle, float target_speed)
{
    static float kp_rot = 3.0;
    static float ki_rot = 0.0;
    static float kd_rot = 0.0;
    float rotate_error = target_angle - current_angle;
    float rotate_error_pid = kp_rot*rotate_error + ki_rot*(rotate_error_i + rotate_error) + kd_rot*(rotate_error - prev_rotate_error);
    prev_rotate_error = rotate_error;
    if (target_speed > last_rotate_speed) last_rotate_speed += 0.1;
    v_ang_left = rotate_error_pid;
    v_ang_right = rotate_error_pid;
    if (v_ang_left > target_speed + 5.0) v_ang_left = target_speed + 5.0;
    if (v_ang_left < 0) v_ang_left = 0;
    if (v_ang_right > target_speed + 5.0) v_ang_right = target_speed + 5.0;
    if (v_ang_right < 0) v_ang_right = 0;
}

float pid_l(motor_t motor, float target)
{
    static float kp = 2.45;
    static float ki = 0.00001;
    static float kd = 0.1; 
    // if (target > ref_new_l) ref_new_l += 0.1;
    // else if (target < ref_new_l) ref_new_l -= 0.1;
    static float ep, ei = 0, ed, u;
    ep = (target - vl_inc) * kp;
    ei_left += ep * ki;
    //if (ei_left > 1.0) ei_left = 1.0;
    ed = kd*(ep - ep_prev_l);
    ep_prev_l = ep;
    u = ep + ei_left + ed;
    ESP_LOGI(TAG_Control, "UL: %f, Target_L: %f", u, target);
    if (u < 0.0) u = 0.0;
    else if (u > 95.0) u = 95.0;
    return u;
}

float pid_r(motor_t motor, float target)
{
    static float kp = 2.7; //2.5
    static float ki = 0.00001; //0.00001
    static float kd = 0.1;    //0.001
    // if (target > ref_new_r) ref_new_r += 0.1;
    // else if (target < ref_new_r) ref_new_r -= 0.1;
    static float ep, ed, u;
    ep = (target - vr_inc) * kp;
    ei_right += ep * ki;
    //if (ei_right > 1.0) ei_right = 1.0;
    ed = kd*(ep - ep_prev_r);
    ep_prev_r = ep;
    u = ep + ei_right + ed;
    ESP_LOGI(TAG_Control, "UR: %f, Target_R: %f", u, target);
    if (u < 0.0) u = 0.0;
    else if (u > 95.0) u = 95.0;
    return u;
}

void rotate_right(float target_angle, encoder_info_queue_t *enc, position_t *robot, motor_t right_motor, motor_t left_motor)
{
    //ESP_LOGW(TAG_Control, "Rotate right");
    if (angle_count_deg < target_angle)
    {
        calc_pos(enc, robot);
        angle_count += robot->ang;
        angle_count_deg = angle_count * 180/PI;
        rotate_control(angle_count_deg, target_angle, 24.0);
        float speed_r = pid_r(right_motor, v_ang_right);
        float speed_l = pid_l(left_motor, v_ang_left);
        // ESP_LOGI("Log", "SR: %f SL: %f", speed_r, speed_l);
        motor_forward(left_motor, speed_l);
        motor_back(right_motor, speed_r);
        robot_rotated = false;
    }
    else robot_rotated = true;
}

void rotate_left(float target_angle, encoder_info_queue_t *enc, position_t *robot, motor_t right_motor, motor_t left_motor)
{
    //ESP_LOGW(TAG_Control, "Rotate Left");
    if (angle_count_deg < target_angle)
    {
        calc_pos(enc, robot);
        angle_count += fabs(robot->ang);
        angle_count_deg = angle_count * 180/PI;
        rotate_control(angle_count_deg, target_angle, 25.0);
        float speed_r = pid_r(right_motor, v_ang_right);
        float speed_l = pid_l(left_motor, v_ang_left);
        motor_forward(right_motor, speed_r);
        motor_back(left_motor, speed_l);
        robot_rotated = false;
    }
    else robot_rotated = true;
}

void go_to_xy(float target_x, float target_y, float speed, encoder_info_queue_t *enc, position_t *robot, motor_t right_motor, motor_t left_motor, pcnt_unit_handle_t pcnt1, pcnt_unit_handle_t pcnt2)
{
    switch (state)
    {
        case ROBOT_CALCS:
            //ESP_LOGI(TAG_Control, "U robot calcs stejtu");
            reset_values();
            start_x = robot->x_pos;
            start_y = robot->y_pos;     
            last_target_speed = 0;
            last_distance_error_traj = 0;
            angle_of_rot = calc_angle(target_x, target_y, robot);
            target_dist = sqrt(pow(target_x-robot->x_pos, 2) + pow(target_y-robot->y_pos, 2));
            state = ROBOT_MOVE_STRAIGHT;
        break;
        case ROBOT_ROTATE:
            /*ESP_LOGI(TAG_Control, "Ang: %f Dist: %f", angle_of_rot, target_dist);
            if (angle_of_rot > 0.0) rotate_right(angle_of_rot, enc, robot, right_motor, left_motor);
            else if (angle_of_rot < 0.0) rotate_left(fabs(angle_of_rot), enc, robot, right_motor, left_motor);
            else robot_rotated = true;
            if (robot_rotated)
            {
                state = ROBOT_STOP;
            }*/
        break;
        case ROBOT_MOVE_STRAIGHT:
            ESP_LOGI(TAG_Control, "Robot se krece pravo");
            distance_error = target_dist;
            move_straight(target_x, target_y, speed, enc, robot, right_motor, left_motor);
            if (robot_moved)
            {
                state = ROBOT_STOP;
            }
        break;
        case ROBOT_STOP:
            motors_stop(right_motor, left_motor);
            disable_all_pcnts(pcnt1, pcnt2);
            ESP_LOGI(TAG_Control, "Robot stoji");
            vTaskDelay(100/portTICK_PERIOD_MS);
            encoder_reset(enc);
            pcnt_enable_clear_start(pcnt1);
            pcnt_enable_clear_start(pcnt2);
            //if (!robot_moved) state = ROBOT_MOVE_STRAIGHT;
            if (robot_moved)
            {
                //motors_stop(right_motor, left_motor);
                finished_movement = true;
                state = ROBOT_CALCS;
            }
        break;
    }
}

float poow(float a, uint8_t b){
    for(int i = 1;i<b;i++){
        a*=a;
    }
    return a;
}

void move_straight(float target_x, float target_y, float speed, encoder_info_queue_t *enc, position_t *robot, motor_t right_motor, motor_t left_motor)
{
        //ESP_LOGI(TAG_Control, "U while si");
        calc_pos(enc, robot);
        float dist_from_start = sqrtf(poow(robot->x_pos - start_x, 2) + poow(robot->y_pos - start_y, 2));
        float distance_error_traj = ((target_y - start_y)*robot->x_pos-(target_x - start_x)*robot->y_pos + target_x*start_y - target_y*start_x)/target_dist;
        distance_error = target_dist - dist_from_start; 
        ESP_LOGW(TAG_Control, "DE: %f", distance_error);   
    
        float vel = distance_error*0.25;
        
        if(vel > speed){
            vel = speed;
        }
        
        if(vel > last_target_speed + 0.3){
            vel = last_target_speed + 0.3;
        }
        v_left = vel;
        v_right = vel;
    
        last_target_speed = vel;
        
        v_left  += distance_error_traj*traj_kp + (distance_error_traj-last_distance_error_traj)*traj_kd;
        v_right -= distance_error_traj*traj_kp + (distance_error_traj-last_distance_error_traj)*traj_kd;
        last_distance_error_traj = distance_error_traj;
        float speed_l = pid_l(left_motor, v_left);
        float speed_r = pid_r(right_motor, v_right);
        motor_forward(right_motor, speed_r);
        motor_forward(left_motor, speed_l);
        if(distance_error < 15){
            ESP_LOGE(TAG_Control, "Zavrsi kaze Filip");
            state = ROBOT_STOP;
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
    angle_count_deg = 0.0;
    dist = 0.0;
    ref_new_l = 0.0;
    ref_new_r = 0.0;
    ei_right = 0.0;
    ei_left = 0.0;
    ep_prev_l = 0.0;
    ep_prev_r = 0.0;
    distance_error_i = 0.0;
    last_target_speed = 0.0;
    prev_distance_error = 0.0;
    prev_rotate_error = 0.0;
    rotate_error_i = 0.0;
}