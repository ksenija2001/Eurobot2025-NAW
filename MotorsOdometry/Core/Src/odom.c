/*
 * odom.c
 *
 *  Created on: Mar 14, 2024
 *      Author: xenia
 */

#include "odom.h"

// Current odometry data
sOdom_t odom = {
		.x = 0,
		.y = 0,
		.theta = M_PI/2
};

// Variables for calculating odometry
float delta_left  = 0;
float delta_right = 0;
float delta_distance = 0;
float delta_theta    = 0;

float left_speed = 0;
float right_speed = 0;

sEncoderWheel_t left = {
		.IO = {
				.A = {
						.pin = Encoder1_A_Pin,
						.port = Encoder1_A_GPIO_Port
				},
				.B = {
						.pin = Encoder1_B_Pin,
						.port = Encoder1_B_GPIO_Port
				}
		}
};
sEncoderWheel_t right = {
		.IO = {
				.A = {
						.pin = Encoder2_A_Pin,
						.port = Encoder2_A_GPIO_Port
				},
				.B = {
						.pin = Encoder2_B_Pin,
						.port = Encoder2_B_GPIO_Port
				}
		}
};

void Init_Encoder(sEncoderWheel_t* wheel, TIM_HandleTypeDef* htim){
	wheel->TIM.tim = htim;

	HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
}

// Sets the value of wheel diameter and distance used for calculating odometry data
void Config_Encoder_Wheel(sEncoderWheel_t* wheel, float diameter, float track){
	wheel->diameter = diameter;
	wheel->track = track;

	// Calculates conversion
	wheel->inc_mm = (wheel->diameter*M_PI)/PPR;
}


// Calculates current position and speeds based on encoder increment readings
sOdom_t* Odometry_Old(void){
	left.last_inc = left.curr_inc;
	right.last_inc = right.curr_inc;

	left.curr_inc  = left.TIM.tim->Instance->CNT;
	right.curr_inc = right.TIM.tim->Instance->CNT;

	// The delta is calulated from increments from current and last encoder readings and converted to mm
	// The cast to int16_t ensures that a jump from 0 to 65535 and vice versa won't happen - given that
	// the rate of reading the encoders is fast enough
	delta_left  = (int16_t)(left.curr_inc  - left.last_inc)  * left.inc_mm;
	delta_right = (int16_t)(right.curr_inc - right.last_inc) * right.inc_mm;

	// Distance traveled from last encoder reading
	delta_distance = (delta_left + delta_right) / 2;
	// Change in orientation from last encoder reading
	delta_theta    = (delta_left - delta_right) / (left.track/2 + right.track/2);

	// Updated odom data
	odom.x += delta_distance * cos(odom.theta + delta_theta/2);
	odom.y += delta_distance * sin(odom.theta + delta_theta/2);
	odom.theta -= delta_theta;

	// The heding angle range is -PI/2 to PI/2
	if (odom.theta > M_PI)
		odom.theta -= 2*M_PI;
	else if(odom.theta < -M_PI)
		odom.theta += 2*M_PI;

	// Derivative of the travelled path by each wheel
	left_speed = delta_left*1000/ODOM_TIME;   // mm/s
	right_speed = delta_right*1000/ODOM_TIME; // mm/s

	// Low-pass filter, where FILTER determines how much of the old value is kept
	odom.wheel_left_speed  = FILTER*left_speed + (1-FILTER)*odom.wheel_left_speed;
	odom.wheel_right_speed = FILTER*right_speed + (1-FILTER)*odom.wheel_right_speed;

	odom.wheel_trans = (odom.wheel_left_speed + odom.wheel_right_speed)/2.0;
	odom.wheel_angular = (odom.wheel_left_speed - odom.wheel_right_speed)/2.0;

	return &odom;
}

// Calculates current position and speeds based on encoder increment readings
sOdom_t* Odometry_New(void){
	left.last_inc = left.curr_inc;
	right.last_inc = right.curr_inc;

	left.curr_inc  = left.TIM.tim->Instance->CNT;
	right.curr_inc = right.TIM.tim->Instance->CNT;

	// The delta is calulated from increments from current and last encoder readings and converted to mm
	// The cast to int16_t ensures that a jump from 0 to 65535 and vice versa won't happen - given that
	// the rate of reading the encoders is fast enough
	delta_left  = (int16_t)(left.curr_inc  - left.last_inc)  * left.inc_mm;
	delta_right = (int16_t)(right.curr_inc - right.last_inc) * right.inc_mm;

	if ( delta_left == delta_right ) {
		return Odometry_Old();
	}

	// Distance traveled from last encoder reading
	delta_distance = (delta_left + delta_right) / 2;
	// Change in orientation from last encoder reading
	delta_theta    = (delta_left - delta_right) / (left.track/2 + right.track/2);

	// Updated odom data
	odom.x     +=  (delta_distance/delta_theta) * (sin(delta_theta + odom.theta) - sin(odom.theta));
	odom.y     += -(delta_distance/delta_theta) * (cos(delta_theta + odom.theta) - cos(odom.theta));
	odom.theta -= delta_theta;

	// The heding angle range is -PI to PI
	if (odom.theta > M_PI)
		odom.theta -= 2*M_PI;
	else if(odom.theta < -M_PI)
		odom.theta += 2*M_PI;

	// Derivative of the travelled path by each wheel
	left_speed = delta_left*1000/ODOM_TIME;   // mm/s
	right_speed = delta_right*1000/ODOM_TIME; // mm/s

	// Low-pass filter, where FILTER determines how much of the old value is kept
	odom.wheel_left_speed  = FILTER*left_speed + (1-FILTER)*odom.wheel_left_speed;
	odom.wheel_right_speed = FILTER*right_speed + (1-FILTER)*odom.wheel_right_speed;

	odom.wheel_trans = (odom.wheel_left_speed + odom.wheel_right_speed)/2.0;
	odom.wheel_angular = (odom.wheel_left_speed - odom.wheel_right_speed)/2.0;

	return &odom;
}

// Resets or initializes odometry data based on input parameter
void Reset_Odometry(sOdom_t* new_odom){
	left.last_inc = 0;
	left.curr_inc = 0;
	right.curr_inc = 0;
	right.last_inc = 0;

	left.TIM.tim->Instance->CNT = 0;
	right.TIM.tim->Instance->CNT = 0;

	odom.x = new_odom->x;
	odom.y = new_odom->y;
	odom.theta = new_odom->theta;
	odom.wheel_left_speed = 0;
	odom.wheel_right_speed = 0;
	odom.wheel_trans = 0;
	odom.wheel_angular = 0;
}


