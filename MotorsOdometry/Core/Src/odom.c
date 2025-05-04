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
		},
		.diameter = 73
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
		},
		.diameter = 73
};

void Init_Encoder(sEncoderWheel_t* wheel, TIM_HandleTypeDef* htim){
	wheel->TIM.tim = htim;

	HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
}

// Sets the value of wheel diameter and distance used for calculating odometry data
void Config_Encoder_Wheel(sEncoderWheel_t* wheel, float gain, float inc_mm, float track){
	wheel->gain = gain;
	wheel->inc_mm = inc_mm;
	wheel->track = track;
}

// Calculates current position and speeds based on encoder increment readings
sOdom_t* Odometry(void){
	left.last_inc = left.curr_inc;
	right.last_inc = right.curr_inc;

	left.curr_inc  = left.TIM.tim->Instance->CNT;
	right.curr_inc = right.TIM.tim->Instance->CNT;

	left.inc += (int16_t)(left.curr_inc  - left.last_inc);
	right.inc += (int16_t)(right.curr_inc - right.last_inc);
	// The delta is calulated from increments from current and last encoder readings and converted to mm
	// The cast to int16_t ensures that a jump from 0 to 65535 and vice versa won't happen - given that
	// the rate of reading the encoders is fast enough
	delta_left  = ((int16_t)(left.curr_inc  - left.last_inc))  * left.inc_mm;
	delta_right = ((int16_t)(right.curr_inc - right.last_inc)) * right.inc_mm;

	// Distance traveled from last encoder reading
	delta_distance = (delta_left * left.gain + delta_right * right.gain) * 0.5;
	// Change in orientation from last encoder reading
	delta_theta    = (delta_left  * left.gain - delta_right * right.gain) / (left.track*0.5 + right.track*0.5);

	// Odometry approximation is used when the robot is moving straight, same increments on both wheels
	if ( delta_left == delta_right){
		odom.x     += delta_distance * cos(odom.theta + delta_theta/2);
		odom.y     += delta_distance * sin(odom.theta + delta_theta/2);
		odom.theta -= delta_theta;
	} else {
		odom.x     +=  (delta_distance/delta_theta) * (sin(delta_theta + odom.theta) - sin(odom.theta));
		odom.y     += -(delta_distance/delta_theta) * (cos(delta_theta + odom.theta) - cos(odom.theta));
		odom.theta -= delta_theta;
	}

	// The heading angle range is -PI to PI
	if (odom.theta > M_PI)
		odom.theta -= 2*M_PI;
	else if(odom.theta < -M_PI)
		odom.theta += 2*M_PI;

	// Velocity
	left.last_vel = left.curr_vel;
	right.last_vel = right.curr_vel;

	// Low-pass filter, where FILTER determines how much of the old value is kept
	left.curr_vel  = FILTER*delta_left* left.gain*1000/ODOM_TIME + (1-FILTER)*left.last_vel;
	right.curr_vel = FILTER*delta_right*right.gain*1000/ODOM_TIME + (1-FILTER)*right.last_vel;

	odom.trans_vel = (left.curr_vel + right.curr_vel)/2.0;
	odom.ang_vel = (left.curr_vel - right.curr_vel)/2.0;

	// Acceleration
	left.last_acc = left.curr_acc;
	right.last_acc = right.curr_acc;

	left.curr_acc = FILTER*(left.curr_vel - left.last_vel)/ODOM_TIME + (1-FILTER)*left.last_acc;
	right.curr_acc = FILTER*(right.curr_vel - right.last_vel)/ODOM_TIME + (1-FILTER)*right.last_acc;

	odom.trans_acc = (left.curr_acc + right.curr_acc)/2.0;
	odom.ang_acc = (left.curr_acc - right.curr_acc)/2.0;

	return &odom;
}

// Resets or initializes odometry data based on input parameter
void Reset_Odometry(sOdom_t* new_odom){
	left.last_inc = 0;
	left.curr_inc = 0;
	left.curr_vel = 0.0;
	left.curr_acc = 0.0;
	left.last_vel = 0.0;
	left.last_acc = 0.0;
	left.inc = 0;

	right.curr_inc = 0;
	right.last_inc = 0;
	right.curr_vel = 0.0;
	right.curr_acc = 0.0;
	right.last_vel = 0.0;
	right.last_acc = 0.0;
	right.inc = 0;

	left.TIM.tim->Instance->CNT = 0;
	right.TIM.tim->Instance->CNT = 0;

	odom.x = new_odom->x;
	odom.y = new_odom->y;
	odom.theta = new_odom->theta;
	odom.trans_vel = 0.0;
	odom.trans_acc = 0.0;
	odom.ang_vel = 0.0;
	odom.ang_acc = 0.0;
}


