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
		.theta = M_PI/2,
		.detection_enable_front = 0,
		.detection_enable_back = 0,
		.detection_activated = 0
};

// Last updated wheel odometry data
sOdom_t wheel_odom = {
		.x = 0,
		.y = 0,
		.theta = M_PI/2
};

sOdom_t last_wheel_odom = {
		.x = 0,
		.y = 0,
		.theta = M_PI/2
};

// Last updated lidar odometry data
sOdom_t lidar_odom = {
		.x = 0,
		.y = 0,
		.theta = 0
};

uint8_t lidar_update = 0;



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

	// Save wheel_odom before update step
	last_wheel_odom = wheel_odom;

	// Odometry approximation is used when the robot is moving straight, same increments on both wheels
	if ( delta_left == delta_right){
		wheel_odom.x     += delta_distance * cos(odom.theta + delta_theta/2);
		wheel_odom.y     += delta_distance * sin(odom.theta + delta_theta/2);
		wheel_odom.theta -= delta_theta;
	} else {
		wheel_odom.x     +=  (delta_distance/delta_theta) * (sin(delta_theta + odom.theta) - sin(odom.theta));
		wheel_odom.y     += -(delta_distance/delta_theta) * (cos(delta_theta + odom.theta) - cos(odom.theta));
		wheel_odom.theta -= delta_theta;
	}

	// The heading angle range is -PI to PI
	if (wheel_odom.theta > M_PI)
		wheel_odom.theta -= 2*M_PI;
	else if(wheel_odom.theta < -M_PI)
		wheel_odom.theta += 2*M_PI;

	// Velocity
	left.last_vel = left.curr_vel;
	right.last_vel = right.curr_vel;

	// Low-pass filter, where FILTER determines how much of the old value is kept
	left.curr_vel  = FILTER*delta_left* left.gain*1000/ODOM_TIME + (1-FILTER)*left.last_vel;
	right.curr_vel = FILTER*delta_right*right.gain*1000/ODOM_TIME + (1-FILTER)*right.last_vel;

	wheel_odom.trans_vel = (left.curr_vel + right.curr_vel)/2.0;
	wheel_odom.ang_vel = (left.curr_vel - right.curr_vel)/2.0;

	// Acceleration
	left.last_acc = left.curr_acc;
	right.last_acc = right.curr_acc;

	left.curr_acc = FILTER*(left.curr_vel - left.last_vel)/ODOM_TIME + (1-FILTER)*left.last_acc;
	right.curr_acc = FILTER*(right.curr_vel - right.last_vel)/ODOM_TIME + (1-FILTER)*right.last_acc;

	wheel_odom.trans_acc = (left.curr_acc + right.curr_acc)/2.0;
	wheel_odom.ang_acc = (left.curr_acc - right.curr_acc)/2.0;

	// Complementary filter if lidar odometry is ready
	if (lidar_update){
		odom.x = (1 - LIDAR_FILTER) * wheel_odom.x + LIDAR_FILTER * lidar_odom.x;
		odom.y = (1 - LIDAR_FILTER) * wheel_odom.y + LIDAR_FILTER * lidar_odom.y;
		odom.theta = (1 - LIDAR_FILTER) * wheel_odom.theta + LIDAR_FILTER * lidar_odom.theta;
		lidar_update = 0;
	} else {
		odom.x += wheel_odom.x - last_wheel_odom.x;
		odom.y += wheel_odom.y - last_wheel_odom.y;
		odom.theta += wheel_odom.theta - last_wheel_odom.theta;
	}

	odom.trans_vel = wheel_odom.trans_vel;
	odom.trans_acc = wheel_odom.trans_acc;
	odom.ang_vel = wheel_odom.ang_vel;
	odom.ang_acc = wheel_odom.ang_acc;

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

	wheel_odom.x = new_odom->x;
	wheel_odom.y = new_odom->y;
	wheel_odom.theta = new_odom->theta;
	wheel_odom.trans_vel = 0.0;
	wheel_odom.trans_acc = 0.0;
	wheel_odom.ang_vel = 0.0;
	wheel_odom.ang_acc = 0.0;
}


