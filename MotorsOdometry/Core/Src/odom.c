/*
 * odom.c
 *
 *  Created on: Mar 14, 2024
 *      Author: xenia
 */

#include "odom.h"

// Current odometry data
sOdom_t odom;

// Helper variables for calculating odometry based on the
// current and last increments read from the encoder timers
int32_t last_left_inc  = 0;
int32_t last_right_inc = 0;
int32_t curr_left_inc  = 0;
int32_t curr_right_inc = 0;

float delta_left  = 0;
float delta_right = 0;
float delta_distance = 0;
float delta_theta    = 0;

float left_speed = 0;
float right_speed = 0;

// Variables used for calculating odometry that can be
// changed by user request (for odometry calibration)
volatile float wheel_diameter = 70;
volatile float wheel_distance = 166.42;
float inc_mm = 1;
float inc_rad = 1;

// Calculates current position and speeds based on encoder increment readings
sOdom_t* Odometry_Old(void){
	last_left_inc  = curr_left_inc;
	last_right_inc = curr_right_inc;

	curr_left_inc  = TIM3->CNT;
	curr_right_inc = TIM1->CNT;

	// The delta is calulated from increments from current and last encoder readings and converted to mm
	// The cast to int16_t ensures that a jump from 0 to 65535 and vice versa won't happen - given that
	// the rate of reading the encoders is fast enough
	delta_left  = (int16_t)(curr_left_inc  - last_left_inc)  * inc_mm;
	delta_right = (int16_t)(curr_right_inc - last_right_inc) * inc_mm;

	// Distance traveled from last encoder reading
	delta_distance = (delta_left + delta_right) / 2;
	// Change in orientation from last encoder reading
	delta_theta    = (delta_left - delta_right) / wheel_distance;

	// Updated odom data
	odom.x += delta_distance * cos(odom.theta + delta_theta/2);
	odom.y += delta_distance * sin(odom.theta + delta_theta/2);
	odom.theta += delta_theta;

	// The heding angle range is -PI/2 to PI/2
	if (odom.theta > M_PI)
		odom.theta -= 2*M_PI;
	else if(odom.theta < -M_PI)
		odom.theta += 2*M_PI;

	odom.left_speed  = delta_left / ODOM_TIME * 1000; // mm/s
	odom.right_speed = delta_right / ODOM_TIME * 1000; // mm/s

	return &odom;
}

// Calculates current position and speeds based on encoder increment readings
sOdom_t* Odometry_New(void){
	last_left_inc  = curr_left_inc;
	last_right_inc = curr_right_inc;

	// TODO switch if needed
	curr_left_inc  = TIM3->CNT;
	curr_right_inc = TIM1->CNT;

	// The delta is calulated from increments from current and last encoder readings and converted to mm
	// The cast to int16_t ensures that a jump from 0 to 65535 and vice versa won't happen - given that
	// the rate of reading the encoders is fast enough
	delta_left  = (int16_t)(curr_left_inc  - last_left_inc)  * inc_mm;
	delta_right = (int16_t)(curr_right_inc - last_right_inc) * inc_mm;

	// Distance traveled from last encoder reading
	delta_distance = (delta_left + delta_right) / 2;
	// Change in orientation from last encoder reading
	delta_theta    = (delta_left - delta_right) / wheel_distance;

	// Updated odom data
	odom.x     +=  (delta_distance/delta_theta) * (sin(delta_theta + odom.theta) - sin(odom.theta));
	odom.y     += -(delta_distance/delta_theta) * (cos(delta_theta + odom.theta) - cos(odom.theta));
	odom.theta += delta_theta;

	// The heding angle range is -PI/2 to PI/2
	if (odom.theta > M_PI)
		odom.theta -= 2*M_PI;
	else if(odom.theta < -M_PI)
		odom.theta += 2*M_PI;

	left_speed = delta_left/ODOM_TIME;   // m/s
	right_speed = delta_right/ODOM_TIME; // m/s

	odom.left_speed  = FILTER*left_speed + (1-FILTER)*odom.left_speed;
	odom.right_speed = FILTER*right_speed + (1-FILTER)*odom.right_speed;

	return &odom;
}

// Resets or initializes odometry data based on input parameter
void Reset_Encoders(sOdom_t* new_odom){
	last_left_inc  = 0;
	last_right_inc = 0;
	curr_left_inc  = 0;
	curr_right_inc = 0;

	delta_left  = 0;
	delta_right = 0;
	delta_distance = 0;
	delta_theta    = 0;

	left_speed = 0;
	right_speed = 0;

	TIM3->CNT = 0;
	TIM1->CNT = 0;

	odom.x = new_odom->x;
	odom.y = new_odom->y;
	odom.theta = new_odom->theta;
	odom.left_speed = 0;
	odom.right_speed = 0;
}

// Sets the value of wheel diameter and distance used for calculating odometry data
void Config(float diameter, float distance){
	wheel_diameter = diameter;
	wheel_distance = distance;

	// Calculates conversions
	inc_mm = (wheel_diameter*M_PI)/PPR;
	inc_rad = inc_mm/wheel_distance;
}

