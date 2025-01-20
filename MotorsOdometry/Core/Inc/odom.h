/*
 * odom.h
 *
 *  Created on: Mar 14, 2024
 *      Author: xenia
 */

#ifndef INC_ODOM_H_
#define INC_ODOM_H_

#include <math.h>
#include <stm32g4xx.h>
#include "interrupts.h"

#define PPR         8192      // 4*2048 inc
#define FILTER      0.5       // determines how much of the old value will be kept

//#define WHEEL_DIAMETER 70
//#define WHEEL_DISTANCE 166.42
//#define INC_MM         0.10069207 // (WHEEL_DIAMETER*PI)/PPR
//#define INC_RAD        0.00143845 // INC_MM/WHEEL_DISTANCE

// Structure for representing odometry data
typedef struct {
	float x;
	float y;
	float theta;
	float left_speed;
	float right_speed;
} sOdom_t;

typedef struct {
	float diameter;
	float track;
	float inc_mm;

	uint16_t curr_inc;
	uint16_t last_inc;
} sEncoderWheel_t;

sOdom_t* Odometry_New(void);
sOdom_t* Odometry_Old(void);
void Reset_Odometry(sOdom_t* new_odom);
void Config_Encoder_Wheel(sEncoderWheel_t* wheel, float diameter, float track);

extern sEncoderWheel_t left;
extern sEncoderWheel_t right;

#endif /* INC_ODOM_H_ */
