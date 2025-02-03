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
#include "struct_types.h"

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
	float wheel_left_speed;
	float wheel_right_speed;

	float wheel_trans;
	float wheel_angular;

	float gyr_angular;
} sOdom_t;

typedef struct {
	sIO_t A;
	sIO_t B;
} sEncoderIO_t;

typedef struct {
	float diameter;
	float track;
	float inc_mm;

	uint16_t curr_inc;
	uint16_t last_inc;

	sEncoderIO_t IO;
	sTIM_t TIM;
} sEncoderWheel_t;


sOdom_t* Odometry_New(void);
sOdom_t* Odometry_Old(void);
void Reset_Odometry(sOdom_t* new_odom);
void Config_Encoder_Wheel(sEncoderWheel_t* wheel, float diameter, float track);
void Init_Encoder(sEncoderWheel_t* wheel, TIM_HandleTypeDef* htim);


extern sEncoderWheel_t left;
extern sEncoderWheel_t right;
extern sOdom_t odom;


#endif /* INC_ODOM_H_ */
