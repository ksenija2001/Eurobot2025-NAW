/*
 * quaternion.h
 *
 *  Created on: Jan 29, 2025
 *      Author: filip
 */

#ifndef INC_QUATERION_H_
#define INC_QUATERION_H_

#include "stm32f4xx_hal.h"
#include <math.h>

typedef struct {
	float w;
	float x;
	float y;
	float z;
} Quaterion;

typedef struct {
	float roll;
	float pitch;
	float yaw;
} RPY; // ROLL PITCH YAW

HAL_StatusTypeDef QUATERION_INIT(Quaterion *q, float w, float x, float y, float z);

float deg2rad(float deg);
float rad2deg(float rad);

HAL_StatusTypeDef QUATERION_HAMILTON_PRODUCT(Quaterion *a, Quaterion *b, Quaterion *c);

HAL_StatusTypeDef QUATERION2EULER(Quaterion *q, RPY *rpy);
HAL_StatusTypeDef QUATERION_NORMALIZE(Quaterion *q);

#endif /* INC_QUATERION_H_ */
