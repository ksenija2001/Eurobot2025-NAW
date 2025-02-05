/*
 * quaterion.c
 *
 *  Created on: Jan 29, 2025
 *      Author: filip
 */

#include "quaterion.h"

HAL_StatusTypeDef QUATERION_INIT(Quaterion *q, float w, float x, float y, float z){
	q->w = w;
	q->x = x;
	q->y = y;
	q->z = z;

	return HAL_OK;
}

float deg2rad(float deg){
	return deg * M_PI / 180.0;
}
float rad2deg(float rad){
	return rad * 180.0 / M_PI;
}

HAL_StatusTypeDef QUATERION_HAMILTON_PRODUCT(Quaterion *a, Quaterion *b, Quaterion *c){
	if(a == NULL) return HAL_ERROR;
	if(b == NULL) return HAL_ERROR;

	uint8_t c_null = 0;
	Quaterion q;

	if(c == NULL) {
		c_null = 1;

		QUATERION_INIT(&q, 1, 0,0,0);
		c = &q;
	}

	c->w = a->w * b->w - a->x * b->x - a->y * b->y - a->z * b->z;
	c->x = a->w * b->x + a->x * b->w + a->y * b->z - a->z * b->y;
	c->y = a->w * b->y - a->x * b->z + a->y * b->w + a->z * b->x;
	c->z = a->w * b->z + a->x * b->y - a->y * b->x + a->z * b->w;
	/*
	 * 	q.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z
    	q.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y
    	q.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x
    	q.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
	 *
	 */

	if(c_null){
		a->w = c->w;
		a->x = c->x;
		a->y = c->y;
		a->z = c->z;
	}

	return HAL_OK;
}

HAL_StatusTypeDef QUATERION2EULER(Quaterion *q, RPY *rpy){
	/*
	 *
	sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
    roll = np.atan2(sinr_cosp, cosr_cosp)

    sinp = np.sqrt(1 + 2 * (q.w * q.y - q.x * q.z))
    cosp = np.sqrt(1 - 2 * (q.w * q.y - q.x * q.z))
    pitch = 2 * np.atan2(sinp, cosp) - np.pi / 2

    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = np.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw
     *
	 */

	float sinr_cosp = 2 * (q->w * q->x + q->y * q->z);
	float cosr_cosp = 1 - 2 * (q->x * q->x + q->y * q->y);
	rpy->roll = rad2deg(atan2(sinr_cosp, cosr_cosp));

	float sinp = sqrt(1 + 2 * (q->w * q->y - q->x * q->z));
	float cosp = sqrt(1 - 2 * (q->w * q->y - q->x * q->z));
	rpy->pitch = rad2deg(2 * atan2(sinp, cosp) - M_PI / 2.0);

	float siny_cosp = 2 * (q->w * q->z + q->x * q->y);
	float cosy_cosp = 1 - 2 * (q->y * q->y + q->z * q->z);
	rpy->yaw = rad2deg(atan2(siny_cosp, cosy_cosp));

	return HAL_OK;
}

HAL_StatusTypeDef QUATERION_NORMALIZE(Quaterion *q){
	float s = 1 / sqrt(q->w *q->w + q->x * q->x + q->y * q->y + q->z * q->z);

	q->w *= s;
	q->x *= s;
	q->y *= s;
	q->z *= s;

	return HAL_OK;
}
