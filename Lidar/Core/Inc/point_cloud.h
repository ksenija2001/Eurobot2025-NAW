/*
 * point_cloud.h
 *
 *  Created on: Jan 9, 2025
 *      Author: xenia
 */

#ifndef POINT_CLOUD_H_
#define POINT_CLOUD_H_

#include "stm32g4xx_hal.h"
#include "vector.h"
#include "arm_math.h"
#include "svd.h"
#include <math.h>

//extern Point_t PointCloud_8x8[64];
//extern Point_t PointCloud_4x4[16];

void ConvertDist2Point(int16_t angle, float_t distance, float rx, float ry, float rtheta, sVector3_t* point);
void Point2Bytes(sVector3_t point_cloud, uint8_t *point_bytes);
void PC_Init_Matrices();
void Cross_Correlation(sVector3_t* a, sVector3_t* b, sVector3_t* mass_centers, arm_matrix_instance_f32* A);
void Translation_Vec(arm_matrix_instance_f32* U, arm_matrix_instance_f32* VT, sVector3_t mass[2], sVector3_t* trans);
void Rotation_Matrix(arm_matrix_instance_f32* U, arm_matrix_instance_f32* VT, sVector3_t* rpy);
void Process_Point_Cloud(sVector3_t point_cloud[], sVector3_t last_point_cloud[]);

extern arm_matrix_instance_f32 H, U, VT, D;

#endif /* POINT_CLOUD_H_ */
