/*
 * point_cloud.c
 *
 *  Created on: Jan 9, 2025
 *      Author: xenia
 */

#include "point_cloud.h"

arm_matrix_instance_f32 U;
arm_matrix_instance_f32 VT;
arm_matrix_instance_f32 D;
arm_matrix_instance_f32 UT;
arm_matrix_instance_f32 V;
arm_matrix_instance_f32 R;
arm_matrix_instance_f32 T;
arm_matrix_instance_f32 H;

float32_t U_f32[MATRIX_DIMENSION*MATRIX_DIMENSION];
float32_t VT_f32[MATRIX_DIMENSION*MATRIX_DIMENSION];
float32_t D_f32[MATRIX_DIMENSION*MATRIX_DIMENSION];
float32_t UT_f32[MATRIX_DIMENSION*MATRIX_DIMENSION];
float32_t V_f32[MATRIX_DIMENSION*MATRIX_DIMENSION];
float32_t R_f32[MATRIX_DIMENSION*MATRIX_DIMENSION];
float32_t T_f32[MATRIX_DIMENSION*MATRIX_DIMENSION];
float32_t H_f32[MATRIX_DIMENSION*MATRIX_DIMENSION];

double deg2rad(double deg){
	return deg * (M_PI/180.0);
}

void ConvertDist2Point(int16_t angle, float_t distance, float rx, float ry, float rtheta, sVector3_t* point){
	if (distance > 0) {
		point->vector[0] = distance * cos(deg2rad(angle));
		point->vector[1] = distance * sin(deg2rad(angle));
		point->vector[2] = 0.0;

		/* Rotate point to global coordinate system rotation */
		Vector_Rotate(point, M_PI, 0.0, M_PI_2 - rtheta);

		/* Translate point to global coordinate system */
		Vector_Translate(point, rx, ry, 400.0);
	}
}


void Cross_Correlation(sVector3_t* a, sVector3_t* b, sVector3_t* mass_centers, arm_matrix_instance_f32* A){
	sVector3_t a0 = {0};
	sVector3_t b0 = {0};

	/* Finding center of mass of both point sets*/
	for(uint16_t i=0; i<VECTOR_SIZE; ++i){
		Vector_Sum(a0, a[i], &a0);
		Vector_Sum(b0,  b[i], &b0);
	}

	Vector_Scale(a0, (float)1/VECTOR_SIZE, &a0);
	Vector_Scale(b0, (float)1/VECTOR_SIZE, &b0);

	mass_centers[0] = a0;
	mass_centers[1] = b0;

	sVector3_t a_t, b_t;
	/* Finding cross correlation matrix */
	for(uint16_t i=0; i<VECTOR_SIZE; ++i){
		Vector_Sub(a[i], a0, &a[i]);
		Vector_Sub(b[i], b0, &b[i]);

		Vector_Scale(a[i], (float)1/Vector_Norm(a[i]), &a_t);
		Vector_Scale(b[i], (float)1/Vector_Norm(b[i]), &b_t);

		Vector_Mult(a_t, b_t, &T);
		arm_mat_add_f32(&T, A, A);
	}
}

void Rotation_Matrix(arm_matrix_instance_f32* U, arm_matrix_instance_f32* VT, sVector3_t* rpy){
	arm_mat_trans_f32(U, &UT);
	arm_mat_trans_f32(VT, &V);

	arm_mat_mult_f32(&V, &UT, &R);

	if(fabs(R.pData[2*3 + 0]) != 1){
		rpy->vector[1] = -asin(R.pData[2*3 + 0]);
		rpy->vector[0] = atan2(R.pData[2*3 + 1]/cos(rpy->vector[1]), R.pData[2*3 + 2]/cos(rpy->vector[1]));
		rpy->vector[2] = atan2(R.pData[1*3 + 0]/cos(rpy->vector[1]), R.pData[0*3 + 0]/cos(rpy->vector[1]));

	} else if (R.pData[2*3 + 0] == -1){
		rpy->vector[2] = 0.0;
		rpy->vector[1] = M_PI/2;
		rpy->vector[0] = atan2(R.pData[0*3 + 1], R.pData[0*3 + 2]);
	} else {
		rpy->vector[2] = 0.0;
		rpy->vector[1] = -M_PI/2;
		rpy->vector[0] = atan2(-R.pData[0*3 + 1], -R.pData[0*3 + 2]);
	}
}

void Translation_Vec(arm_matrix_instance_f32* U, arm_matrix_instance_f32* VT, sVector3_t mass[2], sVector3_t* trans){
	sVector3_t Rx;

	arm_mat_trans_f32(U, &UT);
	arm_mat_trans_f32(VT, &V);

	arm_mat_mult_f32(&V, &UT, &R);

	Mat_Vector_Mult(&R, mass[0], &Rx);
	Vector_Sub(mass[1], Rx, trans);
}

void PC_Init_Matrices(){
	arm_mat_init_f32(&UT, MATRIX_DIMENSION, MATRIX_DIMENSION, (float32_t *)UT_f32);
	arm_mat_init_f32(&R, MATRIX_DIMENSION, MATRIX_DIMENSION, (float32_t*)R_f32);
	arm_mat_init_f32(&T, MATRIX_DIMENSION, MATRIX_DIMENSION, (float32_t*)T_f32);
	arm_mat_init_f32(&V, MATRIX_DIMENSION, MATRIX_DIMENSION, (float32_t*)V_f32);
	arm_mat_init_f32(&H, MATRIX_DIMENSION, MATRIX_DIMENSION, (float32_t*)H_f32);
	arm_mat_init_f32(&U, MATRIX_DIMENSION, MATRIX_DIMENSION, (float32_t*)U_f32);
	arm_mat_init_f32(&VT, MATRIX_DIMENSION, MATRIX_DIMENSION, (float32_t*)VT_f32);
	arm_mat_init_f32(&D, MATRIX_DIMENSION, MATRIX_DIMENSION, (float32_t*)D_f32);
}

void Process_Point_Cloud(sVector3_t point_cloud[], sVector3_t last_point_cloud[]){
	/* Find which indexes from one pc correspond to which indexes in the other pc */

	/* Cross correlation of the two pcs */
	sVector3_t mass[2];
	Cross_Correlation(point_cloud, last_point_cloud, mass, &H);

	/* SVD of cross correlation matrix */
	SVD(&H, &U, &VT, &D);

	/* Find rotation and translation that will align the two pcs */
	sVector3_t rpy, trans;
	Rotation_Matrix(&U, &VT, &rpy);
	Translation_Vec(&U, &VT, mass, &trans);

}
