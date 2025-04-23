/*
 * vector.c
 *
 *  Created on: Jan 31, 2025
 *      Author: xenia
 */

#include "vector.h"

void Vector_Sum(sVector3_t a, sVector3_t b, sVector3_t* dst){
	for (uint16_t i=0; i<VECTOR_SIZE; ++i){
		dst->vector[i] = a.vector[i] + b.vector[i];
	}
}

void Vector_Sub(sVector3_t a, sVector3_t b, sVector3_t* dst){
	for (uint16_t i=0; i<VECTOR_SIZE; ++i){
		dst->vector[i] = a.vector[i] - b.vector[i];
	}
}

void Vector_Scale(sVector3_t a, float scale, sVector3_t* dst){
	for (uint16_t i=0; i<VECTOR_SIZE; ++i){
		dst->vector[i] = a.vector[i] * scale;
	}
}

//void Vector_Mult(sVector3_t a, sVector3_t b, arm_matrix_instance_f32* dst){
//	uint16_t k=0;
//	for (uint16_t i=0; i<VECTOR_SIZE; ++i){
//		for (uint16_t j=0; j<VECTOR_SIZE; ++j){
//			// a is treated as a column vector, b is treated as a row vector
//			dst->pData[k++] = a.vector[i] * b.vector[j];
//		}
//	}
//}
//
//void Mat_Vector_Mult(arm_matrix_instance_f32* A, sVector3_t a, sVector3_t* dst){
//	float sum;
//	float32_t* M = A->pData;
//	uint16_t rowCnt = A->numRows;
//
//	for (uint16_t i=0; i<VECTOR_SIZE; ++i){
//		sum = 0;
//		for (uint16_t j=0; j<VECTOR_SIZE; ++j){
//			sum += M[i*rowCnt + j] * a.vector[j];
//		}
//
//		dst->vector[i] = sum;
//	}
//}
