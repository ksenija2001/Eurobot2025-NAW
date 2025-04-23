/*
 * vector.h
 *
 *  Created on: Jan 31, 2025
 *      Author: xenia
 */

#ifndef LIB_VECTOR_INC_VECTOR_H_
#define LIB_VECTOR_INC_VECTOR_H_

//#include "arm_math.h"
//#include "stm32f4xx.h"

#include <stdint.h>

#define VECTOR_SIZE 3

typedef struct{
	float vector[3];
} sVector3_t;

void Vector_Sum(sVector3_t a, sVector3_t b, sVector3_t* dst);
void Vector_Sub(sVector3_t a, sVector3_t b, sVector3_t* dst);
void Vector_Scale(sVector3_t a, float scale, sVector3_t* dst);
//void Vector_Mult(sVector3_t a, sVector3_t b, arm_matrix_instance_f32* dst);
//void Mat_Vector_Mult(arm_matrix_instance_f32* A, sVector3_t a, sVector3_t* dst);


#endif /* LIB_VECTOR_INC_VECTOR_H_ */
