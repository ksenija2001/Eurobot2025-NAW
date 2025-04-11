/*
 * svd.h
 *
 *  Created on: Jan 28, 2025
 *      Author: xenia
 */

#ifndef INC_SVD_H_
#define INC_SVD_H_

#include "arm_math.h"
#include "vector.h"
#include <stdlib.h>

#define MATRIX_DIMENSION 3 // 3x3
#define MAX_ITERATION 500
#define EPSILON 1e-6

arm_status SVD(arm_matrix_instance_f32* A, arm_matrix_instance_f32* U, arm_matrix_instance_f32* VT, arm_matrix_instance_f32* D);

/* Returns charactestic value for initial guess lambda and coefficients of characteristic equation coeff - Gaus-Newton method */
float32_t eigenvalue(float32_t lambda, float32_t coeff[4]);
/* Returns characteristic vector for characteristic value lambda and matrix A - Row Reduction Echelon Form */
arm_status eigenvector(float32_t eigenv[3], float32_t lambda, arm_matrix_instance_f32* A);
/* Perfoms row reduction on matrix A */
void rref(arm_matrix_instance_f32* A);

float32_t determinante(arm_matrix_instance_f32* A);
float32_t trace(arm_matrix_instance_f32* A);
void calc_coeff(float32_t coeff[4], arm_matrix_instance_f32* A);
float32_t equation(float32_t lambda, float32_t coeff[4], uint8_t order);
float32_t linear_eq(float32_t a, float32_t b, float32_t c);

float32_t pivot(arm_matrix_instance_f32* A, uint16_t rowCnt, uint16_t colCnt, uint16_t currRow, uint16_t currCol);
void swap_rows(arm_matrix_instance_f32* A, uint8_t row1, uint8_t row2);
void swap_cols(arm_matrix_instance_f32* A, uint8_t col1, uint8_t col2);

void SVD_Init_Matrices();

#endif /* INC_SVD_H_ */
