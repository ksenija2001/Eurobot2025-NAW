/*
 * svd.c
 *
 *  Created on: Jan 28, 2025
 *      Author: xenia
 */

#include "svd.h"

float32_t AT_f32[MATRIX_DIMENSION*MATRIX_DIMENSION];
float32_t AT_M_A_f32[MATRIX_DIMENSION*MATRIX_DIMENSION];
float32_t I_f32[MATRIX_DIMENSION*MATRIX_DIMENSION] = {
		1.0, 0.0, 0.0,
		0.0, 1.0, 0.0,
		0.0, 0.0, 1.0
};

float32_t IL_f32[MATRIX_DIMENSION*MATRIX_DIMENSION];
float32_t LI_A_f32[MATRIX_DIMENSION*MATRIX_DIMENSION];
float32_t _UT_f32[MATRIX_DIMENSION*MATRIX_DIMENSION];


/* Transpose of matrix */
arm_matrix_instance_f32 AT;
/* Transmose of matrix multiplied by same matrix */
arm_matrix_instance_f32 AT_M_A;
/* Identity matrix */
arm_matrix_instance_f32 I;
arm_matrix_instance_f32 LI_A;
arm_matrix_instance_f32 IL;

arm_matrix_instance_f32 _UT;


/* Vector of right signular values (eigenvector) */
sVector3_t v;
/* Vector of left signular values (eigenvector) */
sVector3_t u;
/* Vector of eigenvalues */
sVector3_t L;
/* Vector of sqrt of eigenvalues */
sVector3_t Sigma;

/* Characteristic equation coefficients  */
float32_t coeff[4];
/* Trace  */
float32_t tr;
float32_t guess;

float32_t vnorm;
float32_t unorm;

uint16_t rowCnt;
uint16_t colCnt;
float32_t* M;

void SVD_Init_Matrices(){
	arm_mat_init_f32(&I, MATRIX_DIMENSION, MATRIX_DIMENSION, (float32_t *)I_f32);
	arm_mat_init_f32(&IL, MATRIX_DIMENSION, MATRIX_DIMENSION, (float32_t *)IL_f32);
	arm_mat_init_f32(&LI_A, MATRIX_DIMENSION, MATRIX_DIMENSION, (float32_t *)LI_A_f32);
	arm_mat_init_f32(&AT, MATRIX_DIMENSION, MATRIX_DIMENSION, (float32_t *)AT_f32);
	arm_mat_init_f32(&AT_M_A, MATRIX_DIMENSION, MATRIX_DIMENSION, (float32_t *)AT_M_A_f32);
	arm_mat_init_f32(&_UT, MATRIX_DIMENSION, MATRIX_DIMENSION, (float32_t *)_UT_f32);
}

int compare (const void * a, const void * b)
{
  float fa = *(const float*) a;
  float fb = *(const float*) b;
  return (fa < fb) - (fa > fb);
}

/*
 * Performs singular value decomposition.
 * Returns 3 matrices:
 * - U - left singular vectors of matrix A
 * - D - singular values of matrix A in descending order
 * - VT - right singular vectors of matrix A
*/
arm_status SVD(arm_matrix_instance_f32* A, arm_matrix_instance_f32* U, arm_matrix_instance_f32* VT, arm_matrix_instance_f32* D){
	arm_status status;

	/* Transpose of matrix A */
	status = arm_mat_trans_f32(A, &AT);

	/* Mulitplication of AT and A */
	status = arm_mat_mult_f32(&AT, A, &AT_M_A);

	// Coefficients of characteristic equation of AT_M_A
	calc_coeff(coeff, &AT_M_A);
	tr = -coeff[2];
	guess = 0;

	// Ensures the eigenvalues are unique
	L.vector[0] = eigenvalue(0, coeff);
	do{
		if (L.vector[0] == L.vector[1])
			L.vector[1] = eigenvalue(tr - guess, coeff);

		if (L.vector[0] == L.vector[2] || L.vector[1] == L.vector[2])
			L.vector[2] = eigenvalue(-tr + guess, coeff);

		guess += tr/10;
	}
	while (L.vector[1] == L.vector[2] || L.vector[0] == L.vector[2] || L.vector[0] == L.vector[1]);

//	guess=0;
//	do{
//		L[2] = eigenvalue(-tr + guess, coeff);
//		guess += tr/10;
//	}
//	while (L[1] == L[2] || L[0] == L[2]);

	int16_t i,j;
	// float32_t temp;

	/* Sort the eigenvalues in descending order */
	qsort(L.vector, MATRIX_DIMENSION, sizeof(float32_t), compare);

//	for(i=1; i<MATRIX_DIMENSION; ++i){
//		for(j=i-1; j>=0; --j){
//			if (L[j+1] > L[j]){
//				temp = L[j+1];
//				L[j+1] = L[j];
//				L[j] = temp;
//			}
//		}
//	}

	for (i=0; i<MATRIX_DIMENSION; ++i){
		eigenvector(v.vector, L.vector[i], &AT_M_A);
		vnorm = Vector_Norm(v);

		Sigma.vector[i] = sqrt(L.vector[i]);

		if (Sigma.vector[i] != 0){
			Mat_Vector_Mult(A, v, &u);
			unorm = vnorm*Sigma.vector[i];
		} else {
			arm_mat_trans_f32(U, &_UT);
			rref(&_UT);

			u.vector[2] = linear_eq(0, 0, 0);
			u.vector[1] = linear_eq(0, _UT.pData[4], _UT.pData[5]*u.vector[2]);
			u.vector[0] = linear_eq(1, _UT.pData[1]*u.vector[1], _UT.pData[2]*u.vector[2]);

			unorm = Vector_Norm(u);
		}

		for(j=0; j<MATRIX_DIMENSION; ++j){
			VT->pData[i*VT->numRows + j] = v.vector[j]/vnorm;
			U->pData[j*U->numRows + i] = u.vector[j]/unorm;
		}

		D->pData[i*D->numRows + i] = Sigma.vector[i];
	}

	return status;
}

float32_t determinante(arm_matrix_instance_f32* A){
	M = A->pData;

	return M[0*3 + 0]*M[1*3 + 1]*M[2*3 + 2] + M[0*3 + 1]*M[1*3 + 2]*M[2*3 + 0] + M[0*3 + 2]*M[1*3 + 0]*M[2*3 + 1]
		 - M[0*3 + 2]*M[1*3 + 1]*M[2*3 + 0] - M[0*3 + 0]*M[1*3 + 2]*M[2*3 + 1] - M[0*3 + 1]*M[1*3 + 0]*M[2*3 + 2];
}

float32_t trace(arm_matrix_instance_f32* A){
	M = A->pData;

	return M[0*3 + 0] + M[1*3 + 1] + M[2*3 + 2];
}

void calc_coeff(float32_t coeff[4], arm_matrix_instance_f32* A){
	M = A->pData;

	coeff[3] = 1;
	coeff[2] = -trace(A);
	coeff[1] = -(M[0*3 + 1]*M[1*3 + 0] - M[0*3 + 0]*M[1*3 + 1] +
			     M[0*3 + 2]*M[2*3 + 0] - M[0*3 + 0]*M[2*3 + 2] +
			     M[1*3 + 2]*M[2*3 + 1] - M[1*3 + 1]*M[2*3 + 2]);
	coeff[0] = -determinante(A);
}

float32_t equation(float32_t lambda, float32_t coeff[4], uint8_t order){
	if (order == 0)
		return coeff[3]*lambda*lambda*lambda + coeff[2]*lambda*lambda + coeff[1]*lambda + coeff[0];
	else if (order == 1)
		return 3*coeff[3]*lambda*lambda + 2*coeff[2]*lambda + coeff[1];
	else if (order == 2)
		return 6*coeff[3]*lambda + 2*coeff[2];
	else if (order == 3)
		return 6*coeff[3];
	else
		return 0;
}

float32_t eigenvalue(float32_t lambda, float32_t coeff[4]){
	/* Finds a single eigenvalue for a lambda initial guess.
	 * Uses  Gauss-Newtons numerical estimation */
	float32_t eq, deq;
	uint16_t iter = 0;

	while(iter < MAX_ITERATION){

		eq = equation(lambda, coeff, 0);
		deq = equation(lambda, coeff, 1);

		/* Division near zero */
		if (fabs(deq) < EPSILON)
			break;

		lambda -= eq/deq;

		if (fabs(eq) < EPSILON)
			break;
		++iter;
	}

	return lambda;
}

arm_status eigenvector(float32_t eigenv[3], float32_t lambda, arm_matrix_instance_f32* A){
	arm_status status;

	status = arm_mat_scale_f32(&I, lambda, &IL);
	status = arm_mat_sub_f32(&IL, A, &LI_A);

	rowCnt = A->numRows;
	colCnt = A->numCols;
	int16_t k, m;
	float32_t p, el;

	p = pivot(&LI_A, rowCnt, colCnt, 0, 0);
	/* Scale row with pivot */
	for (k=0; k<colCnt && p != 0; ++k){
		LI_A.pData[0*rowCnt + k] = LI_A.pData[0*rowCnt + k] / p;
	}

	// Reduce below the pivot row
	for (k=1; k<rowCnt; ++k){
		el = LI_A.pData[k*rowCnt + 0];
		if (el != 0){
			// Subtract pivot row multiplied by el from the element row
			for (m=0; m<colCnt; ++m){
				LI_A.pData[k*rowCnt + m] = el*LI_A.pData[0*rowCnt + m] - LI_A.pData[k*rowCnt + m];
			}
		}
	}

	eigenv[2] = 1.0;
	eigenv[1] = - LI_A.pData[5]/LI_A.pData[4];
	eigenv[0] = - LI_A.pData[1] * eigenv[1] - LI_A.pData[2];

	//	rref(&LI_A);
	//
	//	eigenv[2] = linear_eq(0, 0, LI_A.pData[8]);
	//	eigenv[1] = linear_eq(0, LI_A.pData[4], LI_A.pData[5]*eigenv[2]);
	//	eigenv[0] = linear_eq(1, LI_A.pData[1]*eigenv[1], LI_A.pData[2]*eigenv[2]);

	return status;
}

void rref(arm_matrix_instance_f32* A){
	rowCnt = A->numRows;
	colCnt = A->numCols;
	int16_t i, j, k, m;
	float32_t p, el;

	/* Reduced echelon form */
	for (i=0,j=0; i<rowCnt && j<colCnt; ++i, ++j){
		p = pivot(A, rowCnt, colCnt, i, j);
		if (p == 0){
			// last row is zero, over
			if (i == rowCnt-1)
				break;

			swap_rows(A, i, i+1);
			--i;
			continue;  // without incrementing row number
		}

		/* Scale row with pivot */
		for (k=j; k<colCnt && p != 0; ++k){
			A->pData[i*rowCnt + k] = A->pData[i*rowCnt + k] / p;
			if (fabs(A->pData[i*rowCnt + k]) < 1e-3)
				A->pData[i*rowCnt + k] = 0.0;
		}

		// Reduce below the pivot row
		for (k=i+1; k<rowCnt; ++k){
			el = A->pData[k*rowCnt + j];
			if (el != 0){
				// Subtract pivot row multiplied by el from the element row
				for (m=j; m<colCnt; ++m){
					A->pData[k*rowCnt + m] = el*A->pData[i*rowCnt + m] - A->pData[k*rowCnt + m];
					if (fabs(A->pData[k*rowCnt + m]) < 1e-3)
						A->pData[k*rowCnt + m] = 0.0;
				}
			}
		}

		// Reduce above the pivot row
		for (k=i-1; k>=0 && i != rowCnt-1; --k){
			el = A->pData[k*rowCnt + j];
			if (el != 0){
				// Subtract pivot row multiplied by el from the element row
				for (m=j; m<colCnt; ++m){
					A->pData[k*rowCnt + m] = -el*A->pData[i*rowCnt + m] + A->pData[k*rowCnt + m];
					if (fabs(A->pData[k*rowCnt + m]) < 1e-3)
						A->pData[k*rowCnt + m] = 0.0;
				}
			}
		}
	}

}

float32_t linear_eq(float32_t a, float32_t b, float32_t c){
	if (a == 0.0 && b == 0 && c == 0){
		// last row - z can be anything
		return 1.0;
	} else if (a == 0 && b == 0 && c == 1){
		// last row - z is zero
		// or second row - z is zero
		return 0.0;
	} else if (a == 0 && b == 1){
		// second row - y = -c z
		return -c;
	} else if (a == 1){
		// first row - x = -b y - c z
		return -b - c;
	}

	// error
	return 1e99;
}

float32_t pivot(arm_matrix_instance_f32* A, uint16_t rowCnt, uint16_t colCnt, uint16_t currRow, uint16_t currCol){
	// Search for the first non zero pivot element, first along the column
	float32_t pivot = 0;
	uint16_t m;

	m = currRow;
	while (pivot == 0 && m != rowCnt){
		pivot = A->pData[m*rowCnt + currCol];
		++m;
	}

	if (pivot != 0)
		return pivot;

	m = currCol;
	// All rows in column contain zeros
	while (pivot == 0 && m != colCnt){
		pivot = A->pData[currRow*rowCnt + m];
		++m;
	}

	return pivot;
}

void swap_rows(arm_matrix_instance_f32* A, uint8_t row1, uint8_t row2){
	float32_t temp;

	if (row1 == row2)
		return;

	uint16_t colCnt = A->numCols;
	uint16_t rowCnt = A->numRows;

	for(uint16_t j=0; j<colCnt; ++j){
		temp = A->pData[row1*rowCnt + j];
		A->pData[row1*rowCnt + j] = A->pData[row2*rowCnt + j];
		A->pData[row2*rowCnt + j] = temp;
	}
}

void swap_cols(arm_matrix_instance_f32* A, uint8_t col1, uint8_t col2){
	float32_t temp;

	if (col1 == col2)
		return;

	//uint16_t colCnt = A->numCols;
	rowCnt = A->numRows;

	for(uint16_t i=0; i<rowCnt; ++i){
		temp = A->pData[i*rowCnt + col1];
		A->pData[i*rowCnt + col1] = A->pData[i*rowCnt + col2];
		A->pData[i*rowCnt + col2] = temp;
	}
}










