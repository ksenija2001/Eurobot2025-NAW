#include "synthesis.h"
#include <math.h>



float coef[6]; 				//[a5, a4, a3, a2, a1, a0]
arm_matrix_instance_f32 COEF;
float current_state[3];
arm_matrix_instance_f32 CURRENT_STATE;
float target_state[3];
arm_matrix_instance_f32 TARGET_STATE;
float next_state[3];
arm_matrix_instance_f32 NEXT_STATE;
float state[6];
arm_matrix_instance_f32 STATE;


float ai[36] = {0};
arm_matrix_instance_f32 AI;
float a[18];
arm_matrix_instance_f32 A;

float x_0, x_1, y_0, y_1;
float total_distance;
float total_T;
float synthesis_start_time = 0;
float trajectory_start_time = 0;
uint8_t synthesis_translation_state = 0;


void synthesis_init(){
	arm_mat_init_f32(&CURRENT_STATE, 3, 1, current_state);
	arm_mat_init_f32(&TARGET_STATE, 3, 1, target_state);
	arm_mat_init_f32(&STATE, 6, 1, state);
	arm_mat_init_f32(&NEXT_STATE, 3, 1, next_state);
	arm_mat_init_f32(&COEF, 6, 1, coef);
	arm_mat_init_f32(&AI, 6, 6, ai);
	arm_mat_init_f32(&A, 3, 6, a);
}

float poow(float a, int exp){
	float res = 1;
	for(int i=0; i<exp; i++){
		res *= a;
	}
	return res;;
}

void synthesis_calc_coef(float T){
	AI.pData[0*6 + 0] = 1;
	AI.pData[0*6 + 1] = 0;
	AI.pData[0*6 + 2] = 0;
	AI.pData[0*6 + 3] = 0;
	AI.pData[0*6 + 4] = 0;
	AI.pData[0*6 + 5] = 0;

	AI.pData[1*6 + 0] = 0;
	AI.pData[1*6 + 1] = 1;
	AI.pData[1*6 + 2] = 0;
	AI.pData[1*6 + 3] = 0;
	AI.pData[1*6 + 4] = 0;
	AI.pData[1*6 + 5] = 0;

	AI.pData[2*6 + 0] = 0;
	AI.pData[2*6 + 1] = 0;
	AI.pData[2*6 + 2] = 0.5;
	AI.pData[2*6 + 3] = 0;
	AI.pData[2*6 + 4] = 0;
	AI.pData[2*6 + 5] = 0;

	AI.pData[3*6 + 0] =  -10 / poow(T,3);
	AI.pData[3*6 + 1] =   -6 / poow(T,2);
	AI.pData[3*6 + 2] = -1.5 / poow(T,1);
	AI.pData[3*6 + 3] =   10 / poow(T,3);
	AI.pData[3*6 + 4] =   -4 / poow(T,2);
	AI.pData[3*6 + 5] =  0.5 / poow(T,1);

	AI.pData[4*6 + 0] =  15 / poow(T,4);
	AI.pData[4*6 + 1] =   8 / poow(T,3);
	AI.pData[4*6 + 2] = 1.5 / poow(T,2);
	AI.pData[4*6 + 3] = -15 / poow(T,4);
	AI.pData[4*6 + 4] =   7 / poow(T,3);
	AI.pData[4*6 + 5] =  -1 / poow(T,2);

	AI.pData[5*6 + 0] =  -6  / poow(T,5);
	AI.pData[5*6 + 1] =  -3  / poow(T,4);
	AI.pData[5*6 + 2] = -0.5 / poow(T,3);
	AI.pData[5*6 + 3] =   6  / poow(T,5);
	AI.pData[5*6 + 4] =  -3  / poow(T,4);
	AI.pData[5*6 + 5] = 0.5  / poow(T,3);

	STATE.pData[0] = CURRENT_STATE.pData[0];
	STATE.pData[1] = CURRENT_STATE.pData[1];
	STATE.pData[2] = CURRENT_STATE.pData[2];
	STATE.pData[3] = TARGET_STATE.pData[0];
	STATE.pData[4] = TARGET_STATE.pData[1];
	STATE.pData[5] = TARGET_STATE.pData[2];

//	A_3.pData[0*3 + 0] = poow(T, 5);
//	A_3.pData[0*3 + 1] = poow(T, 4);
//	A_3.pData[0*3 + 2] = poow(T, 3);
//
//	A_3.pData[1*3 + 0] = 5*poow(T, 4);
//	A_3.pData[1*3 + 1] = 4*poow(T, 3);
//	A_3.pData[1*3 + 2] = 3*poow(T, 2);
//
//	A_3.pData[2*3 + 0] = 20*poow(T, 3);
//	A_3.pData[2*3 + 1] = 12*poow(T, 2);
//	A_3.pData[2*3 + 2] = 6* poow(T, 1);
//
//	arm_mat_inverse_f32(&A_3, &AI_3);
	arm_mat_mult_f32(&AI, &STATE, &COEF);
}

void synthesis_calc_next_state(float t){
	A.pData[0*6 + 0] = poow(t, 0);
	A.pData[0*6 + 1] = poow(t, 1);
	A.pData[0*6 + 2] = poow(t, 2);
	A.pData[0*6 + 3] = poow(t, 3);
	A.pData[0*6 + 4] = poow(t, 4);
	A.pData[0*6 + 5] = poow(t, 5);

	A.pData[1*6 + 0] = 0;
	A.pData[1*6 + 1] = 1* poow(t, 0);
	A.pData[1*6 + 2] = 2* poow(t, 1);
	A.pData[1*6 + 3] = 3* poow(t, 2);
	A.pData[1*6 + 4] = 4* poow(t, 3);
	A.pData[1*6 + 5] = 5* poow(t, 4);

	A.pData[2*6 + 0] = 0;
	A.pData[2*6 + 1] = 0;
	A.pData[2*6 + 2] = 2*  poow(t, 0);
	A.pData[2*6 + 3] = 6*  poow(t, 1);
	A.pData[2*6 + 4] = 12* poow(t, 2);
	A.pData[2*6 + 5] = 20* poow(t, 3);

	arm_mat_mult_f32(&A, &COEF, &NEXT_STATE);
}

void synthesis_set_target_state(float p, float v, float a){
	TARGET_STATE.pData[0] = p;
	TARGET_STATE.pData[1] = v;
	TARGET_STATE.pData[2] = a;
}

void synthesis_set_current_state(float p, float v, float a){
	CURRENT_STATE.pData[0] = p;
	CURRENT_STATE.pData[1] = v;
	CURRENT_STATE.pData[2] = a;
}

float synthesis_calc_Vmax(float Pmax, float Amax){
	return sqrtf((5*Pmax*Amax)/8);
}
float synthesis_calc_Amax(float Pmax, float Vmax){
	return (8*Vmax*Vmax)/(5*Pmax);
}
float synthesis_calc_T_a(float Pmax, float Amax){
	return sqrtf((45*Pmax)/(8*Amax));
}
float synthesis_calc_T_v(float Pmax, float Vmax){
	return (15*Pmax)/(8*Vmax);
}
float synthesis_calc_T(float P, float V, float A){
	if(synthesis_calc_Vmax(P, A) > V){
		return synthesis_calc_T_v(P, V);
	}
	else{
		return synthesis_calc_T_a(P, A);
	}
}

//void synthesis_start_rotation(float Vmax, float Amax, float start_theta, float theta);

void synthesis_start_distance(float Vmax, float Amax, float start_x, float start_y, float start_theta, float distance){
	x_0 = start_x;
	y_0 = start_y;
	x_1 = start_x + distance * cos(start_theta);
	y_1 = start_y + distance * sin(start_theta);
	total_distance = sqrtf((y_1-y_0)*(y_1-y_0) + (x_1-x_0)*(x_1-x_0));
	synthesis_set_target_state(total_distance, 0, 0);
	synthesis_set_current_state(0, 0, 0);
	total_T = synthesis_calc_T(total_distance, Vmax, Amax);
	synthesis_calc_coef(total_T);
	synthesis_start_time = (float)HAL_GetTick() / 1000; //s
	trajectory_start_time = synthesis_start_time;
	synthesis_translation_state = 1;
}

