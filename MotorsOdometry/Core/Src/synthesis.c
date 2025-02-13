#include "synthesis.h"
#include <math.h>


float coef_3[3]; 				//[a5, a4, a3]
arm_matrix_instance_f32 COEF_3;
float coef_6[6]; 				//[a5, a4, a3, a2, a1, a0]
arm_matrix_instance_f32 COEF_6;
float current_state[3];
arm_matrix_instance_f32 CURRENT_STATE;
float target_state[3];
arm_matrix_instance_f32 TARGET_STATE;
float next_state[3];
arm_matrix_instance_f32 NEXT_STATE;

float a_3[9] = {0};
arm_matrix_instance_f32 A_3;
float ai_3[9] = {0};
arm_matrix_instance_f32 AI_3;
float a_6[18];
arm_matrix_instance_f32 A_6;

float x_0, x_1, y_0, y_1;
float total_distance;
float total_T;
uint32_t synthesis_start_time = 0;
uint8_t  synthesis_translation_state = 0;


void synthesis_init(){
	arm_mat_init_f32(&CURRENT_STATE, 3, 1, current_state);
	arm_mat_init_f32(&TARGET_STATE, 3, 1, target_state);
	arm_mat_init_f32(&NEXT_STATE, 3, 1, next_state);
	arm_mat_init_f32(&COEF_3, 3, 1, coef_3);
	arm_mat_init_f32(&COEF_6, 6, 1, coef_6);
	arm_mat_init_f32(&A_3, 3, 3, a_3);
	arm_mat_init_f32(&AI_3, 3, 3, ai_3);
	arm_mat_init_f32(&A_6, 3, 6, a_6);
}

float poow(float a, int exp){
	float res = 1;
	for(int i=0; i<exp; i++){
		res *= a;
	}
	return res;;
}

void synthesis_calc_coef(float T){
	A_3.pData[0*3 + 0] = poow(T, 5);
	A_3.pData[0*3 + 1] = poow(T, 4);
	A_3.pData[0*3 + 2] = poow(T, 3);

	A_3.pData[1*3 + 0] = 5*poow(T, 4);
	A_3.pData[1*3 + 1] = 4*poow(T, 5);
	A_3.pData[1*3 + 2] = 3*poow(T, 2);

	A_3.pData[2*3 + 0] = 20*poow(T, 3);
	A_3.pData[2*3 + 1] = 12*poow(T, 2);
	A_3.pData[2*3 + 2] = 6* poow(T, 1);

	arm_mat_inverse_f32(&A_3, &AI_3);
	arm_mat_mult_f32(&AI_3, &TARGET_STATE, &COEF_3);

	COEF_6.pData[0] = COEF_3.pData[0];	  // a5
	COEF_6.pData[1] = COEF_3.pData[1];    // a4
	COEF_6.pData[2] = COEF_3.pData[2];    // a3
	COEF_6.pData[3] = current_state[2]/2; //a2 = current A / 2
	COEF_6.pData[4] = current_state[1];   //a1 = current V
	COEF_6.pData[5] = current_state[0];   //a0 = current P
}

void synthesis_calc_next_state(float t){
	A_6.pData[0*3 + 0] = poow(t, 5);
	A_6.pData[0*3 + 1] = poow(t, 4);
	A_6.pData[0*3 + 2] = poow(t, 3);
	A_6.pData[0*3 + 3] = poow(t, 2);
	A_6.pData[0*3 + 4] = poow(t, 1);
	A_6.pData[0*3 + 5] = poow(t, 0);

	A_6.pData[1*3 + 0] = 5* poow(t, 4);
	A_6.pData[1*3 + 1] = 4* poow(t, 3);
	A_6.pData[1*3 + 2] = 3* poow(t, 2);
	A_6.pData[1*3 + 3] = 2* poow(t, 1);
	A_6.pData[1*3 + 4] = 1* poow(t, 0);
	A_6.pData[1*3 + 5] = 0;

	A_6.pData[2*3 + 0] = 20* poow(t, 3);
	A_6.pData[2*3 + 1] = 12* poow(t, 2);
	A_6.pData[2*3 + 2] = 6* poow(t, 1);
	A_6.pData[2*3 + 3] = 2* poow(t, 0);
	A_6.pData[2*3 + 4] = 0;
	A_6.pData[2*3 + 5] = 0;

	arm_mat_mult_f32(&A_6, &COEF_6, &NEXT_STATE);
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
	return sqrtf((5000*Pmax*Amax)/(2880));
}
float synthesis_calc_Amax(float Pmax, float Vmax){
	return (2880*Vmax*Vmax)/(5000*Pmax);
}
float synthesis_calc_T_a(float Pmax, float Amax){
	return sqrtf((45*Pmax)/(8*Amax));
}
float synthesis_calc_T_v(float Pmax, float Vmax){
	return (15*Pmax)/(8*Vmax);
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
	float T_Vmax = synthesis_calc_T_v(total_distance, Vmax);
	float T_Amax = synthesis_calc_T_a(total_distance, Amax);
	if(T_Vmax > T_Amax){
		total_T = T_Vmax;
	}
	else{
		total_T = T_Amax;
	}
	synthesis_calc_coef(total_T);
	synthesis_start_time = HAL_GetTick();
	synthesis_translation_state = 1;
}

