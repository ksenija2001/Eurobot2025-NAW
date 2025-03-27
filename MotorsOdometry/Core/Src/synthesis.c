#include "synthesis.h"

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

float total_distance;
float total_theta;
float total_T;
float synthesis_start_time = 0;
int8_t synthesis_phase = -1;
uint8_t synthesis_target_len = 0;
float start_x, start_y, start_theta;
float end_x, end_y, end_theta;
float last_theta;

float _a, _b, _c;
float distance_error;

float distance_from_start = 0, theta_from_start = 0;

sTarget_t synthesis_target[6];

void calc_traj_coef(){
	_a = -(end_y - start_y)/(end_x - start_x);
	_b = 1;
	_c = (end_y - start_y)/(end_x - start_x) - start_y;
}

void calc_distance_from_traj(){
	distance_error = (_a*odom.x + _b*odom.y + _c)/sqrtf(_a*_a + _b*_b);
}

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
//	if(NEXT_STATE.pData[2] >=  2000) NEXT_STATE.pData[2] =  2000;
//	if(NEXT_STATE.pData[2] <= -2000) NEXT_STATE.pData[2] = -2000;
//	if(NEXT_STATE.pData[1] >=  1000) NEXT_STATE.pData[1] =  1000;
//	if(NEXT_STATE.pData[1] <= -1000) NEXT_STATE.pData[1] = -1000;
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

void synthesis_set_next_state(float p, float v, float a){
	NEXT_STATE.pData[0] = p;
	NEXT_STATE.pData[1] = v;
	NEXT_STATE.pData[2] = a;
}

void synthesis_set_init_target_state(){
	synthesis_set_target_state(synthesis_target[0].P,synthesis_target[0].V,synthesis_target[0].A);
}

float synthesis_calc_V(float Pmax, float Amax){
	return sqrtf((5*fabs(Pmax)*Amax)/8);
}
float synthesis_calc_T(float Pmax, float Amax){
	return sqrtf((45*fabs(Pmax))/(8*Amax));
}

float synthesis_calc_P(float T, float Amax){
	return (8.0*Amax*T*T)/45;
}

void synthesis_calc_target_points(float P, float Vmax, float Amax, char type){ //decide is it synthesis 2.0 or 1.0
	if(synthesis_calc_V(P, Amax) > Vmax){
		//synthesis 2.0
		float T1, T2, T3;
		float P1, P2, P3;
		float V1, V2, V3;

		T1 = (3.0*Vmax)/(2.0*Amax);
		T3 = T1;
		P1 = synthesis_calc_P(T1, Amax) * 2;
		P2 = P - P1;
		P3 = P;
		T2 = (P2 - P1)/Vmax;
		V1 = V2 = Vmax;
		V3 = 0;

		synthesis_target[synthesis_target_len].T = T1;
		synthesis_target[synthesis_target_len].P = P1;
		synthesis_target[synthesis_target_len].V = V1;
		synthesis_target[synthesis_target_len].A = 0;		 //accel = 0
		synthesis_target[synthesis_target_len].type = type;
		synthesis_target[synthesis_target_len].version = 2;
		synthesis_target_len++;
		synthesis_target[synthesis_target_len].T = T2;
		synthesis_target[synthesis_target_len].P = P2;
		synthesis_target[synthesis_target_len].V = V2;
		synthesis_target[synthesis_target_len].A = 0;		 //accel = 0
		synthesis_target[synthesis_target_len].type = type;
		synthesis_target[synthesis_target_len].version = 2;
		synthesis_target_len++;
		synthesis_target[synthesis_target_len].T = T3;
		synthesis_target[synthesis_target_len].P = P3;
		synthesis_target[synthesis_target_len].V = V3;
		synthesis_target[synthesis_target_len].A = 0;		 //accel = 0
		synthesis_target[synthesis_target_len].type = type;
		synthesis_target[synthesis_target_len].version = 2;
		synthesis_target_len++;

		total_T = T1+T2+T3;

	}
	else{
		total_T = synthesis_calc_T(P, Amax);

		synthesis_target[synthesis_target_len].T = total_T;
		synthesis_target[synthesis_target_len].P = P;
		synthesis_target[synthesis_target_len].V = 0;
		synthesis_target[synthesis_target_len].A = 0;		 //accel = 0
		synthesis_target[synthesis_target_len].type = type;
		synthesis_target[synthesis_target_len].version = 1;
		synthesis_target_len++;
	}
}

void synthesis_start_XY(float x, float y, char direction, float Vmax, float Amax, float Wmax, float amax){
	synthesis_target_len = 0;
	start_x = odom.x;
	start_y = odom.y;
	start_theta = odom.theta;
	end_x = x;
	end_y = y;
	end_theta = atan2f(end_y - start_y, end_x - start_x);
	total_distance = sqrtf((end_x - start_x) * (end_x - start_x) + (end_y - start_y) * (end_y - start_y));
	if(direction == 'r'){
		end_theta += M_PI;
		if(end_theta > M_PI) end_theta -= 2*M_PI;
		if(end_theta < M_PI) end_theta += 2*M_PI;
		total_distance = -total_distance;
	}
	total_theta = end_theta - start_theta;
	if(total_theta >  M_PI) total_theta -= 2*M_PI;
	if(total_theta < -M_PI) total_theta += 2*M_PI;

	synthesis_calc_target_points(total_theta, Wmax, amax, 'r');
	synthesis_calc_target_points(total_distance, Vmax, Amax, 't');

	synthesis_set_current_state(0, 0, 0);
	synthesis_start_time = (float)HAL_GetTick() / 1000; //s
	distance_from_start = 0;
	theta_from_start = 0;
	synthesis_set_init_target_state();
	last_theta = odom.theta;
	synthesis_phase = 0;
}
void synthesis_start_rotateFor(float theta, float Wmax, float amax){
	synthesis_target_len = 0;
	start_theta = odom.theta;
	total_theta = theta;

	synthesis_calc_target_points(total_theta, Wmax, amax, 'r');

	synthesis_set_current_state(0, 0, 0);
	synthesis_start_time = (float)HAL_GetTick() / 1000; //s
	theta_from_start = 0;
	synthesis_set_init_target_state();
	last_theta = odom.theta;
	synthesis_phase = 0;
}

void synthesis_start_rotateTo(float theta, float Wmax, float amax){
	synthesis_target_len = 0;
	start_theta = odom.theta;
	total_theta = theta - odom.theta;
	if(total_theta >  M_PI) total_theta -= 2*M_PI;
	if(total_theta < -M_PI) total_theta += 2*M_PI;
	end_theta = theta;

	synthesis_calc_target_points(total_theta, Wmax, amax, 'r');

	synthesis_set_current_state(0, 0, 0);
	synthesis_start_time = (float)HAL_GetTick() / 1000; //s
	theta_from_start = 0;
	synthesis_set_init_target_state();
	last_theta = odom.theta;
	synthesis_phase = 0;
}


void synthesis_start_distance(float distance,float Vmax, float Amax){
	synthesis_target_len = 0;
	start_x = odom.x;
	start_y = odom.y;
	start_theta = odom.theta;
	total_distance = distance;
	end_x = start_x + distance*cos(odom.theta);
	end_y = start_y + distance*sin(odom.theta);

	calc_traj_coef();
	synthesis_calc_target_points(distance, Vmax, Amax, 't');

	synthesis_set_current_state(0, 0, 0);
	synthesis_set_next_state(0, 0, 0);
	synthesis_start_time = (float)HAL_GetTick() / 1000; //s
	distance_from_start = 0;
	synthesis_set_init_target_state();
	last_theta = odom.theta;
	synthesis_phase = 0;
}

void synthesis_compute(){
	if(synthesis_phase >= 0){ //If synthesis is activated
		float tolerance;
		if(synthesis_target[synthesis_phase].type == 'r' && synthesis_target[synthesis_phase].V != 0) tolerance = 0.1;
		if(synthesis_target[synthesis_phase].type == 'r' && synthesis_target[synthesis_phase].V == 0) tolerance = 0.01;
		if(synthesis_target[synthesis_phase].type == 't' && synthesis_target[synthesis_phase].V != 0) tolerance = 1;
		if(synthesis_target[synthesis_phase].type == 't' && synthesis_target[synthesis_phase].V == 0) tolerance = 1;

		if(CURRENT_STATE.pData[0] > TARGET_STATE.pData[0] - tolerance){ // if robot passed target position
			synthesis_phase++;								//increment phase
			synthesis_set_current_state(synthesis_target[synthesis_phase-1].P, synthesis_target[synthesis_phase-1].V, synthesis_target[synthesis_phase-1].A);
			synthesis_set_next_state(synthesis_target[synthesis_phase-1].P, synthesis_target[synthesis_phase-1].V, synthesis_target[synthesis_phase-1].A);

			if(synthesis_phase < synthesis_target_len){ // if there is more target in synthesis
				synthesis_set_target_state(synthesis_target[synthesis_phase].P, synthesis_target[synthesis_phase].V, synthesis_target[synthesis_phase].A);
				synthesis_start_time = (float)HAL_GetTick()/1000;
				if(synthesis_target[synthesis_phase - 1].type == 'r' && synthesis_target[synthesis_phase].type == 't'){ //switched from rotation to translation
					start_x = odom.x;
					start_y = odom.y;
					total_distance = sqrtf((end_x - start_x) * (end_x - start_x) + (end_y - start_y) * (end_y - start_y));
					synthesis_target[synthesis_target_len - 1].P = total_distance - synthesis_target[synthesis_target_len - 2].P;
				}
			}
			else{ 										//robot is in end postition
				synthesis_phase = -1;
			}
		}
		else{ //compute synthesis
			float left_time = synthesis_start_time + synthesis_target[synthesis_phase].T - (float)HAL_GetTick()/1000;

			distance_from_start = sqrtf((odom.x - start_x) * (odom.x - start_x) + (odom.y - start_y) * (odom.y - start_y));
			float dTheta = odom.theta - last_theta;
			if(dTheta > M_PI){
				dTheta -= 2*M_PI;
			}
			if(dTheta < -M_PI){
				dTheta += 2*M_PI;
			}
			theta_from_start += dTheta;
			last_theta = odom.theta;

			synthesis_calc_coef(left_time);
			synthesis_calc_next_state(SYNTHESIS_TIME/1000.0);
		}
		if(synthesis_target[synthesis_phase].type == 't'){
			calc_distance_from_traj();
//			if(distance_from_start < total_distance - 100)
//				synthesis_set_current_state(NEXT_STATE.pData[0], NEXT_STATE.pData[1], NEXT_STATE.pData[2]);
//			else
			synthesis_set_current_state(distance_from_start, NEXT_STATE.pData[1], NEXT_STATE.pData[2]);
			Set_Speed(&left_motor, NEXT_STATE.pData[1] - (2*distance_error));
			Set_Speed(&right_motor, NEXT_STATE.pData[1] + (2*distance_error));
		}
		if(synthesis_target[synthesis_phase ].type == 'r'){
			synthesis_set_current_state(NEXT_STATE.pData[0], NEXT_STATE.pData[1], NEXT_STATE.pData[2]);
			Set_Speed(&left_motor, -NEXT_STATE.pData[1] * WHEEL_HALF_DISTANCE);
			Set_Speed(&right_motor, NEXT_STATE.pData[1] * WHEEL_HALF_DISTANCE);
		}

	}
	else{
		Set_RPM(&left_motor, 0);
		Set_RPM(&right_motor, 0);
	}
}

