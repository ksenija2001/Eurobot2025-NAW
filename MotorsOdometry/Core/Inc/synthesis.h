#ifndef INC_SYNTHESIS_H_
#define INC_SYNTHESIS_H_

#include "arm_math.h"
#include "stm32g4xx_hal.h"
#include <math.h>
#include "odom.h"
#include "utils.h"

//extern uint8_t synthesis_translation_state;
//extern float synthesis_start_time;
//extern arm_matrix_instance_f32 NEXT_STATE;
//extern arm_matrix_instance_f32 TARGET_STATE;
//extern arm_matrix_instance_f32 CURRENT_STATE;
//extern float total_T;
//extern uint8_t synthesis_is_v2;
//extern float target[3][4];

typedef struct {
	float T;
	float P;
	float V;
	float A;
	char type;
	// dodati toleranciju
} sTarget_t;

//
//extern float start_x, start_y, start_theta, end_x, end_y, end_theta;
//extern int8_t synthesis_phase;

typedef struct{
	float error;
	float Kp;
}sPreg_t;

typedef struct{
	// Napraviti sPath_t
	float a;
	float b;
	float c;
} sTrajectory_t;

typedef struct{
	float x;
	float y;
	float theta;
} sPose_t;

typedef struct{
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
	float ai[36];
	arm_matrix_instance_f32 AI;
	float a[18];
	arm_matrix_instance_f32 A;

	sTarget_t target[6];
	uint8_t target_len;
	int8_t phase;
	float start_time;
	float total_time;

	// Spojiti u jednu structuru trajecotory
	sPose_t start;
	sPose_t end;
	sTrajectory_t traj;

	// Napraviti strukturu sa total, traveled i current - distance i angle
	float total_distance;
	float total_angle;
	float distance_from_start;
	float angle_from_start;
	float last_angle;
	sPreg_t distance;
	sPreg_t angle;

} sSynthesis_t;

//void synthesis_calc_coef(float T);
//void synthesis_calc_next_state(float t);
void synthesis_init();
int8_t synthesis_state();
void synthesis_stop();
//void synthesis_set_target_state(float p, float v, float a);
//void synthesis_set_current_state(float p, float v, float a);

void synthesis_start_distance(float distance, float Vmax, float Amax);
void synthesis_start_XY(float x, float y, char direction, float Vmax, float Amax, float Wmax, float amax);
void synthesis_start_rotateFor(float theta, float Wmax, float amax);
void synthesis_start_rotateTo(float theta, float Wmax, float amax);
void synthesis_activate_detection(float backing_distance);
void synthesis_compute();

#endif
