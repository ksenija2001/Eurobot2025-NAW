#ifndef INC_SYNTHESIS_H_
#define INC_SYNTHESIS_H_

#include "arm_math.h"
#include "stm32g4xx_hal.h"
#include "odom.h"
#include <math.h>

extern uint8_t synthesis_translation_state;
extern float synthesis_start_time;
extern arm_matrix_instance_f32 NEXT_STATE;
extern arm_matrix_instance_f32 TARGET_STATE;
extern arm_matrix_instance_f32 CURRENT_STATE;
extern float total_T;
extern uint8_t synthesis_is_v2;
extern float target[3][4];

typedef struct {
	float T;
	float P;
	float V;
	float A;
	char type;
	uint8_t version;
} sTarget_t;

extern float start_x, start_y, start_theta, end_x, end_y, end_theta;

void synthesis_calc_coef(float T);
void synthesis_calc_next_state(float t);
float poow(float a, int exp);
void synthesis_init();
void synthesis_set_target_state(float p, float v, float a);
void synthesis_set_current_state(float p, float v, float a);

void synthesis_start_distance(float distance, float Vmax, float Amax);
void synthesis_start_XY(float x, float y, char direction, float Vmax, float Amax, float Wmax, float amax);
void synthesis_start_rotateFor(float theta, float Wmax, float amax);
void synthesis_start_rotateTo(float theta, float Wmax, float amax);

void synthesis_compute();

#endif
