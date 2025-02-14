#include "arm_math.h"
#include "stm32g4xx_hal.h"

extern uint8_t synthesis_translation_state;
extern float synthesis_start_time;
extern float trajectory_start_time;
extern arm_matrix_instance_f32 NEXT_STATE;
extern float total_T;

void synthesis_calc_coef(float T);
void synthesis_calc_next_state(float t);
float poow(float a, int exp);
void synthesis_init();
void synthesis_set_target_state(float p, float v, float a);
void synthesis_set_current_state(float p, float v, float a);
float synthesis_calc_Vmax(float Pmax, float Amax);
float synthesis_calc_Amax(float Pmax, float Vmax);
float synthesis_calc_T_a(float Pmax, float Amax);
float synthesis_calc_T_v(float Pmax, float Vmax);
//void synthesis_start_rotation(float Vmax, float Amax, float start_theta, float theta);
void synthesis_start_distance(float Vmax, float Amax, float start_x, float start_y, float start_theta, float distance);
