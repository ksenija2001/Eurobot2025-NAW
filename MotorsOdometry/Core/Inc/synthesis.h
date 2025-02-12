#include "arm_math.h"

void synthesis_calc_coef(float T);
void synthesis_calc_next_state(float t);
float poow(float a, int exp);
void synthesis_init();
void synthesis_set_target_state(float p, float v, float a);
void synthesis_set_current_state(float p, float v, float a);
