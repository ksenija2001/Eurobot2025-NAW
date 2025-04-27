#ifndef INC_SPLINE_H_
#define INC_SPLINE_H_

#include <stm32g4xx.h>
#include <math.h>
#include "utils.h"
#include "odom.h"
#include "motor_control.h"
#include "fdcan.h"

#define FIND_NUM 20
#define MAX_BEZIERS_IN_SPLINE 4
#define PARAM_DISTANCE 300
#define T_INCREMENT 0.002

typedef struct {
	float p[6];     // parameters of curvature
	uint8_t order;  // order of the polynomial
} sBezier_t;

typedef struct {
	float last_error;
	float current_error;
	float Kp;
	float Kd;
	float reg;
}sPD_t;

typedef struct{
	sBezier_t x_bezier[MAX_BEZIERS_IN_SPLINE];
	sBezier_t y_bezier[MAX_BEZIERS_IN_SPLINE];
	float speed_coef;
	uint8_t num_of_beziers;    // actual number of beziers in spline
	char direction;
	int8_t index;
	sPD_t distance;
	sPD_t angle;
	float t_accel;
	float t_deccel;
	float end_angle;
} sSpline_t;

void spline_move(float *x, float *y, float *theta, uint8_t num_of_points, float max_speed, char direction);
void spline_compute();
void spline_init();
int8_t spline_state();
void spline_stop();

#endif


