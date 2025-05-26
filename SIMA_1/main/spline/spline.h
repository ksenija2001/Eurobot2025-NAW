#ifndef INC_SPLINE_H_
#define INC_SPLINE_H_

#include <math.h>
#include "../utils/utils.h"
#include "../motor_control/motor_control.h"
#include "../control/control.h"
#include "../tof/tof.h"
#include "esp_log.h"

#define FIND_NUM 20
#define MAX_BEZIERS_IN_SPLINE 4
#define PARAM_DISTANCE 250	//250
#define T_INCREMENT 0.004
#define WHEEL_HALF_DISTANCE 42.0

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
	uint8_t detecion;
	bool stopped;
	bool no_detect;
} sSpline_t;

extern sSpline_t spline;

void spline_move(float *x, float *y, float *theta, uint8_t num_of_points, float max_speed, char direction, position_t robot);
void spline_compute(position_t robot, motor_t right_motor, motor_t left_motor, VL53LMZ_Interrupt_Zone int_zone);
void spline_init();
int8_t spline_state();
void spline_stop();
void spline_activate_detection(position_t robot);

#endif


