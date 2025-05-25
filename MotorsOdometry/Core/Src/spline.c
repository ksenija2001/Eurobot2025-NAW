#include "spline.h"

sSpline_t spline;

void spline_init(){
	spline.index = -1;
	spline.distance.Kp = 0.1;
	spline.distance.Kd = 1;
	spline.angle.Kp = 60;
	spline.angle.Kd = 200;
	spline.detecion = 0;
}

int8_t spline_state(){
	return spline.index;
}

void spline_stop(){
	spline.index = -1;
}

float spline_calc(float* params, float t, uint8_t order, uint8_t derivative){
	float result = 0;
	switch(derivative){
	case 0:
		for(uint8_t i=0;i<order+1;i++){
			result += (factorial(order)/(factorial(i)*factorial(order-i))) * poow(1-t, order-i) * poow(t, i) * params[i];
		}
		break;
	case 1:
		for(uint8_t i=0;i<order;i++){
			result += order * (factorial(order-1)/(factorial(i)*factorial(order-i-1))) * poow(1-t, order-i-1) * poow(t, i) * (params[i+1]-params[i]);
		}
		break;
	case 2:
		for(uint8_t i=0;i<order-1;i++){
			result += order * (order-1)*(factorial(order-2)/(factorial(i)*factorial(order-i-2))) * poow(1-t, order-i-2) * poow(t, i) * (params[i+2]-2*params[i+1]+params[i]);
		}
		break;
	}
	return result;
}

float spline_calc_radius(float x_speed, float y_speed, float x_accel, float y_accel){
	return 1/((x_speed*y_accel - y_speed*x_accel)/sqrtf(poow(poow(x_speed, 2)+poow(y_speed, 2), 3)));
}

float spline_find_t(float *x_params, float *y_params, uint8_t order){
	float left_t = 0;
	float right_t = 1;
	float left_x, left_y, right_x, right_y, left_distance, right_distance;

	for(uint8_t i=0;i<FIND_NUM;i++){
		left_x = spline_calc(x_params, left_t, order, 0);
		left_y = spline_calc(y_params, left_t, order, 0);
		right_x = spline_calc(x_params, right_t, order, 0);
		right_y = spline_calc(y_params, right_t, order, 0);
		left_distance = distance(left_x, left_y, odom.x, odom.y);
		right_distance = distance(right_x, right_y, odom.x, odom.y);

		if(right_distance < left_distance){
			left_t = left_t/2 + right_t/2;
		}
		else{
			right_t = left_t/2 + right_t/2;
		}
	}
	return left_t/2 + right_t/2;
}

float spline_find_max_speed(){
	float max = 0;
	float x_speed;
	float y_speed;
	float speed;
	for(uint8_t i=0;i<spline.num_of_beziers;i++){
		for(float t=0;t<=1;t+=0.05){
			x_speed = spline_calc(spline.x_bezier[i].p, t, spline.x_bezier[i].order, 1);
			y_speed = spline_calc(spline.y_bezier[i].p, t, spline.y_bezier[i].order, 1);
			speed = magnitude(x_speed, y_speed);
			if(fabs(speed) > max){
				max = fabs(speed);
			}
		}
	}
	return max;
}

float spline_find_distance(float x, float y){
	float theta1 = atan2(y-odom.y,x-odom.x);
	float d = distance(x, y, odom.x, odom.y);
	if(theta1 - odom.theta > 0){
		return d;
	}
	else{
		return -d;
	}
 }

float spline_find_theta(float x0, float y0, float x, float y){
	return atan2f(y-y0, x-x0);
}

void spline_activate_detection(){
	odom.detection_activated = 1;
//	spline.detecion = 1;
	spline.num_of_beziers = spline.index + 1;
	float reverse = 0;
	if(spline.direction == 'r') reverse = M_PI;

	spline.x_bezier[spline.index].p[2] = odom.x;
	spline.x_bezier[spline.index].p[3] = odom.x + 100*cos(odom.theta + reverse);

	spline.y_bezier[spline.index].p[2] = odom.y;
	spline.y_bezier[spline.index].p[3] = odom.y + 100*sin(odom.theta + reverse);

	spline.end_angle = normalize(odom.theta + reverse);
}

void spline_move(float *x, float *y, float *theta, uint8_t num_of_points, float max_speed, char direction){
	spline.direction = direction;
	float reverse = 0;
	if (direction == 'r'){
		reverse = M_PI;
	}

	spline.num_of_beziers = num_of_points;
	for(uint8_t i = 0; i<spline.num_of_beziers; i++){
		spline.x_bezier[i].order = 3;
		spline.y_bezier[i].order = 3;
	}

	spline.x_bezier[0].p[0] = odom.x;
	spline.x_bezier[0].p[1] = odom.x + PARAM_DISTANCE*cos(odom.theta + reverse);
	spline.x_bezier[0].p[2] = x[0] + PARAM_DISTANCE*cos(theta[0] + reverse + M_PI);
	spline.x_bezier[0].p[3] = x[0];

	spline.y_bezier[0].p[0] = odom.y;
	spline.y_bezier[0].p[1] = odom.y + PARAM_DISTANCE*sin(odom.theta + reverse);
	spline.y_bezier[0].p[2] = y[0] + PARAM_DISTANCE*sin(theta[0] + reverse + M_PI);
	spline.y_bezier[0].p[3] = y[0];

	if(num_of_points > 1){
		for(uint8_t i = 1; i<spline.num_of_beziers; i++){
			spline.x_bezier[i].p[0] = x[i-1];
			spline.x_bezier[i].p[1] = x[i-1] + PARAM_DISTANCE*cos(theta[i-1] + reverse);
			spline.x_bezier[i].p[2] = x[i]   + PARAM_DISTANCE*cos(theta[i]   + reverse + M_PI);
			spline.x_bezier[i].p[3] = x[i];

			spline.y_bezier[i].p[0] = y[i-1];
			spline.y_bezier[i].p[1] = y[i-1] + PARAM_DISTANCE*sin(theta[i-1] + reverse);
			spline.y_bezier[i].p[2] = y[i]   + PARAM_DISTANCE*sin(theta[i]   + reverse + M_PI);
			spline.y_bezier[i].p[3] = y[i];
		}
	}

	spline.speed_coef = max_speed / spline_find_max_speed();
	spline.t_accel = max_speed / 10000.0;
	spline.t_deccel = max_speed / 6000.0; //8000.0;
	spline.end_angle = theta[num_of_points - 1];
	spline.index = 0;
}
float t= 0;
float left_velocity = 0, right_velocity = 0, velocity;
float last_left_velocity = 0, last_right_velocity = 0;
float x[2], y[2];

void spline_compute(){
	if(spline.index >= 0){
		if(odom.trans_vel > 20 && !odom.detection_activated){
			odom.detection_enable_front = 1;
			odom.detection_enable_back = 0;
		}
		else if(odom.trans_vel < -20 && !odom.detection_activated){
			odom.detection_enable_front = 0;
			odom.detection_enable_back = 1;
		}
		else{
			odom.detection_enable_front = 0;
			odom.detection_enable_back = 0;
		}

		t = spline_find_t(spline.x_bezier[spline.index].p, spline.y_bezier[spline.index].p, spline.x_bezier[spline.index].order);
		if(t < 0.01) t = 0.01;
		t += T_INCREMENT;
		x[0] 	  = spline_calc(spline.x_bezier[spline.index].p, t-T_INCREMENT, spline.x_bezier[spline.index].order, 0);
		y[0] 	  = spline_calc(spline.y_bezier[spline.index].p, t-T_INCREMENT, spline.y_bezier[spline.index].order, 0);
		x[1] 	  = spline_calc(spline.x_bezier[spline.index].p, t, spline.x_bezier[spline.index].order, 0);
		y[1] 	  = spline_calc(spline.y_bezier[spline.index].p, t, spline.y_bezier[spline.index].order, 0);
		float x_speed = spline_calc(spline.x_bezier[spline.index].p, t, spline.x_bezier[spline.index].order, 1);
		float y_speed = spline_calc(spline.y_bezier[spline.index].p, t, spline.y_bezier[spline.index].order, 1);
		float x_accel = spline_calc(spline.x_bezier[spline.index].p, t, spline.x_bezier[spline.index].order, 2);
		float y_accel = spline_calc(spline.y_bezier[spline.index].p, t, spline.y_bezier[spline.index].order, 2);
		float radius = spline_calc_radius(x_speed, y_speed, x_accel, y_accel);
		float distance_error = spline_find_distance(x[0], y[0]);
		float t_error = spline_find_theta(x[0], y[0], x[1], y[1]);

		spline.angle.last_error = spline.angle.current_error;
		spline.distance.last_error = spline.distance.current_error;
		if(spline.direction == 'f'){
			spline.angle.current_error = normalize(t_error - odom.theta);
			spline.distance.current_error = distance_error;
		}
		if(spline.direction == 'r') {
			spline.angle.current_error = normalize(t_error - (odom.theta + M_PI));
			spline.distance.current_error = -distance_error;
		}

		spline.angle.reg = spline.angle.Kp*spline.angle.current_error + (spline.angle.current_error - spline.angle.last_error) * spline.angle.Kd;
		spline.distance.reg = spline.distance.Kp*spline.distance.current_error + (spline.distance.current_error - spline.distance.last_error) * spline.distance.Kd;

		velocity = magnitude(x_speed, y_speed) * spline.speed_coef;
		last_left_velocity = left_velocity;
		last_right_velocity = right_velocity;
		left_velocity  = (radius - WHEEL_HALF_DISTANCE) * (velocity / radius);
		right_velocity = (radius + WHEEL_HALF_DISTANCE) * (velocity / radius);
		if(spline.direction == 'r'){
			float temp = left_velocity;
			left_velocity = -right_velocity;
			right_velocity = -temp;
		}
		//acceleration
		if(t<spline.t_accel && spline.index == 0){
			left_velocity *=t*(1/spline.t_accel);
			right_velocity *=t*(1/spline.t_accel);
		}
		//deceleration
		if(t>1-spline.t_deccel && spline.index == spline.num_of_beziers-1){
			left_velocity *= (1-t)*(1/spline.t_deccel);
			right_velocity *= (1-t)*(1/spline.t_deccel);
		}
		Set_Speed(&left_motor,   left_velocity - spline.distance.reg - spline.angle.reg);
		Set_Speed(&right_motor, right_velocity + spline.distance.reg + spline.angle.reg);
		if(t > 0.95 && odom.detection_activated){
			spline.index = -1;
			float reverse = -1;
			if(spline.direction == 'r'){
				reverse = 1;
			}
			synthesis_start_distance(100*reverse, 500, 1000);
		}

		if (t > 0.995){
			if(spline.index < spline.num_of_beziers){
				spline.index++;
			}
			if(spline.index == spline.num_of_beziers){
				spline.index = -1;
				if(!odom.detection_activated)
					synthesis_start_rotateTo(spline.end_angle, 1, 1);
				else{

				}
			}

		}
	}
}
