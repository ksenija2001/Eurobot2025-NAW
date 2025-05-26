#include "spline.h"

sSpline_t spline;

void spline_init(){
	spline.index = -1;	//Defines in which of N Beziers the robot is currently moving in, -1 means no movement
	spline.distance.Kp = 1;   //0.3
	spline.distance.Kd = 12;
	spline.angle.Kp = 10.0;  //150
	spline.angle.Kd = 0.0;
	spline.detecion = 1;
	spline.stopped = false;
	spline.no_detect = true;
}

int8_t spline_state(){
	return spline.index;
}

void spline_stop(){
	spline.index = -1;
}

/* 	Spline_calc is a basic function for calculating the position, velocity and acceleration of a Bezier curve in
	a point t.
	Params are control points for Bezier curve
	t is a position on the curve defined in a 0 to 1 range
	Order defines the order of the Bezier curve - we use cubic (3)
	Derivative defines which derivative of the Bezier we want
*/
float spline_calc(float* params, float t, uint8_t order, uint8_t derivative){
	float result = 0;
	switch(derivative){
	case 0: //Position
	/* 	The result is based on the equation for the Bezier curve and returns the value of the curve at a set point t
		Bernstein polynomial for t is multiplied by the paramater (control point). That is iterated 4 times (number of control points) and summed togehter
	*/
		for(uint8_t i=0;i<order+1;i++){
			result += (factorial(order)/(factorial(i)*factorial(order-i))) * poow(1-t, order-i) * poow(t, i) * params[i];
		}
		break;
	case 1:	//Velocity
	//	Every derivative of the Bezier curve is another Bezier curve of a lower order
		for(uint8_t i=0;i<order;i++){
			result += order * (factorial(order-1)/(factorial(i)*factorial(order-i-1))) * poow(1-t, order-i-1) * poow(t, i) * (params[i+1]-params[i]);
		}
		break;
	case 2:	//Acceleration
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

/*	Spline_find_t helps us find our t, which defines where on our trajectory we currently are
	Left_t is 0 and right_t is 1. This gives us an interval of 0 to 1
	We calculate left and right x and y for left_t and right_t and based on those values compare their distances to the current robot position
	Depending on which distance is bigger, we move the t of that distance (left or right) more towards the middle
	This is iterated a set number of times, so that we can get a good approximation of our t
*/
float spline_find_t(float *x_params, float *y_params, uint8_t order, position_t robot){
	float left_t = 0;
	float right_t = 1;
	float left_x, left_y, right_x, right_y, left_distance, right_distance;

	for(uint8_t i=0;i<FIND_NUM;i++){
		left_x = spline_calc(x_params, left_t, order, 0);
		left_y = spline_calc(y_params, left_t, order, 0);
		right_x = spline_calc(x_params, right_t, order, 0);
		right_y = spline_calc(y_params, right_t, order, 0);
		left_distance = distance_calc(left_x, left_y, robot.x_pos, robot.y_pos);
		right_distance = distance_calc(right_x, right_y, robot.x_pos, robot.y_pos);

		if(right_distance < left_distance){
			left_t = left_t/2 + right_t/2;
		}
		else{
			right_t = left_t/2 + right_t/2;
		}
	}
    //ESP_LOGW("Spline", "t: %f", left_t/2 + right_t/2);
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
    ESP_LOGW("Spline", "Max Speed: %f", max);
	return max;
}

float spline_find_distance(float x, float y, position_t robot){
	float theta1 = atan2(y-robot.y_pos, x-robot.x_pos);
    //ESP_LOGI("Spline", "Thet: %f RealAng: %f", theta1, robot.angle_in_rad);
	float d = distance_calc(x, y, robot.x_pos, robot.y_pos);
	if(theta1 - robot.angle_in_rad > 0){
		return d;
	}
	else{
		return -d;
	}
 }

float spline_find_theta(float x0, float y0, float x, float y){
	return atan2f(y-y0, x-x0);
}

void spline_activate_detection(position_t robot){
	//odom.detection_activated = 1;
//	spline.detecion = 1;
	spline.num_of_beziers = spline.index + 1;
	float reverse = 0;
	if(spline.direction == 'r') reverse = M_PI;

	spline.x_bezier[spline.index].p[2] = robot.x_pos;
	spline.x_bezier[spline.index].p[3] = robot.x_pos + 100*cos(robot.angle_in_rad + reverse);

	spline.y_bezier[spline.index].p[2] = robot.y_pos;
	spline.y_bezier[spline.index].p[3] = robot.y_pos + 100*sin(robot.angle_in_rad + reverse);

	//spline.end_angle = normalize(robot.angle_in_rad + reverse);
}

/*	Spline_move defines the movement we are going to have
	Parameters x and y are arrays of points that are going to define our movement from the second point (not the current position) to the end
	Theta defines targeted orientation
	Num_of_points defines the number of Beziers used for the spline
	Max_speed and direction are pretty obvious
*/
void spline_move(float *x, float *y, float *theta, uint8_t num_of_points, float max_speed, char direction, position_t robot){
	spline.direction = direction;
	spline.stopped = false;
	float reverse = 0;
	if (direction == 'r'){
		reverse = M_PI;
	}

	//Assigning a number of Beziers to the spline
	spline.num_of_beziers = num_of_points;
	for(uint8_t i = 0; i<spline.num_of_beziers; i++){
		spline.x_bezier[i].order = 3;
		spline.y_bezier[i].order = 3;
	}

	/*	Defining the control points of the first Bezier
		First and fourth parameters are start and finish control points
		Second and third parameters are based on the current and targeted orientation
	*/
	spline.x_bezier[0].p[0] = robot.x_pos;
	spline.x_bezier[0].p[1] = robot.x_pos + PARAM_DISTANCE*cos(robot.angle_in_rad + reverse);
	spline.x_bezier[0].p[2] = x[0] + PARAM_DISTANCE*cos(theta[0] + reverse + M_PI);
	spline.x_bezier[0].p[3] = x[0];
    ESP_LOGW("Spline", "XP0: %f XP1: %f XP2: %f XP3: %f", spline.x_bezier[0].p[0], spline.x_bezier[0].p[1], spline.x_bezier[0].p[2], spline.x_bezier[0].p[3]);

	spline.y_bezier[0].p[0] = robot.y_pos;
	spline.y_bezier[0].p[1] = robot.y_pos + PARAM_DISTANCE*sin(robot.angle_in_rad + reverse);
	spline.y_bezier[0].p[2] = y[0] + PARAM_DISTANCE*sin(theta[0] + reverse + M_PI);
	spline.y_bezier[0].p[3] = y[0];
    ESP_LOGW("Spline", "YP0: %f YP1: %f YP2: %f YP3: %f T: %f", spline.y_bezier[0].p[0], spline.y_bezier[0].p[1], spline.y_bezier[0].p[2], spline.y_bezier[0].p[3], theta[0]);


	if(num_of_points > 1){
		for(uint8_t i = 1; i<spline.num_of_beziers; i++){
			spline.x_bezier[i].p[0] = x[i-1];
			spline.x_bezier[i].p[1] = x[i-1] + PARAM_DISTANCE*cos(theta[i-1] + reverse);
			spline.x_bezier[i].p[2] = x[i]   + PARAM_DISTANCE*cos(theta[i]   + reverse + M_PI);
			spline.x_bezier[i].p[3] = x[i];
			ESP_LOGW("Spline", "XP0: %f XP1: %f XP2: %f XP3: %f T: %f", spline.x_bezier[i].p[0], spline.x_bezier[i].p[1], spline.x_bezier[i].p[2], spline.x_bezier[i].p[3], theta[i]);

			spline.y_bezier[i].p[0] = y[i-1];
			spline.y_bezier[i].p[1] = y[i-1] + PARAM_DISTANCE*sin(theta[i-1] + reverse);
			spline.y_bezier[i].p[2] = y[i]   + PARAM_DISTANCE*sin(theta[i]   + reverse + M_PI);
			spline.y_bezier[i].p[3] = y[i];
			ESP_LOGW("Spline", "YP0: %f YP1: %f YP2: %f YP3: %f T: %f", spline.y_bezier[i].p[0], spline.y_bezier[i].p[1], spline.y_bezier[i].p[2], spline.y_bezier[i].p[3], theta[i]);

		}
	}

	spline.speed_coef = max_speed / spline_find_max_speed();
	spline.t_accel = max_speed / 3500.0;
	spline.t_deccel = max_speed / 2000.0;
    ESP_LOGW("Spline", "SC: %f TA: %f DA: %f", spline.speed_coef, spline.t_accel, spline.t_deccel);
	spline.end_angle = theta[num_of_points - 1];
	spline.index = 0;
}
float t= 0;
float left_velocity = 0, right_velocity = 0, velocity;
float last_left_velocity = 0, last_right_velocity = 0;
float x_pos[2], y_pos[2];

/*	Spline_compute is called constantly during movement
*/
void spline_compute(position_t robot, motor_t right_motor, motor_t left_motor, VL53LMZ_Interrupt_Zone int_zone){
	if(spline.index >= 0){

		// It first calculates an approximation of t that is as close as possible to the perfect current position of the robot
		t = spline_find_t(spline.x_bezier[spline.index].p, spline.y_bezier[spline.index].p, spline.x_bezier[spline.index].order, robot);
        if(t < 0.01) t = 0.01;
		//ESP_LOGI("Spline", "t: %f", t);
		//Then it increments it to assume where the robot is going to be in the next iteration
		t += T_INCREMENT;
		x_pos[0] 	  = spline_calc(spline.x_bezier[spline.index].p, t-T_INCREMENT, spline.x_bezier[spline.index].order, 0);
		y_pos[0] 	  = spline_calc(spline.y_bezier[spline.index].p, t-T_INCREMENT, spline.y_bezier[spline.index].order, 0);
		x_pos[1] 	  = spline_calc(spline.x_bezier[spline.index].p, t, spline.x_bezier[spline.index].order, 0);
		y_pos[1] 	  = spline_calc(spline.y_bezier[spline.index].p, t, spline.y_bezier[spline.index].order, 0);
		//We need the speed and acceleration in order to get the radius of the curve at this point
		float x_speed = spline_calc(spline.x_bezier[spline.index].p, t, spline.x_bezier[spline.index].order, 1);
		float y_speed = spline_calc(spline.y_bezier[spline.index].p, t, spline.y_bezier[spline.index].order, 1);
		float x_accel = spline_calc(spline.x_bezier[spline.index].p, t, spline.x_bezier[spline.index].order, 2);
		float y_accel = spline_calc(spline.y_bezier[spline.index].p, t, spline.y_bezier[spline.index].order, 2);
		float radius = spline_calc_radius(x_speed, y_speed, x_accel, y_accel);
		//Distance and theta errors define how far from the target trajectory we are
		float distance_error = spline_find_distance(x_pos[0], y_pos[0], robot);
		float t_error = spline_find_theta(x_pos[0], y_pos[0], x_pos[1], y_pos[1]);
        //ESP_LOGI("Spline", "t: %f X0: %f Y0: %f X1: %f Y1: %f RX: %f RY: %f", t, x_pos[0], y_pos[0], x_pos[1], y_pos[1], robot.x_pos, robot.y_pos);
        //ESP_LOGI("Spline", "DE: %f TE: %f", distance_error, t_error);
        //ESP_LOGI("Spline", "XS0: %f YS0: %f XS1: %f YS1: %f", x_speed, y_speed, x_accel, y_accel);

		//Defining errors for PD
		spline.angle.last_error = spline.angle.current_error;
		spline.distance.last_error = spline.distance.current_error;
		if(spline.direction == 'f'){
			spline.angle.current_error = t_error - robot.angle_in_rad;
            if (spline.angle.current_error > M_PI) spline.angle.current_error -= 2*M_PI;
            else if (spline.angle.current_error < -M_PI) spline.angle.current_error += 2*M_PI;
			spline.distance.current_error = distance_error;
            //ESP_LOGI("Spline", "SDE: %f SAE: %f", spline.distance.current_error, spline.angle.current_error);
		}
		if(spline.direction == 'r') {
			//spline.angle.current_error = normalize(t_error - (robot.angle_in_rad + M_PI));
			spline.distance.current_error = -distance_error;
		}

		spline.angle.reg = spline.angle.Kp*spline.angle.current_error + (spline.angle.current_error - spline.angle.last_error) * spline.angle.Kd;

		if (robot.angle_in_rad < -1.57 || robot.angle_in_rad > 1.57)
		{
			spline.distance.Kp = 0.0;
			spline.distance.Kd = 0.0;
			spline.angle.Kp = 10.0;
			spline.angle.Kd = 0.0;
		}

		spline.distance.reg = spline.distance.Kp*spline.distance.current_error + (spline.distance.current_error - spline.distance.last_error) * spline.distance.Kd;
        ESP_LOGI("Spline", "AReg: %f DReg: %f", spline.angle.reg, spline.distance.reg);

		velocity = magnitude(x_speed, y_speed) * spline.speed_coef;
		last_left_velocity = left_velocity;
		last_right_velocity = right_velocity;
		left_velocity  = (radius - WHEEL_HALF_DISTANCE) * (velocity / radius);
		right_velocity = (radius + WHEEL_HALF_DISTANCE) * (velocity / radius);
        ESP_LOGI("Spline", "LV: %f RV: %f", left_velocity, right_velocity);
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
		//Set_Speed(&left_motor,   left_velocity - spline.distance.reg - spline.angle.reg);
		//Set_Speed(&right_motor, right_velocity + spline.distance.reg + spline.angle.reg);
        float left_speed = left_velocity - spline.distance.reg - spline.angle.reg;
		if (left_speed < 0.0) left_speed = 0.0;
		float right_speed = right_velocity + spline.distance.reg + spline.angle.reg;
		if (right_speed < 0.0) right_speed = 0.0;
		float speed_l = pid_l(left_motor, left_speed);
		float speed_r = pid_r(right_motor, right_speed);
		ESP_LOGW("Spline", "LS: %f RS: %f", left_speed, right_speed);
		motor_forward(left_motor, speed_l);
        motor_forward(right_motor, speed_r);
		/*if(t > 0.95 && odom.detection_activated){
			spline.index = -1;
			float reverse = -1;
			if(spline.direction == 'r'){
				reverse = 1;
			}
			synthesis_start_distance(100*reverse, 500, 1000);
		}*/

		/*if (t > 0.995){
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

		}*/
        if (t > 0.95) 
        {
            if (spline.index < spline.num_of_beziers) spline.index ++;
            if (spline.index == spline.num_of_beziers)
            {
				spline.stopped = true;
				//finished_movement = true;
                spline.index = -1;
                motors_stop(right_motor, left_motor);
            }
        }

		if (t>0.2) spline.no_detect = false;
	}
}
