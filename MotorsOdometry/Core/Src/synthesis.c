#include "synthesis.h"

sSynthesis_t synthesis;

//float total_distance;
//float total_theta;
//float total_T;
//float synthesis_start_time = 0;
//int8_t synthesis_phase = -1;
//uint8_t synthesis_target_len = 0;
//float start_x, start_y, start_theta;
//float end_x, end_y, end_theta;
//float last_theta;
//
//float _a, _b, _c;
//float distance_error;
//float theta_error;
//
//float distance_from_start = 0, theta_from_start = 0;
//float init_angle;
//
//sTarget_t synthesis_target[6];

void synthesis_start(){
	synthesis.phase = 0;
}

int8_t synthesis_state(){
	return synthesis.phase;
}

//void calc_traj_coef(){
//	synthesis.traj.a = -(synthesis.end.y - synthesis.start.y)/(synthesis.end.x - synthesis.start.x);
//	synthesis.traj.b = 1;
//	synthesis.traj.c = (synthesis.end.y - synthesis.start.y)/(synthesis.end.x - synthesis.start.x) - synthesis.start.y;
//}

float calc_distance_from_traj(){
//	return fabs(synthesis.traj.a*odom.x + synthesis.traj.b*odom.y + synthesis.traj.c)/sqrtf(synthesis.traj.a*synthesis.traj.a + synthesis.traj.b*synthesis.traj.b);
	return fabs((synthesis.end.y-synthesis.start.y)*odom.x - (synthesis.end.x-synthesis.start.x)*odom.y + synthesis.end.x*synthesis.start.y - synthesis.end.y*synthesis.start.x)/sqrtf(poow(synthesis.end.y - synthesis.start.y, 2)+poow(synthesis.end.x - synthesis.start.x, 2));
}

void synthesis_init(){
	arm_mat_init_f32(&synthesis.CURRENT_STATE, 3, 1, synthesis.current_state);
	arm_mat_init_f32(&synthesis.TARGET_STATE, 3, 1, synthesis.target_state);
	arm_mat_init_f32(&synthesis.STATE, 6, 1, synthesis.state);
	arm_mat_init_f32(&synthesis.NEXT_STATE, 3, 1, synthesis.next_state);
	arm_mat_init_f32(&synthesis.COEF, 6, 1, synthesis.coef);
	arm_mat_init_f32(&synthesis.AI, 6, 6, synthesis.ai);
	arm_mat_init_f32(&synthesis.A, 3, 6, synthesis.a);

	synthesis.angle.Kp = 300;
	synthesis.distance.Kp = 1.25;

	synthesis.phase = -1;
}

float synthesis_limit(float speed, float limit){
	if(speed > limit) speed = limit;
	if(speed < -limit) speed = -limit;
	return speed;
}

void synthesis_stop(){
	synthesis.phase = -1;
}

uint8_t synthesis_is_stuck(){
	if(fabs(odom.trans_vel - synthesis.NEXT_STATE.pData[1]) > 1000){
		if(odom.x > 2700){
			if(fabs(normalize(odom.theta - 0)) < M_PI_4){ // theta 0
//				odom.x = 3000 - FRONT_HALF_DISTANCE;
//				odom.theta = 0;
				return 1;
			}
			else if(fabs(normalize(odom.theta - M_PI)) < M_PI_4){ // theta PI
//				odom.x = 3000 - BACK_HALF_DISTANCE;
//				odom.theta = M_PI;
				return 1;
			}
		}
		else if(odom.x < 300){
			if(fabs(normalize(odom.theta - 0)) < M_PI_4){ //tehta 0
//				odom.x = BACK_HALF_DISTANCE;
//				odom.theta = 0;
				return 1;
			}
			else if(fabs(normalize(odom.theta - M_PI)) < M_PI_4){ //theta PI
//				odom.x = FRONT_HALF_DISTANCE;
//				odom.theta = M_PI;
				return 1;
			}
		}
		else if(odom.y > 1300){
			if(fabs(normalize(odom.theta - M_PI_2)) < M_PI_4){ //theta PI/2
//				odom.y = 2000-FRONT_HALF_DISTANCE;
//				odom.theta = M_PI_2;
				return 1;
			}
			else if(fabs(normalize(odom.theta - (-M_PI_2))) < M_PI_4){ // theta -PI/2
//				odom.y = 2000-BACK_HALF_DISTANCE;
//				odom.theta = -M_PI_2;
				return 1;
			}
		}

		else if(odom.y < 300){
			if(fabs(normalize(odom.theta - M_PI_2)) < M_PI_4){ //theta PI/2
//				odom.y = BACK_HALF_DISTANCE;
//				odom.theta = M_PI_2;
				return 1;
			}
			else if(fabs(normalize(odom.theta - (-M_PI_2))) < M_PI_4){ // theta -PI/2
//				odom.y = FRONT_HALF_DISTANCE;
//				odom.theta = -M_PI_2;
				return 1;
			}
		}

	}
	return 0;
}

void synthesis_calc_coef(float T){
	synthesis.AI.pData[0*6 + 0] = 1;
	synthesis.AI.pData[0*6 + 1] = 0;
	synthesis.AI.pData[0*6 + 2] = 0;
	synthesis.AI.pData[0*6 + 3] = 0;
	synthesis.AI.pData[0*6 + 4] = 0;
	synthesis.AI.pData[0*6 + 5] = 0;

	synthesis.AI.pData[1*6 + 0] = 0;
	synthesis.AI.pData[1*6 + 1] = 1;
	synthesis.AI.pData[1*6 + 2] = 0;
	synthesis.AI.pData[1*6 + 3] = 0;
	synthesis.AI.pData[1*6 + 4] = 0;
	synthesis.AI.pData[1*6 + 5] = 0;

	synthesis.AI.pData[2*6 + 0] = 0;
	synthesis.AI.pData[2*6 + 1] = 0;
	synthesis.AI.pData[2*6 + 2] = 0.5;
	synthesis.AI.pData[2*6 + 3] = 0;
	synthesis.AI.pData[2*6 + 4] = 0;
	synthesis.AI.pData[2*6 + 5] = 0;

	synthesis.AI.pData[3*6 + 0] =  -10 / poow(T,3);
	synthesis.AI.pData[3*6 + 1] =   -6 / poow(T,2);
	synthesis.AI.pData[3*6 + 2] = -1.5 / poow(T,1);
	synthesis.AI.pData[3*6 + 3] =   10 / poow(T,3);
	synthesis.AI.pData[3*6 + 4] =   -4 / poow(T,2);
	synthesis.AI.pData[3*6 + 5] =  0.5 / poow(T,1);

	synthesis.AI.pData[4*6 + 0] =  15 / poow(T,4);
	synthesis.AI.pData[4*6 + 1] =   8 / poow(T,3);
	synthesis.AI.pData[4*6 + 2] = 1.5 / poow(T,2);
	synthesis.AI.pData[4*6 + 3] = -15 / poow(T,4);
	synthesis.AI.pData[4*6 + 4] =   7 / poow(T,3);
	synthesis.AI.pData[4*6 + 5] =  -1 / poow(T,2);

	synthesis.AI.pData[5*6 + 0] =  -6  / poow(T,5);
	synthesis.AI.pData[5*6 + 1] =  -3  / poow(T,4);
	synthesis.AI.pData[5*6 + 2] = -0.5 / poow(T,3);
	synthesis.AI.pData[5*6 + 3] =   6  / poow(T,5);
	synthesis.AI.pData[5*6 + 4] =  -3  / poow(T,4);
	synthesis.AI.pData[5*6 + 5] = 0.5  / poow(T,3);

	synthesis.STATE.pData[0] = synthesis.CURRENT_STATE.pData[0];
	synthesis.STATE.pData[1] = synthesis.CURRENT_STATE.pData[1];
	synthesis.STATE.pData[2] = synthesis.CURRENT_STATE.pData[2];
	synthesis.STATE.pData[3] = synthesis.TARGET_STATE.pData[0];
	synthesis.STATE.pData[4] = synthesis.TARGET_STATE.pData[1];
	synthesis.STATE.pData[5] = synthesis.TARGET_STATE.pData[2];

	arm_mat_mult_f32(&synthesis.AI, &synthesis.STATE, &synthesis.COEF);
}

void synthesis_calc_next_state(float t){
	synthesis.A.pData[0*6 + 0] = poow(t, 0);
	synthesis.A.pData[0*6 + 1] = poow(t, 1);
	synthesis.A.pData[0*6 + 2] = poow(t, 2);
	synthesis.A.pData[0*6 + 3] = poow(t, 3);
	synthesis.A.pData[0*6 + 4] = poow(t, 4);
	synthesis.A.pData[0*6 + 5] = poow(t, 5);

	synthesis.A.pData[1*6 + 0] = 0;
	synthesis.A.pData[1*6 + 1] = 1* poow(t, 0);
	synthesis.A.pData[1*6 + 2] = 2* poow(t, 1);
	synthesis.A.pData[1*6 + 3] = 3* poow(t, 2);
	synthesis.A.pData[1*6 + 4] = 4* poow(t, 3);
	synthesis.A.pData[1*6 + 5] = 5* poow(t, 4);

	synthesis.A.pData[2*6 + 0] = 0;
	synthesis.A.pData[2*6 + 1] = 0;
	synthesis.A.pData[2*6 + 2] = 2*  poow(t, 0);
	synthesis.A.pData[2*6 + 3] = 6*  poow(t, 1);
	synthesis.A.pData[2*6 + 4] = 12* poow(t, 2);
	synthesis.A.pData[2*6 + 5] = 20* poow(t, 3);

	arm_mat_mult_f32(&synthesis.A, &synthesis.COEF, &synthesis.NEXT_STATE);

//	if(fabs(synthesis.NEXT_STATE.pData[1]) > 50000){
//		Enable_Motor(&left_motor, 0);
//		Enable_Motor(&right_motor, 0);
//	}
}

void synthesis_set_target_state(float p, float v, float a){
	synthesis.TARGET_STATE.pData[0] = p;
	synthesis.TARGET_STATE.pData[1] = v;
	synthesis.TARGET_STATE.pData[2] = a;
}

void synthesis_set_current_state(float p, float v, float a){
	synthesis.CURRENT_STATE.pData[0] = p;
	synthesis.CURRENT_STATE.pData[1] = v;
	synthesis.CURRENT_STATE.pData[2] = a;
}

void synthesis_set_next_state(float p, float v, float a){
	synthesis.NEXT_STATE.pData[0] = p;
	synthesis.NEXT_STATE.pData[1] = v;
	synthesis.NEXT_STATE.pData[2] = a;
}

//void synthesis_set_init_target_state(){
//	synthesis_set_target_state(synthesis_target[0].P,synthesis_target[0].V,synthesis_target[0].A);
//}

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

//		float T[3], P[3], V[3];
//		T[0] = T[2] = (3.0*Vmax)/(2.0*Amax);
//		P[0] = synthesis_calc_P(T[0], Amax) * 2;
//		if (P<0)
//			P[0] *= -1;
//		P[1] = P - P[0];
//		P[2] = P;
//		T[1] = fabs(P[1] - P[0])/Vmax;
//		V[0] = V[1] = Vmax * (P<0) ? -1:1;
//		V[2] = 0;
//
//		for( uint8_t i=synthesis.target_len; i<synthesis.target_len+3; ++i){
//			synthesis.target[i].T = T[i];
//			synthesis.target[i].P = P[i];
//			synthesis.target[i].V = V[i];
//			synthesis.target[i].A = 0;		 //accel = 0
//			synthesis.target[i].type = type;
//		}
//
//		synthesis.target_len += 3;
//		synthesis.total_time = T1+T2+T3;


		T1 = (3.0*Vmax)/(2.0*Amax);
		T3 = T1;
		P1 = synthesis_calc_P(T1, Amax) * 2;
		if (P<0)
			P1 *= -1;
		P2 = P - P1;
		P3 = P;
		T2 = fabs(P2 - P1)/Vmax;
		if (P<0)
			Vmax *= -1;
		V1 = V2 = Vmax;
		V3 = 0;

		synthesis.target[synthesis.target_len].T = T1;
		synthesis.target[synthesis.target_len].P = P1;
		synthesis.target[synthesis.target_len].V = V1;
		synthesis.target[synthesis.target_len].A = 0;		 //accel = 0
		synthesis.target[synthesis.target_len].type = type;
		synthesis.target_len++;
		synthesis.target[synthesis.target_len].T = T2;
		synthesis.target[synthesis.target_len].P = P2;
		synthesis.target[synthesis.target_len].V = V2;
		synthesis.target[synthesis.target_len].A = 0;		 //accel = 0
		synthesis.target[synthesis.target_len].type = type;
		synthesis.target_len++;
		synthesis.target[synthesis.target_len].T = T3;
		synthesis.target[synthesis.target_len].P = P3;
		synthesis.target[synthesis.target_len].V = V3;
		synthesis.target[synthesis.target_len].A = 0;		 //accel = 0
		synthesis.target[synthesis.target_len].type = type;
		synthesis.target_len++;

		synthesis.total_time = T1+T2+T3;

	}
	else{
		synthesis.total_time = synthesis_calc_T(P, Amax);

		synthesis.target[synthesis.target_len].T = synthesis.total_time;
		synthesis.target[synthesis.target_len].P = P;
		synthesis.target[synthesis.target_len].V = 0;
		synthesis.target[synthesis.target_len].A = 0;		 //accel = 0
		synthesis.target[synthesis.target_len].type = type;
		synthesis.target_len++;
	}
}

void synthesis_activate_detection(float backing_distance){
	if(synthesis.phase >= 0 && synthesis.target[synthesis.phase].type == 't'){
		if(odom.trans_vel > 0) backing_distance = -backing_distance;
		synthesis.start.x = odom.x;
		synthesis.start.y = odom.y;
		//synthesis.start.theta = odom.theta;
		synthesis.target_len = 0;

		//synthesis_set_current_state(0, 0, 0);
		//synthesis_set_next_state(0, 0, 0);
		//synthesis_set_target_state(0, 0, 0);

		synthesis.start_time = (float)HAL_GetTick() / 1000; //s
		synthesis.distance_from_start = 0;
		synthesis.angle_from_start = 0;

		synthesis.total_distance = 0;
		synthesis.total_angle = 0;
		synthesis.last_angle = odom.theta;
		synthesis.total_distance = backing_distance;
		synthesis.end.x = synthesis.start.x + backing_distance*cos(odom.theta);
		synthesis.end.y = synthesis.start.y + backing_distance*sin(odom.theta);
		//synthesis.end.theta = odom.theta;

//		calc_traj_coef();
		//synthesis.total_time = synthesis_calc_T(P, Amax);
		synthesis.target[synthesis.target_len].T = 1;
		synthesis.target[synthesis.target_len].P = backing_distance;
		synthesis.target[synthesis.target_len].V = 0;
		synthesis.target[synthesis.target_len].A = 0;		 //accel = 0
		synthesis.target[synthesis.target_len].type = 't';
		synthesis.target_len++;

		synthesis_set_target_state(synthesis.target[0].P, synthesis.target[0].V, synthesis.target[0].A);
		synthesis_start();
	}
	else{
		uint8_t data[1] = {0x01};
		FDCAN_Send_Data(0x4DE, FDCAN_DLC_BYTES_1, 1, data);
	}

}

void synthesis_prepare(){
	synthesis.start.x = odom.x;
	synthesis.start.y = odom.y;
	synthesis.start.theta = odom.theta;
	synthesis.target_len = 0;

	synthesis_set_current_state(0, 0, 0);
	synthesis_set_next_state(0, 0, 0);
	synthesis_set_target_state(0, 0, 0);

	synthesis.start_time = (float)HAL_GetTick() / 1000; //s
	synthesis.distance_from_start = 0;
	synthesis.angle_from_start = 0;

	synthesis.total_distance = 0;
	synthesis.total_angle = 0;
	synthesis.last_angle = odom.theta;
}

void synthesis_start_XY(float x, float y, char direction, float Vmax, float Amax, float Wmax, float amax){
	synthesis_prepare();
//	synthesis_target_len = 0;
//	start_x = odom.x;
//	start_y = odom.y;
//	start_theta = odom.theta;
	synthesis.end.x = x;
	synthesis.end.y = y;
	synthesis.end.theta = atan2f(synthesis.end.y - synthesis.start.y, synthesis.end.x - synthesis.start.x);
//	total_distance = sqrtf((end_x - start_x) * (end_x - start_x) + (end_y - start_y) * (end_y - start_y));
	synthesis.total_distance = distance(synthesis.end.x, synthesis.end.y, synthesis.start.x, synthesis.start.y);
	if(direction == 'r'){
		synthesis.end.theta += M_PI;
		synthesis.end.theta = normalize(synthesis.end.theta);
//		if(synthesis.end.theta > M_PI) synthesis.end.theta -= 2*M_PI;
//		if(synthesis.end.theta < -M_PI) synthesis.end.theta += 2*M_PI;
		synthesis.total_distance = -synthesis.total_distance;
	}
	synthesis.total_angle = synthesis.end.theta - synthesis.start.theta;
	synthesis.total_angle = normalize(synthesis.total_angle);
//	if(synthesis.total_angle >  M_PI) synthesis.total_angle -= 2*M_PI;
//	if(synthesis.total_angle < -M_PI) synthesis.total_angle += 2*M_PI;

	//init_angle = atan2(end_y - start_y, end_x - start_x);

	//TODO: Calc it in compute function when transiting from rotatin to translation!!!
	//calc_traj_coef();
	synthesis_calc_target_points(synthesis.total_angle, Wmax, amax, 'r');
	synthesis_calc_target_points(synthesis.total_distance, Vmax, Amax, 't');

//	synthesis_set_current_state(0, 0, 0);
//	synthesis_start_time = (float)HAL_GetTick() / 1000; //s
//	distance_from_start = 0;
//	theta_from_start = 0;
	synthesis_set_target_state(synthesis.target[0].P, synthesis.target[0].V, synthesis.target[0].A);
//	synthesis_set_init_target_state();
//	last_theta = odom.theta;
//	synthesis_phase = 0;
	synthesis_start();
}
void synthesis_start_rotateFor(float theta, float Wmax, float amax){
	synthesis_prepare();
//	synthesis_target_len = 0;
//	start_theta = odom.theta;
	synthesis.total_angle = theta;

	synthesis_calc_target_points(synthesis.total_angle, Wmax, amax, 'r');

//	synthesis_set_current_state(0, 0, 0);
//	synthesis_set_next_state(0, 0, 0);
//	synthesis_start_time = (float)HAL_GetTick() / 1000; //s
//	theta_from_start = 0;
//	synthesis_set_init_target_state();
	synthesis_set_target_state(synthesis.target[0].P, synthesis.target[0].V, synthesis.target[0].A);
//	last_theta = odom.theta;
//	synthesis_phase = 0;
	synthesis_start();
}

void synthesis_start_rotateTo(float theta, float Wmax, float amax){
	synthesis_prepare();
//	synthesis_target_len = 0;
//	start_theta = odom.theta;
	synthesis.total_angle = theta - odom.theta;
	if(synthesis.total_angle >  M_PI) synthesis.total_angle -= 2*M_PI;
	if(synthesis.total_angle < -M_PI) synthesis.total_angle += 2*M_PI;
	synthesis.end.theta = theta;

	synthesis_calc_target_points(synthesis.total_angle, Wmax, amax, 'r');

//	synthesis_set_current_state(0, 0, 0);
//	synthesis_set_next_state(0, 0, 0);
//	synthesis_start_time = (float)HAL_GetTick() / 1000; //s
//	theta_from_start = 0;
//	synthesis_set_init_target_state();
	synthesis_set_target_state(synthesis.target[0].P, synthesis.target[0].V, synthesis.target[0].A);
//	last_theta = odom.theta;
//	synthesis_phase = 0;
	synthesis_start();
}


void synthesis_start_distance(float distance,float Vmax, float Amax){
	synthesis_prepare();
	//synthesis_target_len = 0;
	//start_x = odom.x;
	//start_y = odom.y;
	//start_theta = odom.theta;
	synthesis.total_distance = distance;
	synthesis.end.x = synthesis.start.x + distance*cos(odom.theta);
	synthesis.end.y = synthesis.start.y + distance*sin(odom.theta);
	synthesis.end.theta = odom.theta;

//	calc_traj_coef();
	synthesis_calc_target_points(distance, Vmax, Amax, 't');
	synthesis_set_target_state(synthesis.target[0].P, synthesis.target[0].V, synthesis.target[0].A);
//	synthesis_set_current_state(0, 0, 0);
//	synthesis_set_next_state(0, 0, 0);
//	synthesis_start_time = (float)HAL_GetTick() / 1000; //s
//	distance_from_start = 0;
//	synthesis_set_init_target_state();
//	last_theta = odom.theta;
	synthesis_start();
}

//void synthesis_compute(){
//	if(synthesis.phase >= 0){ //If synthesis is activated
//		float tolerance;
//		if(synthesis.target[synthesis.phase].type == 'r' && synthesis.target[synthesis.phase].V != 0) tolerance = 0.02;
//		if(synthesis.target[synthesis.phase].type == 'r' && synthesis.target[synthesis.phase].V == 0) tolerance = 0.01;
//		if(synthesis.target[synthesis.phase].type == 't' && synthesis.target[synthesis.phase].V != 0) tolerance = 3;
//		if(synthesis.target[synthesis.phase].type == 't' && synthesis.target[synthesis.phase].V == 0) tolerance = 1;
//
//		if(fabs(synthesis.CURRENT_STATE.pData[0]) > fabs(synthesis.TARGET_STATE.pData[0]) - tolerance){ // if robot passed target position
//			synthesis.phase++;								//increment phase
//			synthesis_set_current_state(synthesis.target[synthesis.phase-1].P, synthesis.target[synthesis.phase-1].V, synthesis.target[synthesis.phase-1].A);
//			synthesis_set_next_state(synthesis.target[synthesis.phase-1].P, synthesis.target[synthesis.phase-1].V, synthesis.target[synthesis.phase-1].A);
//
//			if(synthesis.phase < synthesis.target_len){ // if there is more target in synthesis
//				synthesis_set_target_state(synthesis.target[synthesis.phase].P, synthesis.target[synthesis.phase].V, synthesis.target[synthesis.phase].A);
//				synthesis.start_time = (float)HAL_GetTick()/1000;
//				if(synthesis.target[synthesis.phase - 1].type == 'r' && synthesis.target[synthesis.phase].type == 't'){ //switched from rotation to translation
//					synthesis_set_next_state(0,0,0);
//					synthesis_set_current_state(0,0,0);
//					synthesis.start.theta = synthesis.end.theta;
//					synthesis.distance_from_start = 0;
//					synthesis.start.x = odom.x;
//					synthesis.start.y = odom.y;
//					calc_traj_coef();
//
////					total_distance = sqrtf((end_x - start_x) * (end_x - start_x) + (end_y - start_y) * (end_y - start_y));
////					synthesis_target[synthesis_target_len - 1].P = total_distance - synthesis_target[synthesis_target_len - 2].P;
//				}
//			}
//			else{ 										//robot is in end postition
//				synthesis.phase = -1;
//				uint8_t data[1] = {0x01};
//				FDCAN_Send_Data(0x4D7, FDCAN_DLC_BYTES_1, 1, data);
//			}
//		}
//		 //compute synthesis
//		float left_time = synthesis.start_time + synthesis.target[synthesis.phase].T - (float)HAL_GetTick()/1000;
//
//		synthesis.distance_from_start = distance(odom.x, odom.y, synthesis.start.x, synthesis.start.y);
//		//distance_from_start = sqrtf((odom.x - start_x) * (odom.x - start_x) + (odom.y - start_y) * (odom.y - start_y));
//		float dTheta = odom.theta - synthesis.last_angle;
//		if(dTheta > M_PI){
//			dTheta -= 2*M_PI;
//		}
//		if(dTheta < -M_PI){
//			dTheta += 2*M_PI;
//		}
//		synthesis.angle_from_start += dTheta;
//		synthesis.last_angle = odom.theta;
//
//		synthesis_calc_coef(left_time);
//		synthesis_calc_next_state(SYNTHESIS_TIME/1000.0);
//
//		if(synthesis.target[synthesis.phase].type == 't'){
//			calc_distance_from_traj();
//			synthesis.angle.error = (odom.theta - synthesis.start.theta) * synthesis.angle.Kp ;
//			synthesis.distance.error *= synthesis.distance.Kp;
//			if (synthesis.target[synthesis.phase].P <= 0){
//				synthesis.angle.error *= -1;
//				synthesis.distance.error *= -1;
//				synthesis.distance_from_start *= -1;
//			}
////			if(_c < 0){
////				distance_error *= -1;
////			}
//			//TODO: Add position and speed in calcualtion
//			synthesis_set_current_state(synthesis.distance_from_start, synthesis.NEXT_STATE.pData[1], synthesis.NEXT_STATE.pData[2]);
//			Set_Speed(&left_motor, synthesis.NEXT_STATE.pData[1] - /*distance_error*/ + synthesis.angle.error);
//			Set_Speed(&right_motor, synthesis.NEXT_STATE.pData[1] + /*distance_error*/ - synthesis.angle.error);
//		}
//		if(synthesis.target[synthesis.phase ].type == 'r'){
//			synthesis_set_current_state(synthesis.angle_from_start, synthesis.NEXT_STATE.pData[1], synthesis.NEXT_STATE.pData[2]);
//			Set_Speed(&left_motor, -synthesis.NEXT_STATE.pData[1] * WHEEL_HALF_DISTANCE);
//			Set_Speed(&right_motor, synthesis.NEXT_STATE.pData[1] * WHEEL_HALF_DISTANCE);
//		}
//
//	}
//}

void synthesis_compute(){
	int8_t *phase = &synthesis.phase;
	char *type = &synthesis.target[synthesis.phase].type;
	float *target_V = &synthesis.target[synthesis.phase].V;
	float tolerance;
	if(*type == 'r' && *target_V != 0) tolerance = 0.04;
	if(*type == 'r' && *target_V == 0) tolerance = 0.02;
	if(*type == 't' && *target_V != 0) tolerance = 3;
	if(*type == 't' && *target_V == 0) tolerance = 2;
	//is synthesis activated
	if(synthesis_state() >= 0){
		//is target reached
		if(
			(synthesis.TARGET_STATE.pData[0] > 0 && synthesis.TARGET_STATE.pData[0] - synthesis.CURRENT_STATE.pData[0] < tolerance) || //for positive movement
			(synthesis.TARGET_STATE.pData[0] < 0 && synthesis.TARGET_STATE.pData[0] - synthesis.CURRENT_STATE.pData[0] > -tolerance)	// for negative movement
			){
		//if(fabs(synthesis.CURRENT_STATE.pData[0]) > fabs(synthesis.TARGET_STATE.pData[0]) - tolerance){
			(*phase)++;
			//deactivate synthesis if there is no more steps
			if(*phase == synthesis.target_len){
				*phase = -1;
				uint8_t data[1] = {0x01};
				FDCAN_Send_Data(0x4DE, FDCAN_DLC_BYTES_1, 1, data);
				goto end;
			}
			//set current state to be same as last step target state
//			synthesis_set_current_state(synthesis.target[(*phase)-1].P, synthesis.target[(*phase)-1].V, synthesis.target[(*phase)-1].A);
//			synthesis_set_next_state(synthesis.target[(*phase)-1].P, synthesis.target[(*phase)-1].V, synthesis.target[(*phase)-1].A);
			synthesis_set_current_state(synthesis.distance_from_start, synthesis.target[(*phase)-1].V, 0);
			synthesis_set_next_state(synthesis.distance_from_start, synthesis.target[(*phase)-1].V, 0);

			//load target state
			synthesis_set_target_state(synthesis.target[*phase].P, synthesis.target[*phase].V, synthesis.target[*phase].A);
			//reset start time
			synthesis.start_time = (float)HAL_GetTick()/1000;
			//change from rotation to translation
			if(synthesis.target[(*phase)-1].type == 'r' && synthesis.target[*phase].type == 't'){
				//reset current state
				//TODO: correct total distance, because it can translete while rotation and then miss end point
//				Set_Speed(&left_motor,  0);
//				Set_Speed(&right_motor, 0);
				synthesis_set_current_state(0,0,0);
				synthesis_set_next_state(0,0,0);
				synthesis.start.theta = synthesis.end.theta;
				synthesis.distance_from_start = 0;
				synthesis.start.x = odom.x;
				synthesis.start.y = odom.y;
//				calc_traj_coef();
			}
		}
		float left_time = synthesis.start_time + synthesis.target[*phase].T - (float)HAL_GetTick()/1000;
		//translation
		if(*type == 't'){
			synthesis.distance_from_start = distance(odom.x, odom.y, synthesis.start.x, synthesis.start.y);
			synthesis.angle.error = normalize(odom.theta - synthesis.end.theta);
			synthesis.distance.error = calc_distance_from_traj();
			//correction
			float angle_S2R = atan2(odom.y - synthesis.start.y, odom.x - synthesis.start.x);
			if(normalize(angle_S2R - synthesis.start.theta) < 0){
				synthesis.distance.error *= -1;
			}
			if(fabs(normalize(angle_S2R - synthesis.start.theta)) > M_PI/2){
				synthesis.distance_from_start *= -1;
			}
			if(synthesis.target[*phase].P < 0){
				synthesis.distance.error *= -1;
				//synthesis.distance_from_start *= -1;
			}
			//end of correction
			synthesis_set_current_state(synthesis.distance_from_start, synthesis.NEXT_STATE.pData[1], synthesis.NEXT_STATE.pData[2]);
			synthesis_calc_coef(left_time);
			synthesis_calc_next_state(SYNTHESIS_TIME/1000.0);


			float ls = synthesis.NEXT_STATE.pData[1] + synthesis.angle.error * synthesis.angle.Kp + synthesis.distance.error * synthesis.distance.Kp;
			float rs = synthesis.NEXT_STATE.pData[1] - synthesis.angle.error * synthesis.angle.Kp - synthesis.distance.error * synthesis.distance.Kp;
			ls = synthesis_limit(ls, 1100);
			rs = synthesis_limit(rs, 1100);

			Set_Speed(&left_motor,  ls*1.05);
			Set_Speed(&right_motor, rs*1.05);

			if(synthesis_is_stuck()){
				uint8_t data[1] = {0x01};
				FDCAN_Send_Data(0x4DE, FDCAN_DLC_BYTES_1, 1, data);
				*phase = -1;
				goto end;
			}
		}
		//rotation
		else if(*type == 'r'){
			float dTheta = odom.theta - synthesis.last_angle;
			synthesis.last_angle = odom.theta;
			dTheta = normalize(dTheta);
			synthesis.angle_from_start += dTheta;
			synthesis_set_current_state(synthesis.angle_from_start, synthesis.NEXT_STATE.pData[1], synthesis.NEXT_STATE.pData[2]);
			synthesis_calc_coef(left_time);
			synthesis_calc_next_state(SYNTHESIS_TIME/1000.0);
			float ls = -synthesis.NEXT_STATE.pData[1] * WHEEL_HALF_DISTANCE*1.05;
			float rs =  synthesis.NEXT_STATE.pData[1] * WHEEL_HALF_DISTANCE*1.05;
			ls = synthesis_limit(ls, 1100);
			rs = synthesis_limit(rs, 1100);

			Set_Speed(&left_motor,  ls);
			Set_Speed(&right_motor, rs);
		}

	}
	end:

}

