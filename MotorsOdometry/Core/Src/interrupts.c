/*
 * interrupts.c
 *
 *  Created on: Jan 20, 2025
 *      Author: xenia
 */

#include "interrupts.h"


uint32_t counter = 0;

uint8_t tx_buffer[64];

//float regen_new_time;
//float new_time;


// Odometry interrupt - 1ms
void TIM6_IT(void){
	++counter;

	Odometry();

	if(counter%50 == 0){ //50ms
		Float2Bytes(tx_buffer, 0, odom.x);
		Float2Bytes(tx_buffer, 4, odom.y);
		Float2Bytes(tx_buffer, 8, odom.theta);
		Float2Bytes(tx_buffer, 12,(float)left.inc);
		Float2Bytes(tx_buffer, 16, (float)right.inc);
		Float2Bytes(tx_buffer, 20, odom.trans_vel);
		Float2Bytes(tx_buffer, 24, odom.ang_vel);
		Float2Bytes(tx_buffer, 28, odom.trans_acc);
		Float2Bytes(tx_buffer, 32, odom.ang_acc);

		tx_buffer[36] = odom.detection_enable_front;
		tx_buffer[37] = odom.detection_enable_back;
		FDCAN_Send_Data(0x6FF, FDCAN_DLC_BYTES_48, 38, tx_buffer);
	}
}

void TIM7_IT(void){
//	if(STOP){
//		spline_stop();
//		synthesis_stop();
//		STOP = 0;
//	}

	if(spline_state() == -1 && synthesis_state() == -1){
		Set_RPM(&left_motor, 0);
		Set_RPM(&right_motor, 0);
		odom.detection_enable_back = 0;
		odom.detection_enable_front = 0;
	}
	else{
		synthesis_compute();
		spline_compute();
	}
}






