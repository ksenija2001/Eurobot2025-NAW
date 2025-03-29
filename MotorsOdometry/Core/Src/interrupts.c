/*
 * interrupts.c
 *
 *  Created on: Jan 20, 2025
 *      Author: xenia
 */

#include "interrupts.h"


uint32_t counter = 0;

uint8_t tx_buffer[64];

float regen_new_time;
float new_time;


// Odometry interrupt - 1ms
void TIM6_IT(void){
	++counter;

	if(counter % ODOM_TIME == 0){
		Odometry();

		Float2Bytes(tx_buffer, 0, odom.x);
		Float2Bytes(tx_buffer, 4, odom.y);
		Float2Bytes(tx_buffer, 8, odom.theta);
		Float2Bytes(tx_buffer, 12, left.curr_vel);
		Float2Bytes(tx_buffer, 16, right.curr_vel);
		Float2Bytes(tx_buffer, 20, odom.trans_vel);
		Float2Bytes(tx_buffer, 24, odom.ang_vel);
		Float2Bytes(tx_buffer, 28, odom.trans_acc);
		Float2Bytes(tx_buffer, 32, odom.ang_acc);
//		Float2Bytes(tx_buffer, 28, odom.gyr_angular);


		if(counter%100 == 0){
			FDCAN_Send_Data(0x4FF, FDCAN_DLC_BYTES_32, 32, tx_buffer);
		}
	}

//	if(counter % RPM_TIME == 0){
//		Int162Bytes(tx_buffer, 0, left_motor.currRPM);
//		Int162Bytes(tx_buffer, 2, right_motor.currRPM);
//
//		FDCAN_Send_Data(0x4DF, FDCAN_DLC_BYTES_4, 4, tx_buffer);
//	}

	if(counter % SYNTHESIS_TIME == 0){
		synthesis_compute();
	}

}






