/*
 * interrupts.c
 *
 *  Created on: Jan 20, 2025
 *      Author: xenia
 */

#include "interrupts.h"


uint32_t counter = 0;

uint8_t tx_buffer[64];


// Odometry interrupt - 1ms
void TIM6_IT(void){
	++counter;

	if(counter % ODOM_TIME == 0){
		Odometry_New();

		Float2Bytes(tx_buffer, 0, odom.x);
		Float2Bytes(tx_buffer, 4, odom.y);
		Float2Bytes(tx_buffer, 8, odom.theta);
		Float2Bytes(tx_buffer, 12, odom.left_speed);
		Float2Bytes(tx_buffer, 16, odom.right_speed);

		FDCAN_Send_Data(0x4FF, FDCAN_DLC_BYTES_20, 20, tx_buffer);
	}

	if(counter % RPM_TIME == 0){
		Int162Bytes(tx_buffer, 0, left_motor.currRPM);
		Int162Bytes(tx_buffer, 2, right_motor.currRPM);

		FDCAN_Send_Data(0x4DF, FDCAN_DLC_BYTES_4, 4, tx_buffer);
	}

}






