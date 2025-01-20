/*
 * interrupts.c
 *
 *  Created on: Jan 20, 2025
 *      Author: xenia
 */

#include "interrupts.h"


uint32_t counter = 0;

union U_F{
	float f;
	uint8_t u[4];
}convert_bytes;

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

}


void Float2Bytes(uint8_t* buffer, uint8_t start, float data){
	convert_bytes.f = data;

	for (uint8_t i=0; i<4 ;++i){
		buffer[start+i] = convert_bytes.u[i];
	}

}





