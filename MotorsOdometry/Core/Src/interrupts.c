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
		Float2Bytes(tx_buffer, 12, odom.wheel_left_speed);
		Float2Bytes(tx_buffer, 16, odom.wheel_right_speed);
		Float2Bytes(tx_buffer, 20, odom.wheel_trans);
		Float2Bytes(tx_buffer, 24, odom.wheel_angular);
		Float2Bytes(tx_buffer, 28, odom.gyr_angular);


		FDCAN_Send_Data(0x4FF, FDCAN_DLC_BYTES_32, 32, tx_buffer);
	}

	if(counter % RPM_TIME == 0){
		Int162Bytes(tx_buffer, 0, left_motor.currRPM);
		Int162Bytes(tx_buffer, 2, right_motor.currRPM);

		FDCAN_Send_Data(0x4DF, FDCAN_DLC_BYTES_4, 4, tx_buffer);
	}

	if(counter % SYNTHESIS_REGEN_TIME == 0){

	}

	if(counter % SYNTHESIS_TIME == 0){
		if(synthesis_translation_state){
			synthesis_calc_next_state(HAL_GetTick() - synthesis_start_time);
			Set_Speed(&left_motor, (int16_t)NEXT_STATE.pData[1]);
			Set_Speed(&right_motor,(int16_t)NEXT_STATE.pData[1]);
		}
		if(HAL_GetTick() - synthesis_start_time >= total_T){
			synthesis_translation_state = 0;
			Set_Speed(&left_motor, 0);
			Set_Speed(&right_motor,0);
		}
	}

}






