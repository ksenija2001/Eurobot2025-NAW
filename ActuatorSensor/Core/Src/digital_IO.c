/*
 * digital_IO.c
 *
 *  Created on: Mar 19, 2025
 *      Author: xenia
 */

#include "digital_IO.h"

sIO_t inputs[IN_NUM] = {
		{.pin = In_1_Pin,.port = In_1_GPIO_Port},
		{.pin = In_2_Pin,.port = In_2_GPIO_Port},
		{.pin = In_3_Pin,.port = In_3_GPIO_Port},
		{.pin = In_4_Pin,.port = In_4_GPIO_Port},
		{.pin = In_5_Pin,.port = In_5_GPIO_Port},
		{.pin = In_6_Pin,.port = In_6_GPIO_Port},
		{.pin = In_7_Pin,.port = In_7_GPIO_Port},
		{.pin = In_8_Pin,.port = In_8_GPIO_Port}
};

sIO_t outputs[IN_NUM] = {
		{.pin = Out_1_Pin,.port = Out_1_GPIO_Port},
		{.pin = Out_2_Pin,.port = Out_2_GPIO_Port},
		{.pin = Out_3_Pin,.port = Out_3_GPIO_Port},
		{.pin = Out_4_Pin,.port = Out_4_GPIO_Port}
};


uint8_t data[2];

void Set_Output(uint8_t output_num, uint8_t state){
	HAL_GPIO_WritePin(outputs[output_num-1].port, outputs[output_num-1].pin, state);
}

// 10ms interrupt for polling digital inputs
void TIM7_Input_Poll_IT(TIM_HandleTypeDef* tim){
	for (uint8_t i=0; i<IN_NUM; ++i){
		inputs[i].state = HAL_GPIO_ReadPin(inputs[i].port, inputs[i].pin);
		if ( inputs[i].state != inputs[i].last_state){
			data[0] = i+1;
			data[1] = inputs[i].state;
			FDCAN_Send_Data(0x3FF, FDCAN_DLC_BYTES_2, 2, data);
		}

		inputs[i].last_state = inputs[i].state;
	}
}
