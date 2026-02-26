/*
 * digital_IO.h
 *
 *  Created on: Mar 19, 2025
 *      Author: xenia
 */

#ifndef INC_DIGITAL_IO_H_
#define INC_DIGITAL_IO_H_

#include "main.h"
#include "stm32g4xx_hal.h"
#include "fdcan.h"
#include "struct_types.h"

#define IN_NUM  8
#define OUT_NUM 4

void Set_Output(uint8_t output_num, uint8_t state);
void TIM7_Input_Poll_IT(TIM_HandleTypeDef* tim);

#endif /* INC_DIGITAL_IO_H_ */
