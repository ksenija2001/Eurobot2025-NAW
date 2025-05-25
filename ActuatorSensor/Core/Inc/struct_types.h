/*
 * struct_types.h
 *
 *  Created on: Feb 2, 2025
 *      Author: xenia
 */

#ifndef INC_STRUCT_TYPES_H_
#define INC_STRUCT_TYPES_H_

#include "stm32g431xx.h"
#include "stm32g4xx_hal.h"

typedef struct {
	uint16_t pin;
	GPIO_TypeDef* port;
	uint8_t last_state;
	uint8_t state;
} sIO_t;

typedef struct {
	TIM_HandleTypeDef* tim;
	uint16_t channel;
} sTIM_t;

//typedef struct {
//	ADC_HandleTypeDef* adc;
//	uint32_t output;
//} sADC_t;



#endif /* INC_STRUCT_TYPES_H_ */
