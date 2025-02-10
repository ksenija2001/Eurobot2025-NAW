/*
 * struct_types.h
 *
 *  Created on: Feb 2, 2025
 *      Author: xenia
 */

#ifndef INC_STRUCT_TYPES_H_
#define INC_STRUCT_TYPES_H_

typedef struct {
	uint16_t pin;
	GPIO_TypeDef* port;
} sIO_t;

typedef struct {
	TIM_HandleTypeDef* tim;
	uint16_t channel;
} sTIM_t;

typedef struct {
	ADC_HandleTypeDef* adc;
	uint32_t output;
} sADC_t;



#endif /* INC_STRUCT_TYPES_H_ */
