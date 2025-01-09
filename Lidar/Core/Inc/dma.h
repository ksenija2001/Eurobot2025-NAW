/*
 * dma.h
 *
 *  Created on: Nov 27, 2024
 *      Author: xenia
 */

#ifndef INC_DMA_H_
#define INC_DMA_H_

#include "main.h"
#include "stm32g4xx_hal.h"

#define DMA_BUF_SIZE   84
#define DMA_TIMEOUT_MS 10

typedef struct{
    volatile uint8_t  flag;     /* Timeout event flag */
    uint16_t timeout;             /* Timeout duration in msec */
    uint16_t prevCNDTR;         /* Holds previous value of DMA_CNDTR */
} sDMA_Event_t;

typedef struct{
	volatile uint8_t new_data;
	uint8_t data[DMA_BUF_SIZE];
	uint8_t length;
} sBuffer_t;

void Start_DMA(UART_HandleTypeDef *huart, uint8_t size);
void Change_Size_DMA(uint8_t size);

extern sBuffer_t data_buf;

#endif /* INC_DMA_H_ */
