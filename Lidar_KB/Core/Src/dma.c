/*
 * dma.c
 *
 *  Created on: Nov 27, 2024
 *      Author: xenia
 */

#include "dma.h"

sDMA_Event_t dma_uart_rx = {            /* DMA Timeout event structure */
		.flag = 0,
		.timeout = DMA_TIMEOUT_MS,
		.prevCNDTR = DMA_BUF_SIZE
};

sBuffer_t data_buf = {
		.data = {'\0'},                 /* Data buffer that contains newly received data */
		.length = 0,
		.new_data = 0
};
uint8_t dma_rx_buf[DMA_BUF_SIZE];       /* Circular buffer for DMA */

uint16_t i, pos, start, length;
uint16_t currCNDTR;

UART_HandleTypeDef *huart_t;

void Start_DMA(UART_HandleTypeDef *huart, uint8_t size){

	huart_t = huart;
	/* Start DMA */
	if(HAL_UART_Receive_DMA(huart, dma_rx_buf, size) != HAL_OK){
		Error_Handler();
	}

	/* Disable Half Transfer Interrupt */
	__HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}

void Change_Size_DMA(uint8_t size){
	HAL_UART_DMAStop(huart_t);

	/* Start DMA */
	if(HAL_UART_Receive_DMA(huart_t, dma_rx_buf, size) != HAL_OK){
		Error_Handler();
	}

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
	currCNDTR = __HAL_DMA_GET_COUNTER(huart->hdmarx);

	/* Ignore IDLE Timeout when the received characters exactly filled up the DMA buffer and DMA Rx Complete IT is generated,
	 * but there is no new character during timeout */
	if(dma_uart_rx.flag && currCNDTR == DMA_BUF_SIZE)
	{
		dma_uart_rx.flag = 0;
		return;
	}

	/* Determine start position in DMA buffer based on previous CNDTR value */
	start = (dma_uart_rx.prevCNDTR < DMA_BUF_SIZE) ? (DMA_BUF_SIZE - dma_uart_rx.prevCNDTR) : 0;

	/* Timeout event */
	if(dma_uart_rx.flag)
	{
		/* Determine new data length based on previous DMA_CNDTR value:
		 *  If previous CNDTR is less than DMA buffer size: there is old data in DMA buffer (from previous timeout) that has to be ignored.
		 *  If CNDTR == DMA buffer size: entire buffer content is new and has to be processed.
		*/
		length = (dma_uart_rx.prevCNDTR < DMA_BUF_SIZE) ? (dma_uart_rx.prevCNDTR - currCNDTR) : (DMA_BUF_SIZE - currCNDTR);
		dma_uart_rx.prevCNDTR = currCNDTR;
		dma_uart_rx.flag = 0;
	}
	/* DMA Rx Complete event */
	else
	{
		length = currCNDTR - start;
		dma_uart_rx.prevCNDTR = DMA_BUF_SIZE;
	}

	/* Copy new data */
	for(i=0, pos=start; i<length; ++i,++pos)
	{
		data_buf.data[i] = dma_rx_buf[pos];
	}

	data_buf.length = length;
	data_buf.new_data = 1;
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
}
