/*
 * fdcan.c
 *
 *  Created on: Jan 20, 2025
 *      Author: xenia
 */

#include "fdcan.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;


extern UART_HandleTypeDef huart2;

FDCAN_FilterTypeDef sFilterConfig;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[64];

FDCAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[64];

uint8_t send_status = HAL_ERROR;
uint8_t receive_status = HAL_ERROR;

uint8_t FDCAN_Init(FDCAN_HandleTypeDef *hfdcan)
{
	uint8_t status = HAL_ERROR;

	HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);

	HAL_FDCAN_ConfigTxDelayCompensation(hfdcan, 9, 0);
	HAL_FDCAN_EnableTxDelayCompensation(hfdcan);

	sFilterConfig.IdType = FDCAN_STANDARD_ID;			  // Use standard IDs
	sFilterConfig.FilterIndex = 0;						  // Filter index 0
	sFilterConfig.FilterType = FDCAN_FILTER_RANGE;		  // Use range filter
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // Route accepted messages to RX FIFO 0
	sFilterConfig.FilterID1 = 0x321;					  // Start of ID range

	status = HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig);
	status |= HAL_FDCAN_Start(hfdcan);
	status |= HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

	return status;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	// receive_status = HAL_ERROR;
	//  CHAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
	{
		HAL_GPIO_WritePin(LED_CAN_RX_GPIO_Port, LED_CAN_RX_Pin, GPIO_PIN_SET);
		/* Retreive Rx messages from RX FIFO0 */
		receive_status = HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData);

		receive_status |= HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

		if (receive_status == HAL_OK)
		{
			switch (RxHeader.Identifier)
			{
			case 0x6FF: // Receive odometry
				self.x = Bytes2Float(RxData, 0);
				self.y = Bytes2Float(RxData, 4);
				self.theta = Bytes2Float(RxData, 8);
				self.speed = Bytes2Float(RxData, 20);
				self.ang_speed = Bytes2Float(RxData, 24);
				self.enable_front_det = RxData[36];
				self.enable_back_det = RxData[37];

				speed = 0.5*speed + 0.5*self.speed;
				ang_speed = 0.5*ang_speed + 0.5*self.ang_speed;

				detection.front = abs(speed)/3.33 + 400;
				detection.back  = abs(speed)/3.33 + 400;

				break;
			case 0x4C0: // Start/Stop Lidar
				uint8_t status = RxData[0];
				color = RxData[1];

				if (status){
					start_lidar = 1;
				} else {
					start_lidar = 0;
				}

				break;
			case 0x4C1:
				uint16_t front = (uint16_t)(RxData[1] << 8) & RxData[0];
				uint16_t back = (uint16_t)(RxData[3] << 8) & RxData[2];

				detection.front = front;
				detection.back  = back;

			default:
				receive_status = HAL_ERROR;
			}
		}
	}
}

uint8_t FDCAN_Send_Data(uint32_t id, uint32_t dlc, uint8_t size, uint8_t *data)
{
	 send_status = HAL_ERROR;

	// Configure TX Header for FDCAN
	TxHeader.Identifier = id;
	TxHeader.IdType = FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength = dlc;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_ON;
	TxHeader.FDFormat = FDCAN_FD_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;

	memcpy(TxData, data, size);

	if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0){
		send_status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
		if (send_status != HAL_OK){
			Error_Handler();
		}
	} else {
		uint32_t txFifoRequest = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan1);
		if (HAL_FDCAN_IsTxBufferMessagePending(&hfdcan1, txFifoRequest)) {
			HAL_FDCAN_AbortTxRequest(&hfdcan1, txFifoRequest);
		}
	}

	return send_status;
}
