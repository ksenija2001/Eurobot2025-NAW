/*
 * fdcan.c
 *
 *  Created on: Jan 20, 2025
 *      Author: xenia
 */

#include "fdcan.h"

extern FDCAN_HandleTypeDef hfdcan1;

FDCAN_FilterTypeDef sFilterConfig;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[9];

FDCAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[64];

union U_F{
	float f;
	uint8_t u[4];
}convert_float;

uint8_t FDCAN_Init(FDCAN_HandleTypeDef *hfdcan){
	uint8_t status = HAL_ERROR;

	HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);

	sFilterConfig.IdType = FDCAN_STANDARD_ID; // Use standard IDs
	sFilterConfig.FilterIndex = 0;            // Filter index 0
	sFilterConfig.FilterType = FDCAN_FILTER_RANGE; // Use range filter
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // Route accepted messages to RX FIFO 0
	sFilterConfig.FilterID1 = 0x321;          // Start of ID range

	status = HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig);
	status |= HAL_FDCAN_Start(hfdcan);
	status |= HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

	return status;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	// CHAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {

    /* Retreive Rx messages from RX FIFO0 */
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
		/* Reception Error */
		Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
      /* Notification Error */
      Error_Handler();
    }

    switch(RxHeader.Identifier){
    case 0x4F0:
    	sOdom_t new_odom = {
    			.x = Bytes2Float(RxData, 0),
				.y = Bytes2Float(RxData, 4),
				.theta = Bytes2Float(RxData, 8),
    	};

    	Reset_Odometry(&new_odom);

    	break;
    case 0x4F1:
    	float diameter = Bytes2Float(RxData, 1);
		float track = Bytes2Float(RxData, 5);

		if (RxData[0]){
			Config_Encoder_Wheel(&left, diameter, track);
		} else {
			Config_Encoder_Wheel(&right, diameter, track);
		}

    	break;
    }

  }
}

uint8_t FDCAN_Send_Data(uint32_t id, uint32_t dlc, uint8_t size, uint8_t* data){
	uint8_t status = HAL_ERROR;

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

	memcpy(data, TxData, size);

	status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);

	return status;
}

// Converts uint8_t bytes into a float number
float Bytes2Float(uint8_t msg[], uint8_t start)
{
	convert_float.u[0] = msg[start];
	convert_float.u[1] = msg[start+1];
	convert_float.u[2] = msg[start+2];
	convert_float.u[3] = msg[start+3];

	return convert_float.f;
}
