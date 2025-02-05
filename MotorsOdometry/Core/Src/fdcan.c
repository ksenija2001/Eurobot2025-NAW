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

uint8_t send_status = HAL_ERROR;
uint8_t receive_status = HAL_ERROR;


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

//void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
//{
//   //receive_status = HAL_ERROR;
//	// CHAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
//  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
//  {
//
//    /* Retreive Rx messages from RX FIFO0 */
//    receive_status = HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData);
//
//    receive_status |= HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
//
//    if (receive_status == HAL_OK){
//    	switch(RxHeader.Identifier){
//    	    case 0x4F0: // Reset odometry
//    	    	sOdom_t new_odom = {
//    	    			.x = Bytes2Float(RxData, 0),
//    					.y = Bytes2Float(RxData, 4),
//    					.theta = Bytes2Float(RxData, 8),
//    	    	};
//
//    	    	Reset_Odometry(&new_odom);
//
//    	    	break;
//    	    case 0x4F1:  // Wheel parameters configuration
//    	    	float left_diameter = Bytes2Float(RxData, 0);
//    	    	float right_diameter = Bytes2Float(RxData, 4);
//    			float track = Bytes2Float(RxData, 8);
//
//    			Config_Encoder_Wheel(&left, left_diameter, track);
//    			Config_Encoder_Wheel(&right, right_diameter, track);
//
//    	    	break;
//    	    case 0x4D0:  // Set reference for motor speed
//    	    	int16_t left_speed = Bytes2Int16(RxData, 0);
//    	    	int16_t right_speed = Bytes2Int16(RxData, 2);
//
//    	    	Set_Speed(&left_motor,  left_speed);
//    	    	Set_Speed(&right_motor, right_speed);
//
//    	    	break;
//    	    case 0x4D1:  // Set reference for motor RPM
//    	    	uint16_t left_RPM = Bytes2Int16(RxData, 0);
//    			uint16_t right_RPM = Bytes2Int16(RxData, 2);
//    			uint8_t left_dir = RxData[4];
//    			uint8_t right_dir = RxData[5];
//
//    			Set_RPM(&left_motor, left_RPM);
//    			Set_Direction(&left_motor, left_dir);
//    			Set_RPM(&right_motor, right_RPM);
//    			Set_Direction(&right_motor, right_dir);
//
//    			break;
//    	    default:
//    	    	receive_status = HAL_ERROR;
//    	    }
//
//    }
//  }
//}

uint8_t FDCAN_Send_Data(uint32_t id, uint32_t dlc, uint8_t size, uint8_t* data){
	//send_status = HAL_ERROR;

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

	send_status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);

	return send_status;
}


