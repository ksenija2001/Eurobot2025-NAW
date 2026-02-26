/*
 * fdcan.c
 *
 *  Created on: Jan 20, 2025
 *      Author: xenia
 */

#include "fdcan.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim6;

FDCAN_FilterTypeDef sFilterConfig;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[64];

FDCAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[64];

uint8_t send_status = HAL_ERROR;
uint8_t receive_status = HAL_ERROR;

uint8_t ids[11];
uint16_t angles[11];
uint8_t speeds[11];

uint8_t xl_ids[11];
uint16_t xl_angles[11];
uint16_t xl_speeds[11];
uint8_t servo_id;
uint8_t state;
uint8_t output;

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
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
	{

		/* Retreive Rx messages from RX FIFO0 */
		receive_status = HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData);

		receive_status |= HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

		if (receive_status == HAL_OK)
		{
			switch (RxHeader.Identifier)
			{
			case 0x4CF:
				break;
			case 0x530: // Set servo position and speed
				uint8_t servo_num = RxData[0];
				uint8_t xl_servo_num = 0;
				for (uint8_t i=0, j=0, k=0; i<servo_num; ++i){
					if (RxData[i*4 +1] == 8){
						xl_ids[j] = RxData[i*4 +1];
						xl_angles[j] = ((uint16_t)RxData[i*4 + 1 + 1] << 8) | RxData[i*4 + 2 + 1];
						xl_speeds[j++] = RxData[i*4 + 3 + 1];
						xl_servo_num++;
					} else {
						ids[k] = RxData[i*4 +1];
						angles[k]= ((uint16_t)RxData[i*4 + 1 + 1] << 8) | RxData[i*4 + 2 + 1];
						speeds[k++] = RxData[i*4 + 3 + 1];
					}
				}

				HAL_TIM_Base_Stop_IT(&htim6);

				if (servo_num - xl_servo_num > 0)
					Sync_Set_Goal_Position(&huart1, ids, angles, speeds, servo_num - xl_servo_num);

//				HAL_Delay(1);

				if (xl_servo_num > 0)
					Sync_Set_Goal_Position_XL(&huart1, xl_ids, xl_angles, xl_speeds, xl_servo_num);

				htim6.Instance->CCR1 = 0;

				HAL_TIM_Base_Start_IT(&htim6);

				break;
			case 0x531: // Get servo position
				servo_id = RxData[0];

//				if (servo_id <= 10)
				Get_Present_Position(&huart1, servo_id);
//				else {
//					uint16_t angle = Get_Current_Angle(servo_id-11);
//					uint8_t msg[] = {servo_id, (uint8_t)((angle & 0xFF00) >> 8), (angle & 0x00FF)};
//					FDCAN_Send_Data(0x531, FDCAN_DLC_BYTES_3, 3, msg);
//				}

				break;
//			case 0x532: // Set position for RC servos
//				servo_id = RxData[0];
//				uint8_t position = RxData[1];
//
//				// IDs of RC servos start from 11, but indexing is from 0
//				if (position > 150) position = 150;
//				else if (position < 20) position = 20;
//
//				Set_Target_Angle(servo_id-11, position);
//				Set_Angle(servo_id-11, position);
//
//				break;
			case 0x533: // Enable Torque for all
				state = RxData[0];

				Enable_Torque(&huart1, 0xFE, state);
				Enable_Torque_XL(&huart1, 0x08, state);

				break;
			case 0x3F0: // Enable/disable output pin
				output = RxData[0];
				state = RxData[1];

				Set_Output(output, state & 0x01);

				break;
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
