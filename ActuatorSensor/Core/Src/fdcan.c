/*
 * fdcan.c
 *
 *  Created on: Jan 20, 2025
 *      Author: xenia
 */

#include "fdcan.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern UART_HandleTypeDef huart1;

FDCAN_FilterTypeDef sFilterConfig;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[64];

FDCAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[64];

uint8_t send_status = HAL_ERROR;
uint8_t receive_status = HAL_ERROR;

uint8_t ids[10];
uint16_t angles[10];
uint8_t speeds[10];

uint8_t FDCAN_Init(FDCAN_HandleTypeDef *hfdcan)
{
	uint8_t status = HAL_ERROR;

	HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);

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
			case 0x530: // Set servo position and speed
				uint8_t servo_num = RxData[0];
				for (uint8_t i=0; i<servo_num; ++i){
					ids[i] = RxData[i*4 +1];
					angles[i] = ((uint16_t)RxData[i*4 + 1 + 1] << 8) | RxData[i*4 + 2 + 1];
					speeds[i] = RxData[i*4 + 3 + 1];
				}

				Sync_Set_Goal_Position(&huart1, ids, angles, speeds, servo_num);

				break;
			case 0x531: // Get servo position
				uint8_t id = RxData[0];
				Get_Present_Position(&huart1, id);
//				float left_diameter = Bytes2Float(RxData, 0);
//				float right_diameter = Bytes2Float(RxData, 4);
//				float track = Bytes2Float(RxData, 8);
				break;
			case 0x532: // Set position for RC servos
				uint8_t servo_states = RxData[0];

				Set_Angle(0, (servo_states & 0x01)*180);
				Set_Angle(1, (servo_states & 0x02)*180);
				Set_Angle(2, (servo_states & 0x04)*180);
				Set_Angle(3, (servo_states & 0x08)*180);
				Set_Angle(4, (servo_states & 0x10)*180);
				Set_Angle(5, (servo_states & 0x20)*180);
				Set_Angle(6, (servo_states & 0x40)*180);
				Set_Angle(7, (servo_states & 0x80)*180);

				break;
			case 0x690: // Enable/disable output pin
				uint8_t output = RxData[0];
				uint8_t state = RxData[1];

				Set_Output(output, state);

				break;
			default:
				receive_status = HAL_ERROR;
			}
		}
	}
}

uint8_t FDCAN_Send_Data(uint32_t id, uint32_t dlc, uint8_t size, uint8_t *data)
{
	// send_status = HAL_ERROR;

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
