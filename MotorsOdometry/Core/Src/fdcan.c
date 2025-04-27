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
			case 0x4F0: // Reset odometry
				sOdom_t new_odom = {
					.x = Bytes2Float(RxData, 0),
					.y = Bytes2Float(RxData, 4),
					.theta = Bytes2Float(RxData, 8),
				};

				Reset_Odometry(&new_odom);
				uint8_t data[1] = {0x01};
				FDCAN_Send_Data(0x4DE, FDCAN_DLC_BYTES_1, 1, data);

				break;
			case 0x4F1: // Wheel parameters configuration
				float left_diameter = Bytes2Float(RxData, 0);
				float right_diameter = Bytes2Float(RxData, 4);
				float track = Bytes2Float(RxData, 8);

				Config_Encoder_Wheel(&left, left_diameter, track);
				Config_Encoder_Wheel(&right, right_diameter, track);

				break;
			case 0x4D0: // Set reference for motor speed
				Set_Speed(&left_motor, Bytes2Float(RxData, 0));
				Set_Speed(&right_motor, Bytes2Float(RxData, 4));

				break;
			case 0x4D1: // Set reference for motor RPM

				Set_RPM(&left_motor, Bytes2Float(RxData, 0));
				Set_RPM(&right_motor, Bytes2Float(RxData, 4));

				break;
			case 0x4D2:
				HAL_GPIO_WritePin(LED_CAN_RX_GPIO_Port, LED_CAN_RX_Pin, GPIO_PIN_SET);
				synthesis_start_distance(
						Bytes2Float(RxData, 0), //p
						Bytes2Float(RxData, 4), //v
						Bytes2Float(RxData, 8));//a
				break;
			case 0x4D3:
				synthesis_start_rotateFor(
						Bytes2Float(RxData, 0), //theta
						Bytes2Float(RxData, 4), //w
						Bytes2Float(RxData, 8));//alpha
				break;
			case 0x4D4:
				synthesis_start_rotateTo(
						Bytes2Float(RxData, 0), //theta
						Bytes2Float(RxData, 4), //w
						Bytes2Float(RxData, 8));//alpha
				break;
			case 0x4D5:
				synthesis_start_XY(
						Bytes2Float(RxData, 0), //x
						Bytes2Float(RxData, 4), //y
						RxData[8],				//direction
						Bytes2Float(RxData, 9), //v
						Bytes2Float(RxData, 13), //a
						Bytes2Float(RxData, 17), //w
						Bytes2Float(RxData, 21));//alpha
				break;
			case 0x4D6:
				uint8_t len = RxData[0];
				char direction = RxData[1];
				float speed = Bytes2Float(RxData, 2);
				float x[MAX_BEZIERS_IN_SPLINE];
				float y[MAX_BEZIERS_IN_SPLINE];
				float theta[MAX_BEZIERS_IN_SPLINE];

				for(uint8_t i = 0;i<len;i++){
					x[i] = Bytes2Float(RxData, 6 +i*12);
					y[i] = Bytes2Float(RxData, 10 +i*12);
					theta[i] = Bytes2Float(RxData, 14+i*12);
				}
				spline_move(x, y, theta, len, speed, direction);
				break;
			case 0x4D8:
				//Detection activated
				HAL_GPIO_WritePin(LED_CAN_RX_GPIO_Port, LED_CAN_RX_Pin, GPIO_PIN_RESET);
				synthesis_activate_detection(Bytes2Float(RxData, 0));
				break;
			case 0x4D7:
				spline_stop();
				synthesis_stop();
				FDCAN_Send_Data(0x4DE, FDCAN_DLC_BYTES_1, 1, data);

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
