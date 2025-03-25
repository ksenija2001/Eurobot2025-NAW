/*
 * ax_servo.c
 *
 *  Created on: Mar 10, 2025
 *      Author: xenia
 */

#include "ax_servo.h"

uint8_t rx_buffer[256] = {0};

uint8_t last_command = 0;
uint8_t error, id;
uint8_t crc;
uint8_t moving_status;
volatile uint8_t moving_servos[SERVO_NUM];
uint8_t moving_counter[SERVO_NUM];
uint16_t position, angle, speed, speed_perc;
uint8_t ind = 0;


void Init_AX_Servo(){

}

void AX_Transmit(UART_HandleTypeDef* huart, uint8_t *tx_buffer, uint8_t tx_length, uint8_t rx_length){
	HAL_HalfDuplex_EnableTransmitter(huart);
	HAL_UART_Transmit(huart, tx_buffer, tx_length, 1000);

	if (rx_length > 0){
//		memset(rx_buffer, 0, sizeof(rx_buffer));
		HAL_HalfDuplex_EnableReceiver(huart);
		HAL_UARTEx_ReceiveToIdle_DMA(huart, rx_buffer, rx_length);
	}


}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_TC) {
//		HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
//		rx_set = 1;

//		ind=0;
//		uint8_t ff_counter = 0;
//		while (ff_counter <= 2){
//			if (ff_counter == 2 && rx_buffer[ind] != 0xFF){
//				break;
//			}
//			if (rx_buffer[ind] == 0xFF){
//				++ff_counter;
//			}
//			++ind;
//		}

//		for (uint8_t i=0; i<256; ++i){
//			if (rx_buffer[i] == 0xFF){
//				ind++;
//			} else break;
//		}

		id = rx_buffer[2];
		error = rx_buffer[4];

		if (error & OVERLOAD_MASK) {
			// Current load cannot be controlled by the set torque
			// TODO FDCAN warning
		}

		if (error & OVERHEATING_MASK) {
			// Internal temperature is out of range
			// TODO FDCAN warning
		}

		if (error & ANGLE_LIMIT_MASK) {
			// Goal position is out of range
			// TODO FDCAN warning
		}

// TODO FDCAN transmit for get commands

		switch (last_command) {
//		case PING:
//			break;
//		case TORQUE:
//			break;
//		case LED:
//			break;
//		case GOAL_POSITION:
//			break;
//		case MOVING_SPEED:
//			break;
		case PRESENT_POSITION:
			position =  ((uint16_t)rx_buffer[6] << 8) | rx_buffer[5];
			angle = 300/1023.0 * position;
			uint8_t msg[] = {id, (uint8_t)((angle & 0xFF00) >> 8), (angle & 0x00FF)};
			FDCAN_Send_Data(0x531, FDCAN_DLC_BYTES_3, 3, msg);
			break;
		case PRESENT_SPEED:
			speed =  ((uint16_t)rx_buffer[6] << 8) | rx_buffer[5];
			speed_perc = 100/1023.0 * speed;
			break;
		case PRESENT_LOAD:
			break;
		case MOVING:
			// TODO NOT READING MOVING STATUS CORRECTLY
			moving_status = rx_buffer[5];

			if ( !moving_status ){
				moving_servos[id] = 0;
				moving_counter[id] = 0;

				uint8_t msg[] = {id, 1};
				FDCAN_Send_Data(0x53F, FDCAN_DLC_BYTES_2, 2, msg);
			}
		}
	} else {
//		memset(rx_buffer, 0, sizeof(rx_buffer));
		HAL_UARTEx_ReceiveToIdle_DMA(huart, rx_buffer, Size);
	}

}

// 50ms timer for checking if servos are moving after setting goal position
void TIM6_Moving_IT(TIM_HandleTypeDef* tim, UART_HandleTypeDef* huart){
	for( uint8_t i=1; i<=SERVO_NUM; ++i){
		if (moving_servos[i]) {
//			if (++moving_counter[i] > 30){
//				// Servo has been moving for more than 1s
//				// TODO FDCAN warning that servo can't get into position
//				uint8_t msg[] = {i, 0};
//				FDCAN_Send_Data(0x53F, FDCAN_DLC_BYTES_2, 2, msg);
//			}

			Get_Moving_Status(huart, i);
//				HAL_Delay(10);
//			}
		}



	}
}

uint8_t Checksum(uint8_t* buffer, uint8_t len){
	crc = 0;
	for (uint8_t i=2; i<len; i++){
		crc += buffer[i];
	}

	return ~crc;
}

void Ping(UART_HandleTypeDef* huart, uint8_t ID){
	while( huart->RxState != HAL_UART_STATE_READY);

	uint8_t msg[] = {HEADER, HEADER, ID, 2, PING, 0x00};
	msg[5] = Checksum(msg, 5);

	last_command = REBOOT;
	AX_Transmit(huart, msg, 6, 6);
}

void Reboot(UART_HandleTypeDef* huart, uint8_t ID){
	while( huart->RxState != HAL_UART_STATE_READY);

	uint8_t msg[] = {HEADER, HEADER, ID, 2, REBOOT, 0x00};
	msg[5] = Checksum(msg, 5);

	last_command = REBOOT;
	AX_Transmit(huart, msg, 6, 6);
}

void Enable_Torque(UART_HandleTypeDef* huart, uint8_t ID, uint8_t on_off){
	while( huart->RxState != HAL_UART_STATE_READY);

	uint8_t msg[] = {HEADER, HEADER, ID, 2 + 2, WRITE, TORQUE, (on_off & 0x01), 0x00};
	msg[7] = Checksum(msg, 7);

	last_command = TORQUE;

	if (ID != 0xFE)
		AX_Transmit(huart, msg, 8, 6);
	else
		AX_Transmit(huart, msg, 8, 0);
}

void Enable_LED(UART_HandleTypeDef* huart, uint8_t ID, uint8_t on_off){
	while( huart->RxState != HAL_UART_STATE_READY);

	uint8_t msg[] = {HEADER, HEADER, ID, 2 + 2, WRITE, LED, (on_off & 0x01), 0x00};
	msg[7] = Checksum(msg, 7);

	last_command = LED;
	AX_Transmit(huart, msg, 8, 6);
}

void Sync_Set_Goal_Position(UART_HandleTypeDef* huart, uint8_t* IDs, uint16_t* angles, uint8_t* speeds, uint8_t size){
	while( huart->RxState != HAL_UART_STATE_READY);

	uint8_t length = size * (4+1) + 2 + 2;
	uint8_t msg[64] = {HEADER, HEADER, 0xFE, length, SYNC_WRITE, GOAL_POSITION, 4};  // moving speed is right behind goal position, writing 4 bytes in total

	for (uint8_t i=0; i<size; ++i){
		if ( angles[i] > 300) angles[i] = 300;
		if ( speeds[i] > 100) speeds[i] = 100;

		position = 1023/300.0 * angles[i];
		speed = 1023/100.0 * speeds[i];

		msg[5*(i+1) + 2] = IDs[i];                              // servo ID
		msg[5*(i+1) + 3] = (uint8_t)(position & 0x00FF);        // position lower byte
		msg[5*(i+1) + 4] = (uint8_t)((position & 0xFF00) >> 8); // position upper byte
		msg[5*(i+1) + 5] = (uint8_t)(speed & 0x00FF);           // speed lower byte
		msg[5*(i+1) + 6] = (uint8_t)((speed & 0xFF00) >> 8);    // speed upper byte
	}

	msg[length + 3] = Checksum(msg, length + 3);

	AX_Transmit(huart, msg, length + 4, 0); // Sync Write uses Broadcast ID -> no status response

	for (uint8_t i=0; i<size; ++i){
		moving_servos[IDs[i]] = 1;
	}
}

void Set_Goal_Position(UART_HandleTypeDef* huart, uint8_t ID, uint16_t angle){
	while( huart->RxState != HAL_UART_STATE_READY);

	if ( angle > 300) angle = 300;
	position = 1023/300.0 * angle;
	uint8_t msg[] = {HEADER, HEADER, ID, 3 + 2, WRITE, GOAL_POSITION, (uint8_t)(position & 0x00FF), (uint8_t)((position & 0xFF00) >> 8), 0x00};
	msg[8]  = Checksum(msg, 8);

	last_command = GOAL_POSITION;
	AX_Transmit(huart, msg, 9, 6);
//	Get_Moving_Status(huart, ID);
	moving_servos[ID] = 1;
}

void Set_Moving_Speed(UART_HandleTypeDef* huart, uint8_t ID, uint8_t speed_percentage){
	while( huart->RxState != HAL_UART_STATE_READY);

	if ( speed_percentage > 100 ) speed_percentage = 100;
	speed = 1023/100.0 * speed_percentage;
	uint8_t msg[] = {HEADER, HEADER, ID, 3 + 2, WRITE, MOVING_SPEED, (uint8_t)(speed & 0x00FF), (uint8_t)((speed & 0xFF00) >> 8), 0x00};
	msg[8] = Checksum(msg, 8);

	last_command = MOVING_SPEED;
	AX_Transmit(huart, msg, 9, 6);
}

void Get_Present_Position(UART_HandleTypeDef* huart, uint8_t ID){
	while( huart->RxState != HAL_UART_STATE_READY);

	uint8_t msg[] = {HEADER, HEADER, ID, 2 + 2 , READ, PRESENT_POSITION, 0x02, 0x00};
	msg[7] = Checksum(msg, 7);

	last_command = PRESENT_POSITION;
	AX_Transmit(huart, msg, 8, 8);
}

void Get_Present_Speed(UART_HandleTypeDef* huart, uint8_t ID){
	while( huart->RxState != HAL_UART_STATE_READY);

	uint8_t msg[] = {HEADER, HEADER, ID, 2 + 2 , READ, PRESENT_SPEED, 0x02, 0x00};
	msg[7] = Checksum(msg, 7);

	last_command = PRESENT_SPEED;
	AX_Transmit(huart, msg, 8, 8);
}

void Get_Moving_Status(UART_HandleTypeDef* huart, uint8_t ID){
	while( huart->RxState != HAL_UART_STATE_READY);

	uint8_t msg[] = {HEADER, HEADER, ID, 2 + 2 , READ, MOVING, 0x01, 0x00};
	msg[7] = Checksum(msg, 7);

	last_command = MOVING;
	AX_Transmit(huart, msg, 8, 7);
}



