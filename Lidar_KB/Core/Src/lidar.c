#include <lidar.h>

//uint8_t tx_buff[BUFFER_SIZE];
uint8_t rx_buff[BUFFER_SIZE];
sDescriptor_t descriptor = {
	.length = 0,
	.send_mode = 4,
	.data_type = 0,
	.packet_num = 0
};

// Enables DMA receive, data won't be received if not called again
void Lidar_Receive_Response(UART_HandleTypeDef *huart, uint8_t length){
	HAL_UART_Receive_DMA(huart, rx_buff, length);
}

uint8_t Lidar_CRC(uint8_t *msg, uint8_t length){
	uint8_t crc = 0;
	uint8_t i;

	for(i=0; i<length; ++i){
		crc ^= msg[i];
	}
	return crc;
}

// Triggered when data is ready
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(descriptor.send_mode == 4){
		descriptor.length = (uint32_t)((rx_buff[5]<<24) &  0x3FFFFFFF) | (uint32_t)(rx_buff[4]<<16) | (uint32_t)(rx_buff[3]<<8) | (uint32_t)rx_buff[2];
		descriptor.send_mode = rx_buff[5] >> 6;
		descriptor.data_type = rx_buff[6];
	} else {
		descriptor.packet_num++;

		switch(descriptor.data_type){
		case 15:
			break;
		case 20:
			break;
		default:
			break;
		}
	}

	// Single response mode, 0 data packets have been received or Multiple response mode
	if((descriptor.send_mode == 0 && descriptor.packet_num == 0) || descriptor.send_mode == 1){
		Lidar_Receive_Response(huart, descriptor.length);
	} else {
		descriptor.packet_num = 0;
		descriptor.send_mode = 4; // all packets have been received
	}

}

// Stops the current scanning state and enters IDLE state
void Lidar_Stop(UART_HandleTypeDef *huart){
	uint8_t msg[] = {START, STOP};
	HAL_UART_Transmit_DMA(huart, msg, 2);
	// No response exists for this command, host system should wait for at least 1ms before sending another request
	HAL_Delay(1);
}

// Initiates a core reset, after rebooting the LIDAR will enter IDLE state
void Lidar_Reset(UART_HandleTypeDef *huart){
	uint8_t msg[] = {START, RESET};
	HAL_UART_Transmit_DMA(huart, msg, 2);
	// No response exists for this command, host system should wait for at least 2ms before sending another request
	HAL_Delay(2);
}

void Lidar_Get_Health(UART_HandleTypeDef *huart){
	uint8_t msg[] = {START, GET_HEALTH};
	HAL_UART_Transmit_DMA(huart, msg, 2);
	// No response exists for this command, host system should wait for at least 1ms before sending another request
	HAL_Delay(1);

	Lidar_Receive_Response(huart, 7);
}

void Lidar_Get_Samplerate(UART_HandleTypeDef *huart){
	uint8_t msg[8] = {START, GET_SAMPLERATE};

	HAL_UART_Transmit_DMA(huart, msg, 2);
	// No response exists for this command, host system should wait for at least 1ms before sending another request
	//HAL_Delay(1);

	Lidar_Receive_Response(huart, 7);
}

void Lidar_Get_Lidar_Conf(UART_HandleTypeDef *huart){
	uint8_t msg[8] = {START, GET_LIDAR_CONF, 0x04, 0xB9, 0x01, 0xFC, 0x00};
	msg[7] = Lidar_CRC(&msg, 7);

	HAL_UART_Transmit_DMA(huart, msg, 8);
	// No response exists for this command, host system should wait for at least 1ms before sending another request
	//HAL_Delay(1);

	Lidar_Receive_Response(huart, 7);
}

// Sets motor speed in RPM, IDLE state canbe achieved by setting RPM=0
void Lidar_Motor_Speed(UART_HandleTypeDef *huart, uint16_t rpm){
	if(rpm > 600){
		rpm = 600;
	}

	uint8_t msg[6] = {START, MOTOR_SPEED, 2, (uint8_t)(rpm & 0xff), (uint8_t)(rpm >> 8)};
	uint8_t crc = Lidar_CRC(&msg, 5);
	msg[5] = crc;

	HAL_UART_Transmit_DMA(huart, msg, 6);
	// No response exists for this command, host system should wait for at least 1ms before sending another request
	HAL_Delay(1);
}

void Lidar_Express_Scan(UART_HandleTypeDef *huart){

}

