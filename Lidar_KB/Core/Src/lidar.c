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

uint8_t Lidar_CRC(uint8_t msg[], uint8_t length){
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
		case 04: // GET_INFO - returns 20 bytes
			break;
		case 06: // GET_HEALTH - returns 3 bytes
			if(rx_buff[0] == 0x00) {} // Good
			else if(rx_buff[0] == 0x01) {} // Warning
			else if(rx_buff[0] == 0x02) {
				uint16_t error_code = (uint16_t)(rx_buff[2] << 8) | rx_buff[1];
			} // Error
			break;
		case 15: // GET_SAMPLERATE - return 4 bytes
			break;
		case 20: // GET_LIDAR_CONF
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
	HAL_UART_Transmit(huart, msg, 2, 100);
	// No response exists for this command, host system should wait for at least 1ms before sending another request
	//HAL_Delay(1);
}

// Initiates a core reset, after rebooting the LIDAR will enter IDLE state
void Lidar_Reset(UART_HandleTypeDef *huart){
	uint8_t msg[] = {START, RESET};
	HAL_UART_Transmit(huart, msg, 2, 100);
	// No response exists for this command, host system should wait for at least 2ms before sending another request
	//HAL_Delay(2);
}

void Lidar_Unknown(UART_HandleTypeDef *huart){
	uint8_t msg[] = {START, 0xFF, 0x04, 0x00, 0x00, 0x00, 0x00, 0x5E};
	HAL_UART_Transmit(huart, msg, 8, 100);
	// No response exists for this command, host system should wait for at least 2ms before sending another request
	//HAL_Delay(2);
}

void Lidar_Get_Health(UART_HandleTypeDef *huart){
	uint8_t msg[] = {START, GET_HEALTH};
	HAL_UART_Transmit(huart, msg, 2, 100);

	//HAL_Delay(1);

	//Lidar_Receive_Response(huart, 7);
}

void Lidar_Get_Samplerate(UART_HandleTypeDef *huart){
	uint8_t msg[8] = {START, GET_SAMPLERATE};

	HAL_UART_Transmit(huart, msg, 2, 100);
	//HAL_Delay(1);

	//Lidar_Receive_Response(huart, 7);
}

void Lidar_Get_Info(UART_HandleTypeDef *huart){
	uint8_t msg[8] = {START, GET_INFO};

	HAL_UART_Transmit(huart, msg, 2, 100);

	//HAL_Delay(1);

	//Lidar_Receive_Response(huart, 7);
}

void Lidar_Get_Lidar_Conf(UART_HandleTypeDef *huart, uint8_t config, uint8_t request_length, uint8_t mode){
	uint8_t length = request_length + 3; // total length of message
	// 0x01 config is unknows, but appareantly needed at the start and end of get lidar conf
	// 0x7C payload signifies typical scan mode command
	// 0x71 payload signifies us cost per measurement sample
	// 0x74 payload signifies max measurement distance
	// 0x75 payload signifies type of answer command
	// 0x7F payload signifies name of selected scan mode
	uint8_t msg[] = {START, GET_LIDAR_CONF, request_length, config, 0x00, 0x00, 0x00, mode, 0x00, 0x00};
	msg[length] = Lidar_CRC(msg, length);

	HAL_UART_Transmit(huart, msg, length+1, 100);

	//HAL_Delay(1);

	//Lidar_Receive_Response(huart, 7);
}

// Sets motor speed in RPM, IDLE state can be achieved by setting RPM=0
//void Lidar_Motor_Speed(UART_HandleTypeDef *huart, uint16_t rpm){
//	if(rpm > 600){
//		rpm = 600;
//	}
//
//	uint8_t msg[] = {START, MOTOR_SPEED, 0x02, (uint8_t)(rpm & 0xff), (uint8_t)(rpm >> 8), 0x00};
//	uint8_t crc = Lidar_CRC(msg, 5);
//	msg[5] = crc;
//
//	HAL_UART_Transmit(huart, msg, 6, 100);
//	// No response exists for this command, host system should wait for at least 1ms before sending another request
//	//HAL_Delay(1);
//}

void Lidar_Motor_Start(TIM_HandleTypeDef *tim, uint8_t channel){
	HAL_TIM_PWM_Start(tim, channel);
}

void Lidar_Motor_Stop(TIM_HandleTypeDef *tim, uint8_t channel){
	HAL_TIM_PWM_Stop(tim, channel);
}

void Lidar_Motor_Speed(TIM_HandleTypeDef *tim, uint8_t channel, uint16_t rpm){
	// Cap max speed of lidar to 800RPM
	if(rpm > 800) rpm = 800;

	// Duty cycle with regards to the experimental sensitivity measured for this lidar
	uint16_t duty = round(PWM_ARR * (PWM_SENS * rpm + PWM_SENS_OFFSET) / 100);
	if(rpm == 0) duty = 0;

	TIM3->CCR1 = duty;
	HAL_TIM_PWM_Start(tim, channel);
	//__HAL_TIM_SET_COMPARE(tim, channel, duty);
}

void Lidar_Scan(UART_HandleTypeDef *huart){
	uint8_t msg[8] = {START, SCAN};

	HAL_UART_Transmit(huart, msg, 2, 100);
	// No response exists for this command, host system should wait for at least 1ms before sending another request
	//HAL_Delay(1);

	//Lidar_Receive_Response(huart, 7);
}

void Lidar_Express_Scan(UART_HandleTypeDef *huart){
	uint8_t msg[9] = {START, EXPRESS_SCAN, 0x05, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00};
	msg[8] = Lidar_CRC(msg, 8);

	HAL_UART_Transmit(huart, msg, 9, 100);
	// No response exists for this command, host system should wait for at least 1ms before sending another request
	//HAL_Delay(1);

	//Lidar_Receive_Response(huart, 132);
}

void Start_Connection(UART_HandleTypeDef *huart){
	// GET_INFO - return 20 bytes
	// Unknown - 0xA5 0xFF 0x04 0x00 0x00 0x00 0x00 0x5E, no response
	// GET_INFO - retruns 20 bytes
	// GET_HEALTH - returns 3 bytes
	// GET_LIDAR_CONF weird - 0xA5 0x84 0x04 0x01 0x00 0x00 0x00 0x24
	// STOP
	// GET_INFO
	// GET_INFO
	// GET_LIDAR_CONF for Typical Scan Mode
	// STOP
	// GET_INFO
	// GET_LIDAR_CONF for Scan Mode us Per Sample - for scan mode returned in previous get lidar conf
	// GET_LIDAR_CONF for Scan Mode Max Distance - for scan mode returned in ||
	// GET_LIDAR_CONF for Scan Mode Ans Type - ||
	// GET_LIDAR_CONF for Scan Mode Name - ||
	// GET_LIDAR_CONF weird - 0xA5 0x84 0x04 0x01 0x00 0x00 0x00 0x24
	// EXPRESS_SCAN
}

