#include <lidar.h>

uint8_t rx_buff[BUFFER_SIZE];
uint8_t last_rx_buff[BUFFER_SIZE];

sDescriptor_t response_desc = {
	.length = 0,
	.send_mode = 0, // 0 - signel response, 1 - multiple responses
	.data_type = 0,
	.packet_num = 0
};

uint8_t awaiting_response_desc;  // flag that indicates whether the next received packet will be a descriptor or data
sResponse_t response;
sResponse_t last_response = {0};
sCabin_t cabin;
float_t delta_theta1;
float_t delta_theta2;
float_t angle_diff;

// Debug variables
sInfo_t lidar_info;
sScanMode_t scan_mode;  // debugging structure for finding differences between scan modes
sHealth_t health;
uint16_t num_scan_modes = 5;  // this lidar supports 5 scan modes
uint16_t typ_scan_mode = 3;   // typical scan mode is Sensitivity

// Since Sensitivity utilizes ultra capsulated data format - which is hard to decode, scan mode 1 will be used

// Scan Mode 0 - sample_duration = 0x1B9us (441us == 2.2kHz)
// 				 answer_type = 0x81  // legacy
// 				 max_distance = 0x10m = 16m
//               name = Standard

// Scan Mode 1 - sample_duration = 0xFCus (252us == 4kHz)
// 				 answer_type = 0x82 // capsulated
// 				 max_distance = 0x10m = 16m
//               name = Express

// Scan Mode 2 - sample_duration = 0xFCus (126us == 8kHz)
// 				 answer_type = 0x84  // ultra capsulated
// 				 max_distance = 0x10m = 16m
//               name = Boost

// Scan Mode 3 - sample_duration = 0xFCus (126us == 8kHz)
// 				 answer_type = 0x84  // ultra capsulated
// 				 max_distance = 0x10m = 16m
//               name = Sensitivity

// Scan Mode 4 - sample_duration = 0xFCus (200us == 5kHz)
// 				 answer_type = 0x84 // ultra capsulated
// 				 max_distance = 0x10m = 16m
//               name = Stability

// Calculates XOR of all bytes in message to be sent to lidar
uint8_t Lidar_CRC(uint8_t msg[], uint8_t length, uint8_t start){
	uint8_t crc = 0;
	uint8_t i;

	for(i=start; i<length; ++i){
		crc ^= msg[i];
	}
	return crc;
}

// Nromalization of angles defined by rplidar protocol
float_t Angle_Diff(float_t w1, float_t w2){
	if(w1 <= w2){
		return w2 - w1;
	} else {
		return 360 + w2 - w1;
	}
}

// Transmit complete callback that initiates DMA receive of descriptor
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(awaiting_response_desc){
		HAL_UART_Receive_DMA(huart, rx_buff, 7);
	}
}

// Receive complete callback, triggered when data is ready to be read
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(awaiting_response_desc){
		// Before all useful data a descriptor is sent with information about the useful data: number of bytes, single/multiple packets, data(command) type
		awaiting_response_desc = 0;
		response_desc.length = (uint32_t)((rx_buff[5]<<24) &  0x3FFFFFFF) | (uint32_t)(rx_buff[4]<<16) | (uint32_t)(rx_buff[3]<<8) | rx_buff[2];
		response_desc.send_mode = rx_buff[5] >> 6;
		response_desc.data_type = rx_buff[6];
	} else {
		// Every time a packet with data is received it's counted so as to know when to stop listening for new packets
		response_desc.packet_num++;

		switch(response_desc.data_type){
		case 0x04: // GET_INFO
			lidar_info.model          = rx_buff[0];
			lidar_info.firmware_minor = rx_buff[1];
			lidar_info.firmware_major = rx_buff[2];
			lidar_info.hardware       = rx_buff[3];
			// serial_number in rx_buff[4] to rx_buff[19] is discarded
			break;
		case 0x06: // GET_HEALTH - returns 3 bytes
			health.status = rx_buff[0];
			switch(health.status){
			case 0x00:  // Good
				break;
			case 0x01:  // Warning
				// TODO handle warning
				break;
			case 0x02:  // Error
				health.error_code = (uint16_t)(rx_buff[2] << 8) | rx_buff[1];
				// TODO handle error
				break;
			default:
				break;
			}

			break;
		case 0x15: // GET_SAMPLERATE
			lidar_info.standard_samplerate = (uint16_t)(rx_buff[1]<<8) | rx_buff[0]; // 0x1B9 = 441us
			lidar_info.express_samplerate = (uint16_t)(rx_buff[3]<<8) | rx_buff[2];  // 0x0FC = 252us
			break;
		case 0x20: // GET_LIDAR_CONF
			uint8_t response_type = rx_buff[0]; // other 3 bytes of type are not used

			switch(response_type){
			case 0x70:
				num_scan_modes = (uint16_t)(rx_buff[5]<<8) | rx_buff[4];
				break;
			case 0x71:
				scan_mode.sample_duration = (uint32_t)(rx_buff[7]<<24) | (uint32_t)(rx_buff[6]<<16) | (uint32_t)(rx_buff[5]<<8) | rx_buff[4];
				scan_mode.sample_duration /= (1<<8);
				break;
			case 0x74:
				scan_mode.max_distance = (uint32_t)(rx_buff[7]<<24) | (uint32_t)(rx_buff[6]<<16) | (uint32_t)(rx_buff[5]<<8) | rx_buff[4];
				scan_mode.max_distance /= (1<<8);
				break;
			case 0x75:
				scan_mode.answer_type = rx_buff[4];
				break;
			case 0x7C:
				typ_scan_mode = (uint16_t)(rx_buff[5]<<8) | rx_buff[4];
				break;
			case 0x7F:
				uint8_t i = 4;
				for(i=4; rx_buff[i] != 0x00; ++i)
					scan_mode.name[i-4] = rx_buff[i];
				scan_mode.name[i-4] = '\0';
				break;
			default:
				break;
			}

			break;
		case 0x84:  // Express scan in scan mode 1
			// TODO Measure time it takes to process one response packet

			response.sync        = (rx_buff[1] & 0xF0) | (rx_buff[0] & 0xF0);     // should be 0x5A
			response.checksum    = (rx_buff[1] & 0x0F) | (rx_buff[0] & 0x0F);
			response.start_angle = (float_t)((uint16_t)((rx_buff[3] & 0x7F) << 8) | rx_buff[2]) / 64.0;
			response.S           = rx_buff[3] & 0x80;

			if(response.checksum != Lidar_CRC(rx_buff, response_desc.length, 4)){
				// bad message
				break;
			}

			// information about the next start_angle is needed to calculate theta for this data response
			if(last_response.sync != 0){
				angle_diff = Angle_Diff(last_response.start_angle, response.start_angle);

				// iterates through the rest of buffer, 80 bytes = 16 * 5 bytes(cabin)
				uint8_t k = 1;
				for(uint8_t i=4; i<response_desc.length; i=i+5){
					cabin.distance1 = (float_t)((uint16_t)(last_rx_buff[i+1] << 5) | (last_rx_buff[i] & 0xFC)) / 4.0;  // TODO check if /4 is needed - it's mentioned in the SCAN section, but not in EXPRESS SCAN
					delta_theta1    = (float_t)((uint16_t)(last_rx_buff[i] & 0x18) | (last_rx_buff[i+4] & 0x0F));
					cabin.theta1    = last_response.start_angle + angle_diff/32 * k - delta_theta1;

					cabin.distance2 = (float_t)((uint16_t)(last_rx_buff[i+3] << 5) | (last_rx_buff[i+2] & 0xFC)) / 4.0;
					delta_theta2    = (float_t)((uint16_t)(last_rx_buff[i+2] & 0x18) | (last_rx_buff[i+4] & 0xF0));
					cabin.theta2    = last_response.start_angle + angle_diff/32 * (k+1) - delta_theta1;

					k = k+2;

					// TODO Send two samples to host system
					//HAL_UART_Transmit(huart, pData, Size, Timeout);
				}
			}

			last_response = response;
			memcpy(rx_buff, last_rx_buff, sizeof(rx_buff));

			break;
		default:
			break;
		}
	}

	// Single response mode will send a descriptor and only one data packet
	// Multiple response mode will send a descriptor and multiple data packets
	if((response_desc.send_mode == 0 && response_desc.packet_num == 0) ||
	   (response_desc.send_mode == 1 && response_desc.packet_num < response_desc.length)){
		// Enables DMA receive, data won't be received if not called again
		HAL_UART_Receive_DMA(huart, rx_buff, response_desc.length);
	} else {
		response_desc.packet_num = 0;
	}

}

// Stops the current scanning state and enters IDLE state
void Lidar_Stop(UART_HandleTypeDef *huart){
	uint8_t msg[] = {START, STOP};

	// No response exists for this command, host system should wait for at least 1ms before sending another request
	awaiting_response_desc = 0;
	HAL_UART_Transmit_DMA(huart, msg, 2);

	HAL_Delay(1);
}

// Initiates a core reset, after rebooting the LIDAR will enter IDLE state
void Lidar_Reset(UART_HandleTypeDef *huart){
	uint8_t msg[] = {START, RESET};

	// No response exists for this command, host system should wait for at least 10ms before sending another request
	awaiting_response_desc = 0;
	HAL_UART_Transmit_DMA(huart, msg, 2);

	HAL_Delay(10);
}

// Function of this command is unknown, it's sent from the SDK in this order
void Lidar_Unknown(UART_HandleTypeDef *huart){
	uint8_t msg[] = {START, 0xFF, 0x04, 0x00, 0x00, 0x00, 0x00, 0x5E};

	// No response exists for this command, host system should wait for at least 10ms before sending another request
	awaiting_response_desc = 0;
	HAL_UART_Transmit_DMA(huart, msg, 8);

	HAL_Delay(1);
}

// Returns the current status of lidar: good, warning or error
void Lidar_Get_Health(UART_HandleTypeDef *huart){
	uint8_t msg[] = {START, GET_HEALTH};

	awaiting_response_desc = 1;
	HAL_UART_Transmit_DMA(huart, msg, 2);

	// Minimal time needed for the response to come in before sending next command
	HAL_Delay(2);
}

// Returns samplerate for Standard scan mode - first two bytes, and Express scan mode - second two bytes
void Lidar_Get_Samplerate(UART_HandleTypeDef *huart){
	uint8_t msg[8] = {START, GET_SAMPLERATE};

	awaiting_response_desc = 1;
	HAL_UART_Transmit_DMA(huart, msg, 2);

	// Minimal time needed for the response to come in before sending next command
	HAL_Delay(2);

}

// Returns hardware information about lidar
void Lidar_Get_Info(UART_HandleTypeDef *huart){
	uint8_t msg[8] = {START, GET_INFO};

	awaiting_response_desc = 1;
	HAL_UART_Transmit_DMA(huart, msg, 2);
	// Minimal time needed for the response to come in before sending next command
	HAL_Delay(3);
}

// Returns information about scan modes of lidar
void Lidar_Get_Lidar_Conf(UART_HandleTypeDef *huart, uint8_t config, uint8_t request_length, uint8_t mode){
	uint8_t length = request_length + 3; // total length of message
	// 0x01 config is unknown, but appareantly needed at the start and end of get lidar conf
	// 0x70 payload signifies scan mode count
	// 0x7C payload signifies typical scan mode command
	// 0x71 payload signifies us cost per measurement sample
	// 0x74 payload signifies max measurement distance
	// 0x75 payload signifies type of answer command
	// 0x7F payload signifies name of selected scan mode
	uint8_t msg[] = {START, GET_LIDAR_CONF, request_length, config, 0x00, 0x00, 0x00, mode, 0x00, 0x00};
	msg[length] = Lidar_CRC(msg, length, 0);

	awaiting_response_desc = 1;
	HAL_UART_Transmit_DMA(huart, msg, length+1);

	HAL_Delay(2);
}

// Stops PWM channel that is used to generate PWM signal for lidar
void Lidar_Motor_Stop(TIM_HandleTypeDef *tim, uint8_t channel){
	HAL_TIM_PWM_Stop(tim, channel);
}

// Sets duty cycle for lidar PWM signal and starts the PWM timer
void Lidar_Motor_Speed(TIM_HandleTypeDef *tim, uint8_t channel, uint16_t rpm){
	// Cap max speed of lidar to 800RPM
	if(rpm > 800) rpm = 800;

	// Duty cycle with regards to the experimental sensitivity measured for this lidar
	uint16_t duty = round(PWM_ARR * (PWM_SENS * rpm + PWM_SENS_OFFSET) / 100);
	if(rpm == 0) duty = 0;

	TIM3->CCR1 = duty;
	HAL_TIM_PWM_Start(tim, channel);

	HAL_Delay(10);
}

// Each measurement sample will be sent out individually in data packets of 5 bytes
void Lidar_Scan(UART_HandleTypeDef *huart){
	uint8_t msg[8] = {START, SCAN};

	awaiting_response_desc = 1;
	HAL_UART_Transmit(huart, msg, 2, 100);
	// No response exists for this command, host system should wait for at least 1ms before sending another request
	HAL_Delay(1);
}

// Sends 32 measurements at once
void Lidar_Express_Scan(UART_HandleTypeDef *huart, uint8_t scan_mode_id){
	uint8_t msg[9] = {START, EXPRESS_SCAN, 0x05, scan_mode_id, 0x00, 0x00, 0x00, 0x00, 0x00};
	msg[8] = Lidar_CRC(msg, 8, 0);

	awaiting_response_desc = 1;
	HAL_UART_Transmit_DMA(huart, msg, 9);

	HAL_Delay(1);
}

// Order of commands sent by SDK when using this lidar
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


