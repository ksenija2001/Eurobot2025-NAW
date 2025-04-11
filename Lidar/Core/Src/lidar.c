#include "lidar.h"

uint8_t rx_buff[BUFFER_SIZE];
uint8_t last_rx_buff[BUFFER_SIZE];

sDescriptor_t response_desc = {
	.length = 0,
	.send_mode = 0, // 0 - signel response, 1 - multiple responses
	.data_type = 0,
	.packet_num = 0
};

// Variables for parsing lidar express scan data
sResponse_t response;
sResponse_t last_response = {0};
//sCabin_t cabin;
//float angle_diff;

// Variables for lidar PWM control
sPWM_t lidar_pwm = {
		.ccr1 = 0,
		.inc = 1
};

sVector3_t point_cloud[360];
uint16_t pc_index = 0;
sVector3_t last_point_cloud[360];
uint16_t lpc_index = 0;

// Odometry data
sOdom_t opponent;
sOdom_t self;


// Debug variables
sInfo_t lidar_info;
sScanMode_t scan_mode;  // debugging structure for finding differences between scan modes
sHealth_t health;
uint16_t num_scan_modes = 5;  // this lidar supports 5 scan modes
uint16_t typ_scan_mode = 3;   // typical scan mode is Sensitivity

union U_F{
	float f;
	uint8_t u[4];
} convert_theta1, convert_theta2, convert_distance1, convert_distance2, convert_x, convert_y, convert_z;


//UART_HandleTypeDef *huart2_pc;

//uint8_t cabin_bytes[17];
int8_t delta_theta;
uint8_t u_delta_theta;
uint16_t u_distance;
float distance, theta;

uint8_t front = 0, back = 0;
uint32_t last_detection = 0;


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

void Lidar_Start(TIM_HandleTypeDef* motor_htim, TIM_HandleTypeDef* ramp_htim, TIM_HandleTypeDef* parse_htim, UART_HandleTypeDef* huart){
	  HAL_TIM_Base_Start_IT(parse_htim);   /* Timer for parsing LIDAR data*/

	  Lidar_Get_Health(huart);
	  Lidar_Motor_Speed(motor_htim, TIM_CHANNEL_1, 660, ramp_htim);
	  Lidar_Stop(huart);
	  Lidar_Get_Info(huart);
	  Lidar_Get_Lidar_Conf(huart, 0x01, 0x04, 0x00);
	  Lidar_Express_Scan(huart, 0x00); // Legacy Express Scan
}

void Lidar_Stop_All(TIM_HandleTypeDef* motor_htim, TIM_HandleTypeDef* ramp_htim, TIM_HandleTypeDef* parse_htim,  UART_HandleTypeDef* huart){
	Lidar_Stop(huart);

    Lidar_Motor_Speed(motor_htim, TIM_CHANNEL_1, 0, ramp_htim);
    Lidar_Motor_Stop(motor_htim, TIM_CHANNEL_1);
    Lidar_Get_Health(huart);

    HAL_TIM_Base_Stop_IT(parse_htim);
}

// Calculates XOR of all bytes in message to be sent to lidar
uint8_t Lidar_CRC(uint8_t msg[], uint8_t length, uint8_t start){
	uint8_t crc = 0;

	for(uint8_t j=start; j<length; ++j){
		crc ^= msg[j];
	}
	return crc; // ^ 0xff;
}

// Nromalization of angles defined by rplidar protocol
float Angle_Diff(float w1, float w2){
	if(w1 <= w2){
		return w2 - w1;
	} else {
		return 360 + w2 - w1;
	}
}

// Stops the current scanning state and enters IDLE state
void Lidar_Stop(UART_HandleTypeDef *huart){
	uint8_t msg[] = {START, STOP};

	// No response exists for this command, host system should wait for at least 1ms before sending another request
	HAL_UART_Transmit_DMA(huart, msg, 2);

	HAL_Delay(1);
}

// Initiates a core reset, after rebooting the LIDAR will enter IDLE state
void Lidar_Reset(UART_HandleTypeDef *huart){
	uint8_t msg[] = {START, RESET};

	// No response exists for this command, host system should wait for at least 10ms before sending another request
	HAL_UART_Transmit_DMA(huart, msg, 2);

	HAL_Delay(10);
}

// Function of this command is unknown, it's sent from the SDK in this order
void Lidar_Unknown(UART_HandleTypeDef *huart){
	uint8_t msg[] = {START, 0xFF, 0x04, 0x00, 0x00, 0x00, 0x00, 0x5E};

	// No response exists for this command, host system should wait for at least 10ms before sending another request
	HAL_UART_Transmit_DMA(huart, msg, 8);

	HAL_Delay(1);
}

// Returns the current status of lidar: good, warning or error
void Lidar_Get_Health(UART_HandleTypeDef *huart){
	uint8_t msg[] = {START, GET_HEALTH};

	Change_Size_DMA(7);
	HAL_UART_Transmit_DMA(huart, msg, 2);

	// Minimal time needed for the response to come in before sending next command
	HAL_Delay(2);
}

// Returns samplerate for Standard scan mode - first two bytes, and Express scan mode - second two bytes
void Lidar_Get_Samplerate(UART_HandleTypeDef *huart){
	uint8_t msg[8] = {START, GET_SAMPLERATE};

	Change_Size_DMA(7);
	HAL_UART_Transmit_DMA(huart, msg, 2);

	// Minimal time needed for the response to come in before sending next command
	HAL_Delay(2);

}

// Returns hardware information about lidar
void Lidar_Get_Info(UART_HandleTypeDef *huart){
	uint8_t msg[8] = {START, GET_INFO};

	Change_Size_DMA(7);
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

	Change_Size_DMA(7);
	HAL_UART_Transmit_DMA(huart, msg, length+1);

	HAL_Delay(2);
}

// Stops PWM channel that is used to generate PWM signal for lidar
void Lidar_Motor_Stop(TIM_HandleTypeDef *tim, uint8_t channel){
	HAL_TIM_PWM_Stop(tim, channel);
}

// Sets duty cycle for lidar PWM signal and starts the PWM timer
void Lidar_Motor_Speed(TIM_HandleTypeDef *tim, uint8_t channel, uint16_t rpm, TIM_HandleTypeDef *tim_ramp){
	// Cap max speed of lidar to 800RPM
	if(rpm > 800) rpm = 800;
	float duty = (PWM_SENS * rpm + PWM_SENS_OFFSET) / 100;
	lidar_pwm.ccr1 = round(PWM_ARR * duty);

	if(rpm == 0) lidar_pwm.ccr1 = 0;
	lidar_pwm.inc = lidar_pwm.ccr1 > TIM3->CCR1 ? 1:-1;

	// Increments rpm every 0.5ms for
	HAL_TIM_Base_Start_IT(tim_ramp);

	HAL_TIM_PWM_Start(tim, channel);

	// Waits for lidar to reach speed
	HAL_Delay(500);
}

// Called from stm32g4xx_it.c in HAL interrupt handler for TIM6 every 0.5ms
void TIM6_IT(TIM_HandleTypeDef *tim){
	if(TIM3->CCR1 != lidar_pwm.ccr1){
		TIM3->CCR1 += lidar_pwm.inc;
	} else {
		HAL_TIM_Base_Stop_IT(tim);
	}
}

// Each measurement sample will be sent out individually in data packets of 5 bytes
void Lidar_Scan(UART_HandleTypeDef *huart){
	uint8_t msg[8] = {START, SCAN};

	Change_Size_DMA(7);
	HAL_UART_Transmit(huart, msg, 2, 100);
	// No response exists for this command, host system should wait for at least 1ms before sending another request
	HAL_Delay(1);
}

// Sends 32 measurements at once
void Lidar_Express_Scan(UART_HandleTypeDef *huart, uint8_t scan_mode_id){
	uint8_t msg[9] = {START, EXPRESS_SCAN, 0x05, scan_mode_id, 0x00, 0x00, 0x00, 0x00, 0x00};
	msg[8] = Lidar_CRC(msg, 8, 0);

	Change_Size_DMA(7);
	HAL_UART_Transmit_DMA(huart, msg, 9);

	HAL_Delay(1);
}

// 10us interrupt - lidar sends response 14us after request
void TIM7_IT(TIM_HandleTypeDef *tim){
	/* Waits for new data to be received by DMA */
	if(data_buf.new_data){
		data_buf.new_data = 0;

		/* Saves current state of buffer in case a new DMA receive interrupt occurs */
		memcpy(rx_buff, data_buf.data, data_buf.length);

		if(data_buf.length == 7){
			// Before all useful data a descriptor is sent with information about the useful data: number of bytes, single/multiple packets, data(command) type
			response_desc.length = (uint32_t)((rx_buff[5]<<24) &  0x3FFFFFFF) | (uint32_t)(rx_buff[4]<<16) | (uint32_t)(rx_buff[3]<<8) | rx_buff[2];
			//response_desc.send_mode = rx_buff[5] >> 6;
			response_desc.data_type = rx_buff[6];

			Change_Size_DMA(response_desc.length);
		} else {
			// Every time a packet with data is received it's counted so as to know when to stop listening for new packets
			//response_desc.packet_num++;

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
					uint8_t j = 4;
					for(j=4; rx_buff[j] != 0x00; ++j)
						scan_mode.name[j-4] = rx_buff[j];
					scan_mode.name[j-4] = '\0';
					break;
				default:
					break;
				}

				break;
			case 0x82:  // Express scan in scan mode 1
				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
				response.sync        = (rx_buff[1] & 0xF0) | ((rx_buff[0] & 0xF0) >> 4);     // should be 0x5A
				response.checksum    = ((rx_buff[1] & 0x0F) << 4) | (rx_buff[0] & 0x0F);
				response.start_angle = (float)((((uint16_t)(rx_buff[3] & 0x7F) << 8) | rx_buff[2]) / 64);
				response.S           = rx_buff[3] & 0x80;
				uint8_t crc = Lidar_CRC(rx_buff, response_desc.length, 2);  // excluding sync bytes
				if(response.checksum != crc){
					// bad message
					HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
					HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
					last_response = response;
					memcpy(last_rx_buff, rx_buff, sizeof(rx_buff));

					HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);

					break;
				}

				// information about the next start_angle is needed to calculate theta for this data response
				if(last_response.sync != 0){
					float angle_diff = Angle_Diff(last_response.start_angle, response.start_angle);

					// iterates through the rest of buffer, 80 bytes = 16 * 5 bytes(cabin)
					for(uint8_t i=4, k=1; i<response_desc.length; i=i+5, k=k+2){

						// distance1 and theta1
						u_distance = ( ((uint16_t)last_rx_buff[i+1] << 8) | (last_rx_buff[i] & 0xFC) ) >> 2;
						distance = (float)u_distance;

						u_delta_theta = ((last_rx_buff[i] & 0x03) << 4) | (last_rx_buff[i+4] & 0x0F);
						delta_theta = (u_delta_theta ^ (1<<5)) - (1<<5);  // 2s complement

						theta = last_response.start_angle + ((angle_diff/32.0) * k) - (float)delta_theta / 8.0;

						Process_Distance(distance, theta);

						// distance2 and theta2
						u_distance = ( ((uint16_t)last_rx_buff[i+3] << 8) | (last_rx_buff[i+2] & 0xFC) ) >> 2;
						distance = (float)u_distance;

						u_delta_theta = ((last_rx_buff[i+2] & 0x03) << 4) | ((last_rx_buff[i+4] & 0xF0) >> 4);
						delta_theta = (u_delta_theta ^ (1<<5)) - (1<<5);  // 2s complement

						theta = last_response.start_angle + ((angle_diff/32.0) * (k+1)) - (float)delta_theta / 8.0;

						Process_Distance(distance, theta);
					}
				}

//				if(front){
//					  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1);
//				}
//				else{
//					  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0);
//				}
//
//				if(HAL_GetTick() > last_detection + 150){
//					front = 0;
//				}

				last_response = response;
				memcpy(last_rx_buff, rx_buff, sizeof(rx_buff));

				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
				break;
			default:
				break;
			}
		}
	}
}

void Point_Cloud_To_Bytes(sVector3_t pc[], uint16_t size, uint8_t* bytes){
	bytes[0] = START;
	for (uint16_t i=0; i<size; ++i){
		convert_x.f = pc[i].vector[0];
		convert_y.f = pc[i].vector[1];
		convert_z.f = pc[i].vector[2];

		for (uint8_t j=i*12+1; j<i+1+4; ++j){
			bytes[j] = convert_x.u[j];
			bytes[j+4] = convert_y.u[j];
			bytes[j+8] = convert_z.u[j];
		}

	}
}

void Cabin_To_Bytes(sCabin_t cabin, uint8_t* cabin_bytes){
	cabin_bytes[0] = START;

	convert_theta1.f = cabin.theta1;
	convert_theta2.f = cabin.theta2;
	convert_distance1.f = cabin.distance1;
	convert_distance2.f = cabin.distance2;

	for(int j=0; j<4; ++j){
		cabin_bytes[j+1]    = convert_theta1.u[j];
		cabin_bytes[j+1+4]  = convert_distance1.u[j];
		cabin_bytes[j+1+8]  = convert_theta2.u[j];
		cabin_bytes[j+1+12] = convert_distance2.u[j];
	}

}

sVector3_t Process_Distance(float distance, float angle){
	sVector3_t point;

	// Normalize angle
	if(angle>360) {
//		Get_Opponent();  // TODO get opponent position periodically because LIDAR sends data every 10us
		angle -= 360;
	}

	// Convert angle and distance to a point in global coordinate system
	ConvertDist2Point((int16_t)angle, distance, self.x, self.y, self.theta, &point);
	if (point.vector[0] <= 2950 || point.vector[0] >= 50 ||
		point.vector[1] <= 1950 || point.vector[1] >= 50) {
		// Point in bounds of table

		point_cloud[pc_index++] = point;
		// TODO how to save points to be able to search them easiliy for

	} else if (point.vector[0] > 2950 || point.vector[0] <= 3050 ||
			   point.vector[1] > 1950 || point.vector[1] <= 2050) {
		// Point in region of beacons
	}

	return point;
}

float norm(sVector3_t a, sVector3_t b){
	return sqrt((a.vector[0] - b.vector[0])*(a.vector[0] - b.vector[0]) + (a.vector[1] - b.vector[1])*(a.vector[1] - b.vector[1]));
}


void Get_Opponent(){
	// Segment point cloud into groups of points
	uint8_t i=0, j=0, k=0;

	while (i < pc_index-1) {
		sVector3_t pivot = point_cloud[i];

		// Average close points
		for (j=i+1; j < pc_index; ++j){
			if (norm(pivot, point_cloud[j]) <= BEACON_SUPPORT_DIAMETER){
				pivot.vector[0] = (pivot.vector[0] + point_cloud[j].vector[0])/2.0;
				pivot.vector[1] = (pivot.vector[1] + point_cloud[j].vector[1])/2.0;
			}
		}

		// Shift point cloud to the left to get rid of averaged points
		point_cloud[i] = pivot;
		pc_index -= j-i-1;
		for (k=i+1; k < pc_index; ++k){
			point_cloud[k] = point_cloud[k+(j-i-1)];
		}

		++i;
	}

	// Average first and last group if they are close enough
	if (norm(point_cloud[0], point_cloud[pc_index-1]) <= BEACON_SUPPORT_DIAMETER){
		point_cloud[0].vector[0] = (point_cloud[0].vector[0] + point_cloud[pc_index-1].vector[0])/2.0;
		point_cloud[0].vector[1] = (point_cloud[0].vector[1] + point_cloud[pc_index-1].vector[1])/2.0;

		pc_index -= 1;
	}

	sVector3_t new_op = point_cloud[0];
	sVector3_t last_op = {.vector= {opponent.x, opponent.y, 400.0} };
	float min = norm(last_op, new_op);
	float dist;
	for(i=1; i<pc_index; ++i){  // if there is only one candidate it will stay point_cloud[0]
		dist = norm(last_op, point_cloud[i]);
		if (dist < min){
			new_op = point_cloud[i];
			min = dist;
		}
	}

	float dir = (new_op.vector[1] - self.y)/(new_op.vector[0] - self.x);
	float theta = atan(dir);

	// Filter last known opponent position and new estimate
	opponent.x = 0.5*opponent.x + 0.5*(new_op.vector[0] + cos(theta)*42.5);  // max diameter = (70+100)/2 = 85/2 = 42.5
	opponent.y = 0.5*opponent.y + 0.5*(new_op.vector[1] + sin(theta)*42.5);
	opponent.theta = 0.5*opponent.theta + 0.5*theta;

	// Reset point cloud
//	memcpy(last_point_cloud, point_cloud, sizeof(point_cloud));
//	lpc_index = pc_index;
	memset(point_cloud, 0, sizeof(point_cloud));
	pc_index = 0;
}

