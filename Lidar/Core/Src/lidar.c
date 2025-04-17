#include "lidar.h"

extern TIM_HandleTypeDef htim17;

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

// Variables for lidar PWM control
sPWM_t lidar_pwm = {
		.ccr1 = 0,
		.inc = 1
};

uint16_t last_angle;
uint32_t last_timestamp;

sVector3_t point_cloud[100];
sVector3_t beacon_pc[100];
uint16_t pc_index = 0;
uint16_t b_pc_index = 0;

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
} convert_theta1, convert_theta2, convert_distance1, convert_distance2, convert_x, convert_y, convert_z, convert_t;


//UART_HandleTypeDef *huart2_pc;

//uint8_t cabin_bytes[17];
int8_t delta_theta;
uint8_t u_delta_theta;
uint16_t u_distance;
float distance, theta;

sDetection_t detection = {
		.front = 500, .back = 500
};

uint8_t express_scan_status = 0;
uint8_t proccessing_status = 0;


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

void Timer_Delay(uint16_t count){
	uint16_t i=0;

	// 1ms interrupt
	HAL_TIM_Base_Start_IT(&htim17);

	while(i < count){
		if((TIM17->SR & 0x02) >> 1){
			TIM17->SR &= ~(0x02);
			TIM17->CNT = 0;
			++i;

		}
	}

	HAL_TIM_Base_Stop_IT(&htim17);
}

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

//	Timer_Delay(1);
	HAL_Delay(1);
}

// Initiates a core reset, after rebooting the LIDAR will enter IDLE state
void Lidar_Reset(UART_HandleTypeDef *huart){
	uint8_t msg[] = {START, RESET};

	// No response exists for this command, host system should wait for at least 10ms before sending another request
	HAL_UART_Transmit_DMA(huart, msg, 2);

//	Timer_Delay(10);

	HAL_Delay(10);
}

// Function of this command is unknown, it's sent from the SDK in this order
void Lidar_Unknown(UART_HandleTypeDef *huart){
	uint8_t msg[] = {START, 0xFF, 0x04, 0x00, 0x00, 0x00, 0x00, 0x5E};

	// No response exists for this command, host system should wait for at least 10ms before sending another request
	HAL_UART_Transmit_DMA(huart, msg, 8);

//	Timer_Delay(1);

	HAL_Delay(1);
}

// Returns the current status of lidar: good, warning or error
void Lidar_Get_Health(UART_HandleTypeDef *huart){
	uint8_t msg[] = {START, GET_HEALTH};

	Change_Size_DMA(7);
	HAL_UART_Transmit_DMA(huart, msg, 2);

//	Timer_Delay(2);

	// Minimal time needed for the response to come in before sending next command
	HAL_Delay(2);
}

// Returns samplerate for Standard scan mode - first two bytes, and Express scan mode - second two bytes
void Lidar_Get_Samplerate(UART_HandleTypeDef *huart){
	uint8_t msg[8] = {START, GET_SAMPLERATE};

	Change_Size_DMA(7);
	HAL_UART_Transmit_DMA(huart, msg, 2);

//	Timer_Delay(2);

	// Minimal time needed for the response to come in before sending next command
	HAL_Delay(2);

}

// Returns hardware information about lidar
void Lidar_Get_Info(UART_HandleTypeDef *huart){
	uint8_t msg[8] = {START, GET_INFO};

	Change_Size_DMA(7);
	HAL_UART_Transmit_DMA(huart, msg, 2);

//	Timer_Delay(3);

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

//	Timer_Delay(2);

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

//	while (huart->RxState != HAL_UART_STATE_READY);

	// Waits for lidar to reach speed
//	Timer_Delay(500);

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

//	Timer_Delay(1);

	// No response exists for this command, host system should wait for at least 1ms before sending another request
	HAL_Delay(1);
}

// Sends 32 measurements at once
void Lidar_Express_Scan(UART_HandleTypeDef *huart, uint8_t scan_mode_id){
	uint8_t msg[9] = {START, EXPRESS_SCAN, 0x05, scan_mode_id, 0x00, 0x00, 0x00, 0x00, 0x00};
	msg[8] = Lidar_CRC(msg, 8, 0);

	Change_Size_DMA(7);
	HAL_UART_Transmit_DMA(huart, msg, 9);

//	Timer_Delay(1);

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
				express_scan_status = 1;

				response.sync        = (rx_buff[1] & 0xF0) | ((rx_buff[0] & 0xF0) >> 4);     // should be 0x5A
				response.checksum    = ((rx_buff[1] & 0x0F) << 4) | (rx_buff[0] & 0x0F);
				response.start_angle = (float)((((uint16_t)(rx_buff[3] & 0x7F) << 8) | rx_buff[2]) / 64);
				response.S           = rx_buff[3] & 0x80;
				uint8_t crc = Lidar_CRC(rx_buff, response_desc.length, 2);  // excluding sync bytes
				if(response.checksum != crc){
					// bad message
//					HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
//					HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
					last_response = response;
					memcpy(last_rx_buff, rx_buff, sizeof(rx_buff));

//					HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);

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

						Process_Distance(distance, (uint16_t)theta);

						// distance2 and theta2
						u_distance = ( ((uint16_t)last_rx_buff[i+3] << 8) | (last_rx_buff[i+2] & 0xFC) ) >> 2;
						distance = (float)u_distance;

						u_delta_theta = ((last_rx_buff[i+2] & 0x03) << 4) | ((last_rx_buff[i+4] & 0xF0) >> 4);
						delta_theta = (u_delta_theta ^ (1<<5)) - (1<<5);  // 2s complement

						theta = last_response.start_angle + ((angle_diff/32.0) * (k+1)) - (float)delta_theta / 8.0;

						Process_Distance(distance, (uint16_t)theta);
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

void Polar2Cartesian(float distance, uint16_t angle, sVector3_t* point){
	point->vector[0] = self.x + cos(deg2rad(angle) - self.theta) * distance;
	point->vector[1] = self.y + sin(self.theta - deg2rad(angle)) * distance;
	point->vector[2] = 400.0;
}

uint8_t Process_Detection(float distance, uint16_t angle){
	// LIDAR relative x
	float x = distance * sin(deg2rad(angle));

	// Front detection
	if ( (angle > (360 - LIDAR_FOV/2) || angle < LIDAR_FOV/2) && distance < detection.front && fabs(x) < LIDAR_SIDE_DISTANCE){
		return 'F';
	}
	// Back detection
	else if ( (angle > (180 - LIDAR_FOV/2) && angle < (180 + LIDAR_FOV/2)) && distance < detection.back && fabs(x) < LIDAR_SIDE_DISTANCE ){
		return 'B';
	}
	// No detection
	else{
		return 0;
	}
}

void Process_Distance(float distance, uint16_t angle){
	sVector3_t point;

	// Normalize angle
	angle += 353;
	angle %= 360;

	if (last_angle > 357 && last_angle < 360){
		if (pc_index >= 1 ) Get_Opponent();
		if (b_pc_index >= 1) Get_Beacons();

		memset(beacon_pc, 0, sizeof(beacon_pc));
		b_pc_index = 0;
		memset(point_cloud, 0, sizeof(point_cloud));
		pc_index = 0;
	}

	if (distance > 0 ) {
		Polar2Cartesian(distance, angle, &point);

		if ((point.vector[0] <= 1000 && point.vector[0] >= 0) &&
			(point.vector[1] <= 1000 && point.vector[1] >= 0)) {
			// Point in bounds of table

			uint8_t det = Process_Detection(distance, angle);

			if (det != 0){
				uint8_t msg[1] = {det};
				FDCAN_Send_Data(0x4CF, FDCAN_DLC_BYTES_1, 1, msg);
			}

			point_cloud[pc_index++] = point;
			if (pc_index >= 100) pc_index = 0;
		}
		else {
			// Point in some beacon region
			sVector3_t beacon = Choose_Beacon(point);

			if (beacon.vector[0] != 0){
				beacon.vector[0] = point.vector[0];
				beacon.vector[1] = point.vector[1];
				beacon.vector[2] = distance;

				beacon_pc[b_pc_index++] = beacon;
				if (b_pc_index > 300) b_pc_index = 0;
			}
//
//				convert_x.f = point.vector[0];
//				convert_y.f = point.vector[1];
//				convert_z.f = distance;
//
//				uint8_t bytes[13];
//				for(j=0; j<4; ++j){
//					bytes[j]    = convert_x.u[j];
//					bytes[j+4]  = convert_y.u[j];
//					bytes[j+8]  = convert_z.u[j];
//				}
//
//				bytes[12] = beacon;
//
//				FDCAN_Send_Data(0x4CD, FDCAN_DLC_BYTES_12, 9, bytes);
//			}

		}

	}

	last_angle = angle;

}

float norm(sVector3_t a, sVector3_t b){
	return sqrt((a.vector[0] - b.vector[0])*(a.vector[0] - b.vector[0]) + (a.vector[1] - b.vector[1])*(a.vector[1] - b.vector[1]));
}

uint16_t Segment_PC(sVector3_t* pc, uint16_t ind, uint8_t radius){
	// Segment point cloud into groups of points
	uint8_t i=0, j=0, k=0;
	float sum_x=0, sum_y=0;

	while (i < ind-1) {
		// Average close points
		for (j=i+1; j < ind; ++j){
			if (norm(pc[i], pc[j]) > radius){
				break;
			}
		}

		// Average all points
		sum_x = sum_y = 0;
		for (k=i; k<j; ++k){
			sum_x += pc[k].vector[0];
			sum_y += pc[k].vector[1];
		}

		pc[i].vector[0] = sum_x/(j-i);
		pc[i].vector[1] = sum_y/(j-i);

		// Shift point cloud to the left to get rid of averaged points
		ind -= j-i-1;
		for (k=i+1; k < ind; ++k){
			pc[k] = pc[k+(j-i-1)];
		}

		++i;
	}

	// Average first and last group if they are close enough
	if (ind > 1 && norm(pc[0], pc[ind-1]) <= BEACON_SUPPORT_DIAMETER){
		pc[0].vector[0] = (pc[0].vector[0] + pc[ind-1].vector[0])/2.0;
		pc[0].vector[1] = (pc[0].vector[1] + pc[ind-1].vector[1])/2.0;

		ind -= 1;
	}

	return ind;
}

sVector3_t Choose_Beacon(sVector3_t position){
	sVector3_t point;
	// upper left beacon
	if ((position.vector[0] > -150 && position.vector[0] <= 0) &&
	    (position.vector[1] > 1850 && position.vector[1] <= 2000)){
		point.vector[0] = -40 - 50;
		point.vector[1] = 2000 - 55;
	} // middle left beacon
	else if ((position.vector[0] > -150 && position.vector[0] <= 0) &&
			  (position.vector[1] > 925  && position.vector[1] <= 1075)){
		point.vector[0] = -40 - 50;
		point.vector[1] = 1000;
	} // lower left beacon
	else if ((position.vector[0] > -150 && position.vector[0] <= 0) &&
			  (position.vector[1] > 0    && position.vector[1] <= 150)){
		point.vector[0] = -40 - 50;
		point.vector[1] = 55;
	} // upper right beacon
	else if ((position.vector[0] > 3000 && position.vector[0] <= 3150) &&
			  (position.vector[1] > 1850 && position.vector[1] <= 2000)){
		point.vector[0] = 3000 + 40 + 50;
		point.vector[1] = 2000 - 55;
	} // middle right beacon
	else if ((position.vector[0] > 3000 && position.vector[0] <= 3150) &&
			  (position.vector[1] > 925  && position.vector[1] <= 1075)){
		point.vector[0] = 3000 + 40 + 50;
		point.vector[1] = 1000;
	} // lower right beacon
	else if ((position.vector[0] > 3000 && position.vector[0] <= 3150) &&
			  (position.vector[1] > 0    && position.vector[1] <= 150)){
		point.vector[0] = 3000 + 40 + 50;
		point.vector[1] = 55;
	}

	point.vector[2] = position.vector[2];
	return point;
}

void Get_Beacons(){
	uint8_t i=0, j=0;

	b_pc_index = Segment_PC(beacon_pc, b_pc_index, 100);

	uint8_t bytes[48];
	// At least two beacons
	if (b_pc_index > 1){
		for (i=0; i<b_pc_index; i++){
			sVector3_t point = Choose_Beacon(beacon_pc[i]);
			// If the point can fall in one of the beacon regions send it
			if (point.vector[0] != 0 && point.vector[1] != 0){
				convert_x.f = point.vector[0];
				convert_y.f = point.vector[1];
				convert_z.f = point.vector[2];

				for(j=0; j<4; ++j){
					bytes[j]    = convert_x.u[j];
					bytes[j+4]  = convert_y.u[j];
					bytes[j+8]  = convert_z.u[j];
				}

			}
 		}

		FDCAN_Send_Data(0x4CD, FDCAN_DLC_BYTES_48, 48, bytes);
	}
}

void Get_Opponent(){
	uint8_t i=0;

	pc_index = Segment_PC(point_cloud, pc_index, BEACON_SUPPORT_DIAMETER);

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

	float op_x = opponent.x, op_y=opponent.y;

	// Filter last known opponent position and new estimate
	opponent.x = 0.5*op_x + 0.5*(new_op.vector[0]); // + cos(theta)*42.5);  // max diameter = (70+100)/2 = 85/2 = 42.5
	opponent.y = 0.5*op_y + 0.5*(new_op.vector[1]); // + sin(theta)*42.5);
	opponent.theta = atan2((opponent.y - op_y),(opponent.x - op_x));

	uint32_t timestamp = HAL_GetTick();

	opponent.speed = (sqrt( (op_x - opponent.x)*(op_x - opponent.x) + (op_y - opponent.y)*(op_y - opponent.y)))/(timestamp - last_timestamp);

	last_timestamp = timestamp;

	// x, y, theta, timestamp = 4+4+4+4 bytes
	uint8_t bytes[16];
	convert_x.f = opponent.x;
	convert_y.f = opponent.y;
	convert_z.f = opponent.theta;
	convert_t.f = opponent.speed;

	for(int j=0; j<4; ++j){
		bytes[j]    = convert_x.u[j];
		bytes[j+4]  = convert_y.u[j];
		bytes[j+8]  = convert_z.u[j];
		bytes[j+12] = convert_t.u[j];
	}

	FDCAN_Send_Data(0x4CE, FDCAN_DLC_BYTES_16, 16, bytes);
}

