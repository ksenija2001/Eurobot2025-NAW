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

uint32_t VBS_SCALED_BASE[5] = {RPLIDAR_VARBITSCALE_X16_DEST_VAL,
                   RPLIDAR_VARBITSCALE_X8_DEST_VAL,
                   RPLIDAR_VARBITSCALE_X4_DEST_VAL,
                   RPLIDAR_VARBITSCALE_X2_DEST_VAL,
                   0
};

uint32_t VBS_SCALED_LVL[5] = {4, 3, 2, 1, 0};

uint32_t VBS_TARGET_BASE[5] = {(0x1 << RPLIDAR_VARBITSCALE_X16_SRC_BIT),
                   (0x1 << RPLIDAR_VARBITSCALE_X8_SRC_BIT),
                   (0x1 << RPLIDAR_VARBITSCALE_X4_SRC_BIT),
                   (0x1 << RPLIDAR_VARBITSCALE_X2_SRC_BIT),
                   0
};

// Variables for parsing lidar express scan data
sResponse_t last_response = {0};
uint16_t counter = 0;

// Variables for lidar PWM control
sPWM_t lidar_pwm = {
		.ccr1 = 0,
		.inc = 1
};

uint16_t last_angle;
uint32_t last_timestamp;

sVector3_t point_cloud[100];
sVector3_t beacon_pc[300];
uint16_t pc_index = 0;
uint16_t b_pc_index = 0;

sVector3_t beacons[6][100] = {0};
uint8_t beacon_indexes[6] = {0};

// Odometry data
sOdom_t opponent;
sOdom_t self;
sOdom_t self_lidar;
float speed = 0;
float ang_speed = 0;
float speed_cnt = 0;

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

uint8_t process_beacon = 0;
uint8_t process_opponent = 0;
int8_t det = 0;
int8_t last_det = 0;

sVector3_t new_robot = {0};
uint8_t color = 0;


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
	  Lidar_Motor_Speed(motor_htim, TIM_CHANNEL_1, 400, ramp_htim);
	  Lidar_Stop(huart);
	  Lidar_Get_Info(huart);
	  Lidar_Get_Lidar_Conf(huart, 0x01, 0x04, 0x00);
	  Lidar_Express_Scan(huart, typ_scan_mode); // Legacy Express Scan - 0x00
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
			sResponse_t response = {0};
			uint8_t crc;

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
			case 0x82:  // Express scan in scan mode 1 - 0x82
				response.sync        = (rx_buff[1] & 0xF0) | ((rx_buff[0] & 0xF0) >> 4);     // should be 0x5A
				response.checksum    = ((rx_buff[1] & 0x0F) << 4) | (rx_buff[0] & 0x0F);
				response.start_angle = (float)(((((uint16_t)(rx_buff[3] & 0x7F)) << 8) | rx_buff[2])/64.0);
				response.S           = rx_buff[3] >> 7;
				crc = Lidar_CRC(rx_buff, response_desc.length, 2);  // excluding sync bytes
				if(response.checksum != crc){
					// bad message
					last_response = response;
					break;
				}

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

						Process_Distance(distance, (uint16_t)theta, response.S);

						// distance2 and theta2
						u_distance = ( ((uint16_t)last_rx_buff[i+3] << 8) | (last_rx_buff[i+2] & 0xFC) ) >> 2;
						distance = (float)u_distance;

						u_delta_theta = ((last_rx_buff[i+2] & 0x03) << 4) | ((last_rx_buff[i+4] & 0xF0) >> 4);
						delta_theta = (u_delta_theta ^ (1<<5)) - (1<<5);  // 2s complement

						theta = last_response.start_angle + ((angle_diff/32.0) * (k+1)) - (float)delta_theta / 8.0;

						Process_Distance(distance, (uint16_t)theta, response.S);
					}
				}

				last_response = response;
				break;
			case 0x84:  // Express scan in scan mode 2 and 3 - 0x84
				response.sync        = (rx_buff[1] & 0xF0) | ((rx_buff[0] & 0xF0) >> 4);     // should be 0x5A
				response.checksum    = ((rx_buff[1] & 0x0F) << 4) | (rx_buff[0] & 0x0F);
				response.start_angle_q6 = ((((uint16_t)(rx_buff[3] & 0x7F)) << 8) | rx_buff[2]); // / 64);
				response.S           = rx_buff[3] >> 7;
				crc = Lidar_CRC(rx_buff, response_desc.length, 2);  // excluding sync bytes
				if(response.checksum != crc){
					// bad message
					last_response = response;
					break;
				}

				RPlidarUltraCabin cabin = {0};
				for(uint8_t i=4, k=0; i<response_desc.length; i+=4, ++k){
					cabin.major = (((int32_t)(rx_buff[i+1] & 0x0F)) << 8) | rx_buff[i];
					cabin.predict1 = (((uint32_t)(rx_buff[i+2] & 0x3F)) << 4) | ((rx_buff[i+1] >> 4) & 0x0F);
					cabin.predict2 = (((uint32_t)(rx_buff[i+3] & 0xFF)) << 2) | ((rx_buff[i+2] >> 6) & 0x03);

					// Sign extension for 10-bit signed values
//					if (cabin.predict1 & 0x200) cabin.predict1 |= 0xFFFFFC00;
//					if (cabin.predict2 & 0x200) cabin.predict2 |= 0xFFFFFC00;

					response.ultra_cabins[k].major = cabin.major;
					response.ultra_cabins[k].predict1 = (int32_t)((cabin.predict1 << 10) >> 22);
					response.ultra_cabins[k].predict2 = (int32_t)(cabin.predict2 >> 22);
				}

				// information about the next start_angle is needed to calculate theta for this data response
				if(last_response.sync != 0){
					uint32_t curr_angle_q8 = ((uint32_t)response.start_angle_q6) << 2;
					uint32_t prev_angle_q8 = ((uint32_t)last_response.start_angle_q6) << 2;
					int32_t diff_angle_q8 = curr_angle_q8 - prev_angle_q8;
					if (prev_angle_q8 > curr_angle_q8)
						diff_angle_q8 += (((uint32_t)360) << 8);

					uint32_t angle_inc_q16 = (diff_angle_q8 << 3)/3;
					uint32_t current_angle_raw_q16 = prev_angle_q8 << 8;

					for (uint8_t pos = 0; pos < MAX_ULTRA_CABINS; pos++) {
						int32_t major = last_response.ultra_cabins[pos].major;
						int32_t predict1 = last_response.ultra_cabins[pos].predict1;
						int32_t predict2 = last_response.ultra_cabins[pos].predict2;

						int32_t major2 = (pos == MAX_ULTRA_CABINS - 1) ? response.ultra_cabins[0].major : last_response.ultra_cabins[pos + 1].major;


						uint32_t base1, base2;
						uint8_t scale1 = VarbitScale_Decode(major, &base1);
						uint8_t scale2 = VarbitScale_Decode(major2, &base2);

						if (!base1 && base2) {
							base1 = base2;
							scale1 = scale2;
						}

						uint32_t dist_q2[3] = {0};
						dist_q2[0] = base1 << 2;

						if ((uint32_t)predict1 == 0xFFFFFE00 || (uint32_t)predict1 == 0x1FF){
							dist_q2[1] = 0;
						} else {
							predict1 = predict1 << scale1;
							dist_q2[1] = (predict1 + base1) << 2;
						}

						if ((uint32_t)predict2 == 0xFFFFFE00 || (uint32_t)predict2 == 0x1FF){
							dist_q2[2] = 0;
						} else {
							predict2 = predict2 << scale2;
							dist_q2[2] = (predict2 + base2) << 2;
						}

						for (uint8_t c = 0; c < 3; c++) {
							uint8_t sync = (((current_angle_raw_q16 + angle_inc_q16) % (((uint32_t)360) << 16)) < angle_inc_q16) ? 1 : 0;

							int32_t offset_q16 = (int32_t)(7.5f * 3.1415926535f * (((uint32_t)1) << 16) / 180.0f);

							if (dist_q2[c] >= 200) {
								int32_t k1 = 98361;
								int32_t k2 = (int32_t)(k1 / dist_q2[c]);
								offset_q16 = (int32_t)(8.0f * 3.1415926535f * (((uint32_t)1) << 16) / 180.0f) - (k2 << 6) - (int32_t)((k2 * k2 * k2)/98304.0);
							}

							int32_t angle_q6 = (int32_t)(current_angle_raw_q16 - (uint32_t)(offset_q16 * 180 / 3.14159265f)) >> 10;
							current_angle_raw_q16 += angle_inc_q16;

							if (angle_q6 < 0) angle_q6 += (((int32_t)360) << 6);
							if (angle_q6 >= (((uint32_t)360) << 6)) angle_q6 -= (((int32_t)360) << 6);

							sync = sync | ((!sync) << 1);

							int32_t angle_q14 = (int32_t)((angle_q6 << 8)/90);

							Process_Distance((float)(dist_q2[c]/4.0), (float)(((angle_q14 * 90) >> 8)/64.0), sync);
						}
					}
				}

				last_response = response;
				break;
			default:
				break;
			}
		}
	}
}

uint8_t VarbitScale_Decode(int32_t scaled, uint32_t *decoded) {
	uint8_t scale_level = 0;
    for (uint8_t i = 0; i < 5; i++) {
        int32_t remain = scaled - VBS_SCALED_BASE[i];
        if (remain >= 0) {
            scale_level = VBS_SCALED_LVL[i];
            *decoded = VBS_TARGET_BASE[i] + (remain << scale_level);
            return scale_level;
        }
    }
    *decoded = 0;
    return 0;
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

uint8_t Check_Beacon_Points(){
	if (color == 'y' && beacon_indexes[0] > MIN_POINTS &&
						beacon_indexes[2] > MIN_POINTS &&
						beacon_indexes[4] > MIN_POINTS){
		return 1;
	} else if (color == 'b' && beacon_indexes[1] > MIN_POINTS &&
							   beacon_indexes[3] > MIN_POINTS &&
							   beacon_indexes[5] > MIN_POINTS){
		return 1;
	}

	return 0;
}
void Process_Distance(float distance, float angle, uint8_t new_scan){
	sVector3_t point;

	// Normalize angle
	if (angle > 360) angle -= 360;

	if (last_angle > 357 && last_angle < 360){
		if (pc_index > 20) process_opponent = 1;
		last_det = 0;  // reset detection
	} else if ((last_angle > 357 && last_angle < 360) || process_opponent == 2){
		memset(point_cloud, 0, sizeof(point_cloud));
		pc_index = 0;
		process_opponent = 0;
	}

	if (Check_Beacon_Points() && process_beacon == 0){
		process_beacon = 1;
	}

	if (distance > 0) {
		Polar2Cartesian(distance, angle, &point);

		// Point in bounds of table
		if ((point.vector[0] <= 2900 && point.vector[0] >= 100) &&
			(point.vector[1] <= 1900 && point.vector[1] >= 100)) {

			det = Process_Detection(distance, angle);
//			det += (curr_det != 0) ? 1 : -1;
//
//			if (det < 0) det = 0;
//
//			if (det > 10){
//				uint8_t msg[1] = {det};
//				FDCAN_Send_Data(0x4CF, FDCAN_DLC_BYTES_1, 1, msg);
//			}

			// React only on new detections
			if (det != 0 && det != last_det){
				uint8_t msg[1] = {det};
				FDCAN_Send_Data(0x4CF, FDCAN_DLC_BYTES_1, 1, msg);
			}

			last_det = det;

			// Opponent point cloud
			point_cloud[pc_index++] = point;
			if (pc_index >= 100) pc_index = 0;
		}
		// Point in some beacon region while robot is stationary
		else {
			sVector3_t beacon = {.vector={0,0,0}};
			point.vector[2] = angle;
			uint8_t beacon_num = Choose_Beacon(&point, &beacon);

			if (beacon_num < 6){
				uint8_t index = beacon_indexes[beacon_num];
				beacons[beacon_num][index] = beacon;

				if (++beacon_indexes[beacon_num] >= 100){
					beacon_indexes[beacon_num] = 0;
				}
			}
		}

	}

	last_angle = angle;
}

float norm(sVector3_t a, sVector3_t b){
	return sqrt((a.vector[0] - b.vector[0])*(a.vector[0] - b.vector[0]) + (a.vector[1] - b.vector[1])*(a.vector[1] - b.vector[1]));
}

float triangulationPierlot(sVector3_t *new_robot, sVector3_t beacon1, sVector3_t beacon2, sVector3_t beacon3){
	// Three beacon cotangents
	float cot_12 = Cot( beacon2.vector[2] - beacon1.vector[2]) ; // Changed for CW
	float cot_23 = Cot( beacon3.vector[2] - beacon2.vector[2] ) ; // Changed for CW

	// In practice, we have to avoid Inf or NaN values in the floating point computations.
	// Limiting the cot(.) value to a minimumr maximum value, corresponding to a small angle that is far below the measurement precision.
	cot_12 = adjust_value_to_bounds( cot_12 , COT_MAX ) ;
	cot_23 = adjust_value_to_bounds( cot_23 , COT_MAX ) ;
	float cot_31 = ( 1.0 - cot_12 * cot_23 ) / ( cot_12 + cot_23 ) ;
	cot_31 = adjust_value_to_bounds( cot_31 , COT_MAX ) ;

	// Modified beacon coordinates
	float x1_ = beacon1.vector[0] - beacon2.vector[0] , y1_ = beacon1.vector[1] - beacon2.vector[1] , x3_ = beacon3.vector[0] - beacon2.vector[0] , y3_ = beacon3.vector[1] - beacon2.vector[1] ;

	// Modified circle center coordinates
	float c12x = x1_ + cot_12 * y1_ ;  // +
	float c12y = y1_ - cot_12 * x1_ ;  // -

	float c23x = x3_ - cot_23 * y3_ ;  // -
	float c23y = y3_ + cot_23 * x3_ ;  // +

	float c31x = (x3_ + x1_) + cot_31 * (y3_ - y1_) ;  // +
	float c31y = (y3_ + y1_) - cot_31 * (x3_ - x1_) ;  // -

	float k31 = (x3_ * x1_) + (y3_ * y1_) + cot_31 * ( (y3_ * x1_) - (x3_ * y1_) ) ;

	// The denominator D is equal to 0 when the circle centers are collinear or coincide.
	// For noncollinear beacons, this situation occurs when the beacons and the robot are concyclic; they all stand on the same circle.
	float D = (c12x - c23x) * (c23y - c31y) - (c23x - c31x) * (c12y - c23y) ;
	float invD = 1.0 / D ;
	float K = k31 * invD ;

	// New robot position based only on beacon coordinates and LIDAR angles
	new_robot->vector[0] = K * (c12y - c23y) + beacon2.vector[0] ;
	new_robot->vector[1] = K * (c23x - c12x) + beacon2.vector[1] ;

	// Finally, it should be noted that the robot orientation θ_R may be determined by using any beacon B_i and its corresponding angle φ_i , once the robot position is known:
	// θ_R = atan2(y i − y R , xi − xR ) − φ i
	new_robot->vector[2] = atan2(beacon2.vector[1]-new_robot->vector[1], beacon2.vector[0]-new_robot->vector[0]) - beacon2.vector[2];

	if (new_robot->vector[2] > M_PI){
		new_robot->vector[2] -= 2*M_PI;
	} else if (new_robot->vector[2] < -M_PI){
		new_robot->vector[2] += 2*M_PI;
	}

	return invD ; // 1/|D| is a good approximation of the position error.
}

uint16_t Segment_PC(sVector3_t* pc, uint16_t ind, uint16_t radius){
	// Segment point cloud into groups of points
	uint8_t i=0, j=0, k=0;
	float sum_x=0, sum_y=0, sum_z=0;

	while(i < ind-1){
		sum_x = pc[i].vector[0];
		sum_y = pc[i].vector[1];
		sum_z = pc[i].vector[2];

		for(j=i+1, k=0; j<ind; ++j){
			pc[j-k] = pc[j];
			if (norm(pc[i], pc[j-k]) <= radius){
				sum_x += pc[j-k].vector[0];
				sum_y += pc[j-k].vector[1];
				sum_z += pc[j-k].vector[2];

				++k;
			}
		}

		pc[i].vector[0] = sum_x/(k+1);
		pc[i].vector[1] = sum_y/(k+1);
		pc[i].vector[2] = sum_z/(k+1);

		ind -= k;
		++i;
	}

	return ind;
}

uint8_t Choose_Beacon(sVector3_t* position, sVector3_t* point){
	float angle = position->vector[2];
	point->vector[2] = (360 - angle) * M_PI/180.0; // transform to coutner clockwise direction

	// upper left beacon
	if ((position->vector[0] > -240 && position->vector[0] <= 60) &&
	    (position->vector[1] > 1800 && position->vector[1] <= 2100)){
		point->vector[0] = -90;
		point->vector[1] = 1950;
		return 0;
	} // middle left beacon
	else if ((position->vector[0] > -240 && position->vector[0] <= 60) &&
			 (position->vector[1] > 850  && position->vector[1] <= 1150)){
		point->vector[0] = -90;
		point->vector[1] = 1000;
		return 1;
	} // lower left beacon
	else if ((position->vector[0] > -240 && position->vector[0] <= 60) &&
			 (position->vector[1] > -100 && position->vector[1] <= 200)){
		point->vector[0] = -90;
		point->vector[1] = 50;
		return 2;
	} // upper right beacon
	else if ((position->vector[0] > 2940 && position->vector[0] <= 3240) &&
			 (position->vector[1] > 1800 && position->vector[1] <= 2100)){
		point->vector[0] = 3090;
		point->vector[1] = 1950;
		return 3;
	} // middle right beacon
	else if ((position->vector[0] > 2940 && position->vector[0] <= 3240) &&
			  (position->vector[1] > 850 && position->vector[1] <= 1150)){
		point->vector[0] = 3090;
		point->vector[1] = 1000;
		return 4;
	} // lower right beacon
	else if ((position->vector[0] > 2940 && position->vector[0] <= 3240) &&
			  (position->vector[1] > -100 && position->vector[1] <= 200)){
		point->vector[0] = 3090;
		point->vector[1] = 50;
		return 5;
	}

	// Point does not belong to any beacon
	return 6;
}

void Get_Beacons(){
	memset(new_robot.vector, 0, sizeof(new_robot.vector));

	if (color == 'y'){
		for (uint8_t i=1; i<beacon_indexes[0]; ++i)
			beacons[0][0].vector[2] += beacons[0][i].vector[2];

		beacons[0][0].vector[2] /= beacon_indexes[0];

		for (uint8_t i=1; i<beacon_indexes[2]; ++i)
			beacons[2][0].vector[2] += beacons[2][i].vector[2];

		beacons[2][0].vector[2] /= beacon_indexes[2];

		for (uint8_t i=1; i<beacon_indexes[4]; ++i)
			beacons[4][0].vector[2] += beacons[4][i].vector[2];

		beacons[4][0].vector[2] /= beacon_indexes[4];

		triangulationPierlot(&new_robot, beacons[0][0], beacons[2][0], beacons[4][0]);
	} else if (color == 'b'){
		for (uint8_t i=1; i<beacon_indexes[1]; ++i)
			beacons[1][0].vector[2] += beacons[1][i].vector[2];

		beacons[1][0].vector[2] /= beacon_indexes[1];

		for (uint8_t i=1; i<beacon_indexes[3]; ++i)
			beacons[3][0].vector[2] += beacons[3][i].vector[2];

		beacons[3][0].vector[2] /= beacon_indexes[3];

		for (uint8_t i=1; i<beacon_indexes[5]; ++i)
			beacons[5][0].vector[2] += beacons[5][i].vector[2];

		beacons[5][0].vector[2] /= beacon_indexes[5];

		triangulationPierlot(&new_robot, beacons[1][0], beacons[3][0], beacons[5][0]);
	}

	convert_x.f = new_robot.vector[0];
	convert_y.f = new_robot.vector[1];
	convert_t.f = new_robot.vector[2];

	uint8_t bytes[13] = {0};
	for (uint8_t i = 0; i < 4; ++i)
	{
		bytes[i] = convert_x.u[i];
		bytes[i+4] = convert_y.u[i];
		bytes[i+8] = convert_t.u[i];
	}

	// TODO check how often lidar odometry is published
//	FDCAN_Send_Data(0x4FE, FDCAN_DLC_BYTES_16, 13, bytes);

	for (uint8_t i=0; i<6; ++i){
		beacon_indexes[i] = 0;
		memset(beacons[i], 0, sizeof(beacons[i]));
	}

	// Two beacons
//	else if (b_pc_index > 1){
//		// Correction of lidar angle
//		float fi1 = beacon_pc[0].vector[2] + self.theta;
//		float fi2 = beacon_pc[1].vector[2] + self.theta;
//
//		fi1 -= (fi1 > 6.28) ? 6.28 : 0;
//		fi2 -= (fi2 > 6.28) ? 6.28 : 0;
//
//		float tan_fi1 = tan(fi1), tan_fi2 = tan(fi2);
//
//		float x1 = beacon_pc[0].vector[0], y1 = beacon_pc[0].vector[1];
//		float x2 = beacon_pc[1].vector[0], y2 = beacon_pc[1].vector[1];
//
//		new_robot.vector[0] = (tan_fi1*x1 - tan_fi2*x2 - (y1-y2))/(tan_fi1 - tan_fi2);
//		new_robot.vector[1] = tan_fi1*(new_robot.vector[0] - x1) + y1;
//		new_robot.vector[2] = self.theta;
//
//		// Take into account only if the new position is inside of table
//		if ((new_robot.vector[0] <= 2900 && new_robot.vector[0] >= 100) &&
//			(new_robot.vector[1] <= 1900 && new_robot.vector[1] >= 100)){
//			reliability = 1;
//		}
//	}

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

//	FDCAN_Send_Data(0x6CE, FDCAN_DLC_BYTES_16, 16, bytes);
}

