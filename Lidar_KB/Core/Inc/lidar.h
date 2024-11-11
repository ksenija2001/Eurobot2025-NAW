
#ifndef INC_LIDAR_H_
#define INC_LIDAR_H_

#include <stdint.h>
#include <string.h>
#include <math.h>
#include "stm32g4xx_hal.h"

#define BUFFER_SIZE 128
#define PWM_ARR 5759
#define PWM_SENS 0.0735294117647
#define PWM_SENS_OFFSET 6.176470588235

// Request packet
#define START          0xA5
#define STOP           0x25
#define RESET          0x40
#define SCAN           0x20
#define EXPRESS_SCAN   0x82
#define FORCE_SCAN     0x21
#define GET_INFO       0x50
#define GET_HEALTH     0x52
#define GET_SAMPLERATE 0x59
#define GET_LIDAR_CONF 0x84
#define MOTOR_SPEED    0xA8

// Response packet
#define START2         0x5A

// Response descriptor struct
typedef struct {
	uint32_t length;
	uint8_t send_mode; // 0 - signel response, 1 - multiple response, 4 - all responses received
	uint8_t data_type;
	uint8_t packet_num;
} sDescriptor_t;

// Lidar hardware information
typedef struct{
	uint8_t model;
	uint8_t firmware_minor;
	uint8_t firmware_major;
	uint8_t hardware;
	uint16_t standard_samplerate;
	uint16_t express_samplerate;
} sInfo_t;

// Lidar scan mode informations
typedef struct{
	uint32_t sample_duration;
	uint32_t max_distance;
	uint8_t  answer_type;
	uint8_t  name[20];
} sScanMode_t;

// Lidar health data
typedef struct{
	uint8_t status;
	uint16_t error_code;
} sHealth_t;

typedef struct{
	uint8_t sync;          // identifies the start of a new response packet - 0xA5
	uint8_t checksum;      // XOR of all data bytes in response packet
	float_t start_angle;  // reference value for the angle data in current response packet
	uint8_t S;             // start flag of new scan
} sResponse_t;

typedef struct{
	float_t distance1;
	float_t distance2;
	float_t theta1;
	float_t theta2;
} sCabin_t;

// Commands without response
void Lidar_Stop(UART_HandleTypeDef *huart);
void Lidar_Reset(UART_HandleTypeDef *huart);
void Lidar_Unknown(UART_HandleTypeDef *huart);

// Commands with response
void Lidar_Get_Health(UART_HandleTypeDef *huart);
void Lidar_Get_Samplerate(UART_HandleTypeDef *huart);
void Lidar_Get_Lidar_Conf(UART_HandleTypeDef *huart, uint8_t config, uint8_t request_length, uint8_t mode);
void Lidar_Get_Info(UART_HandleTypeDef *huart);
void Lidar_Scan(UART_HandleTypeDef *huart);
void Lidar_Express_Scan(UART_HandleTypeDef *huart, uint8_t scan_mode_id);

// Helper functions
void Lidar_Motor_Stop(TIM_HandleTypeDef *tim, uint8_t channel);
void Lidar_Motor_Speed(TIM_HandleTypeDef *tim, uint8_t channel, uint16_t rpm);
uint8_t Lidar_CRC(uint8_t msg[], uint8_t length, uint8_t start);


#endif /* INC_LIDAR_H_ */
