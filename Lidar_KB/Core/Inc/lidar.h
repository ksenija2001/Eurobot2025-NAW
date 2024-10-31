
#ifndef INC_LIDAR_H_
#define INC_LIDAR_H_

#include <stdint.h>
#include "stm32g4xx_hal.h"

#define BUFFER_SIZE 128

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


void Lidar_Receive_Response(UART_HandleTypeDef *huart, uint8_t length);
void Lidar_Stop(UART_HandleTypeDef *huart);
void Lidar_Reset(UART_HandleTypeDef *huart);
void Lidar_Get_Health(UART_HandleTypeDef *huart);
void Lidar_Motor_Speed(UART_HandleTypeDef *huart, uint16_t rpm);
void Lidar_Get_Samplerate(UART_HandleTypeDef *huart);
void Lidar_Get_Lidar_Conf(UART_HandleTypeDef *huart);

#endif /* INC_LIDAR_H_ */
