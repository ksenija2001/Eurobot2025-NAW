/*
 * ax_servo.h
 *
 *  Created on: Mar 10, 2025
 *      Author: xenia
 */

#ifndef INC_AX_SERVO_H_
#define INC_AX_SERVO_H_

#include "main.h"
#include "stm32g4xx_hal.h"
#include "fdcan.h"

#define SERVO_NUM 10

#define HEADER 0xFF

/* Instruction */
#define PING       0x01
#define READ       0x02
#define WRITE      0x03
#define REBOOT     0x08
#define SYNC_WRITE 0x83

/* Control table */
#define TORQUE 24
#define LED 25
#define GOAL_POSITION 30
#define MOVING_SPEED 32
#define PRESENT_POSITION 36
#define PRESENT_SPEED 38
#define PRESENT_LOAD 40
#define MOVING 46

/* Error mask */
#define INPUT_VOLTAGE_MASK 0x01
#define ANGLE_LIMIT_MASK   0x02
#define OVERHEATING_MASK   0x04
#define RANGE_MASK         0x08
#define CHECKSUM_MASK      0x10
#define OVERLOAD_MASK      0x20
#define INSTRUCTION_MASK   0x40

void TIM6_Moving_IT(TIM_HandleTypeDef* tim, UART_HandleTypeDef* huart);

void AX_Transmit(UART_HandleTypeDef* huart, uint8_t *tx_buffer, uint8_t tx_length, uint8_t rx_length);
uint8_t Checksum(uint8_t* buffer, uint8_t len);
void Enable_Torque(UART_HandleTypeDef* huart, uint8_t ID, uint8_t on_off);
void Enable_LED(UART_HandleTypeDef* huart, uint8_t ID, uint8_t on_off);
void Sync_Set_Goal_Position(UART_HandleTypeDef* huart, uint8_t* IDs, uint16_t* angles, uint8_t* speeds, uint8_t size);
void Get_Present_Position(UART_HandleTypeDef* huart, uint8_t ID);
void Get_Present_Speed(UART_HandleTypeDef* huart, uint8_t ID);
void Get_Moving_Status(UART_HandleTypeDef* huart, uint8_t ID);
void Set_Goal_Position(UART_HandleTypeDef* huart, uint8_t ID, uint16_t angle);
void Set_Moving_Speed(UART_HandleTypeDef* huart, uint8_t ID, uint8_t speed_percentage);


extern uint8_t rx_buffer[256];

#endif /* INC_AX_SERVO_H_ */
