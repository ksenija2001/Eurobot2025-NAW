/*
 * fdcan.h
 *
 *  Created on: Jan 20, 2025
 *      Author: xenia
 */

#ifndef INC_FDCAN_H_
#define INC_FDCAN_H_

#include "main.h"
#include "odom.h"

uint8_t FDCAN_Init(FDCAN_HandleTypeDef *hfdcan);
uint8_t FDCAN_Send_Data(uint32_t id, uint32_t dlc, uint8_t* data);
float Bytes2Float(uint8_t msg[], uint8_t start);

#endif /* INC_FDCAN_H_ */
