/*
 * fdcan.h
 *
 *  Created on: Jan 20, 2025
 *      Author: xenia
 */

#ifndef INC_FDCAN_H_
#define INC_FDCAN_H_

#include "main.h"
//#include "odom.h"
//#include "motor_control.h"
#include "string.h"
#include "stdlib.h"
#include "utils.h"

uint8_t FDCAN_Init(FDCAN_HandleTypeDef *hfdcan);
uint8_t FDCAN_Send_Data(uint32_t id, uint32_t dlc, uint8_t size, uint8_t* data);

extern uint8_t send_status;
extern uint8_t receive_status;



#endif /* INC_FDCAN_H_ */
