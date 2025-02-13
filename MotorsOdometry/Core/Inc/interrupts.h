/*
 * interrupts.h
 *
 *  Created on: Jan 20, 2025
 *      Author: xenia
 */

#ifndef INC_INTERRUPTS_H_
#define INC_INTERRUPTS_H_

#include "main.h"
#include "stm32g431xx.h"
#include "odom.h"
#include "fdcan.h"
#include "utils.h"
#include "synthesis.h"

#define ODOM_TIME 1 // ms
#define RPM_TIME  1 // ms
#define SYNTHESIS_REGEN_TIME 100 //ms
#define SYNTHESIS_TIME 10 //ms

void TIM6_IT(void);
void Float2Bytes(uint8_t* buffer, uint8_t start, float data);



#endif /* INC_INTERRUPTS_H_ */
