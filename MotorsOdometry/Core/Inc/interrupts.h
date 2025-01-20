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

#define ODOM_TIME 1 // ms

void TIM6_IT(void);


#endif /* INC_INTERRUPTS_H_ */
