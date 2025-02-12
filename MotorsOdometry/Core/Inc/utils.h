/*
 * utils.h
 *
 *  Created on: Jan 31, 2025
 *      Author: xenia
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#include "stm32g4xx.h"

void Float2Bytes(uint8_t *buffer, uint8_t start, float data);
float Bytes2Float(uint8_t msg[], uint8_t start);
int16_t Bytes2Int32(uint8_t msg[], uint8_t start);
void Int162Bytes(uint8_t *buffer, uint8_t start, int16_t data);

#endif /* INC_UTILS_H_ */
