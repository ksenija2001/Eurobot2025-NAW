/*
 * ism330dhcx_utils.h
 *
 *  Created on: Feb 4, 2025
 *      Author: filip
 */

#ifndef INC_ISM330DHCX_UTILS_H_
#define INC_ISM330DHCX_UTILS_H_

#if defined(STM32F407xx) || defined(STM32F401xE)
	#include "stm32f4xx_hal.h"
#elif defined(STM32G431xx) || defined(STM32G441xx)
	#include "stm32g4xx_hal.h"
#else

#endif

#include <stdint.h>
#include "ism330dhcx_settings.h"

typedef struct Raw{
	int16_t x;
	int16_t y;
	int16_t z;
}Raw;

typedef struct Acceleration{
	float x;
	float y;
	float z;
}Acceleration;
typedef struct Accelerometer{
	Raw raw;
	Acceleration acceleration;
} Accelerometer;

typedef int16_t Raw_Temp;
typedef float Degrees;
typedef struct Temperature{
	Raw_Temp raw;
	Degrees degrees;
} Temperature;

typedef struct Angle{
	float x;
	float y;
	float z;
}Angle;
typedef struct Gyroscope{
	Raw   raw;
	Angle angle;
}Gyroscope;

typedef struct ISM330DHCX_Data{
	Accelerometer accelerometer;
	Temperature   temperature;
	Gyroscope     gyroscope;
} ISM330DHCX_Data;

typedef struct {
	uint8_t initialized;

	uint8_t whoami;
	uint8_t address;

	I2C_HandleTypeDef* i2c;
	uint8_t user_convert;

	ISM330DHCX_Status lastStatus;
	ISM330DHCX_Data data;
} ISM330DHCX;

ISM330DHCX_Status writeReg(ISM330DHCX *ism, uint8_t reg, uint8_t *data);
ISM330DHCX_Status writeRegs(ISM330DHCX *ism, uint8_t reg, uint8_t *data, uint8_t size);

ISM330DHCX_Status readReg(ISM330DHCX *ism, uint8_t reg, uint8_t *data);
ISM330DHCX_Status readRegs(ISM330DHCX *ism, uint8_t reg, uint8_t *data, uint8_t size);

ISM330DHCX_Status setReg(ISM330DHCX *ism, uint8_t reg, uint8_t bits, uint8_t mask, ISM330DHCX_Status error);

#endif /* INC_ISM330DHCX_UTILS_H_ */
