/*
 * ism330dhcx_utils.c
 *
 *  Created on: Feb 4, 2025
 *      Author: filip
 */

#include "ism330dhcx_utils.h"

ISM330DHCX_Status writeReg(ISM330DHCX *ism, uint8_t reg, uint8_t *data){
	if(HAL_I2C_Mem_Write(ism->i2c, ism->address, reg, 1, data, 1, HAL_MAX_DELAY) == HAL_OK){
		return ISM_OK;
	}

	return ISM_ERROR_WRITE;
}
ISM330DHCX_Status writeRegs(ISM330DHCX *ism, uint8_t reg, uint8_t *data, uint8_t size){
	if(HAL_I2C_Mem_Write(ism->i2c, ism->address, reg, 1, data, size, HAL_MAX_DELAY) == HAL_OK){
		return ISM_OK;
	}

	return ISM_ERROR_WRITE;
}

ISM330DHCX_Status readReg(ISM330DHCX *ism, uint8_t reg, uint8_t *data){
	if(HAL_I2C_Mem_Read(ism->i2c, ism->address, reg, 1, data, 1, HAL_MAX_DELAY) == HAL_OK){
		return ISM_OK;
	}

	return ISM_ERROR_READ;
}

ISM330DHCX_Status readRegs(ISM330DHCX *ism, uint8_t reg, uint8_t *data, uint8_t size){
	if(HAL_I2C_Mem_Read(ism->i2c, ism->address, reg, 1, data, size, HAL_MAX_DELAY) == HAL_OK){
		return ISM_OK;
	}

	return ISM_ERROR_READ;
}

ISM330DHCX_Status setReg(ISM330DHCX *ism, uint8_t reg, uint8_t bits, uint8_t mask, ISM330DHCX_Status error){
	uint8_t buff;

	if(readReg(ism, reg, &buff) != ISM_OK){
		ism->lastStatus = ISM_ERROR_SET_REG;
		return ism->lastStatus;
	}

	if(mask != 0x00) buff &= ~mask;

	buff |= bits;
	if(writeReg(ism, reg, &buff) != ISM_OK){
		ism->lastStatus = ISM_ERROR_SET_REG;
		return ism->lastStatus;
	}

	if(readReg(ism, reg, &buff) != ISM_OK){
		ism->lastStatus = ISM_ERROR_SET_REG;
		return ism->lastStatus;
	}

	if((buff & bits) != bits){
		if(error == ISM_NONE) 	ism->lastStatus = ISM_ERROR_SET_REG;
		else					ism->lastStatus = error;

		return ism->lastStatus;
	}

	ism->lastStatus = ISM_OK;
	return ism->lastStatus;


}
