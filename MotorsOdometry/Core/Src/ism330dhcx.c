/*
 * ism330dhcx.h
 *
 *  Created on: Feb 4, 2025
 *      Author: filip
 */

#include "ism330dhcx.h"

Angle (*user_convert_Raw_Gyroscope)(ISM330DHCX* ism, float ms) = NULL;
Acceleration (*user_convert_Raw_Accelerometer)(ISM330DHCX* ism, float ms) = NULL;

ISM330DHCX_Status init_ISM330DHCX(ISM330DHCX *ism, uint8_t address, I2C_HandleTypeDef *i2c){
	uint8_t reg_data;

	ism->initialized = 0;
	ism->user_convert = 0;

	ism->address = address;
	ism->i2c = i2c;

	// Read whoami
	if(readReg(ism, ISM_REG_WHOAMI, &(ism->whoami)) != ISM_OK){
		ism->lastStatus = ISM_ERROR_WHOAMI;
		return ism->lastStatus;
	}

	if(ism->whoami != ISM_REG_DATA_WHOAMI){
		ism->lastStatus = ISM_ERROR_WHOAMI;
		return ism->lastStatus;
	}

	// Software (SW) reset
	if(readReg(ism, ISM_REG_CTRL3_C, &reg_data) != ISM_OK){
		ism->lastStatus = ISM_ERROR_SW;
		return ism->lastStatus;
	}

	reg_data |= 0x01;
	if(writeReg(ism, ISM_REG_CTRL3_C, &reg_data) != ISM_OK){
		ism->lastStatus = ISM_ERROR_SW;
		return ism->lastStatus;
	}

	HAL_Delay(20);

	// Device configuration bit
	if(setReg(ism, ISM_REG_CTRL9_XL, 0x02, 0x00, ISM_ERROR_CONFIG_BIT) != ISM_OK){
		return ism->lastStatus;
	}

	// Block data update (BDU) config
	if(setReg(ism, ISM_REG_CTRL3_C, 0x40, 0x00, ISM_ERROR_BDU) != ISM_OK){
		return ism->lastStatus;
	}

	ism->initialized = 1;
	return ism->lastStatus;
}

ISM330DHCX_Status set_Fullscale_Gyroscope(ISM330DHCX *ism, ISM330DHCX_FS_GYROSCOPE fs){
	setReg(ism, ISM_REG_CTRL2_G,  fs, 0x00, ISM_ERROR_FS_GYRO);

	return ism->lastStatus;
}
ISM330DHCX_Status set_Fullscale_Accelerometer(ISM330DHCX *ism, ISM330DHCX_FS_ACCELEROMETER fs){
	setReg(ism, ISM_REG_CTRL1_XL, fs << 2, 0x00, ISM_ERROR_FS_ACC);

	return ism->lastStatus ;
}

ISM330DHCX_Status set_OutputDataRate_Gyroscope(ISM330DHCX *ism, ISM330DHCX_ODR odr){
	setReg(ism, ISM_REG_CTRL2_G,  odr << 4, 0xF0, ISM_ERROR_ODR_GYRO);

	return ism->lastStatus;
}
ISM330DHCX_Status set_OutputDataRate_Accelerometer(ISM330DHCX *ism, ISM330DHCX_ODR odr){
	setReg(ism, ISM_REG_CTRL1_XL, odr << 4, 0xF0, ISM_ERROR_ODR_ACC);

	return ism->lastStatus;
}

ISM330DHCX_Status get_Axies_Raw_All(ISM330DHCX *ism){
	uint8_t buffer[12];
	//readReg
	if(readRegs(ism, ISM_REG_OUT, buffer, 12) != ISM_OK){
		return ism->lastStatus;
	}

	ism->data.gyroscope.raw.x = ((uint16_t) buffer[0x1]) << 8 | buffer[0x0];
	ism->data.gyroscope.raw.y = ((uint16_t) buffer[0x3]) << 8 | buffer[0x2];
	ism->data.gyroscope.raw.z = ((uint16_t) buffer[0x5]) << 8 | buffer[0x4];

	ism->data.accelerometer.raw.x = ((uint16_t) buffer[0x7]) << 8 | buffer[0x6];
	ism->data.accelerometer.raw.y = ((uint16_t) buffer[0x9]) << 8 | buffer[0x8];
	ism->data.accelerometer.raw.z = ((uint16_t) buffer[0xB]) << 8 | buffer[0xA];

	return ism->lastStatus;
}
ISM330DHCX_Status get_Axies_Raw_Gyroscope(ISM330DHCX *ism){
	uint8_t buffer[6];

	// readReg
	if(readRegs(ism, ISM_REG_OUT_GYRO, buffer, 6) != ISM_OK){
		return ism->lastStatus;
	}

	ism->data.gyroscope.raw.x = ((uint16_t) buffer[0x1]) << 8 | buffer[0x0];
	ism->data.gyroscope.raw.y = ((uint16_t) buffer[0x3]) << 8 | buffer[0x2];
	ism->data.gyroscope.raw.z = ((uint16_t) buffer[0x5]) << 8 | buffer[0x4];

	return ism->lastStatus;
}
ISM330DHCX_Status get_Axies_Raw_Accelerometer(ISM330DHCX *ism){
	uint8_t buffer[6];

	// readReg
	if(readRegs(ism, ISM_REG_OUT_ACC, buffer, 6) != ISM_OK){
		return ism->lastStatus;
	}

	ism->data.accelerometer.raw.x = ((uint16_t) buffer[0x1]) << 8 | buffer[0x0];
	ism->data.accelerometer.raw.y = ((uint16_t) buffer[0x3]) << 8 | buffer[0x2];
	ism->data.accelerometer.raw.z = ((uint16_t) buffer[0x5]) << 8 | buffer[0x4];

	return ism->lastStatus;
}

ISM330DHCX_Status get_Axies_All(ISM330DHCX *ism, float ms){

	get_Axies_Raw_All(ism);

	if(ism->lastStatus == ISM_OK){
		convert_Raw(ism, ms);
	}

	return ISM_OK;
}

ISM330DHCX_Status get_Axies_Gyroscope(ISM330DHCX *ism, float ms){
	// readReg
	return ISM_OK;
}
ISM330DHCX_Status get_Axies_Accelerometer(ISM330DHCX *ism, float ms){
	// readReg
	return ISM_OK;
}

ISM330DHCX_Status convert_Raw(ISM330DHCX* ism, float ms){
	Angle angle;
	Acceleration acceleration;

	if(ism->user_convert && user_convert_Raw_Gyroscope != NULL && user_convert_Raw_Accelerometer != NULL){
		angle = user_convert_Raw_Gyroscope(ism, ms);
		acceleration = user_convert_Raw_Accelerometer(ism, ms);
	}else{
		angle = convert_Raw_Gyroscope(ism, ms);
		acceleration= convert_Raw_Accelerometer(ism, ms);
	}

	ism->data.gyroscope.angle.x += angle.x;
	ism->data.gyroscope.angle.y += angle.y;
	ism->data.gyroscope.angle.z += angle.z;

	ism->data.accelerometer.acceleration.x = acceleration.x;
	ism->data.accelerometer.acceleration.y = acceleration.y;
	ism->data.accelerometer.acceleration.z = acceleration.z;

	return ISM_OK;
}

Acceleration convert_Raw_Accelerometer(ISM330DHCX* ism, float ms){
	float x = (float) ism->data.accelerometer.raw.x * ISM_FS_ACC_2G_SENSITIVITY;
	float y = (float) ism->data.accelerometer.raw.y * ISM_FS_ACC_2G_SENSITIVITY;
	float z = (float) ism->data.accelerometer.raw.z * ISM_FS_ACC_2G_SENSITIVITY;

	return (Acceleration) {x, y, z};
}

float getFABS(float val){
	if(val > 0) return val;

	return val * (-1);
}

Angle convert_Raw_Gyroscope(ISM330DHCX* ism, float ms){
	float x_angle = ((float) ism->data.gyroscope.raw.x) * ISM_FS_GYRO_250_SENSITIVITY * ms / 1000.0;
	float y_angle = ((float) ism->data.gyroscope.raw.y) * ISM_FS_GYRO_250_SENSITIVITY * ms / 1000.0;
	float z_angle = ((float) ism->data.gyroscope.raw.z) * ISM_FS_GYRO_250_SENSITIVITY * ms / 1000.0;

	float min_angle = ISM_ZERO_RATE_LEVEL_GYRO * ms;

	if(getFABS(x_angle) <= min_angle ) x_angle = 0.0;
	if(getFABS(y_angle) <= min_angle ) y_angle = 0.0;
	if(getFABS(z_angle) <= min_angle ) z_angle = 0.0;

	return (Angle) {x_angle, y_angle, z_angle};
}


