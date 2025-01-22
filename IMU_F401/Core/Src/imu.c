/*
 * imu.c
 *
 *  Created on: Dec 18, 2024
 *  Modified on: Jan 08, 2025
 *       Author: Filip Goldberger
 */

#include "imu.h"

int16_t IMU_RAW_DATA_ACC_X = 0;
int16_t IMU_RAW_DATA_ACC_Y = 0;
int16_t IMU_RAW_DATA_ACC_Z = 0;

float IMU_DATA_ACC_X 	 = 0;
float IMU_DATA_ACC_Y 	 = 0;
float IMU_DATA_ACC_Z 	 = 0;

/** --------------------- **/

int16_t IMU_RAW_DATA_GYR_X = 0;
int16_t IMU_RAW_DATA_GYR_Y = 0;
int16_t IMU_RAW_DATA_GYR_Z = 0;

float IMU_DATA_GYR_X 	 = 0;
float IMU_DATA_GYR_Y	 = 0;
float IMU_DATA_GYR_Z	 = 0;

/** --------------------- **/
/** DMA settings and data **/

uint8_t IMU_DMA_TX_DATA=0;
uint8_t IMU_DMA_RX_DATA[IMU_REG_SIZE_ACC_AND_GYR]={0};

/** --------------------- **/

HAL_StatusTypeDef status = HAL_ERROR;
uint8_t read_buffer[IMU_REG_SIZE_ACC_AND_GYR];

HAL_StatusTypeDef IMU_Init(IMU* imu, uint8_t dev_addr, I2C_HandleTypeDef* comm_line){
	imu->addr = dev_addr;
	imu->line = comm_line;
	imu->whoami = 0x00;
	imu->initialized = false;

	imu->offset_acc_x = 0;
	imu->offset_acc_y = 0;
	imu->offset_acc_z = 0;

	HAL_Delay(50);
	uint8_t tries = 3;
	while(tries){
		status = HAL_I2C_Mem_Read(imu->line, imu->addr, IMU_WHO_AM_I_REG, 1, &(imu->whoami), 1, HAL_MAX_DELAY);

		if(imu->whoami == 0x00) {
			__IMU_RESET(imu);
		}
		else break;

		HAL_Delay(50);
		tries--;
	}

	if(status != HAL_OK) return status;
	if(imu->whoami != IMU_WHO_AM_I) return HAL_ERROR;

	// Set bit for proper device configuration
	uint8_t imu_conf = 0xE2;
	status = HAL_I2C_Mem_Write(imu->line, imu->addr, IMU_CTRL9_XL_REG, 1, &imu_conf, 1, HAL_MAX_DELAY);
	if(status != HAL_OK) return status;

	// Setting accelerometer register and mode
	uint8_t imu_acc = ((uint8_t) IMU_ACC_ODR_CONFIG) << 4 | ((uint8_t) IMU_ACC_FS_CONFIG) << 2  | 0b00;
	status = HAL_I2C_Mem_Write(imu->line, imu->addr, IMU_CTRL1_XL_REG, 1, &imu_acc, 1, HAL_MAX_DELAY);
	if(status != HAL_OK) return status;

	// Setting gyroscope register and mode
	uint8_t imu_gyr = ((uint8_t) IMU_GYR_ODR_CONFIG) << 4 | ((uint8_t) IMU_GYR_FS_CONFIG) << 2  | 0b00;
	status = HAL_I2C_Mem_Write(imu->line, imu->addr, IMU_CTRL2_G_REG, 1, &imu_gyr, 1, HAL_MAX_DELAY);
	if(status != HAL_OK) return status;

	// Calibrating sensor
	imu->initialized = true;
	__IMU_Calibrate(imu);

	return HAL_OK;
}

HAL_StatusTypeDef __IMU_RESET(IMU *imu){
	// Powering off accelerometer and gyrscope before reset
	uint8_t data = 0x00;
	status = HAL_I2C_Mem_Write(imu->line, imu->addr, IMU_CTRL1_XL_REG, 1, &data, 1, HAL_MAX_DELAY);
	status = HAL_I2C_Mem_Write(imu->line, imu->addr, IMU_CTRL2_G_REG, 1, &data, 1, HAL_MAX_DELAY);

	// Software reset the sensor
	data = 0x05;
	status = HAL_I2C_Mem_Write(imu->line, imu->addr, IMU_CTRL3_C_REG, 1, &data, 1, HAL_MAX_DELAY);

	return HAL_OK;
}

HAL_StatusTypeDef __IMU_Calibrate(IMU *imu){
	int32_t p_offset_acc_x = 0;
	int32_t p_offset_acc_y = 0;
	int32_t p_offset_acc_z = 0;

	int32_t iterations = 1000;
	for(int i = 0; i < iterations; i++){
		//HAL_Delay(10);
		if(IMU_Read_ACC(imu) == HAL_ERROR){
			return HAL_ERROR;
		}

		p_offset_acc_x += IMU_RAW_DATA_ACC_X;
		p_offset_acc_y += IMU_RAW_DATA_ACC_Y;
		p_offset_acc_z += IMU_RAW_DATA_ACC_Z;
	}
	p_offset_acc_x /= iterations;
	p_offset_acc_y /= iterations;
	p_offset_acc_z /= iterations;

	imu->offset_acc_x = p_offset_acc_x;
	imu->offset_acc_y = p_offset_acc_y;
	imu->offset_acc_z = p_offset_acc_z;

	return HAL_OK;
}

// Substracting offset from raw data
HAL_StatusTypeDef IMU_OFFSET_SUBTRACT(IMU *imu){
	IMU_RAW_DATA_ACC_X -= imu->offset_acc_x;
	IMU_RAW_DATA_ACC_Y -= imu->offset_acc_y;
	IMU_RAW_DATA_ACC_Z -= imu->offset_acc_z;
	return HAL_OK;
}

// Reading accelerometer and gyroscope registers
HAL_StatusTypeDef IMU_Read_DATA(IMU* imu){
	if(imu->initialized == false) return HAL_ERROR;
	status = HAL_I2C_Mem_Read(imu->line, imu->addr, IMU_DATA_GYR_ACC, 1, read_buffer, 12, HAL_MAX_DELAY);

	if(status == HAL_ERROR) return status;

	IMU_RAW_DATA_GYR_X = (int16_t) ((uint16_t) read_buffer[0x1] ) << 8 | ((uint16_t) read_buffer[0x0]) << 0;
	IMU_RAW_DATA_GYR_Y = (int16_t) ((uint16_t) read_buffer[0x3] ) << 8 | ((uint16_t) read_buffer[0x2]) << 0;
	IMU_RAW_DATA_GYR_Z = (int16_t) ((uint16_t) read_buffer[0x5] ) << 8 | ((uint16_t) read_buffer[0x4]) << 0;

	IMU_RAW_DATA_ACC_X = (int16_t) ((uint16_t) read_buffer[0x7] ) << 8 | ((uint16_t) read_buffer[0x6]) << 0;
	IMU_RAW_DATA_ACC_Y = (int16_t) ((uint16_t) read_buffer[0x9] ) << 8 | ((uint16_t) read_buffer[0x8]) << 0;
	IMU_RAW_DATA_ACC_Z = (int16_t) ((uint16_t) read_buffer[0xB] ) << 8 | ((uint16_t) read_buffer[0xA]) << 0;

	IMU_OFFSET_SUBTRACT(imu);

	IMU_DATA_ACC_X = __IMU_Convert_ACC(IMU_RAW_DATA_ACC_X);
	IMU_DATA_ACC_Y = __IMU_Convert_ACC(IMU_RAW_DATA_ACC_Y);
	IMU_DATA_ACC_Z = __IMU_Convert_ACC(IMU_RAW_DATA_ACC_Z);

	return status;
}

// Reading accelerometer registers
HAL_StatusTypeDef IMU_Read_ACC(IMU* imu){
	if(imu->initialized == false) return HAL_ERROR;
	status = HAL_I2C_Mem_Read(imu->line, imu->addr, IMU_ACC_OUT, 1, read_buffer, 6, HAL_MAX_DELAY);

	if(status == HAL_ERROR) return status;

	IMU_RAW_DATA_ACC_X = (int16_t) ((uint16_t) read_buffer[0x1] ) << 8 | ((uint16_t) read_buffer[0x0]) << 0;
	IMU_RAW_DATA_ACC_Y = (int16_t) ((uint16_t) read_buffer[0x3] ) << 8 | ((uint16_t) read_buffer[0x2]) << 0;
	IMU_RAW_DATA_ACC_Z = (int16_t) ((uint16_t) read_buffer[0x5] ) << 8 | ((uint16_t) read_buffer[0x4]) << 0;

	IMU_OFFSET_SUBTRACT(imu);

	IMU_DATA_ACC_X = __IMU_Convert_ACC(IMU_RAW_DATA_ACC_X);
	IMU_DATA_ACC_Y = __IMU_Convert_ACC(IMU_RAW_DATA_ACC_Y);
	IMU_DATA_ACC_Z = __IMU_Convert_ACC(IMU_RAW_DATA_ACC_Z);

	return status;
}

HAL_StatusTypeDef IMU_Read_ACC_X(IMU* imu){
	if(imu->initialized == false) return HAL_ERROR;
	status = HAL_I2C_Mem_Read(imu->line, imu->addr, IMU_ACC_OUT_X, 1, read_buffer, 2, HAL_MAX_DELAY);

	if(status == HAL_ERROR) return status;

	IMU_RAW_DATA_ACC_X = (int16_t) ((uint16_t) read_buffer[1] ) << 8 | ((uint16_t) read_buffer[0]) << 0;
	IMU_DATA_ACC_X = __IMU_Convert_ACC(IMU_RAW_DATA_ACC_X);

	return status;
}

HAL_StatusTypeDef IMU_Read_ACC_Y(IMU* imu){
	if(imu->initialized == false) return HAL_ERROR;
	status = HAL_I2C_Mem_Read(imu->line, imu->addr, IMU_ACC_OUT_Y, 1, read_buffer, 2, HAL_MAX_DELAY);

	if(status == HAL_ERROR) return status;

	IMU_RAW_DATA_ACC_Y = (int16_t) ((uint16_t) read_buffer[1] ) << 8 | ((uint16_t) read_buffer[0]) << 0;
	IMU_DATA_ACC_Y = __IMU_Convert_ACC(IMU_RAW_DATA_ACC_Y);

	return status;
}

HAL_StatusTypeDef IMU_Read_ACC_Z(IMU* imu){
	if(imu->initialized == false) return HAL_ERROR;
	status = HAL_I2C_Mem_Read(imu->line, imu->addr, IMU_ACC_OUT_Z, 1, read_buffer, 2, HAL_MAX_DELAY);

	if(status == HAL_ERROR) return status;

	IMU_RAW_DATA_ACC_Z = (int16_t) ((uint16_t) read_buffer[1] ) << 8 | ((uint16_t) read_buffer[0]) << 0;
	IMU_DATA_ACC_Z = __IMU_Convert_ACC(IMU_RAW_DATA_ACC_Z);

	return status;
}

// Reading gyroscope registers
HAL_StatusTypeDef IMU_Read_GYR(IMU* imu){
	if(imu->initialized == false) return HAL_ERROR;
	status = HAL_I2C_Mem_Read(imu->line, imu->addr, IMU_GYR_OUT, 1, read_buffer, 6, HAL_MAX_DELAY);

	if(status == HAL_ERROR) return status;

	IMU_RAW_DATA_GYR_X = (int16_t) ((uint16_t) read_buffer[0x1] ) << 8 | ((uint16_t) read_buffer[0x0]) << 0;
	IMU_RAW_DATA_GYR_Y = (int16_t) ((uint16_t) read_buffer[0x3] ) << 8 | ((uint16_t) read_buffer[0x2]) << 0;
	IMU_RAW_DATA_GYR_Z = (int16_t) ((uint16_t) read_buffer[0x5] ) << 8 | ((uint16_t) read_buffer[0x4]) << 0;

	//IMU_DATA_ACC_X = __IMU_Convert_ACC(IMU_RAW_DATA_ACC_X);
	//IMU_DATA_ACC_Y = __IMU_Convert_ACC(IMU_RAW_DATA_ACC_Y);
	//IMU_DATA_ACC_Z = __IMU_Convert_ACC(IMU_RAW_DATA_ACC_Z);

	return status;
}

HAL_StatusTypeDef IMU_Read_GYR_X(IMU* imu){
	if(imu->initialized == false) return HAL_ERROR;
	status = HAL_I2C_Mem_Read(imu->line, imu->addr, IMU_GYR_OUT_X, 1, read_buffer, 2, HAL_MAX_DELAY);

	if(status == HAL_ERROR) return status;

	IMU_RAW_DATA_GYR_X = (int16_t) ((uint16_t) read_buffer[1] ) << 8 | ((uint16_t) read_buffer[0]) << 0;

	return status;
}

HAL_StatusTypeDef IMU_Read_GYR_Y(IMU* imu){
	if(imu->initialized == false) return HAL_ERROR;
	status = HAL_I2C_Mem_Read(imu->line, imu->addr, IMU_GYR_OUT_Y, 1, read_buffer, 2, HAL_MAX_DELAY);

	if(status == HAL_ERROR) return status;

	IMU_RAW_DATA_GYR_Y = (int16_t) ((uint16_t) read_buffer[1] ) << 8 | ((uint16_t) read_buffer[0]) << 0;

	return status;
}

HAL_StatusTypeDef IMU_Read_GYR_Z(IMU* imu){
	if(imu->initialized == false) return HAL_ERROR;
	status = HAL_I2C_Mem_Read(imu->line, imu->addr, IMU_GYR_OUT_Z, 1, read_buffer, 2, HAL_MAX_DELAY);

	if(status == HAL_ERROR) return status;

	IMU_RAW_DATA_GYR_Z = (int16_t) ((uint16_t) read_buffer[1] ) << 8 | ((uint16_t) read_buffer[0]) << 0;

	return status;
}

// Cycling I2C SCL line in order to reset sensor state
HAL_StatusTypeDef __IMU_RESET_CLOCK_LINE(){
	__IMU_INIT_GPIO();

	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == 1) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
		//__IMU_INIT_I2C();
		return HAL_OK;
	}
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

	for(int i=0; i < IMU_CLOCK_RESET_COUNT; i++){
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6);
		HAL_Delay(1);
	}
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

	//__IMU_INIT_I2C();
	return HAL_OK;
}

// Generate 1 pulse on SCL line
HAL_StatusTypeDef __IMU_GENERATE_CLOCK_PULSE(){
	__IMU_INIT_GPIO();
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);

	//__IMU_INIT_I2C();
	return HAL_OK;
}

// Initializing STM32 GPIO port B to output mode
HAL_StatusTypeDef __IMU_INIT_GPIO(){
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitStruct.Pin = GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	return HAL_OK;
}

// Initializing STM32 I2C mode on port B
HAL_StatusTypeDef __IMU_INIT_I2C(){
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 400000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}

	return HAL_OK;
}

// Converting sensors raw data from accelerometer to G force
float __IMU_Convert_ACC(int16_t data){
	return  ((float) data) * ( ((float) (IMU_ACC_CONVERSION - 0)) / ((float) (32767 - (-32768))) );
}

float __IMU_Convert_GYR(int16_t data){
	return 0;
}
