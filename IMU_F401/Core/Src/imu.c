/*
 * imu.c
 *
 *  Created on: Nov 8, 2024
 *      Author: Matija Kukobat
 */

#include "../Inc/imu.h"

int16_t temp_raw;
float temp_real;

void imu_init(I2C_HandleTypeDef hi2c)
{
	uint8_t init_acc = 0x60;
	uint8_t init_gyro = 0x60;
	HAL_I2C_Mem_Write(&hi2c, 0xd6, 0x10, 1, &init_acc, 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Write(&hi2c, 0xd6, 0x11, 1, &init_gyro, 1, HAL_MAX_DELAY);
}

void imu_read_x_acc(I2C_HandleTypeDef hi2c)
{
	HAL_I2C_Mem_Read(&hi2c, 0xd6, 0x28, 1, &x_acc, 2, HAL_MAX_DELAY);
}

void imu_read_y_acc(I2C_HandleTypeDef hi2c)
{
	HAL_I2C_Mem_Read(&hi2c, 0xd6, 0x2a, 1, &y_acc, 2, HAL_MAX_DELAY);
}

void imu_read_z_acc(I2C_HandleTypeDef hi2c)
{
	HAL_I2C_Mem_Read(&hi2c, 0xd6, 0x2c, 1, &z_acc, 2, HAL_MAX_DELAY);
}

void imu_read_all_acc(I2C_HandleTypeDef hi2c)
{
	imu_read_x_acc(hi2c);
	imu_read_y_acc(hi2c);
	imu_read_z_acc(hi2c);
}

void imu_read_x_gyro(I2C_HandleTypeDef hi2c)
{
	HAL_I2C_Mem_Read(&hi2c, 0xd6, 0x22, 1, &x_gyro, 2, HAL_MAX_DELAY);
}

void imu_read_y_gyro(I2C_HandleTypeDef hi2c)
{
	HAL_I2C_Mem_Read(&hi2c, 0xd6, 0x24, 1, &y_gyro, 2, HAL_MAX_DELAY);
}

void imu_read_z_gyro(I2C_HandleTypeDef hi2c)
{
	HAL_I2C_Mem_Read(&hi2c, 0xd6, 0x26, 1, &z_gyro, 2, HAL_MAX_DELAY);
}

void imu_read_all_gyro(I2C_HandleTypeDef hi2c)
{
	imu_read_x_gyro(hi2c);
	imu_read_y_gyro(hi2c);
	imu_read_z_gyro(hi2c);
}

void imu_get_temp(I2C_HandleTypeDef hi2c)
{
	HAL_I2C_Mem_Read(&hi2c, 0xd6, 0x20, 1, &temp_raw, 2, HAL_MAX_DELAY);
	temp_real = 25 + (temp_raw/16.0);
}
