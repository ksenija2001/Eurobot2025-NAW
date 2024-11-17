/*
 * imu.h
 *
 *  Created on: Nov 8, 2024
 *      Author: Matija Kukobat
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"

//These variables should maybe be moved to the source file
uint16_t x_acc;
uint16_t y_acc;
uint16_t z_acc;

uint16_t x_gyro;
uint16_t y_gyro;
uint16_t z_gyro;

//*All functions are void for testing purposes, could be changed to have a return type if need be

//Initialization
void imu_init(I2C_HandleTypeDef hi2c);

//Acceleration data readings
void imu_read_x_acc(I2C_HandleTypeDef hi2c);
void imu_read_y_acc(I2C_HandleTypeDef hi2c);
void imu_read_z_acc(I2C_HandleTypeDef hi2c);
void imu_read_all_acc(I2C_HandleTypeDef hi2c);

//Gyroscope data readings
void imu_read_x_gyro(I2C_HandleTypeDef hi2c);
void imu_read_y_gyro(I2C_HandleTypeDef hi2c);
void imu_read_z_gyro(I2C_HandleTypeDef hi2c);
void imu_read_all_gyro(I2C_HandleTypeDef hi2c);

//Temperature measurement
void imu_get_temp(I2C_HandleTypeDef hi2c);

#endif /* INC_IMU_H_ */
