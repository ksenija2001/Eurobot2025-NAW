/*
 * imu.h
 *
 *  Created on: Dec 18, 2024
 *  Modified on: Jan 08, 2025
 *       Author: Filip Goldberger
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "types.h"

#include <stdint.h>
#include "stm32f4xx_hal.h"

#define IMU_ACC_ODR_416_HZ
#define IMU_ACC_FS_2G

#define IMU_GYR_ODR_416_HZ
#define IMU_GYR_FS_250

#include "imu_regs.h"
#define IMU_WHO_AM_I 0x6b

// Datasheet recommends to send 9 pulses to SCL when sensor is stuck in one state
// 9 pulses == 18 toggles of SCL lines
#define IMU_CLOCK_RESET_COUNT 9
#define IMU_CLOCK_RESET_COUNT_TOGGLES 2*IMU_CLOCK_RESET_COUNT

#define IMU_REG_SIZE_ACC_AND_GYR 12

typedef struct {
	// WHOAMI
	uint8_t whoami;

	// Device Address
	uint8_t addr;

	// communication line
	I2C_HandleTypeDef* line;

	uint8_t initialized;

	int16_t offset_acc_x;
	int16_t offset_acc_y;
	int16_t offset_acc_z;
	//uint16_t temp;

} IMU;

extern float IMU_DATA_ACC_X;
extern float IMU_DATA_ACC_Y;
extern float IMU_DATA_ACC_Z;

extern int16_t IMU_RAW_DATA_ACC_X;
extern int16_t IMU_RAW_DATA_ACC_Y;
extern int16_t IMU_RAW_DATA_ACC_Z;

extern float IMU_DATA_ANGLE_X;
extern float IMU_DATA_ANGLE_Y;
extern float IMU_DATA_ANGLE_Z;

extern int16_t IMU_RAW_DATA_GYR_X;
extern int16_t IMU_RAW_DATA_GYR_Y;
extern int16_t IMU_RAW_DATA_GYR_Z;

extern I2C_HandleTypeDef hi2c1;
extern DMA_HandleTypeDef hdma_i2c1_tx;
extern DMA_HandleTypeDef hdma_i2c1_rx;

extern void Error_Handler();

HAL_StatusTypeDef __IMU_RESET_CLOCK_LINE();
HAL_StatusTypeDef __IMU_GENERATE_CLOCK_PULSE();
HAL_StatusTypeDef __IMU_INIT_GPIO();
HAL_StatusTypeDef __IMU_INIT_I2C();

HAL_StatusTypeDef IMU_INIT(IMU* imu, uint8_t dev_addr, I2C_HandleTypeDef* comm_line);
HAL_StatusTypeDef IMU_INIT_DMA(IMU *imu, uint8_t dev_addr, I2C_HandleTypeDef *comm_line);
HAL_StatusTypeDef __IMU_RESET(IMU *imu);

HAL_StatusTypeDef IMU_I2C_DMA_Callback(IMU *imu, I2C_HandleTypeDef *hi2c);

HAL_StatusTypeDef __IMU_Calibrate(IMU *imu);
HAL_StatusTypeDef __IMU_OFFSET_SUBTRACT(IMU *imu);

HAL_StatusTypeDef IMU_DATA_EXTRACT();
HAL_StatusTypeDef IMU_DATA_EXTRACT_AND_CONVERT(uint8_t dt);
HAL_StatusTypeDef IMU_Read_DATA(IMU* imu);

HAL_StatusTypeDef IMU_Read_ACC(IMU* imu);
HAL_StatusTypeDef IMU_Read_ACC_X(IMU* imu);
HAL_StatusTypeDef IMU_Read_ACC_Y(IMU* imu);
HAL_StatusTypeDef IMU_Read_ACC_Z(IMU* imu);

HAL_StatusTypeDef IMU_Read_GYR(IMU* imu);
HAL_StatusTypeDef IMU_Read_GYR_X(IMU* imu);
HAL_StatusTypeDef IMU_Read_GYR_Y(IMU* imu);
HAL_StatusTypeDef IMU_Read_GYR_Z(IMU* imu);

float __IMU_Convert_ACC(int16_t data);
float __IMU_Convert_GYR(int16_t raw_data, float prev_angle, uint8_t dt);

#endif /* INC_IMU_H_ */
