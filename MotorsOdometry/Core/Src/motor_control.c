/*
 * motor_control.c
 *
 *  Created on: Jan 30, 2025
 *      Author: xenia
 */

#include "motor_control.h"

sMotorConfig_t left_motor = {
		.TIM = {
				.channel = TIM_CHANNEL_1},
		.IO = {
				.RPM = { .pin = Motor1_RPM_Pin, .port = Motor1_RPM_GPIO_Port},
				.PWM = { .pin = Motor1_PWM_Pin, .port = Motor1_PWM_GPIO_Port},
				.EN  = { .pin = Motor1_EN_Pin,  .port = Motor1_EN_GPIO_Port},
				.Direction = { .pin = Motor1_Dir_Pin, .port = Motor1_Dir_GPIO_Port}
		}
};

sMotorConfig_t right_motor = {
		.TIM = {
				.channel = TIM_CHANNEL_2},
		.IO = {
				.RPM = { .pin = Motor2_RPM_Pin, .port = Motor2_RPM_GPIO_Port},
				.PWM = { .pin = Motor2_PWM_Pin, .port = Motor2_PWM_GPIO_Port},
				.EN  = { .pin = Motor2_EN_Pin,  .port = Motor2_EN_GPIO_Port},
				.Direction = { .pin = Motor2_Dir_Pin, .port = Motor2_Dir_GPIO_Port}
		}
};

uint8_t Init_Motor(sMotorConfig_t* motor, TIM_HandleTypeDef* htim){
	uint8_t status = HAL_OK;

	motor->TIM.tim = htim;

	HAL_GPIO_WritePin(motor->IO.EN.port, motor->IO.EN.pin, GPIO_PIN_SET);

	status |= HAL_TIM_PWM_Start(htim, motor->TIM.channel);

	return status;
}

void Set_Duty_Cycle(sMotorConfig_t* motor, float duty_cycle){
	motor->TIM.tim->Instance->CCR1 = duty_cycle;
}

void Set_RPM(sMotorConfig_t* motor, uint32_t rpm){
	if (rpm > RPM_MAX) rpm = RPM_MAX;

	float duty_cycle = ARR_MIN + (float)(rpm/RPM_MAX) * (ARR_MAX - ARR_MIN);
	Set_Duty_Cycle(motor, duty_cycle);
}

void Set_Speed(sMotorConfig_t* motor, float speed, uint8_t direction){
	uint32_t rpm = (uint32_t)(speed * 60 * REDUCTION) / (M_PI * WHEEL_DIAMETER);

	HAL_GPIO_WritePin(motor->IO.Direction.port, motor->IO.Direction.pin, direction);
	Set_RPM(motor, rpm);
}


