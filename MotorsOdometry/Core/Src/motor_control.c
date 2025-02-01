/*
 * motor_control.c
 *
 *  Created on: Jan 30, 2025
 *      Author: xenia
 */

#include "motor_control.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

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

uint8_t Init_Motor(sMotorConfig_t* motor, TIM_HandleTypeDef* htim, ADC_HandleTypeDef* hadc){
	uint8_t status = HAL_OK;

	motor->TIM.tim = htim;
	motor->ADC.adc = hadc;

	status |= HAL_ADCEx_Calibration_Start(hadc, ADC_SINGLE_ENDED);
    status |= HAL_ADC_Start_DMA(hadc, (uint32_t*)motor->ADC.output, 1);

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

void Set_Direction(sMotorConfig_t* motor, uint8_t direction){
	HAL_GPIO_WritePin(motor->IO.Direction.port, motor->IO.Direction.pin, (direction & 0x01));
}

// speed is always positive, direction defines if the rotation is in the positive or negative direction
void Set_Speed(sMotorConfig_t* motor, uint16_t speed, uint8_t direction){
	uint32_t rpm = (uint32_t)(speed * 60 * REDUCTION) / (M_PI * WHEEL_DIAMETER);

	Set_Direction(motor, direction);
	Set_RPM(motor, rpm);
}

// SamplingTime = 640.5cycles => (640.5+12.5)/144MHz => Conversions every 4.53us
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	float voltage;
	if (hadc == left_motor.ADC.adc){
		voltage = (float)((V_MAX - V_MIN)/ADC_RES) * left_motor.ADC.output;
		left_motor.currRPM = -RPM_MAX + (uint16_t)(2*RPM_MAX/V_MAX * voltage);
	} else if (hadc == right_motor.ADC.adc){
		voltage = (float)((V_MAX - V_MIN)/ADC_RES) * right_motor.ADC.output;
		right_motor.currRPM = -RPM_MAX + (uint16_t)(2*RPM_MAX/V_MAX * voltage);
	}
}


