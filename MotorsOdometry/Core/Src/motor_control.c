/*
 * motor_control.c
 *
 *  Created on: Jan 30, 2025
 *      Author: xenia
 */

#include "motor_control.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

int16_t duty_cycle;

sMotorConfig_t left_motor = {
		.TIM = {
				.channel = TIM_CHANNEL_1},
		.IO = {
				.RPM = { .pin = Motor1_RPM_Pin, .port = Motor1_RPM_GPIO_Port},
				.PWM = { .pin = Motor1_PWM_Pin, .port = Motor1_PWM_GPIO_Port},
				.Direction  = { .pin = Motor1_EN_Pin,  .port = Motor1_EN_GPIO_Port},
				.EN = { .pin = Motor1_Dir_Pin, .port = Motor1_Dir_GPIO_Port}
		}
};

sMotorConfig_t right_motor = {
		.TIM = {
				.channel = TIM_CHANNEL_2},
		.IO = {
				.RPM = { .pin = Motor2_RPM_Pin, .port = Motor2_RPM_GPIO_Port},
				.PWM = { .pin = Motor2_PWM_Pin, .port = Motor2_PWM_GPIO_Port},
				.Direction  = { .pin = Motor2_EN_Pin,  .port = Motor2_EN_GPIO_Port},
				.EN = { .pin = Motor2_Dir_Pin, .port = Motor2_Dir_GPIO_Port}
		}
};

uint8_t Init_Motor(sMotorConfig_t* motor, TIM_HandleTypeDef* htim, ADC_HandleTypeDef* hadc){
	uint8_t status = HAL_OK;

	motor->TIM.tim = htim;
	motor->ADC.adc = hadc;

	status |= HAL_ADCEx_Calibration_Start(hadc, ADC_SINGLE_ENDED);
    status |= HAL_ADC_Start_DMA(hadc, &motor->ADC.output, 1);

    Enable_Motor(motor, 1);
	status |= HAL_TIM_PWM_Start(htim, motor->TIM.channel);
	Set_RPM(motor, 0);

	return status;
}

void Enable_Motor(sMotorConfig_t* motor, uint8_t enable){
	HAL_GPIO_WritePin(motor->IO.EN.port, motor->IO.EN.pin, (enable & 0x01));
}

void Stop_Motor(sMotorConfig_t* motor){
	Set_RPM(motor, 0);
	Enable_Motor(motor, 0);
	HAL_ADC_Stop_DMA(motor->ADC.adc);
	HAL_TIM_PWM_Stop(motor->TIM.tim, motor->TIM.channel);
}

void Set_Duty_Cycle(sMotorConfig_t* motor, uint16_t duty_cycle){
	__HAL_TIM_SET_COMPARE(motor->TIM.tim, motor->TIM.channel, duty_cycle);
}

void Set_RPM(sMotorConfig_t* motor, float rpm){
	if (abs(rpm) > RPM_MAX) rpm = rpm/(abs(rpm+1e-6)) * RPM_MAX;

	float duty_rpm = (float)abs(rpm)/RPM_MAX;
	duty_cycle = (uint16_t)(ARR_MIN + duty_rpm * (ARR_MAX - ARR_MIN));

	if (motor == &left_motor) Set_Direction(motor, rpm > 0); //PROVERI RETARDE MOZDA JE OVDE PROBLEM STO TI ROBOT NE RADI
	else Set_Direction(motor, rpm < 0);

	Set_Duty_Cycle(motor, abs(duty_cycle));
}

void Set_Direction(sMotorConfig_t* motor, uint8_t direction){
	HAL_GPIO_WritePin(motor->IO.Direction.port, motor->IO.Direction.pin, (direction & 0x01));
}

// speed is always positive, direction defines if the rotation is in the positive or negative direction
void Set_Speed(sMotorConfig_t* motor, float speed){
	float rpm = (speed * 60 * REDUCTION) / (M_PI * WHEEL_DIAMETER);

	Set_RPM(motor, rpm);
}

// SamplingTime = 640.5cycles => (640.5+12.5)/144MHz => Conversions every 4.53us
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	float voltage;
	if (hadc == left_motor.ADC.adc){
		voltage = ((float)(V_MAX - V_MIN)/ADC_RES) * left_motor.ADC.output;
		left_motor.currRPM = -RPM_MAX + (uint16_t)(2*RPM_MAX/V_MAX * voltage);
	} else if (hadc == right_motor.ADC.adc){
		voltage = ((float)(V_MAX - V_MIN)/ADC_RES) * right_motor.ADC.output;
		right_motor.currRPM = -RPM_MAX + (uint16_t)(2*RPM_MAX/V_MAX * voltage);
	}

}


