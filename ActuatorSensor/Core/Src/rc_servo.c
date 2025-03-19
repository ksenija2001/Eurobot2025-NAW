/*
 * rc_servo.c
 *
 *  Created on: Mar 6, 2025
 *      Author: xenia
 */

#include "rc_servo.h"

sRC_Servo_t rc_servos[CHANNEL_NUM];
uint32_t adc_output;
uint8_t adc_channel = 0;
uint8_t adc_sample_num = 0;

void Init_RC_Servo(uint8_t index, TIM_HandleTypeDef* htim, uint16_t tim_channel){
	rc_servos[index].TIM.tim = htim;
	rc_servos[index].TIM.channel = tim_channel;

	__HAL_TIM_SET_COMPARE(htim, tim_channel, ARR_MIN);
	HAL_TIM_PWM_Start(htim, tim_channel);
}

void Set_Angle(uint8_t index, uint8_t angle){
	float duty = (ARR_MAX - ARR_MIN)/180.0 * angle + ARR_MIN;
	__HAL_TIM_SET_COMPARE(rc_servos[index].TIM.tim, rc_servos[index].TIM.channel, duty);
}

void Set_ADC_Channel(uint8_t index){
	HAL_GPIO_WritePin(S0_GPIO_Port, S0_Pin, (index & 0x01));
	HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, (index & 0x02));
	HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, (index & 0x04));
}

// 1/(144MHz/4) * (24.5 + 12) = 1.01us conversion rate
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	float I = adc_output * mA_LSB;
	rc_servos[adc_channel].curr_I = 0.9 * rc_servos[adc_channel].curr_I + 0.1 * I;

	// Safety for servos
	if (rc_servos[adc_channel].curr_I >= I_MAX){
		float curr_duty = __HAL_TIM_GET_COMPARE(rc_servos[adc_channel].TIM.tim, rc_servos[adc_channel].TIM.channel);
		__HAL_TIM_SET_COMPARE(rc_servos[adc_channel].TIM.tim, rc_servos[adc_channel].TIM.channel, curr_duty);
	}

	if (++adc_sample_num >= SAMPLE_NUM) {
		adc_sample_num = 0;
		if (++adc_channel >= CHANNEL_NUM) adc_channel = 0;
		Set_ADC_Channel(adc_channel);
	}
}
