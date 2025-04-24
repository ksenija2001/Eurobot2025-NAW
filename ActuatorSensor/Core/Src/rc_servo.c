/*
 * rc_servo.c
 *
 *  Created on: Mar 6, 2025
 *      Author: xenia
 */

#include "rc_servo.h"

sRC_Servo_t rc_servos[CHANNEL_NUM];
uint8_t init_angles[] = {30 ,150, 30, 150, 40 ,30, 150, 140};
uint32_t adc_output;
uint8_t adc_channel = 0;
uint8_t adc_sample_num = 0;
uint32_t test = 0;

void Init_RC_Servo(uint8_t index, TIM_HandleTypeDef* htim, uint16_t tim_channel){
	rc_servos[index].TIM.tim = htim;
	rc_servos[index].TIM.channel = tim_channel;

	HAL_TIM_PWM_Start(htim, tim_channel);
	Set_Angle(index, init_angles[index]);
	rc_servos[index].curr_angle = init_angles[index];
	rc_servos[index].target_angle = init_angles[index];
}

void Set_Angle(uint8_t index, uint8_t angle){
	float duty = (ARR_MAX - ARR_MIN)/180.0 * angle + ARR_MIN;
	__HAL_TIM_SET_COMPARE(rc_servos[index].TIM.tim, rc_servos[index].TIM.channel, duty);
}

void Set_Target_Angle(uint8_t index, uint8_t angle){
	rc_servos[index].target_angle = angle;
	rc_servos[index].state = 0;
}

void Set_ADC_Channel(uint8_t index){
	HAL_GPIO_WritePin(S0_GPIO_Port, S0_Pin, (index & 0x01));
	HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, (index & 0x02));
	HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, (index & 0x04));
}

// 1/(144MHz/4) * (24.5 + 12) = 1.01us conversion rate
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	float I = adc_output * mA_LSB;
	if (adc_channel == 5 || adc_channel == 6) {
		I /= 10;
	}
	rc_servos[adc_channel].curr_I = 0.999 * rc_servos[adc_channel].curr_I + 0.001 * I;

	switch (rc_servos[adc_channel].state){
	case 0: // target changed
		rc_servos[adc_channel].state_cnt++;
		if ((rc_servos[adc_channel].curr_I < I_MAX || rc_servos[adc_channel].state_cnt < 200) && (uint8_t)rc_servos[adc_channel].target_angle != (uint8_t)rc_servos[adc_channel].curr_angle){
			rc_servos[adc_channel].curr_angle += (rc_servos[adc_channel].target_angle > rc_servos[adc_channel].curr_angle) ? ANGLE_STEP : -ANGLE_STEP;
			// Set_Angle(adc_channel, rc_servos[adc_channel].curr_angle);
		} else if (rc_servos[adc_channel].curr_I >= I_MAX){
			rc_servos[adc_channel].state = 1;
		} else {
			rc_servos[adc_channel].state = 2;
			uint8_t msg[2] = {adc_channel+11, 1};
			FDCAN_Send_Data(0x53F, FDCAN_DLC_BYTES_2, 2, msg);
		}
		break;
	case 1: // target reached by overcurrent
//		rc_servos[adc_channel].target_angle = rc_servos[adc_channel].curr_angle;
		rc_servos[adc_channel].state = 2;
		uint8_t msg[2] = {adc_channel+11, 1};
		FDCAN_Send_Data(0x53F, FDCAN_DLC_BYTES_2, 2, msg);
		break;
	case 2: // target reached
		rc_servos[adc_channel].state_cnt = 0;
		rc_servos[adc_channel].curr_I = 0.0;
		break;
	}

	if (++adc_sample_num >= SAMPLE_NUM) {
		adc_sample_num = 0;
		if (++adc_channel >= CHANNEL_NUM) adc_channel = 0;
		Set_ADC_Channel(adc_channel);
	}

	HAL_ADC_Start_DMA(hadc, &adc_output, 1);

}
