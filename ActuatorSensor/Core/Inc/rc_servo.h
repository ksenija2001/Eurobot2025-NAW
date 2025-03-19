/*
 * rc_servo.h
 *
 *  Created on: Mar 6, 2025
 *      Author: xenia
 */

#ifndef INC_RC_SERVO_H_
#define INC_RC_SERVO_H_

#include "struct_types.h"
#include "main.h"

/* PWM timer settings */
#define ARR 59999       // 50Hz
#define ARR_MAX ( (int16_t)(ARR * 0.1) )
#define ARR_MIN ( (int16_t)(ARR * 0.05) )

/* RPM ADC settings */
#define mA_LSB 0.4
#define V_MIN 0
#define V_MAX 3.3
#define ADC_RES 4095  // 2^12-1

#define CHANNEL_NUM 1
#define I_MAX 100  // mA
#define SAMPLE_NUM 100 // number of ADC samples taken from a single channel

typedef struct {
	sTIM_t TIM;
	float curr_I;
} sRC_Servo_t;

void Init_RC_Servo(uint8_t index, TIM_HandleTypeDef* htim, uint16_t tim_channel);
void Set_Angle(uint8_t index, uint8_t angle);
void Set_ADC_Channel(uint8_t index);

extern uint32_t adc_output;

#endif /* INC_RC_SERVO_H_ */
