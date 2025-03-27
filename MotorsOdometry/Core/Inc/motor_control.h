/*
 * motor_control.h
 *
 *  Created on: Jan 30, 2025
 *      Author: xenia
 */

#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_

#include "main.h"
#include "stm32g431xx.h"
#include "struct_types.h"
#include "math.h"

/* PWM timer settings */
#define ARR 28799
#define ARR_MAX ( (int32_t)(ARR * 0.90) )
#define ARR_MIN ( (int32_t)(ARR * 0.092) )

/* Motor characteristics */
#define RPM_MAX 9380
#define REDUCTION 26
#define WHEEL_DIAMETER 25.4*2.5 // mm

/* RPM ADC settings */
#define V_MIN 0
#define V_MAX 3.3
#define ADC_RES 4095  // 2^12-1

typedef struct {
	sIO_t EN;
	sIO_t RPM;
	sIO_t PWM;
	sIO_t Direction;
} sMotorIO_t;

typedef struct {
	sMotorIO_t IO;
	sTIM_t TIM;
	sADC_t ADC;
	int16_t currRPM;
} sMotorConfig_t;

/* Motor utility functions */
uint8_t Init_Motor(sMotorConfig_t* motor, TIM_HandleTypeDef* htim, ADC_HandleTypeDef* hadc);
void Enable_Motor(sMotorConfig_t* motor, uint8_t enable);
void Stop_Motor(sMotorConfig_t* motor);
void Set_Duty_Cycle(sMotorConfig_t* motor, uint16_t duty_cycle);
void Set_RPM(sMotorConfig_t* motor, float rpm);
void Set_Speed(sMotorConfig_t* motor, float speed);
void Set_Direction(sMotorConfig_t* motor, uint8_t direction);

extern sMotorConfig_t left_motor;
extern sMotorConfig_t right_motor;




#endif /* INC_MOTOR_CONTROL_H_ */
