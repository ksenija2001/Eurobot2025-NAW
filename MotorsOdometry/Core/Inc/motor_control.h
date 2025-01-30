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
#include "math.h"

/* PWM timer settings */
#define ARR 28799
#define ARR_MAX ( (int32_t)(ARR * 0.9) )
#define ARR_MIN ( (int32_t)(ARR * 0.1) )

/* Motor characteristics */
#define RPM_MAX 1200
#define REDUCTION 21
#define WHEEL_DIAMETER 70 // mm


typedef struct {
	uint16_t pin;
	GPIO_TypeDef* port;
} sIO_t;

typedef struct {
	TIM_HandleTypeDef* tim;
	uint16_t channel;
} sTIM_t;

typedef struct {
	sIO_t EN;
	sIO_t RPM;
	sIO_t PWM;
	sIO_t Direction;
} sMotorIO_t;

typedef struct {
	sMotorIO_t IO;
	sTIM_t TIM;
} sMotorConfig_t;


/* Motor utility functions */
uint8_t Init_Motor(sMotorConfig_t* motor, TIM_HandleTypeDef* htim);
void Set_Duty_Cycle(sMotorConfig_t* motor, float duty_cycle);
void Set_RPM(sMotorConfig_t* motor, uint32_t rpm);
void Set_Speed(sMotorConfig_t* motor, float speed, uint8_t direction);




#endif /* INC_MOTOR_CONTROL_H_ */
