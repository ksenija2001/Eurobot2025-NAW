/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Encoder1_A_Pin GPIO_PIN_0
#define Encoder1_A_GPIO_Port GPIOC
#define Encoder1_B_Pin GPIO_PIN_1
#define Encoder1_B_GPIO_Port GPIOC
#define Motor1_EN_Pin GPIO_PIN_2
#define Motor1_EN_GPIO_Port GPIOC
#define Motor1_Dir_Pin GPIO_PIN_3
#define Motor1_Dir_GPIO_Port GPIOC
#define Motor1_PWM_Pin GPIO_PIN_0
#define Motor1_PWM_GPIO_Port GPIOA
#define Motor2_PWM_Pin GPIO_PIN_1
#define Motor2_PWM_GPIO_Port GPIOA
#define Motor2_EN_Pin GPIO_PIN_2
#define Motor2_EN_GPIO_Port GPIOA
#define Motor2_Dir_Pin GPIO_PIN_3
#define Motor2_Dir_GPIO_Port GPIOA
#define Encoder2_A_Pin GPIO_PIN_4
#define Encoder2_A_GPIO_Port GPIOA
#define Encoder2_B_Pin GPIO_PIN_6
#define Encoder2_B_GPIO_Port GPIOA
#define IN1_Pin GPIO_PIN_0
#define IN1_GPIO_Port GPIOB
#define IN2_Pin GPIO_PIN_1
#define IN2_GPIO_Port GPIOB
#define LED_R_Pin GPIO_PIN_11
#define LED_R_GPIO_Port GPIOB
#define LED_G_Pin GPIO_PIN_12
#define LED_G_GPIO_Port GPIOB
#define LED_B_Pin GPIO_PIN_13
#define LED_B_GPIO_Port GPIOB
#define ERROR_In_Pin GPIO_PIN_6
#define ERROR_In_GPIO_Port GPIOC
#define ERROR_Out_Pin GPIO_PIN_7
#define ERROR_Out_GPIO_Port GPIOC
#define LED_CAN_RX_Pin GPIO_PIN_9
#define LED_CAN_RX_GPIO_Port GPIOA
#define LED_CAN_TX_Pin GPIO_PIN_10
#define LED_CAN_TX_GPIO_Port GPIOA
#define IMU_SCL_Pin GPIO_PIN_15
#define IMU_SCL_GPIO_Port GPIOA
#define IMU_SMBA_Pin GPIO_PIN_5
#define IMU_SMBA_GPIO_Port GPIOB
#define IMU_SDA_Pin GPIO_PIN_7
#define IMU_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
