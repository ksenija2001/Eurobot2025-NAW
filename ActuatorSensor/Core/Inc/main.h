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
#define PWM_1_Pin GPIO_PIN_0
#define PWM_1_GPIO_Port GPIOC
#define PWM_2_Pin GPIO_PIN_1
#define PWM_2_GPIO_Port GPIOC
#define PWM_3_Pin GPIO_PIN_2
#define PWM_3_GPIO_Port GPIOC
#define PWM_4_Pin GPIO_PIN_3
#define PWM_4_GPIO_Port GPIOC
#define In_1_Pin GPIO_PIN_1
#define In_1_GPIO_Port GPIOA
#define In_2_Pin GPIO_PIN_2
#define In_2_GPIO_Port GPIOA
#define In_3_Pin GPIO_PIN_3
#define In_3_GPIO_Port GPIOA
#define In_4_Pin GPIO_PIN_4
#define In_4_GPIO_Port GPIOA
#define In_5_Pin GPIO_PIN_5
#define In_5_GPIO_Port GPIOA
#define In_6_Pin GPIO_PIN_6
#define In_6_GPIO_Port GPIOA
#define In_7_Pin GPIO_PIN_7
#define In_7_GPIO_Port GPIOA
#define In_8_Pin GPIO_PIN_4
#define In_8_GPIO_Port GPIOC
#define Out_1_Pin GPIO_PIN_5
#define Out_1_GPIO_Port GPIOC
#define Out_2_Pin GPIO_PIN_0
#define Out_2_GPIO_Port GPIOB
#define Out_3_Pin GPIO_PIN_1
#define Out_3_GPIO_Port GPIOB
#define Out_4_Pin GPIO_PIN_2
#define Out_4_GPIO_Port GPIOB
#define LED_R_Pin GPIO_PIN_11
#define LED_R_GPIO_Port GPIOB
#define LED_G_Pin GPIO_PIN_12
#define LED_G_GPIO_Port GPIOB
#define LED_B_Pin GPIO_PIN_13
#define LED_B_GPIO_Port GPIOB
#define PWM_5_Pin GPIO_PIN_6
#define PWM_5_GPIO_Port GPIOC
#define PWM_6_Pin GPIO_PIN_7
#define PWM_6_GPIO_Port GPIOC
#define PWM_7_Pin GPIO_PIN_8
#define PWM_7_GPIO_Port GPIOC
#define PWM_8_Pin GPIO_PIN_9
#define PWM_8_GPIO_Port GPIOC
#define LED_CAN_RX_Pin GPIO_PIN_9
#define LED_CAN_RX_GPIO_Port GPIOA
#define LED_CAN_TX_Pin GPIO_PIN_10
#define LED_CAN_TX_GPIO_Port GPIOA
#define S0_Pin GPIO_PIN_10
#define S0_GPIO_Port GPIOC
#define S1_Pin GPIO_PIN_11
#define S1_GPIO_Port GPIOC
#define S2_Pin GPIO_PIN_12
#define S2_GPIO_Port GPIOC
#define Protocol_1_Pin GPIO_PIN_6
#define Protocol_1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
