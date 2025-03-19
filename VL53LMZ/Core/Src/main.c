/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "custom_bus.h"
#include "custom_tof.h"
#include "vl53lmz_api.h"

#include "point_cloud.h"
#include "vector.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/


UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
VL53LMZ_Object center = {
		.io = {
			.LPn_port = LPn_GPIO_Port,
			.LPn_pin = LPn_Pin,
			.RST_port = I2C_RST_GPIO_Port,
			.RST_pin = I2C_RST_Pin,
			.PWR_EN_port = PWR_EN_GPIO_Port,
			.PWR_EN_pin = PWR_EN_Pin
		},
		.trans_offset = { .vector = {0.0, 282.0, 0.0} },
		.orient_offset = { .vector = {0} }
};

VL53LMZ_Object right = {
		.io = {
			.LPn_port = LPn_R_GPIO_Port,
			.LPn_pin = LPn_R_Pin,
			.RST_port = I2C_RST_R_GPIO_Port,
			.RST_pin = I2C_RST_R_Pin,
			.PWR_EN_port = PWR_EN_R_GPIO_Port,
			.PWR_EN_pin = PWR_EN_R_Pin
		},
		.trans_offset = {
			//.vector = {-147.15, 282.0, 46.2862}  // curved, focus on 200mm
			//.vector = {-58.0, 282.0, 0.0}        // straight, narrow, VL53L7
			//.vector = {-45.0, 282.0, 0.0}          // straight, narrow, VL53L5
			//.vector = {-112.165, 282.0, 0.0}     // straight, wide, VL53L7
			.vector = {-80.0 ,282.0, 0.0}         // straight, wide, VL53L5
		},
		.orient_offset = {
			//.vector = {0.0, M_PI_4, 0.0}        // curved, focus on 200mm
			.vector = {0.0, 0.0, 0.0}
		}
};

VL53LMZ_Object left = {
		.io = {
			.LPn_port = LPn_L_GPIO_Port,
			.LPn_pin = LPn_L_Pin,
			.RST_port = I2C_RST_L_GPIO_Port,
			.RST_pin = I2C_RST_L_Pin,
			.PWR_EN_port = PWR_EN_L_GPIO_Port,
			.PWR_EN_pin = PWR_EN_L_Pin
		},
		.trans_offset = {
			//.vector = {149.12, 282.0, 60.347}    // curved, focus on 200mm
			//.vector = {58.0, 282.0, 0.0}         // straight, narrow, VL53L7
			//.vector = {45.0, 282.0, 0.0}          // straight, narrow, VL53L5
			//.vector = {118.895, 282.0, 0.0}      // straight, wide, VL53L7
			.vector = {85.0, 282.0, 0.0}         // straight, wide, VL53L5
		},
		.orient_offset = {
			//.vector = {0.0, -M_PI_4, 0.0}     // curved, focus on 200mm
			.vector = {0.0, 0.0, 0.0}
		}
};

uint8_t rx_buffer[4] = {0, 0, 0, 0};
uint8_t which_TOF = 0;
uint8_t status;
VL53LMZ_Result_t data;
sVector3_t Point_Cloud[1024];
uint16_t pc_index = 0;
uint8_t initialized = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if (huart->Instance == USART2){

		if ( rx_buffer[0] == 0xFA && rx_buffer[2] == 0xFB ){
			which_TOF = rx_buffer[1];
		}
		__HAL_UART_CLEAR_OREFLAG(huart);

		HAL_UART_Receive_IT(huart, rx_buffer, 3);
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  GenerateTables();

  VL53LMZ_Reset(&center.io);
  VL53LMZ_Reset(&right.io);
  VL53LMZ_Reset(&left.io);

  status = VL53LMZ_Init(&center, VL53LMZ_DEFAULT_I2C_ADDRESS);
  if ( status != VL53LMZ_STATUS_OK ){
	  Error_Handler();
  }

  status = VL53LMZ_Init(&right, VL53LMZ_DEFAULT_I2C_ADDRESS + 2);
  if ( status != VL53LMZ_STATUS_OK ){
	  Error_Handler();
  }

  status = VL53LMZ_Init(&left, VL53LMZ_DEFAULT_I2C_ADDRESS + 4);
  if ( status != VL53LMZ_STATUS_OK ){
	  Error_Handler();
  }

  /* Configure devices: resolution, ranging_mode, integration_time(only for autonomous mode), ranging_frequency, sharpener percent */
  HAL_GPIO_WritePin(center.io.LPn_port, center.io.LPn_pin, GPIO_PIN_SET);
  status = VL53LMZ_Config(&center.conf, VL53LMZ_RESOLUTION_8X8, VL53LMZ_RANGING_MODE_CONTINUOUS, 30, 15, 30);
  if ( status != VL53LMZ_STATUS_OK ){
	  Error_Handler();
  }
  HAL_GPIO_WritePin(center.io.LPn_port, center.io.LPn_pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(right.io.LPn_port, right.io.LPn_pin, GPIO_PIN_SET);
  status = VL53LMZ_Config(&right.conf, VL53LMZ_RESOLUTION_8X8, VL53LMZ_RANGING_MODE_CONTINUOUS, 30, 15, 30);
  if ( status != VL53LMZ_STATUS_OK ){
	  Error_Handler();
  }
  HAL_GPIO_WritePin(right.io.LPn_port, right.io.LPn_pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(left.io.LPn_port, left.io.LPn_pin, GPIO_PIN_SET);
  status = VL53LMZ_Config(&left.conf, VL53LMZ_RESOLUTION_8X8, VL53LMZ_RANGING_MODE_CONTINUOUS, 30, 15, 30);
  if ( status != VL53LMZ_STATUS_OK ){
	  Error_Handler();
  }

  left.conf.module_type = VL53LMZ_MODULE_TYPE_L5;
  right.conf.module_type = VL53LMZ_MODULE_TYPE_L5;

  /* Enable communication for all devices */
  HAL_GPIO_WritePin(center.io.LPn_port, center.io.LPn_pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(right.io.LPn_port, right.io.LPn_pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(left.io.LPn_port, left.io.LPn_pin, GPIO_PIN_SET);

  /* Start ranging */
  status = vl53lmz_start_ranging(&center.conf);
  status |= vl53lmz_start_ranging(&right.conf);
  status |= vl53lmz_start_ranging(&left.conf);

  if ( status != VL53LMZ_STATUS_OK ){
  	  Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

//  HAL_Delay(100);
  which_TOF = 0x00;
  initialized = 1;
  __HAL_UART_CLEAR_OREFLAG(&huart2);

  HAL_UART_Receive_IT(&huart2, rx_buffer, 3);
//  int milis = HAL_GetTick();
//  int time;

  while (1)
  {
	  status = VL53LMZ_STATUS_OK;

	  switch (which_TOF) {
		case 'C':
			status |= VL53LMZ_Get_Distance(&center.conf, &data);
			status |= ConvertDist2Point(&data, &center, 450.0);

			which_TOF = 0x00;
			break;
		case 'R':
			status |= VL53LMZ_Get_Distance(&right.conf, &data);
			status |= ConvertDist2Point(&data, &right, 450.0);
//			time = HAL_GetTick() - milis;
//			milis = HAL_GetTick();
			which_TOF = 0x00;
			break;
		case 'L':
			status |= VL53LMZ_Get_Distance(&left.conf, &data);
			status |= ConvertDist2Point(&data, &left, 450.0);
			which_TOF = 0x00;
			break;
		default:
			break;
	  	}

//	  	HAL_Delay(1);


	  	if (status != VL53LMZ_STATUS_OK)
	  	{
	  		  Error_Handler();
	  	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}


/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PWR_EN_R_GPIO_Port, PWR_EN_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, PWR_EN_L_Pin|LD2_Pin|I2C_RST_L_Pin|I2C_RST_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PWR_EN_Pin|LPn_L_Pin|I2C_RST_Pin|LPn_Pin
                          |LPn_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PWR_EN_R_Pin */
  GPIO_InitStruct.Pin = PWR_EN_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PWR_EN_R_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PWR_EN_L_Pin LD2_Pin I2C_RST_L_Pin I2C_RST_R_Pin */
  GPIO_InitStruct.Pin = PWR_EN_L_Pin|LD2_Pin|I2C_RST_L_Pin|I2C_RST_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : INT_Pin INT_R_Pin */
  GPIO_InitStruct.Pin = INT_Pin|INT_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PWR_EN_Pin LPn_L_Pin I2C_RST_Pin LPn_Pin
                           LPn_R_Pin */
  GPIO_InitStruct.Pin = PWR_EN_Pin|LPn_L_Pin|I2C_RST_Pin|LPn_Pin
                          |LPn_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : INT_L_Pin */
  GPIO_InitStruct.Pin = INT_L_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INT_L_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */

/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if (GPIO_Pin == INT_Pin && initialized){
		which_TOF = 'C';
//		status = VL53LMZ_Get_Distance(&center.conf, &data);
//		status |= ConvertDist2Point(&data, &center, 450.0);
//		for (uint16_t i=0; i<64; ++i){
//			if (center.point_cloud[i].vector[0] != 0 || center.point_cloud[i].vector[1] != 0)
//				Point_Cloud[pc_index++] = center.point_cloud[i];
//		}
	} else if (GPIO_Pin == INT_R_Pin && initialized){
		which_TOF = 'R';
//		status = VL53LMZ_Get_Distance(&right.conf, &data);
//		status |= ConvertDist2Point(&data, &right, 450.0);
//		for (uint16_t i=0; i<64; ++i){
//			if (right.point_cloud[i].vector[0] != 0 || right.point_cloud[i].vector[1] != 0)
//				Point_Cloud[pc_index++] = right.point_cloud[i];
//		}
	} else if (GPIO_Pin == INT_L_Pin && initialized){
		which_TOF = 'L';
//		status = VL53LMZ_Get_Distance(&left.conf, &data);
//		status |= ConvertDist2Point(&data, &left, 450.0);
//		for (uint16_t i=0; i<64; ++i){
//			if (left.point_cloud[i].vector[0] != 0 || left.point_cloud[i].vector[1] != 0)
//				Point_Cloud[pc_index++] = left.point_cloud[i];
//		}
	}


}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
