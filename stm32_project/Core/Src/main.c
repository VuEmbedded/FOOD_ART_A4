/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define digitalWrite(pin_t,status) HAL_GPIO_WritePin(output_table[pin_t].type,output_table[pin_t].pin,status);
#define digitalRead(pin_t)  HAL_GPIO_ReadPin(input_table[pin_t].type,input_table[pin_t].pin)


enum output_define
{
  PIN_RELAY_FAN = 0,
  PIN_RELAY_FAN_UV,
  PIN_STEP_EN,
  PIN_STEP_DIR,
  PIN_STEP_PUL,
  PIN_ONOFF_BUTTON,
  PIN_CONTROL1,
  PIN_CONTROL2,
  PIN_CONTROL3,
  PIN_CONTROL4,
  PIN_CONTROL_MOTOR_1,
  PIN_CONTROL_MOTOR_2,
  PIN_OUTPUT_MAX
};

enum input_define
{
  PIN_SWITCH_1 = 0,
  PIN_SWITCH_2,
  PIN_SWITCH_3,
  PIN_SWITCH_4,
  PIN_TABLE_CONTROL_1,
  PIN_TABLE_CONTROL_2,
  PIN_TABLE_CONTROL_3,
  PIN_TABLE_CONTROL_4,
  PIN_TABLE_CONTROL_5,
  PIN_TABLE_CONTROL_6,
  PIN_TABLE_CONTROL_7,
  PIN_TABLE_CONTROL_8,
  PIN_SENSOR_1,
  PIN_SENSOR_2,
  PIN_SENSOR_3,
  PIN_SENSOR_4,
  PIN_INPUT_MAX
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
typedef struct  GPIO_configs
{
  GPIO_TypeDef *type;
  uint16_t pin;
}GPIO_config;

GPIO_config output_table[PIN_OUTPUT_MAX] =
{
  {GPIOA,GPIO_PIN_12},
  {GPIOA,GPIO_PIN_11},
  {GPIOA,GPIO_PIN_10},
  {GPIOA,GPIO_PIN_9},
  {GPIOA,GPIO_PIN_8},
  {GPIOC,GPIO_PIN_9},
  {GPIOC,GPIO_PIN_8},
  {GPIOC,GPIO_PIN_7},
  {GPIOC,GPIO_PIN_6},
  {GPIOB,GPIO_PIN_15},
  {GPIOB,GPIO_PIN_14},
  {GPIOB,GPIO_PIN_13}
};

GPIO_config input_table[PIN_INPUT_MAX] =
{
  {GPIOA,GPIO_PIN_15},
  {GPIOC,GPIO_PIN_10},
  {GPIOC,GPIO_PIN_11},
  {GPIOC,GPIO_PIN_12},
  {GPIOD,GPIO_PIN_2},
  {GPIOB,GPIO_PIN_3},
  {GPIOB,GPIO_PIN_4},
  {GPIOB,GPIO_PIN_5},
  {GPIOB,GPIO_PIN_6},
  {GPIOB,GPIO_PIN_7},
  {GPIOB,GPIO_PIN_8},
  {GPIOB,GPIO_PIN_9},
  {GPIOA,GPIO_PIN_1},
  {GPIOA,GPIO_PIN_2},
  {GPIOA,GPIO_PIN_3},
  {GPIOA,GPIO_PIN_4}
};


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    digitalWrite(PIN_RELAY_FAN,GPIO_PIN_SET);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 63;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, NoName6_Pin|Control_3_Pin|Control_2_Pin|Control_1_Pin
                          |ON_OFF_button_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, NoName5_Pin|NoName4_Pin|NoName3_Pin|NoName2_Pin
                          |NoName1_Pin|Relay_control_motor_2_Pin|Relay_control_motor_1_Pin|Control_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Step_PUL_Pin|Step_DIR_Pin|Step_EN_Pin|Relay_UV_Pin
                          |Relay_UV_Fan_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Encorder_1_Pin Encorder_2_Pin Encorder_3_Pin Encorder_4_Pin */
  GPIO_InitStruct.Pin = Encorder_1_Pin|Encorder_2_Pin|Encorder_3_Pin|Encorder_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Sensor_1_Pin Sensor_2_Pin Sensor_3_Pin Sensor_4_Pin
                           Cong_tac_HT_1_Pin */
  GPIO_InitStruct.Pin = Sensor_1_Pin|Sensor_2_Pin|Sensor_3_Pin|Sensor_4_Pin
                          |Cong_tac_HT_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : NoName6_Pin Control_3_Pin Control_2_Pin Control_1_Pin
                           ON_OFF_button_Pin */
  GPIO_InitStruct.Pin = NoName6_Pin|Control_3_Pin|Control_2_Pin|Control_1_Pin
                          |ON_OFF_button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : NoName5_Pin NoName4_Pin NoName3_Pin NoName2_Pin
                           NoName1_Pin Relay_control_motor_2_Pin Relay_control_motor_1_Pin Control_4_Pin */
  GPIO_InitStruct.Pin = NoName5_Pin|NoName4_Pin|NoName3_Pin|NoName2_Pin
                          |NoName1_Pin|Relay_control_motor_2_Pin|Relay_control_motor_1_Pin|Control_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Step_PUL_Pin Step_DIR_Pin Step_EN_Pin Relay_UV_Pin
                           Relay_UV_Fan_Pin */
  GPIO_InitStruct.Pin = Step_PUL_Pin|Step_DIR_Pin|Step_EN_Pin|Relay_UV_Pin
                          |Relay_UV_Fan_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Cong_tac_HT_2_Pin Cong_tac_HT_3_Pin Cong_tac_HT_4_Pin */
  GPIO_InitStruct.Pin = Cong_tac_HT_2_Pin|Cong_tac_HT_3_Pin|Cong_tac_HT_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Table_control_1_Pin */
  GPIO_InitStruct.Pin = Table_control_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Table_control_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Table_control_2_Pin Table_control_3_Pin Table_control_4_Pin Table_control_5_Pin
                           Table_control_6_Pin Table_control_7_Pin Table_control_8_Pin */
  GPIO_InitStruct.Pin = Table_control_2_Pin|Table_control_3_Pin|Table_control_4_Pin|Table_control_5_Pin
                          |Table_control_6_Pin|Table_control_7_Pin|Table_control_8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
