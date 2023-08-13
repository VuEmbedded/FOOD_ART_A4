/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

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
#define Encorder_1_Pin GPIO_PIN_0
#define Encorder_1_GPIO_Port GPIOC
#define Encorder_1_EXTI_IRQn EXTI0_IRQn
#define Encorder_2_Pin GPIO_PIN_1
#define Encorder_2_GPIO_Port GPIOC
#define Encorder_2_EXTI_IRQn EXTI1_IRQn
#define Encorder_3_Pin GPIO_PIN_2
#define Encorder_3_GPIO_Port GPIOC
#define Encorder_3_EXTI_IRQn EXTI2_IRQn
#define Encorder_4_Pin GPIO_PIN_3
#define Encorder_4_GPIO_Port GPIOC
#define Encorder_4_EXTI_IRQn EXTI3_IRQn
#define Input_Analog_Pin GPIO_PIN_0
#define Input_Analog_GPIO_Port GPIOA
#define Sensor_1_Pin GPIO_PIN_1
#define Sensor_1_GPIO_Port GPIOA
#define Sensor_2_Pin GPIO_PIN_2
#define Sensor_2_GPIO_Port GPIOA
#define Sensor_3_Pin GPIO_PIN_3
#define Sensor_3_GPIO_Port GPIOA
#define Sensor_4_Pin GPIO_PIN_4
#define Sensor_4_GPIO_Port GPIOA
#define PWM_Pin GPIO_PIN_6
#define PWM_GPIO_Port GPIOA
#define NoName6_Pin GPIO_PIN_5
#define NoName6_GPIO_Port GPIOC
#define NoName5_Pin GPIO_PIN_0
#define NoName5_GPIO_Port GPIOB
#define NoName4_Pin GPIO_PIN_1
#define NoName4_GPIO_Port GPIOB
#define NoName3_Pin GPIO_PIN_2
#define NoName3_GPIO_Port GPIOB
#define NoName2_Pin GPIO_PIN_10
#define NoName2_GPIO_Port GPIOB
#define NoName1_Pin GPIO_PIN_11
#define NoName1_GPIO_Port GPIOB
#define Relay_control_motor_2_Pin GPIO_PIN_13
#define Relay_control_motor_2_GPIO_Port GPIOB
#define Relay_control_motor_1_Pin GPIO_PIN_14
#define Relay_control_motor_1_GPIO_Port GPIOB
#define Control_4_Pin GPIO_PIN_15
#define Control_4_GPIO_Port GPIOB
#define Control_3_Pin GPIO_PIN_6
#define Control_3_GPIO_Port GPIOC
#define Control_2_Pin GPIO_PIN_7
#define Control_2_GPIO_Port GPIOC
#define Control_1_Pin GPIO_PIN_8
#define Control_1_GPIO_Port GPIOC
#define ON_OFF_button_Pin GPIO_PIN_9
#define ON_OFF_button_GPIO_Port GPIOC
#define Step_PUL_Pin GPIO_PIN_8
#define Step_PUL_GPIO_Port GPIOA
#define Step_DIR_Pin GPIO_PIN_9
#define Step_DIR_GPIO_Port GPIOA
#define Step_EN_Pin GPIO_PIN_10
#define Step_EN_GPIO_Port GPIOA
#define Relay_UV_Pin GPIO_PIN_11
#define Relay_UV_GPIO_Port GPIOA
#define Relay_UV_Fan_Pin GPIO_PIN_12
#define Relay_UV_Fan_GPIO_Port GPIOA
#define Cong_tac_HT_1_Pin GPIO_PIN_15
#define Cong_tac_HT_1_GPIO_Port GPIOA
#define Cong_tac_HT_2_Pin GPIO_PIN_10
#define Cong_tac_HT_2_GPIO_Port GPIOC
#define Cong_tac_HT_3_Pin GPIO_PIN_11
#define Cong_tac_HT_3_GPIO_Port GPIOC
#define Cong_tac_HT_4_Pin GPIO_PIN_12
#define Cong_tac_HT_4_GPIO_Port GPIOC
#define Table_control_1_Pin GPIO_PIN_2
#define Table_control_1_GPIO_Port GPIOD
#define Table_control_2_Pin GPIO_PIN_3
#define Table_control_2_GPIO_Port GPIOB
#define Table_control_3_Pin GPIO_PIN_4
#define Table_control_3_GPIO_Port GPIOB
#define Table_control_4_Pin GPIO_PIN_5
#define Table_control_4_GPIO_Port GPIOB
#define Table_control_5_Pin GPIO_PIN_6
#define Table_control_5_GPIO_Port GPIOB
#define Table_control_6_Pin GPIO_PIN_7
#define Table_control_6_GPIO_Port GPIOB
#define Table_control_7_Pin GPIO_PIN_8
#define Table_control_7_GPIO_Port GPIOB
#define Table_control_8_Pin GPIO_PIN_9
#define Table_control_8_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
