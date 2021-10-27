/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define ledGREEN_Pin GPIO_PIN_12
#define ledGREEN_GPIO_Port GPIOD
#define ledORANGE_Pin GPIO_PIN_13
#define ledORANGE_GPIO_Port GPIOD
#define ledRED_Pin GPIO_PIN_14
#define ledRED_GPIO_Port GPIOD
#define ledBLUE_Pin GPIO_PIN_15
#define ledBLUE_GPIO_Port GPIOD
#define buttDOWN_Pin GPIO_PIN_6
#define buttDOWN_GPIO_Port GPIOC
#define buttDOWN_EXTI_IRQn EXTI9_5_IRQn
#define buttUP_Pin GPIO_PIN_8
#define buttUP_GPIO_Port GPIOC
#define buttUP_EXTI_IRQn EXTI9_5_IRQn
#define buttRIGHT_Pin GPIO_PIN_9
#define buttRIGHT_GPIO_Port GPIOC
#define buttRIGHT_EXTI_IRQn EXTI9_5_IRQn
#define buttMIDDLE_Pin GPIO_PIN_15
#define buttMIDDLE_GPIO_Port GPIOA
#define buttMIDDLE_EXTI_IRQn EXTI15_10_IRQn
#define buttLEFT_Pin GPIO_PIN_11
#define buttLEFT_GPIO_Port GPIOC
#define buttLEFT_EXTI_IRQn EXTI15_10_IRQn
/* USER CODE BEGIN Private defines */
#define ledGREEN_Tim TIM4->CCR1
#define ledORANGE_Tim TIM4->CCR2
#define ledRED_Tim TIM4->CCR3
#define ledBLUE_Tim TIM4->CCR4
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
