/* USER CODE BEGIN Header */
/*
 * -----------------------------------------------------------------------------
 * Nightlight.
 * It has 5 operating modes - 3 normal and 2 smooth (with PWM).
 * 4 LEDS: green(PD12), orange(PD13), red(PD14), blue(PD15).
 * 5 buttons: up(PC8), down(PC6), left(PC11), right(PC9), middle(PA15).
 * Key control:
 * UP/DOWN - increase/decrease the blink rate
 * LEFT/RIGHT - previous/next mode
 * MIDDLE - start/stop blinking
 * -----------------------------------------------------------------------------
 */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
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
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
volatile uint32_t * ledArray[] =
{
		// Defined in main.h
		&ledGREEN_Tim,
		&ledORANGE_Tim,
		&ledRED_Tim,
		&ledBLUE_Tim
};
const uint32_t SIZEledArray = sizeof(ledArray)/sizeof(ledArray[0]);

// Leds menu. The modes will switch in the same sequence as defined in this array.
uint8_t favModesArray[] = {1, 2, 3, 4, 5};
const uint8_t SIZEfavModesArray = sizeof(favModesArray)/sizeof(favModesArray[0]);

// Set the default led mode to 1.
volatile uint8_t ledMode = 1;
volatile uint8_t last_ledMode;
volatile int8_t modeCounter;
_Bool stop_flag = 0;

/* Blink rate change factor.
 * Varies from 1 (fast blinking) to 10 (slow blinking).
 * In normal mode, the delay is organized in the format: 100ms*ledSpeed
 * In PWM mode, ledSpeed is used as a divider of a PWM step: 10/ledSpeed  */
volatile uint8_t ledSpeed = 10;

// pwm_max must be equal to the (Counter Period + 1)
uint16_t pwm_max = 500;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void DelayCtrl();
void schemeCircle();
void schemePairs();
void schemeSnake();
void schemeCirclePWM();
void schemeSnakePWM();
void schemeSTOP();
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
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  switch (ledMode)
	  {
	  	  /* To add or remove a mode from the
	  	   * led menu it is necessary to edit
	  	   * mode numbers in favModesArray. */

	  	  // Turn off all LEDs:
	      case 99: schemeSTOP(); break;
	      // The LEDs light up one after the other in a circle:
	      case 1: schemeCircle(); break;
	      // LEDs light up in pairs (opposite each other):
	      case 2: schemePairs(); break;
	      // LEDs light up alternately until they form a complete
	      // circle, and in the same way go out:
	      case 3: schemeSnake(); break;
	      // LEDs light up smoothly one after the other in a circle:
	      case 4: schemeCirclePWM(); break;
	      // LEDs light up smoothly alternately until they form
	      // a complete circle, and in the same way go out.
	      case 5: schemeSnakePWM(); break;
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 64-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 500-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pins : buttDOWN_Pin buttUP_Pin buttRIGHT_Pin buttLEFT_Pin */
  GPIO_InitStruct.Pin = buttDOWN_Pin|buttUP_Pin|buttRIGHT_Pin|buttLEFT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : buttMIDDLE_Pin */
  GPIO_InitStruct.Pin = buttMIDDLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(buttMIDDLE_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void DelayCtrl()
{
	/* Delay for normal mode.
	 * If ledSpeed = 10, the LEDs will blink with
	 * a delay of 1 sec at the slowest speed.*/
	HAL_Delay(100*ledSpeed);
}
void schemeCircle()
{
	/* The LEDs light up one after the other in a circle. */
	for (int i = 0; i < SIZEledArray; i++)
	{
		*ledArray[i] = pwm_max;
		DelayCtrl();
		*ledArray[i] = 0;
	}
}
void schemePairs()
{
	/* The LEDs light up in pairs (opposite each other). */
	for (int i = 0; i < SIZEledArray / 2; i++)
	{
		*ledArray[i] = pwm_max;
		*ledArray[i + 2] = pwm_max;
		DelayCtrl();
		*ledArray[i] = 0;
		*ledArray[i + 2] = 0;
	}
}
void schemeSnake()
{
	/* The LEDs light up alternately until they form a complete circle,
	 * and in the same way go out. */
	for (int i = 0; i < SIZEledArray; i++)
	{
		*ledArray[i] = pwm_max;
		DelayCtrl();
	}
	for (int i = 0; i < SIZEledArray; i++)
	{
		*ledArray[i] = 0;
		DelayCtrl();
	}
}
void schemeCirclePWM()
{
	/* The LEDs light up smoothly one after the other in a circle. */
	for (int i = 0; i < SIZEledArray; i++)
	{
		for (int j = 0; j <= pwm_max; )
		{
			*ledArray[i] = j;
			HAL_Delay(5);
			j += 10/ledSpeed;
		}
		for (int j = pwm_max; j >= 0; )
		{
			*ledArray[i] = j;
			HAL_Delay(5);
			j -= 10/ledSpeed;
		}
	}
}
void schemeSnakePWM()
{
	/* The LEDs light up smoothly alternately until they form a complete
	 * circle, and in the same way go out. */
	for (int i = 0; i < SIZEledArray; i++)
	{
		for (int j = 0; j <= pwm_max; )
		{
			*ledArray[i] = j;
			HAL_Delay(5);
			j += 10/ledSpeed;
		}
	}
	for (int i = 0; i < SIZEledArray; i++)
	{
		for (int j = pwm_max; j >= 0; )
		{
			*ledArray[i] = j;
			HAL_Delay(5);
			j -= 10/ledSpeed;;
		}
	}
}
void schemeSTOP()
{
	/* Turn off all LEDs. */
	for (int i = 0; i < SIZEledArray; i++)
	{
		*ledArray[i] = 0;
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	/* External interrupt handler function. */
	/* Tracking on which pin the interrupt was triggered. */
	switch (GPIO_Pin)
	{
	/*--- MODE CONTROL BEGIN ---*/
		case buttLEFT_Pin:
			if (!stop_flag)
			{
				// Get the number of the previous mode.
				if (--modeCounter < 0)
					modeCounter = SIZEfavModesArray - 1;
				ledMode = favModesArray[modeCounter];
			}
			break;
		case buttRIGHT_Pin:
			if (!stop_flag)
			{
				// Get the next mode number.
				if (++modeCounter >= SIZEfavModesArray)
					modeCounter = 0;
				ledMode = favModesArray[modeCounter];
			}
				break;
	/*--- MODE CONTROL END ---*/
	/*--- BEGIN SPEED CONTROL BEGIN ---*/
		case buttUP_Pin:
			if (ledSpeed > 1)
				ledSpeed--;
			break;
		case buttDOWN_Pin:
			if (ledSpeed < 10)
				ledSpeed++;
			break;
	/*--- SPEED CONTROL END ---*/
	/*--- BEGIN START/STOP BLINKING BEGIN ---*/
		case buttMIDDLE_Pin:
			// Turns off all LEDs, or turns on
			// the last active mode.
			stop_flag = !stop_flag;
			if (stop_flag)
			{
				last_ledMode = ledMode;
				ledMode = 99;
			}
			else
			{
				ledMode = last_ledMode;
			}
			break;
		default:
			__NOP();
			break;
	/*--- START/STOP BLINKING END ---*/
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
