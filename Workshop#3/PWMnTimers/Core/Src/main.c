/* USER CODE BEGIN Header */
/*
 * -----------------------------------------------------------------------------
 * PWM and timers.
 * 4 pins output: green(PD12), orange(PD13), red(PD14), blue(PD15).
 * 5 buttons: up(PC8), down(PC6), left(PC11), right(PC9), middle(PA15).
 * Initial input frequency(HSE) = 8MHz.
 * Initial ARR = 200-1.
 * Initial PCS = 0 (without prescaler).
 * Initial PWM duty cycle = 50%
 * Key control:
 * UP/DOWN - set signal frequency (+/- 5 kHz step).
 * LEFT/RIGHT - set duty cycle (+/- 5% step).
 * MIDDLE - select signal output (PD15, PD14, PD13, PD12, no output)
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
#include "math.h"
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
volatile uint32_t * pwmArray[] =
{
		// Defined in main.h
		&ledGREEN_Tim,
		&ledORANGE_Tim,
		&ledRED_Tim,
		&ledBLUE_Tim
};
const uint32_t SIZEpwmArray = sizeof(pwmArray)/sizeof(pwmArray[0]);

// Menu signal output.
uint8_t favModesArray[] = {0, 1, 2, 3, 99};
const uint8_t SIZEfavModesArray = sizeof(favModesArray)/sizeof(favModesArray[0]);

// Set the default pwm mode to 0(ledGREEN_Tim).
volatile uint8_t pwmMode = 0;
volatile int8_t modeCounter = 0;
/* Enter the timer settings that you specify in the settings. */
uint32_t inputFrequency = 8000000; // Input frequency (HSI/HSE) (Hz)
uint16_t period_value = 200; // Initial period_value
uint32_t sigFrequency;
uint16_t freq_step = 5000; // Frequency step(Hz)
uint16_t pwm_max = 200; // Initial maximum PWM value. Must be = initial period_value.
uint16_t pwm_value;
float pwm_step = 0.05; // The step of changing the duty cycle (5% - 0.05)
float pwm_rate = 0.50; // Duty cycle rate 0-100% (0.00 - 1.00)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void recSetting();
void pwmSTOP();
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
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
	/* The first recalculation of the PWM settings */
	recSetting();
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

	  switch (pwmMode)
	  {

	  	  // PWM : no output
	  	  case 99: pwmSTOP(); break;
	  	  // Submission of a signal to a certain output
	  	  // depending on the value of pwmMode
	  	  case 0: // Green
	  	  case 1: // Orange
	  	  case 2: // Red
	  	  case 3: // Blue
	  		  for (int i = 0; i < SIZEpwmArray; i++)
	  		  {
	  			  if (i == pwmMode)
	  				*pwmArray[pwmMode] = pwm_value;
	  			  else
	  				*pwmArray[i] = 0;
	  		  }
	  		  break;
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
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
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 200-1;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pins : buttUP_Pin buttDOWN_Pin buttRIGHT_Pin buttLEFT_Pin */
  GPIO_InitStruct.Pin = buttUP_Pin|buttDOWN_Pin|buttRIGHT_Pin|buttLEFT_Pin;
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
void recSetting()
{
	/* Recalculation of settings */
	sigFrequency = round((inputFrequency/(float)period_value))-1;
	pwm_max = period_value;
	pwm_value = round(pwm_max*pwm_rate);
}
void pwmSTOP()
{
	/* Set 0 pwm. */
	for (int i = 0; i < SIZEpwmArray; i++)
	{
		*pwmArray[i] = 0;
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	/* External interrupt handler function. */
	/* Tracking on which pin the interrupt was triggered. */
	switch (GPIO_Pin)
	{
	/*--- DUTY CYCLE CONTROL BEGIN ---*/
		case buttLEFT_Pin:
			if (pwm_rate - pwm_step > 0)
				pwm_rate -= pwm_step;
			else
				pwm_rate = 0;
			break;
		case buttRIGHT_Pin:
			if (pwm_rate + pwm_step < 1)
				pwm_rate += pwm_step;
			else
				pwm_rate = 1;
			break;
	/*--- DUTY CYCLE CONTROL END ---*/
	/*--- SIGNAL FREQUENCY CONTROL BEGIN ---*/
		case buttUP_Pin: // Increase the signal frequency.
			if (sigFrequency <= freq_step)
				sigFrequency = freq_step-1;
			else
				sigFrequency -= freq_step;
			period_value = round(inputFrequency/(float)sigFrequency);
			// Set the period and reload the register.
			TIM4->ARR = period_value-1;
			TIM4->EGR = TIM_EGR_UG;
			break;
		case buttDOWN_Pin: // Decrease the signal frequency.
			if (sigFrequency >= inputFrequency)
				sigFrequency = inputFrequency-1;
			else
				sigFrequency += freq_step;
			period_value = round(inputFrequency/(float)sigFrequency);
			// Set the period and reload the register.
			TIM4->ARR = period_value-1;
			TIM4->EGR = TIM_EGR_UG;
			break;
	/*--- SIGNAL FREQUENCY CONTROL END ---*/
	/*--- SWITCH SIGNAL OUTPUT BEGIN ---*/
		case buttMIDDLE_Pin:
			// Get the next mode number.
			if (++modeCounter >= SIZEfavModesArray)
				modeCounter = 0;
			pwmMode = favModesArray[modeCounter];
			break;
		default:
			__NOP();
			break;
	/*--- SWITCH SIGNAL OUTPUT END ---*/
	}
	// Make changes to the PWM settings by recalculating them.
	recSetting();
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
