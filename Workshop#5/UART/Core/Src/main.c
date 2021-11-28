/* USER CODE BEGIN Header */
/*
 * -----------------------------------------------------------------------------
 * UART.
 * ADC1 - external temp sensor(lm335)
 * 4 pins output:
 * green(PD12), orange(PD13), red(PD14), blue(PD15) - digital.
 * 4 buttons:
 * up(PC6), down(PC8), left(PC9), right(PC11).
 * Key control:
 * UP/DOWN/LEFT/RIGHT - red/green/orange/blue.
 * Printing the status of the LEDs (Y-on/N-off) and temperature on the LSD display.
 * The LEDs are controlled by buttons on the board or from a computer via UART.
 * Every 5 seconds (or when the state of the LEDs changes) the board sends data on the UART.
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
#include <stdio.h>
#include <string.h>
#include "hd44780.h"
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
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
_Bool flag_green = 0;
_Bool flag_orange = 0;
_Bool flag_red = 0;
_Bool flag_blue = 0;
uint32_t adcEXT = 0;	// External temp sensor(lm335)
uint32_t sendTimer = 0;
uint16_t sendInterval = 5000;
float tempEXT;
// Degree symbol
uint8_t symbDegree[FONT_HEIGHT] =
{
		0b01100,
		0b10010,
		0b10010,
		0b01100,
		0b00000,
		0b00000,
		0b00000,
		0b00000
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void sendState();
void printDataLCD();
float calcTempEXT(uint16_t inputADC_EXT);
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
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_IT(&hadc1);
  //Initialization of the LCD display
  lcdInit();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Get ADC value from lm335
	  HAL_ADCEx_InjectedStart(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1,100);
	  adcEXT = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
	  HAL_ADCEx_InjectedStop(&hadc1);
	  // Calculating the value of temperature in degrees Celsius.
	  tempEXT = calcTempEXT(adcEXT);
	  printDataLCD();

	  // On/Off leds according to the flags
	  HAL_GPIO_WritePin(GPIOD, ledGREEN_Pin, flag_green);
	  HAL_GPIO_WritePin(GPIOD, ledORANGE_Pin, flag_orange);
	  HAL_GPIO_WritePin(GPIOD, ledRED_Pin, flag_red);
	  HAL_GPIO_WritePin(GPIOD, ledBLUE_Pin, flag_blue);

	  // UART poll
	  uint8_t rcvBuf[1];
	  HAL_StatusTypeDef result;
	  result = HAL_UART_Receive(&huart3, rcvBuf, 1, 20);
	  if (result == HAL_OK)
	  {
		  switch (rcvBuf[0])
		  {
		  		case '1':
		  			flag_red = !flag_red;
		  			break;
		  		case '2':
		  			flag_green = !flag_green;
		  			break;
		  		case '3':
		  			flag_orange = !flag_orange;
		  			break;
		  		case '4':
		  			flag_blue = !flag_blue;
		  			break;
		  		default:
		  			__NOP();
		  			break;
		  }
		  sendState();
		  rcvBuf[0] = 0;
	  }
	  // Every 5 sec
	  if((HAL_GetTick() - sendTimer) > sendInterval)
	  {
		  sendState();
		  sendTimer = HAL_GetTick();
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_9;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_480CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_NONE;
  sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LCD_RS_Pin|LCD_RW_Pin|LCD_E_Pin|LCD_D4_Pin
                          |LCD_D5_Pin|LCD_D6_Pin|LCD_D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, ledGREEN_Pin|ledORANGE_Pin|ledRED_Pin|ledBLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LCD_RS_Pin LCD_RW_Pin LCD_E_Pin LCD_D4_Pin
                           LCD_D5_Pin LCD_D6_Pin LCD_D7_Pin */
  GPIO_InitStruct.Pin = LCD_RS_Pin|LCD_RW_Pin|LCD_E_Pin|LCD_D4_Pin
                          |LCD_D5_Pin|LCD_D6_Pin|LCD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : ledGREEN_Pin ledORANGE_Pin ledRED_Pin ledBLUE_Pin */
  GPIO_InitStruct.Pin = ledGREEN_Pin|ledORANGE_Pin|ledRED_Pin|ledBLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : buttUP_Pin buttDOWN_Pin buttLEFT_Pin buttRIGHT_Pin */
  GPIO_InitStruct.Pin = buttUP_Pin|buttDOWN_Pin|buttLEFT_Pin|buttRIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void printDataLCD()
{
	/* Function for displaying values. */
   lcdClrScr();	// Clear the display

   lcdLoadChar(symbDegree,6); // Load the degree symbol into memory

   lcdGoto(LCD_1st_LINE,0);
   lcdPuts("Temp: ");
   lcdFtos(tempEXT,1); // 1 decimal place
   lcdPutc(6); // Print the degree symbol

   lcdGoto(LCD_1st_LINE,0);
   lcdPuts("\nLedState ");
   _Bool flags[4] =
   {
		   flag_green,
		   flag_orange,
		   flag_red,
		   flag_blue,
   };
   for (int i = 0; i < 4; i++)
   {
	   if (flags[i] == 1)
		   lcdPuts("Y");
	   else
		   lcdPuts("N");
	   if (i < 3)
		   lcdPuts("/");
   }
}

float calcTempEXT(uint16_t inputADC_EXT)
{
	/*
	 * We get the inverted value of the ADC.
	 * (the higher the temperature, the lower the value)
	 * -24 *C = 2.50v
	 *  0  *С = 2.02v(Vo)
	 *  50 *C = 1.02v
	 *  100*C = 0.02v
	 *  => 1*C = 0.02v
	 * voltage = inputADC_EXT*2.98/4096;
	 * where 2.98 is voltage on the sensor;
	 * temp*C = (Vo - voltage) / 0.02
	 */
	return (2.02 - (inputADC_EXT*2.98/(float)4096)) / 0.02;
}

void sendState()
{
	char str[30];
	sprintf (str, "%d%d%d%d%li", flag_red, flag_green, flag_orange, flag_blue, adcEXT);
	HAL_UART_Transmit(&huart3, (uint8_t *)str, strlen(str), 10);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	/* External interrupt handler function. */
	/* Tracking on which pin the interrupt was triggered. */
	switch (GPIO_Pin)
	{
		case buttLEFT_Pin:
			flag_orange = !flag_orange;
			break;
		case buttRIGHT_Pin:
			flag_blue = !flag_blue;
			break;
		case buttUP_Pin:
			flag_red = !flag_red;
			break;
		case buttDOWN_Pin:
			flag_green = !flag_green;
			break;
		default:
			__NOP();
			break;
	}
	sendState();
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
