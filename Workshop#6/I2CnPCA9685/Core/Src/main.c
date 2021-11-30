/* USER CODE BEGIN Header */
/*
 * -----------------------------------------------------------------------------
 * I2C.PCA9685
 * 16 channels output.
 * The LEDs are controlled from a computer via UART.
 * Use UartController16 to control LEDs, frequency and duty cycle.
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
#include <ctype.h>
#include "PCA9685lib.h"


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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
int ledCount = 16;
// All possible commands for LED control
uint8_t ledCMD[] =
{
	's',
	'e',
	'd',
	'r',
	'f',
	't',
	'g',
	'y',
	'h',
	'u',
	'j',
	'i',
	'k',
	'o',
	'l',
	'p'
};
// A flag indicating whether the same PWM has been set for all LEDs
_Bool allDutyFlag = 0;
_Bool sleepFlag = 0;
_Bool allLedOffFlag = 0;

uint32_t sendTimer = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void sendState();
float myRound(float inp);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// For Serial Debug
/*
int _write(int file, char *ptr, int len)
{
	int i;
	for (i = 0; i < len; i++)
		ITM_SendChar(*ptr++);
	return len;
}
*/
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
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  initPCA(&hi2c1, 0x80);
  setAllLedOff();
  setPWMFrequency(500);
  sendState();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {
	  // UART poll
	  uint8_t rcvBuf[1];
	  HAL_StatusTypeDef result;
	  result = HAL_UART_Receive(&huart3, rcvBuf, 1, 10);

	  if (result == HAL_OK)
	  {
		  // If uppercase character
		  if (isupper(rcvBuf[0]))
		  {
			  for (int i = 0; i < ledCount; i++)
			  {
				  if (toupper(ledCMD[i]) == rcvBuf[0])
				  {
					  // Get the PWM value from the desired channel
					  // and subtract 25% from it
					  int val = getPWM(i+1);
					  val = val - (4096/4);
					  if (val < 0)
						  val = 0;
					  if (val == 0)
						  setLedPWM(i+1, val);
					  else
						  setLedPWM(i+1, (val-1));
					  allDutyFlag = 0;
				  	  break;
				  }
			  }
		  }
		  else
		  {
			  if (rcvBuf[0] == '&')
			  {
				  // Accept, convert to an integer value and set the frequency
		  		  char freqBuf[6];
		  		  HAL_UART_Receive(&huart3, freqBuf, 6, 50);
			  	  setPWMFrequency(atoi(freqBuf));
			  	  // clear buff
			  	  for (int i = 0; i < sizeof(freqBuf)/sizeof(freqBuf[0]); i++)
			  		  freqBuf[i] = 0;
			  }
			  if (rcvBuf[0] == '$')
			  {
				  // Accept, convert to an integer value and set the duty cycle
			  	  char pwmBuf[6];
			  	  HAL_UART_Receive(&huart3, pwmBuf, 6, 50);
			  	  setAllLedPWM(atoi(pwmBuf));
			  	  allDutyFlag = 1;
			  	  // clear buff
			  	  for (int i = 0; i < sizeof(pwmBuf)/sizeof(pwmBuf[0]); i++)
			  		  pwmBuf[i] = 0;
			  }
			  if (rcvBuf[0] == '*')
			  {
			  	  sleepPCA();
			  	  sleepFlag = 1;
			  }
			  if (rcvBuf[0] == '+')
			  {
			  	  wakeupPCA();
			  	  sleepFlag = 0;
		  	  }
			  if (rcvBuf[0] == '5')
			  {
				  allLedOffFlag = !allLedOffFlag;
				  if (allLedOffFlag)
					  setAllLedOff();
				  else
					  setAllLedOn();
			  }
			  if (rcvBuf[0] == '0')
				  resetDevice();

			  for (int i = 0; i < ledCount; i++)
			  {


				  if (ledCMD[i] == rcvBuf[0])
				  {
					  // Get the PWM value from the desired channel
					  // and add 25% to it
					  int val = getPWM(i+1);
				  	  val = val + (4096/4);
				  	  if (val > 4096)
				  		  val = 4096;
				  	  setLedPWM(i+1, (val-1));
				  	  allDutyFlag = 0;
			  		  break;
				  }
			  }
		  }
		  // Anti-rattle protection
		  if((HAL_GetTick() - sendTimer) > 300)
		  {
			  sendState();
			  sendTimer = HAL_GetTick();
		  }
		  // clear buff
	  	  rcvBuf[0] = 0;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void sendState()
{
	char str[76];
	str[0] = 0;
	int freq;
	int dutyCycle;;
	int sleepMode;

	freq = getPWMFrequency();
	if (allDutyFlag)
		dutyCycle = getPWM(1);
	else
		// show that dutyCycle is not equal for all channels
		dutyCycle = 9999;
	sleepMode = sleepFlag;
	// Get the PWM value
	uint16_t ledPWM[16];
	for (int i = 0; i < sizeof(ledPWM) / sizeof(ledPWM[0]); i++)
		ledPWM[i] = getPWM(i+1);
	// Form a string
	for(int i = 0; i < sizeof(ledPWM) / sizeof(ledPWM[0]); ++i)
	{
			  sprintf(str + strlen(str), "%04d", ledPWM[i]);
	}

	sprintf(str + strlen(str), "%04d%04d%04d",freq,dutyCycle,sleepMode);
	printf("%i\n", strlen(str));
	// Send by UART
	HAL_UART_Transmit(&huart3, (uint8_t *)&str, sizeof(str), 10);


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
