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
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdbool.h>
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
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
HAL_StatusTypeDef ok_notok = HAL_BUSY;
uint8_t CMD0 [] = {0x40,0x00,0x00, 0x00, 0x00, 0x95};
uint8_t high [] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t CMD1 [] = {0x41,0x00,0x00, 0x00, 0x00, 0x79};
uint8_t CMD8 [] = {0x48, 0x00, 0x00, 0x01, 0xAA, 0x87};
uint8_t CMD55 [] = {0x77, 0x00, 0x00, 0x00, 0x00, 0x65};
uint8_t ACMD41 [] = {0x69, 0x40, 0x00, 0x00, 0x00, 0x77};
uint8_t highByte [] = {0xFF};

uint8_t CMD0_Response [2]; //8 (max wait) + 2 (payload)
uint8_t CMD1_Response [2];
uint8_t CMD8_Response [7];
uint8_t CMD55_Response [2];
uint8_t ACMD41_Response [2];
int count = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
void myprintf(const char *fmt, ...);
void configure_sd(void);
int acdm55(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void myprintf(const char *fmt, ...) {
  static char buffer[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  int len = strlen(buffer);
  HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, -1);

}

int acdm55(void) {
	ok_notok = HAL_SPI_Transmit(&hspi2, highByte, 1, 1000);
	ok_notok = HAL_SPI_Transmit(&hspi2, highByte, 1, 1000);
	ok_notok = HAL_SPI_Transmit(&hspi2, CMD55, 6, 1000); //Sending in Blocking mode
		HAL_Delay(10);
		HAL_SPI_Receive(&hspi2, CMD55_Response, 2, 1000);
		//HAL_Delay(1000);
		count = 0;
		while(count < 20 && CMD55_Response[0] != 0x0 && CMD55_Response[1] != 0x1){
				count++;
				for(int i = 0; i<2; i++){
							  myprintf("(%x)", CMD55_Response[i]);
						  }
				myprintf("\r\n");
			  ok_notok = HAL_SPI_Transmit(&hspi2, CMD55, 6, 1000); //Sending in Blocking mode
			  HAL_Delay(10);
			  HAL_SPI_Receive(&hspi2, CMD55_Response, 2, 1000);
		}
		if(count == 20){
			//myprintf("CMD 55 Timeout \r\n");
				return 0;
			}
		myprintf("CMD55: ");
		for(int i = 0; i<2; i++){
				  myprintf("(%x)", CMD55_Response[i]);
			  }
		myprintf("\r\n");
		ok_notok = HAL_SPI_Transmit(&hspi2, highByte, 1, 1000);
		ok_notok = HAL_SPI_Transmit(&hspi2, highByte, 1, 1000);
		//ACMD41
		ok_notok = HAL_SPI_Transmit(&hspi2, ACMD41, 6, 1000); //Sending in Blocking mode
		HAL_Delay(100);
		HAL_SPI_Receive(&hspi2, ACMD41_Response, 2, 1000);
		count = 0;
		if(ACMD41_Response[0] != 0x0 && ACMD41_Response[1] != 0x0)
		{
			myprintf("ACMD41 Fail once: ");
			for(int i = 0; i<2; i++){
				myprintf("(%x)", ACMD41_Response[i]);
			}
			myprintf("\r\n");
			return 0;
		}
		myprintf("ACMD41: ");
		for(int i = 0; i<2; i++){
				  myprintf("(%x)", ACMD41_Response[i]);
			  }
		myprintf("\r\n");
		return 1;
}

void configure_sd(void) {

	HAL_GPIO_WritePin(GPIOB, SD_CS_Pin, SET);
	ok_notok = HAL_SPI_Transmit(&hspi2, high, 10, 1000); //Sending in Blocking mode
	HAL_GPIO_WritePin(GPIOB, SD_CS_Pin, RESET);
	//HAL_GPIO_WritePin(GPIOA, Green_LED_Pin, SET);
	//		  ok_notok = HAL_UART_Transmit(&huart2, (uint8_t *)hw, len, 100);

	//CMD0:
	ok_notok = HAL_SPI_Transmit(&hspi2, CMD0, 6, 1000); //Sending in Blocking mode
	HAL_Delay(100);
	HAL_SPI_Receive(&hspi2, CMD0_Response, 2, 1000);
	count = 0;
	while(CMD0_Response[1] != 0x1 && count < 20){
	  count++;
	  myprintf("0 Failed once: \r\n");
	  ok_notok = HAL_SPI_Transmit(&hspi2, CMD0, 6, 1000); //Sending in Blocking mode
	  HAL_Delay(100);
	  HAL_SPI_Receive(&hspi2, CMD0_Response, 2, 1000);
	}
	if(count == 20){
		myprintf("Timeout \r\n");
		return;
	}
	myprintf("CMD0: ");
	for(int i = 0; i<2; i++){
			  myprintf("(%x)", CMD0_Response[i]);
		  }
	myprintf("\r\n");
	//HAL_Delay(100);
	ok_notok = HAL_SPI_Transmit(&hspi2, highByte, 1, 1000);
	ok_notok = HAL_SPI_Transmit(&hspi2, highByte, 1, 1000);

	//CMD8:
	ok_notok = HAL_SPI_Transmit(&hspi2, CMD8, 6, 1000); //Sending in Blocking mode
	HAL_Delay(100);
	HAL_SPI_Receive(&hspi2, CMD8_Response, 7, 1000);
	//HAL_Delay(1000);
	count = 0;
	while(count < 20 && (CMD8_Response[1] != 0x1 || CMD8_Response[5] != 0xAA)){
			count++;
			  myprintf("8 Failed once: \r\n");
			  for(int i = 0; i<7; i++){
			  			  myprintf("(%x)", CMD8_Response[i]);
			  		  }
			  ok_notok = HAL_SPI_Transmit(&hspi2, CMD8, 6, 1000); //Sending in Blocking mode
			  HAL_Delay(100);
			  HAL_SPI_Receive(&hspi2, CMD8_Response, 7, 1000);
	}
	if(count == 20){
		myprintf("Timeout \r\n");
			return;
		}
	myprintf("CMD8: ");
	for(int i = 0; i<7; i++){
			  myprintf("(%x)", CMD8_Response[i]);
		  }
	myprintf("\r\n");
	ok_notok = HAL_SPI_Transmit(&hspi2, highByte, 1, 1000);
	ok_notok = HAL_SPI_Transmit(&hspi2, highByte, 1, 1000);

	//CMD55:
	int counter = 0;
	int worked = acdm55();
	myprintf("Worked: (%i)\r\n", worked);
	while(worked == 0 && counter<20){
			worked = acdm55();
			//myprintf("Worked: (%i)\r\n", worked);
			counter++;
			//myprintf("Count: (%x)\r\n", count);
			if(worked==1){
				myprintf("yay!\r\n");
			}
	}
	myprintf("Count: %i\r\n", counter);
	myprintf("Worked: %i\r\n", worked);
	if(counter==20){
		myprintf("Timeout\r\n");
		return;
	}

	myprintf("End of Start Up\r\n");
	if(ok_notok == HAL_OK){
	  HAL_GPIO_WritePin(GPIOA, Green_LED_Pin, SET);
	}
	HAL_Delay(1000);
	HAL_GPIO_WritePin(GPIOA, Green_LED_Pin, RESET);
	HAL_GPIO_WritePin(GPIOB, SD_CS_Pin, SET);
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
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  const char *hw = "Hello World! \r\n";
  HAL_GPIO_WritePin(GPIOA, Green_LED_Pin, RESET);
  HAL_Delay(3000); //a short delay is important to let the SD card settle
  GPIO_PinState pressed;

  int len = 15;
  ok_notok = HAL_UART_Transmit(&huart2, (uint8_t *)hw, len, 100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  pressed = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
	  if(pressed == 0){
		  configure_sd();
		  HAL_Delay(1000);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Green_LED_GPIO_Port, Green_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Green_LED_Pin */
  GPIO_InitStruct.Pin = Green_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Green_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
