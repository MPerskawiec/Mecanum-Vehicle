/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define LED_1_ON HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
#define LED_1_OFF HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);

#define LED_2_ON HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
#define LED_2_OFF HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);

#define LED_3_ON HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);
#define LED_3_OFF HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);

#define LED_4_ON HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_RESET);
#define LED_4_OFF HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_SET);

#define LED_5_ON HAL_GPIO_WritePin(LED_5_GPIO_Port, LED_5_Pin, GPIO_PIN_RESET);
#define LED_5_OFF HAL_GPIO_WritePin(LED_5_GPIO_Port, LED_5_Pin, GPIO_PIN_SET);


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

volatile int rodzaj_ruchu;
volatile int speed;

uint8_t data[50]; // Tablica przechowujaca wysylana wiadomosc.
uint16_t size = 0; // Rozmiar wysylanej wiadomosci


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
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
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  rodzaj_ruchu = 0;
  speed = 10;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {



	  if(rodzaj_ruchu != 1 && HAL_GPIO_ReadPin(KEY_1_GPIO_Port, KEY_1_Pin) == GPIO_PIN_RESET){
		  rodzaj_ruchu = 1;
		  size = sprintf(data, "P R 1   \r\n");
		  HAL_UART_Transmit_IT(&huart3, data, size);


	  } else if(rodzaj_ruchu != 2 && HAL_GPIO_ReadPin(KEY_2_GPIO_Port, KEY_2_Pin) == GPIO_PIN_RESET){
		  rodzaj_ruchu = 2;
		  size = sprintf(data, "P R 2   \r\n");
		  HAL_UART_Transmit_IT(&huart3, data, size);

	  }else if(rodzaj_ruchu != 3 && HAL_GPIO_ReadPin(KEY_3_GPIO_Port, KEY_3_Pin) == GPIO_PIN_RESET){
		  rodzaj_ruchu = 3;
		  size = sprintf(data, "P R 3   \r\n");
		  HAL_UART_Transmit_IT(&huart3, data, size);

	  }else if(rodzaj_ruchu != 4 && HAL_GPIO_ReadPin(KEY_4_GPIO_Port, KEY_4_Pin) == GPIO_PIN_RESET){
		  rodzaj_ruchu = 4;
		  size = sprintf(data, "P R 4   \r\n");
		  HAL_UART_Transmit_IT(&huart3, data, size);

	  } else if(rodzaj_ruchu != 5 && HAL_GPIO_ReadPin(KEY_5_GPIO_Port, KEY_5_Pin) == GPIO_PIN_RESET){
		  rodzaj_ruchu = 5;
		  size = sprintf(data, "P R 0   \r\n");
		  HAL_UART_Transmit_IT(&huart3, data, size);

	  }else if(rodzaj_ruchu != 6 && HAL_GPIO_ReadPin(KEY_6_GPIO_Port, KEY_6_Pin) == GPIO_PIN_RESET){
		  rodzaj_ruchu = 6;
		  size = sprintf(data, "P R 6   \r\n");
		  HAL_UART_Transmit_IT(&huart3, data, size);

	  }else if(rodzaj_ruchu != 7 && HAL_GPIO_ReadPin(KEY_7_GPIO_Port, KEY_7_Pin) == GPIO_PIN_RESET){
		  rodzaj_ruchu = 7;
		  size = sprintf(data, "P R 7   \r\n");
		  HAL_UART_Transmit_IT(&huart3, data, size);

	  }else if(rodzaj_ruchu != 8 && HAL_GPIO_ReadPin(KEY_8_GPIO_Port, KEY_8_Pin) == GPIO_PIN_RESET){
		  rodzaj_ruchu = 8;
		  size = sprintf(data, "P R 8   \r\n");
		  HAL_UART_Transmit_IT(&huart3, data, size);

	  }else if(rodzaj_ruchu != 9 && HAL_GPIO_ReadPin(KEY_9_GPIO_Port, KEY_9_Pin) == GPIO_PIN_RESET){
		  rodzaj_ruchu = 9;
		  size = sprintf(data, "P R 9   \r\n");
		  HAL_UART_Transmit_IT(&huart3, data, size);

	  }else if(rodzaj_ruchu != 10 && HAL_GPIO_ReadPin(KEY_10_GPIO_Port, KEY_10_Pin) == GPIO_PIN_RESET){
		  rodzaj_ruchu = 10;
		  size = sprintf(data, "P R 10  \r\n");
		  HAL_UART_Transmit_IT(&huart3, data, size);

	  }else if(rodzaj_ruchu != 11 && HAL_GPIO_ReadPin(KEY_11_GPIO_Port, KEY_11_Pin) == GPIO_PIN_RESET){
		  rodzaj_ruchu = 11;
		  size = sprintf(data, "P R 11  \r\n");
		  HAL_UART_Transmit_IT(&huart3, data, size);

	  }else if( HAL_GPIO_ReadPin(KEY_12_GPIO_Port, KEY_12_Pin) == GPIO_PIN_RESET){
		  speed++;
		  if(speed >= 15) speed = 15;
		  size = sprintf(data, "P S %d  \r\n", speed);
		  HAL_UART_Transmit_IT(&huart3, data, size);
		  HAL_Delay(350);

	  }else if(HAL_GPIO_ReadPin(KEY_13_GPIO_Port, KEY_13_Pin) == GPIO_PIN_RESET){
		  speed--;
		  if(speed <= 10) speed = 10;
		  size = sprintf(data, "P S %d  \r\n", speed);
		  HAL_UART_Transmit_IT(&huart3, data, size);
		  HAL_Delay(350);

	  }else if(rodzaj_ruchu != 14 && HAL_GPIO_ReadPin(KEY_14_GPIO_Port, KEY_14_Pin) == GPIO_PIN_RESET){
		  rodzaj_ruchu = 14;
		  size = sprintf(data, "P R 14  \r\n");
		  HAL_UART_Transmit_IT(&huart3, data, size);

	  }else if(rodzaj_ruchu != 15 && HAL_GPIO_ReadPin(KEY_15_GPIO_Port, KEY_15_Pin) == GPIO_PIN_RESET){
		  rodzaj_ruchu = 15;
		  size = sprintf(data, "P R 15  \r\n");
		  HAL_UART_Transmit_IT(&huart3, data, size);

	  }else if(rodzaj_ruchu != 16 && HAL_GPIO_ReadPin(KEY_16_GPIO_Port, KEY_16_Pin) == GPIO_PIN_RESET){
		  rodzaj_ruchu = 16;
		  size = sprintf(data, "P R 16  \r\n");
		  HAL_UART_Transmit_IT(&huart3, data, size);

	  }else if(rodzaj_ruchu != 17 && HAL_GPIO_ReadPin(KEY_17_GPIO_Port, KEY_17_Pin) == GPIO_PIN_RESET){
		  rodzaj_ruchu = 17;
		  size = sprintf(data, "P R 17  \r\n");
		  HAL_UART_Transmit_IT(&huart3, data, size);

	  }else if(rodzaj_ruchu != 18 && HAL_GPIO_ReadPin(KEY_18_GPIO_Port, KEY_18_Pin) == GPIO_PIN_RESET){
		  rodzaj_ruchu = 18;
		  size = sprintf(data, "P R 0   \r\n");
		  HAL_UART_Transmit_IT(&huart3, data, size);

	  }else if(rodzaj_ruchu != 19 && HAL_GPIO_ReadPin(KEY_19_GPIO_Port, KEY_19_Pin) == GPIO_PIN_RESET){
		  rodzaj_ruchu = 19;
		  size = sprintf(data, "P R 19  \r\n");
		  HAL_UART_Transmit_IT(&huart3, data, size);

	  }else if(rodzaj_ruchu != 20 && HAL_GPIO_ReadPin(KEY_20_GPIO_Port, KEY_20_Pin) == GPIO_PIN_RESET){
		  rodzaj_ruchu = 20;
		  size = sprintf(data, "P R 20  \r\n");
		  HAL_UART_Transmit_IT(&huart3, data, size);

	  }else if(rodzaj_ruchu != 21 && HAL_GPIO_ReadPin(KEY_21_GPIO_Port, KEY_21_Pin) == GPIO_PIN_RESET){
		  rodzaj_ruchu = 21;
		  size = sprintf(data, "P R 21  \r\n");
		  HAL_UART_Transmit_IT(&huart3, data, size);

	  }else if(rodzaj_ruchu != 22 && HAL_GPIO_ReadPin(KEY_22_GPIO_Port, KEY_22_Pin) == GPIO_PIN_RESET){
		  rodzaj_ruchu = 22;
		  size = sprintf(data, "P R 22  \r\n");
		  HAL_UART_Transmit_IT(&huart3, data, size);

	  }


	  if(speed == 10){
		  LED_1_OFF;
		  LED_2_OFF;
		  LED_3_OFF;
		  LED_4_OFF;
		  LED_5_OFF;
	  } else if(speed == 11){
		  LED_1_ON;
		  LED_2_OFF;
		  LED_3_OFF;
		  LED_4_OFF;
		  LED_5_OFF;
	  } else if(speed == 12){
		  LED_1_ON;
		  LED_2_ON;
		  LED_3_OFF;
		  LED_4_OFF;
		  LED_5_OFF;
	  } else if(speed == 13){
		  LED_1_ON;
		  LED_2_ON;
		  LED_3_ON;
		  LED_4_OFF;
		  LED_5_OFF;
	  } else if(speed == 14){
		  LED_1_ON;
		  LED_2_ON;
		  LED_3_ON;
		  LED_4_ON;
		  LED_5_OFF;
	  } else if(speed == 15){
		  LED_1_ON;
		  LED_2_ON;
		  LED_3_ON;
		  LED_4_ON;
		  LED_5_ON;
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

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, STATUS_Pin|LED_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_4_Pin|LED_5_Pin|LED_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : STATUS_Pin LED_1_Pin */
  GPIO_InitStruct.Pin = STATUS_Pin|LED_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY_16_Pin KEY_15_Pin KEY_21_Pin KEY_7_Pin 
                           KEY_8_Pin KEY_5_Pin KEY_9_Pin KEY_2_Pin */
  GPIO_InitStruct.Pin = KEY_16_Pin|KEY_15_Pin|KEY_21_Pin|KEY_7_Pin 
                          |KEY_8_Pin|KEY_5_Pin|KEY_9_Pin|KEY_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_4_Pin LED_5_Pin LED_2_Pin */
  GPIO_InitStruct.Pin = LED_4_Pin|LED_5_Pin|LED_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY_12_Pin KEY_10_Pin KEY_17_Pin KEY_4_Pin 
                           KEY_1_Pin KEY_19_Pin */
  GPIO_InitStruct.Pin = KEY_12_Pin|KEY_10_Pin|KEY_17_Pin|KEY_4_Pin 
                          |KEY_1_Pin|KEY_19_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY_14_Pin KEY_18_Pin KEY_22_Pin KEY_20_Pin 
                           KEY_13_Pin KEY_6_Pin KEY_11_Pin */
  GPIO_InitStruct.Pin = KEY_14_Pin|KEY_18_Pin|KEY_22_Pin|KEY_20_Pin 
                          |KEY_13_Pin|KEY_6_Pin|KEY_11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_3_Pin */
  GPIO_InitStruct.Pin = LED_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY_3_Pin */
  GPIO_InitStruct.Pin = KEY_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY_3_GPIO_Port, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
