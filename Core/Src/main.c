/* USER CODE BEGIN Header */
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void Set_States(uint8_t * blinky_on, uint8_t * brightness_set, uint8_t * brightness_value, int8_t btn_state, uint8_t brightness_steps);
int8_t Button_Check (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint16_t boarder_press_time_short_long);
void Blinky(uint16_t on_time, uint16_t off_time, GPIO_TypeDef* GPIO_TypeDef_LED, uint16_t GPIO_Pin_LED, GPIO_TypeDef* GPIO_TypeDef_BUTTON, uint16_t GPIO_Pin_BUTTON, uint8_t brightness_steps);
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
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(RGB_BL_GPIO_Port, RGB_BL_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RGB_RT_GPIO_Port, RGB_RT_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RGB_GN_GPIO_Port, RGB_GN_Pin, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	// Aufruf Blinky -> mit Blinkintervall (on,off), Input- und Output Pins, sowie die Anzahl der Helligkeitsstufen
	Blinky( 400, 200, RGB_BL_GPIO_Port, RGB_BL_Pin,BUTTON_GPIO_Port, BUTTON_Pin, 3);

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_4;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RGB_BL_Pin|RGB_RT_Pin|RGB_GN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RGB_BL_Pin RGB_RT_Pin RGB_GN_Pin */
  GPIO_InitStruct.Pin = RGB_BL_Pin|RGB_RT_Pin|RGB_GN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

int8_t Button_Check (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint16_t boarder_press_time_short_long){

	// Zeit wie lange Button gedrückt
	static uint32_t button_pressed_time_start;

	// Status-Variablen-Button
	uint8_t button_state_current;
	static uint8_t button_state_previous;
	static uint8_t button_longpress;
	int8_t button_presskind=0;

	// ermitteln des aktuellen Button-Status
	button_state_current = !HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);

	// wenn Button gedrückt
	if(button_state_current == 1)
	{
		// wenn Button vorher nicht gedrückt war -> Start einer Button-gedrückt-Zeitmessung
		if(!button_state_previous)
		{
			button_pressed_time_start = HAL_GetTick();
			button_state_previous = button_state_current;
		}

		// wenn Button "lange gedrückt" -> Rückgabe "lange gedrückt"
		if(button_state_previous && (HAL_GetTick() - button_pressed_time_start >= boarder_press_time_short_long) && !button_longpress )
		{
			button_presskind=-1;
			button_longpress=1;
		}
	}
	else
	{
		// wenn Button gehalten wird und nicht als "lange gedrückt" erkannt
		if(button_state_previous && !button_longpress)
		{
			// wenn Button "kurz" gedrückt -> Rückgabe "kurz gedrückt"
			if(HAL_GetTick() - button_pressed_time_start < boarder_press_time_short_long )
			{
				button_presskind=1;
			}
		}
		button_longpress=0;
		button_state_previous = button_state_current;
	}

	return button_presskind;
}

void Set_States(uint8_t * blinky_on, uint8_t * brightness_set, uint8_t * brightness_value, int8_t btn_state, uint8_t brightness_steps){

	// wenn Button "kurz gedrückt" -> toggeln der Helligkeits-Variable
	if(btn_state == 1)
	{
		*brightness_set =!(*brightness_set);
		btn_state=0;
	}

	// wenn Button "lange gedrückt" -> toggeln der "Blinken_Ein"-Variable
	if(btn_state == -1)
	{
		*blinky_on =!(*blinky_on);
		btn_state=0;
	}

	// wenn Helligkeits-Variable gesetzt, dann Helligkeitswert erhöhen
	if(*brightness_set)
	{
		if(*brightness_value<100)
		{
			*brightness_value+=((100/brightness_steps)+1);
			if(*brightness_value>100)
			*brightness_value=0;
		}
		else
			*brightness_value=0;

		*brightness_set=0;
	}
}

void Blinky(uint16_t on_time, uint16_t off_time, GPIO_TypeDef* GPIO_TypeDef_LED, uint16_t GPIO_Pin_LED, GPIO_TypeDef* GPIO_TypeDef_BUTTON, uint16_t GPIO_Pin_BUTTON, uint8_t brightness_steps)
{
	// Status-Variablen-Ledsteuerung
	static uint8_t blinky_on;
	static uint8_t brightness_set;
	static uint8_t brightness_value=0;

	// Setzen der Statusvariablen
	Set_States(&blinky_on, &brightness_set, &brightness_value, Button_Check(GPIO_TypeDef_BUTTON, GPIO_Pin_BUTTON, 1000), brightness_steps);

	// wenn Blink-Variable gesetzt
	if(blinky_on)
	{
		// Blinken mit dem eingestellten Intervall
		uint8_t dutycycle = 20;
		for(int i =0 ; i<on_time/dutycycle;i++)
		{
			HAL_GPIO_WritePin(GPIO_TypeDef_LED, GPIO_Pin_LED, GPIO_PIN_RESET);
			HAL_Delay((uint32_t)dutycycle*((float)brightness_value/100));
			HAL_GPIO_WritePin(GPIO_TypeDef_LED, GPIO_Pin_LED, GPIO_PIN_SET);
			HAL_Delay((uint32_t)dutycycle*((100-(float)brightness_value)/100));
			Set_States(&blinky_on, &brightness_set, &brightness_value, Button_Check(GPIO_TypeDef_BUTTON, GPIO_Pin_BUTTON, 1000), brightness_steps);
		}

		HAL_Delay(off_time);
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
