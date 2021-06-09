
#include "main.h"
#include "cmsis_os.h"

#include "mk_dht11.h"
//--------------------------------------------------
#define DS_PORT GPIOB
#define DS_PIN GPIO_PIN_2
#define Shift_PORT GPIOB
#define Shift_PIN GPIO_PIN_0
#define Clock_PORT GPIOB
#define Clock_PIN GPIO_PIN_1
#define Buzzer_PORT GPIOA
#define Buzzer_PIN GPIO_PIN_2


void napBit0()
{
	HAL_GPIO_WritePin(DS_PORT, DS_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Shift_PORT, Shift_PIN, GPIO_PIN_RESET);
	//HAL_Delay(20);
	HAL_GPIO_WritePin(Shift_PORT, Shift_PIN, GPIO_PIN_SET);
	//HAL_Delay(20);
	HAL_GPIO_WritePin(Shift_PORT, Shift_PIN, GPIO_PIN_RESET);
}
void napBit1()
{
	HAL_GPIO_WritePin(DS_PORT, DS_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Shift_PORT, Shift_PIN, GPIO_PIN_RESET);
	//HAL_Delay(20);
	HAL_GPIO_WritePin(Shift_PORT, Shift_PIN, GPIO_PIN_SET);
	//HAL_Delay(20);
	HAL_GPIO_WritePin(Shift_PORT, Shift_PIN, GPIO_PIN_RESET);
}
void xoaBoNho()
{
	for(int i = 0;i < 8;i++)
	{
		napBit1();
	}
}
void led7DoanInit()
{
	HAL_GPIO_WritePin(DS_PORT, DS_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Shift_PORT, Shift_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Clock_PORT, Clock_PIN, GPIO_PIN_RESET);
	xoaBoNho();
}
void convert(uint8_t data)
{
	uint8_t x;
	for(int i = 0;i<7;i++)
	{
		if(data & 0x80)x = 1;
		else x = 0;
		if(x==0)
		{
			napBit1();
		}
		else
		{
			napBit0();
		}
		data = data<<1;
	}
}
void hienThi()
{
	HAL_GPIO_WritePin(Clock_PORT, Clock_PIN, GPIO_PIN_SET);
	//HAL_Delay(20);
	HAL_GPIO_WritePin(Clock_PORT, Clock_PIN, GPIO_PIN_RESET);
}
uint8_t data[] = {0b01111110,0b00001100,0b10110110,0b10011110,0b11001100,0b11011010,0b11111010,0b00001110,0b11111110,0b11011110};
int dataUpdate;
void show(float a)
{
	for(int j=1;j<=24;j++)
	{
		xoaBoNho();
		convert(data[(int)a/10]);
		hienThi();
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
		HAL_Delay(5);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

		xoaBoNho();
		convert(data[(int)a%10]);
		hienThi();
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
		HAL_Delay(5);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	}
}

void playBipBip()
{
	for(int i = 0;i < 10;i++)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
		HAL_Delay(100);
	}
}
//--------------------------------------------------

TIM_HandleTypeDef htim2;

osThreadId Task1Handle;
osThreadId Task2Handle;
osThreadId Task3Handle;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);

dht11_t dht;
float temp;
uint8_t keu = 0;
int main(void)
{

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM2_Init();
  //Khởi tạo DHT11:
  init_dht11(&dht, &htim2, GPIOB, GPIO_PIN_12);
  readDHT11(&dht);

  osThreadDef(Task1, StartDefaultTask, osPriorityIdle, 0, 128);
  Task1Handle = osThreadCreate(osThread(Task1), NULL);

  osThreadDef(Task2, StartTask02, osPriorityIdle, 1, 128);
  Task2Handle = osThreadCreate(osThread(Task2), NULL);

  osThreadDef(Task3, StartTask03, osPriorityIdle, 2, 128);
  Task3Handle = osThreadCreate(osThread(Task3), NULL);

  osKernelStart();

  while (1)
  {

  }

}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 32-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA2 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  for(;;)
  {
	  	osDelay(1);
	  	readDHT11(&dht);
	  	temp = dht.temperature;

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask01 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  //Tắt 2 LED:
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
  led7DoanInit();
  for(;;)
  {
      osDelay(1);
      show(temp);
  }

}

void StartTask03(void const * argument)
{
  //Tắt 2 LED:

	  int state = 0;
	  int lastState = 0;
	  uint32_t time = 0;
	  //uint32_t time_refresh;
	  //uint8_t sounded = 0;
	  for(;;)
	  {
		  if(temp < 27.0)
		  {
			  	state = 1;

			  //playBipBip();

		  }
		  else state = 0;
		  if(state !=  lastState)
		  {
		  	if(state == 1)
		  	{
		  		if(HAL_GetTick() - time > 5*60*1000) //Nếu nhiệt độ < 27 và từ lúc < 27 đến hiện tại lớn hơn 5p
		  		{
		  			time = HAL_GetTick();
		  		}

		  		//time_refresh = time;
		  	}
		  	lastState = state;
		  }
		  if(HAL_GetTick() - time <= 5000)
		  {
			  playBipBip();
		  }
	  }

}

void Error_Handler(void)
{

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
