/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

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
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

osThreadId UART_TaskHandle;
osThreadId SPI_SlaveTaskHandle;
osThreadId SPI_DacTaskHandle;
osThreadId SPI_ReceiveHostHandle;
osMessageQId SlaveToDAC_QueueHandle;
osMessageQId BufferQueueHandle;
osMessageQId SPIQueueHandle;
osSemaphoreId SaveToFlashBinarySemHandle;
/* USER CODE BEGIN PV */
osSemaphoreId SPI3_BinarySemHandle;
FLASH_EraseInitTypeDef hflash;
extern FlashMapTypeDef FlashMap;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_DMA_Init(void);
void StartUART_Task(void const * argument);
void StartSPI_Slave(void const * argument);
void StartSPI_DAC(void const * argument);
void SPI_ReceiveHostTask(void const * argument);

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
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  MX_DMA_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of SaveToFlashBinarySem */
  osSemaphoreDef(SaveToFlashBinarySem);
  SaveToFlashBinarySemHandle = osSemaphoreCreate(osSemaphore(SaveToFlashBinarySem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  osSemaphoreDef(USART1_BinarySem);
  USART1_BinarySemHandle = osSemaphoreCreate(osSemaphore(USART1_BinarySem), 1);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of SlaveToDAC_Queue */
  osMessageQDef(SlaveToDAC_Queue, 4, uint32_t);
  SlaveToDAC_QueueHandle = osMessageCreate(osMessageQ(SlaveToDAC_Queue), NULL);

  /* definition and creation of BufferQueue */
  osMessageQDef(BufferQueue, 4, uint8_t);
  BufferQueueHandle = osMessageCreate(osMessageQ(BufferQueue), NULL);

  /* definition and creation of SPIQueue */
  osMessageQDef(SPIQueue, 4, uint32_t);
  SPIQueueHandle = osMessageCreate(osMessageQ(SPIQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of UART_Task */
  osThreadDef(UART_Task, StartUART_Task, osPriorityBelowNormal, 0, 128);
  UART_TaskHandle = osThreadCreate(osThread(UART_Task), NULL);

  /* definition and creation of SPI_SlaveTask */
  osThreadDef(SPI_SlaveTask, StartSPI_Slave, osPriorityAboveNormal, 0, 128);
  SPI_SlaveTaskHandle = osThreadCreate(osThread(SPI_SlaveTask), NULL);

  /* definition and creation of SPI_DacTask */
  osThreadDef(SPI_DacTask, StartSPI_DAC, osPriorityNormal, 0, 128);
  SPI_DacTaskHandle = osThreadCreate(osThread(SPI_DacTask), NULL);

  /* definition and creation of SPI_ReceiveHost */
  osThreadDef(SPI_ReceiveHost, SPI_ReceiveHostTask, osPriorityNormal, 0, 128);
  SPI_ReceiveHostHandle = osThreadCreate(osThread(SPI_ReceiveHost), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  //DAC_channel_init();
  //Flash_ReadParams(); 
  NVIC_EnableIRQ(SPI3_IRQn);
  uint32_t temp1 = VoltInterpolation(0x01);
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  while (1)
  {
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
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
  hspi2.Init.Direction = SPI_DIRECTION_1LINE;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_SLAVE;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RED_LED_Pin|SPI2_MASTER_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RED_LED_Pin SPI2_MASTER_CS_Pin */
  GPIO_InitStruct.Pin = RED_LED_Pin|SPI2_MASTER_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
		static hostDataTypeDef xHostDataSlave;
  static portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  uint8_t data_rx[3];
		if (GPIO_Pin == SPI3_SLAVE_CS_Pin)
		{
				
    
				if (HAL_SPI_Receive(&hspi3, data_rx, sizeof(uint8_t) * 3, 1000) == HAL_OK)
						{
								
								xHostDataSlave.command = data_rx[0];
								xHostDataSlave.channel = data_rx[1];
        xHostDataSlave.data = data_rx[2];
								//printf("%d", data_rx[0]);
        HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
								xQueueSendFromISR(SPIQueueHandle, &xHostDataSlave, &xHigherPriorityTaskWoken);
						}
						taskYIELD();
		}

}

void USART3_IRQHandler(void)
{

}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartUART_Task */
/**
  * @brief  Function implementing the UART_Task thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartUART_Task */
void StartUART_Task(void const * argument)
{
  /* USER CODE BEGIN 5 */
		char data_rx[256];
		double receive_coeff;
  /* Infinite loop */
  for(;;)
  {
    taskENTER_CRITICAL();
				HAL_UART_Receive(&huart1, (uint8_t*)data_rx, 10, 1000);
    taskEXIT_CRITICAL();
				//printf("Str %s\n", data_rx);

				switch(data_rx[0])
				{
						case('1'):
        receive_coeff = atof(&data_rx[2]);
        FlashMap.coeff_a = receive_coeff;
								memset(data_rx, 0, sizeof(data_rx));
								break;
						case('2'):
								receive_coeff = atof(&data_rx[2]);
        FlashMap.coeff_b = receive_coeff;
								memset(data_rx, 0, sizeof(data_rx));
								break;
						case('3'):
								receive_coeff = atof(&data_rx[2]);
        FlashMap.coeff_c = receive_coeff;
								memset(data_rx, 0, sizeof(data_rx));
								break;
						case('4'):
								/**Create save serial number*/
						case('5'):
								WriteStruct_to_FLASH ();
								break;
						default:
							break;
				}
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_StartSPI_Slave */
/**
* @brief Function implementing the SPI_SlaveTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSPI_Slave */
void StartSPI_Slave(void const * argument)
{
  /* USER CODE BEGIN StartSPI_Slave */
  hostDataTypeDef xHostDataFromSPI;
  portBASE_TYPE xStatus;
  /* Infinite loop */
  for(;;)
  {
				if (xQueueReceive(SPIQueueHandle, &xHostDataFromSPI, 100) == pdTRUE)
				{
						//printf("SlaveSPI\n");
						switch (xHostDataFromSPI.command)
						{
								case CHECK_CONNECTION: //Check connection
										if ((xStatus = xQueueSend(BufferQueueHandle, &xHostDataFromSPI, 100)) != errQUEUE_FULL)
												taskYIELD();
										break;
								case RECEIVE_SERIAL_NUM: //Receive serial number
										if ((xStatus = xQueueSend(BufferQueueHandle, &xHostDataFromSPI, 100)) != errQUEUE_FULL) 
												taskYIELD();
										break;
								case TRANSMIT_TO_DAC: //Transmit to DAC
										//if ((xStatus = xQueueSend(BufferQueueHandle, &xHostDataSlave, 100)) != errQUEUE_FULL)
												if ((xStatus = xQueueSend(SlaveToDAC_QueueHandle, &xHostDataFromSPI, 100)) != errQUEUE_FULL)
										break;
								default:
										taskYIELD();
						}
						taskYIELD();
				}
		}
  /* USER CODE END StartSPI_Slave */
}

/* USER CODE BEGIN Header_StartSPI_DAC */
/**
* @brief Function implementing the SPI_DacTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSPI_DAC */
void StartSPI_DAC(void const * argument)
{
  /* USER CODE BEGIN StartSPI_DAC */
  hostDataTypeDef xHostData;
		uint8_t DAC_setup;
  /* Infinite loop */
  for(;;)
  {
				if (xQueueReceive(SlaveToDAC_QueueHandle, &xHostData, 10) == pdPASS)
				{
						taskENTER_CRITICAL();
						printf("Data from SPI %d\n", xHostData.data);
						if (xHostData.data > 0)
								DAC_setup = VoltInterpolation((double)xHostData.data);
						else
								DAC_setup = 0;
						//DAC_SPI_Transmit(xHostData.data_1, DAC_WRITE_N_AND_UPDATE_ALL, xHostData.data_0);
      DAC_SPI_Transmit(DAC_setup, DAC_WRITE_N_AND_UPDATE_ALL, xHostData.channel);
						printf("Data in DAC %d\n", DAC_setup);
      taskEXIT_CRITICAL();
				}
  }
  /* USER CODE END StartSPI_DAC */
}

/* USER CODE BEGIN Header_SPI_ReceiveHostTask */
/**
* @brief Function implementing the SPI_ReceiveHost thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SPI_ReceiveHostTask */
void SPI_ReceiveHostTask(void const * argument)
{
  /* USER CODE BEGIN SPI_ReceiveHostTask */
  hostDataTypeDef xHostData;
  FlashMapTypeDef FlashMap;
  /* Infinite loop */
  for(;;)
  {
				if ((xQueueReceive(BufferQueueHandle, &xHostData, 100) == pdPASS) || (HAL_GPIO_ReadPin(SPI3_SLAVE_CS_GPIO_Port, SPI3_SLAVE_CS_Pin) == 0))
				{
						switch (xHostData.command)
						{
								case (RECEIVE_SERIAL_NUM):
										HAL_SPI_Transmit(&hspi3, &FlashMap.serial_number, sizeof(uint8_t), 100);
										taskYIELD();
										break;
								default:
										HAL_SPI_Transmit(&hspi3, &xHostData.command, sizeof(uint8_t), 100);
          taskYIELD();
										break;
						}
				}
    osDelay(1);
  }
  /* USER CODE END SPI_ReceiveHostTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM18 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM18) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
