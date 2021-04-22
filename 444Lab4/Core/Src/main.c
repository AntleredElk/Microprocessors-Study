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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32l4s5i_iot01_accelero.h"
#include "stm32l4s5i_iot01_gyro.h"
#include "stm32l4s5i_iot01_hsensor.h"
#include "stm32l4s5i_iot01_magneto.h"
#include "stm32l4s5i_iot01_nfctag.h"
#include "stm32l4s5i_iot01_psensor.h"
#include "stm32l4s5i_iot01_qspi.h"
#include "stm32l4s5i_iot01_tsensor.h"
#include "stm32l4s5i_iot01.h"

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

UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
osThreadId myTask03Handle;
osMutexId msgMutexHandle;
osMutexId sensorSelectorMutexHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
void buttonPressTask(void const * argument);
void transmitDataTask(void const * argument);
void readSensorDataTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//#######################################################################################################################################



//Global Sensor Variables + Buffer
int sensorSelector = 0; // 0->Humidity,   1->Pressure,   2->Magnetometer,   3->Accelerometer

uint16_t sensorReading; //Not shared, exclusive to reading data only
int16_t sensorReadingXYZ[3];

char msg[100];



//#######################################################################################################################################
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
  //#######################################################################################################################################



  BSP_HSENSOR_Init(); // Initialize humidity sensor
  BSP_MAGNETO_Init(); // Initialize magnetometer
  BSP_PSENSOR_Init(); // Initialize pressure
  BSP_ACCELERO_Init(); // Initialize accelerometer



  //#######################################################################################################################################
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of msgMutex */
  osMutexDef(msgMutex);
  msgMutexHandle = osMutexCreate(osMutex(msgMutex));

  /* definition and creation of sensorSelectorMutex */
  osMutexDef(sensorSelectorMutex);
  sensorSelectorMutexHandle = osMutexCreate(osMutex(sensorSelectorMutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, buttonPressTask, osPriorityBelowNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, transmitDataTask, osPriorityNormal, 0, 128);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* definition and creation of myTask03 */
  osThreadDef(myTask03, readSensorDataTask, osPriorityNormal, 0, 128);
  myTask03Handle = osThreadCreate(osThread(myTask03), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //#######################################################################################################################################
  while (1)
  {

	  if(HAL_GPIO_ReadPin(button_GPIO_Port,button_Pin) == 0){ //If button is pressed
		  sensorSelector = (sensorSelector + 1)%4; //Change sensor
		  HAL_GPIO_TogglePin(led_GPIO_Port, led_Pin); //Flip LED
	  }

	  if(sensorSelector == 0){
		  sensorReading = (uint16_t)(BSP_HSENSOR_ReadHumidity());
		  sprintf(msg, "\rHumidity: %d                                                             ", sensorReading);
	  }

	  if(sensorSelector == 1){
		  sensorReading = (uint16_t)(BSP_PSENSOR_ReadPressure());
		  sprintf(msg, "\rPressure: %d                                                             ", sensorReading);
	  }

	  if(sensorSelector == 2){
		  BSP_MAGNETO_GetXYZ(&sensorReadingXYZ);
		  sprintf(msg, "\rMagnetometer X: %d, Magnetometer Y: %d, Magnetometer Z: %d               ", sensorReadingXYZ[0], sensorReadingXYZ[1], sensorReadingXYZ[2]);
	  }

	  if(sensorSelector == 3){
		  BSP_ACCELERO_AccGetXYZ(&sensorReadingXYZ);
		  sprintf(msg, "\rAccelerometer X: %d, Accelerometer Y: %d, Accelerometer Z: %d\0          ", sensorReadingXYZ[0], sensorReadingXYZ[1], sensorReadingXYZ[2]);
	  }

	  while(HAL_GPIO_ReadPin(button_GPIO_Port,button_Pin) == 0); //Wait until unpress
	  HAL_UART_Transmit(&huart1, &msg, sizeof(msg), HAL_MAX_DELAY);
	  HAL_Delay(100);





    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  //#######################################################################################################################################
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
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : button_Pin */
  GPIO_InitStruct.Pin = button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : led_Pin */
  GPIO_InitStruct.Pin = led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(led_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_buttonPressTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_buttonPressTask */
void buttonPressTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
//#######################################################################################################################################

  /* Infinite loop */
  for(;;)
  {
    osDelay(1);

    //When button has been pressed, update LED and pick next sensor
    if (HAL_GPIO_ReadPin(button_GPIO_Port,button_Pin) == 0) {

    	//Flip LED
    	HAL_GPIO_TogglePin(led_GPIO_Port, led_Pin);


    	//Change the shared resource: sensorSelector
    	osMutexWait(sensorSelectorMutexHandle, osWaitForever);
    	osMutexWait(msgMutexHandle, osWaitForever);
    	sensorSelector = (sensorSelector + 1)%4;
    	osMutexRelease(msgMutexHandle);
    	osMutexRelease(sensorSelectorMutexHandle);


    }
    while(HAL_GPIO_ReadPin(button_GPIO_Port,button_Pin) == 0); //Wait until unpress

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_transmitDataTask */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_transmitDataTask */
void transmitDataTask(void const * argument)
{
  /* USER CODE BEGIN transmitDataTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(100); //Put thread to sleep for 100ms for 10 Hz sampling time

    //Make sure msg is free to be used
    osMutexWait(msgMutexHandle, osWaitForever);

    //Transmit
    HAL_UART_Transmit(&huart1, &msg, sizeof(msg), HAL_MAX_DELAY);

    //Release msg
    osMutexRelease(msgMutexHandle);
  }
  /* USER CODE END transmitDataTask */
}

/* USER CODE BEGIN Header_readSensorDataTask */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_readSensorDataTask */
void readSensorDataTask(void const * argument)
{
  /* USER CODE BEGIN readSensorDataTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(100); //Put thread to sleep for 100ms for 10 Hz sampling time

	//Make sure sensorSelector and msg are free for being used
	osMutexWait(sensorSelectorMutexHandle, osWaitForever);
	osMutexWait(msgMutexHandle, osWaitForever);

	switch (sensorSelector) {
		case 0: // 0->Humidity

			sensorReading = (uint16_t)(BSP_HSENSOR_ReadHumidity());
			sprintf(msg, "\rHumidity: %d                                                             ", sensorReading);
			break;

		case 1: // 1->Pressure

			sensorReading = (uint16_t)(BSP_PSENSOR_ReadPressure());
			sprintf(msg, "\rPressure: %d                                                             ", sensorReading);
			break;

		case 2: // 2->Magnetometer

			BSP_MAGNETO_GetXYZ(&sensorReadingXYZ);
			sprintf(msg, "\rMagnetometer X: %d, Magnetometer Y: %d, Magnetometer Z: %d               ", sensorReadingXYZ[0], sensorReadingXYZ[1], sensorReadingXYZ[2]);
			break;

		case 3: // 3->Accelerometer

			BSP_ACCELERO_AccGetXYZ(&sensorReadingXYZ);
			sprintf(msg, "\rAccelerometer X: %d, Accelerometer Y: %d, Accelerometer Z: %d\0          ", sensorReadingXYZ[0], sensorReadingXYZ[1], sensorReadingXYZ[2]);
			break;

	}

	//Release sensorSelector and msg
	osMutexRelease(msgMutexHandle);
	osMutexRelease(sensorSelectorMutexHandle);


  }
  //#######################################################################################################################################

  /* USER CODE END readSensorDataTask */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
