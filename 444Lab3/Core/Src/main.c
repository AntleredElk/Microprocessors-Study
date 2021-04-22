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
#define ARM_MATH_CM4
#define pi 3.14159265358979323846
#include "arm_math.h"

int signalSelection = 0;
int partNum = 3; //Chooses if you want to see part 1, 2, or 3

#define C6size 42 //Should be 43
#define E6size 32 //Should be 33
#define G6size 28 //Should be 28

uint16_t C6tone[C6size];
uint16_t E6tone[E6size];
uint16_t G6tone[G6size];

uint16_t buffer[672];
int bufferFlag = 0;

int C6index = 0;
int E6index = 0;
int G6index = 0;

uint16_t C6signal;
uint16_t E6signal;
uint16_t G6signal;

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
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
DAC_ChannelConfTypeDef sConfigGlobal; //For self-calibration

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/*
 *
 * DEFINE INTERRUPT HANDLERS
 *
 */

//GPIO Button
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == ButtOn_Pin) {

		HAL_GPIO_TogglePin(LED_GReeN_GPIO_Port, LED_GReeN_Pin);

		signalSelection = (signalSelection + 1)%3;

		bufferFlag = 1;
	}
}


//Timer 2
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim2 && partNum == 2) {

		//Increment everything
		C6index = (C6index + 1)%C6size;
		E6index = (E6index + 1)%E6size;
		G6index = (G6index + 1)%G6size;

		C6signal = C6tone[C6index];
		E6signal = E6tone[E6index];
		G6signal = G6tone[G6index];

		switch (signalSelection) {

			case 0:
				HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, C6tone[C6index]);
				break;
			case 1:
				HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, E6tone[E6index]);
				break;
			case 2:
				HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, G6tone[G6index]);
				break;
		}
	}
}


//DMA Complete
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
	if (bufferFlag) {

		switch (signalSelection) {

			case 0:
				for (int i = 0; i < 672; i++) {
					buffer[i] = C6tone[i%C6size];
				}
				break;
			case 1:
				for (int i = 0; i < 672; i++) {
					buffer[i] = E6tone[i%E6size];
				}
				break;
			case 2:
				for (int i = 0; i < 672; i++) {
					buffer[i] = G6tone[i%G6size];
				}
				break;

		}

		bufferFlag = 0;
	}

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
  MX_DMA_Init();
  MX_DAC1_Init();
  MX_TIM2_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  /*
   *
   * CHANNEL CALIBRATION
   *
   */

  HAL_DACEx_SelfCalibrate(&hdac1, &sConfigGlobal, DAC_CHANNEL_1);
  HAL_DACEx_SelfCalibrate(&hdac1, &sConfigGlobal, DAC_CHANNEL_2);



  /*
   *
   * INITIALIZATION
   *
   */

  //Manual signals
  uint32_t saw = 0; //Starting position for saw signal
  uint32_t triangle = 0; //Starting position for triangle signal
  int32_t incrementSaw = 7; //For the saw and triangle signals to have same frequency, use 7 and 15 as increments and only go up to a maximum of 105
  int32_t incrementTriangle = 15;
  uint32_t sawSignal; //Scaled signal to send to DAC
  uint32_t triangleSignal; //Scaled signal to send to DAC
  float32_t sineSignal = 0; //Scaled signal to send to DAC
  int sineIndex = 0; //Keep track/increment the index through the sineValues array to generate the signal
  int channel = 2; // Selects DAC channel


  //Timer interrupts/DAC
  if (partNum == 2) HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
  if (partNum == 1) HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
  if (partNum == 2 || partNum == 3) HAL_TIM_Base_Start_IT(&htim2); //Start the timer in interrupt mode



  /*
   *
   * SINE WAVE ARRAY GENERATION
   *
   * Calculate the x values inputted into the sine wave generation
   * This prevents choppy sine waves due to long float calculation speeds
   *
   */

  float32_t sineValues[15];
  for (int i = 0; i < 15; i++) {
	  sineValues[i] = (arm_sin_f32(2*pi*i/15)+1)*(0xFFF/2);
  }



  /*
   *
   * 3 FREQUENCY SIGNAL GENERATION (C6, E6, G6)
   *
   */

  for (int i = 0; i < C6size; i++) {
	  C6tone[i] = (uint16_t)(2*(arm_sin_f32(i*2*pi/C6size) + 1)*(0xFFF/2)/3);
  }

  for (int i = 0; i < E6size; i++) {
	  E6tone[i] = (uint16_t)(2*(arm_sin_f32(i*2*pi/E6size) + 1)*(0xFFF/2)/3);
  }

  for (int i = 0; i < G6size; i++) {
	  G6tone[i] = (uint16_t)(2*(arm_sin_f32(i*2*pi/G6size) + 1)*(0xFFF/2)/3);
  }



  /*
   *
   * INITIALIZE BUFFER AND START DMA DAC PROCESS
   *
   */

  for (int i = 0; i < 672; i++) {
	  buffer[i] = C6tone[i%C6size];
  }

  if (partNum == 3) HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)buffer, (uint32_t)672, DAC_ALIGN_12B_R); //672 = LCM(42, 32, 28)



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (partNum == 1) {

		  /*
		   *
		   * MANUAL SIGNAL GENERATION
		   *
		   */

		  //Generate saw signal
		  saw += incrementSaw;
		  incrementSaw = 7;

		  if (saw >= 105) {
			  incrementSaw = -105;
		  }

		  //Generate triangle signal
		  triangle += incrementTriangle;

		  if (triangle <= 0 || triangle >= 105) {
			  incrementTriangle *= -1;
		  }

		  //Increment the sine signal -> circular
		  sineIndex = (sineIndex + 1)%15;

		  //Scaling the signal to generate sound
		  sawSignal = (float)saw*4095/105;
		  triangleSignal = (float)triangle*4095/105;
		  sineSignal = sineValues[sineIndex];



		  /*
		   *
		   * SENDING CHOSEN SIGNAL TO DAC
		   *
		   */

		  if (channel == 1) {
			  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t)sawSignal); //Useless because we dont use channel 1 anymore for part 1
		  } else if (channel == 2) {
			  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (uint32_t)sawSignal);
		  }

		  HAL_Delay(1);
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
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_DISABLE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT2 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */
  sConfigGlobal = sConfig;
  /* USER CODE END DAC1_Init 2 */

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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1814;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GReeN_GPIO_Port, LED_GReeN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ButtOn_Pin */
  GPIO_InitStruct.Pin = ButtOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ButtOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_GReeN_Pin */
  GPIO_InitStruct.Pin = LED_GReeN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GReeN_GPIO_Port, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
