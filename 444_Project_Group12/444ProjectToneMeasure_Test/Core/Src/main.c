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
#define ARM_MATH_CM0
#define pi 3.14159265358979323846
#include "arm_math.h"
#include "arm_const_structs.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ITM_Port32(n) (*((volatile unsigned long *) (0xE0000000+4*n)))
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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//############################################################################################################################################



int getFrequencies(arm_rfft_fast_instance_f32* handler, int sampling_rate ,float* buffer_in, float* buffer_out) {

	arm_rfft_fast_f32(handler, buffer_in, buffer_out,0);

	int frequencies[1024];
	int frequency_index = 0;
	int noise = 150; //variable noise offset
	int max = 0;
	int temp_max = 0;
	int frequency = 0;
	//calculate abs values and linear-to-dB
	for (int i=0; i<2048; i=i+2) {

		int real = buffer_out[i];
		int complex = buffer_out[i+1];
		int result = sqrt(real*real+complex*complex);

		frequencies[frequency_index] = (int)(20*log10f(result));
		if (frequencies[frequency_index]<0) frequencies[frequency_index]=0;

		temp_max = frequencies[frequency_index];

		if(frequency_index > 0 && temp_max > max){
			max = temp_max;
			frequency = (int) frequency_index*sampling_rate/2048;
		}
		frequency_index++;
	}
	return frequency;
}


int watcher;

//Sine wave generation function -> returns a pointer to the array stored in mem
//Returns arrays completely ready for sending through DAC to speaker!
//Size of the arrays are 4000 to mimic the received signal of the microphone
int* sineWaveGenerator(int freq, int soundClipSize) {

	//Sampling rate = 8k -> Does not generate tones with lengths that are different enough, lets do 16k, no 32k
	//Size of array = (sampling rate)/(freq)
	int arraySize = 32000/40;
	int* sineWave = malloc(arraySize*sizeof(int));

	for (int i = 0; i < soundClipSize; i++) {
		sineWave[i] = (int)((arm_sin_f32(i*2*pi/arraySize) + 1)*(4095/2));
	}

	return sineWave;
}

//Measure period function -> returns the period of the tone sent through it
//DOESNT WORK VERY WELL, RETURNS 10 for all c6, e6, g6
float measurePeriod(int* soundClip, int clipSize, int DCOffset) {

	int lastSign = 0;
	int sign = 0; //This variable indicates if the sound signal is below or above the DC Offset: 0 = above, 1 = below
	int periodCounter = 0;

	for (int i = 0; i < clipSize; i++) {

		if ( (soundClip[i] - DCOffset) >= 0 ) { //If above DCOffset
			sign = 0;
		} else {
			sign = 1;
		}

		if (lastSign != sign) {
			periodCounter++;
			lastSign = !lastSign;
		}
	}

	//Now get the length for one period of the sound wave
	return 2*(float)clipSize/(float)(periodCounter -1);

}

//Measure the frequency by fft
float32_t* measureFrequencyFFT(int* soundClip, int clipSize, int DCOffset) {

	arm_rfft_fast_instance_f32* init;
	float32_t* output = malloc(clipSize*sizeof(float32_t));

	//Initialize fft
	arm_rfft_fast_init_f32(&init, (uint16_t)clipSize); //Supports lengths that look like 2^something

	//Do fft
	arm_rfft_fast_f32(&init, (float32_t*)soundClip, output, 0);

	return output;
}


//############################################################################################################################################
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
//############################################################################################################################################

	/*
	 * SET FREQUENCY FOR TESTING HERE
	 */
	int test_frequency = 750;
	// USE this to test array
	int arraySize = 48000/test_frequency;
	int test_size = arraySize*sizeof(int);
	int* test = malloc(arraySize*sizeof(int));

	for (int i = 0; i < arraySize; i++) {
		test[i] = (int)((arm_sin_f32(i*2*pi/arraySize) + 1)*(4095/2));
	}

	float use_this_signal[] = {-0.193207, -0.169879, -0.14191, -0.11115, -0.0769944, -0.041726, -0.00486888, 0.0326922, 0.0682754, 0.100679, 0.132158, 0.163183, 0.190152, 0.215109, 0.238101, 0.255922, 0.269523, 0.277578, 0.281176, 0.280139, 0.274308, 0.26411, 0.247938, 0.226575, 0.201794, 0.1741, 0.143944, 0.113993, 0.0815677, 0.0450895, 0.00824386, -0.0285772, -0.0655515, -0.101127, -0.133882, -0.164063, -0.189885, -0.211087, -0.229038, -0.243586, -0.254433, -0.259993, -0.260314, -0.256299, -0.246047, -0.231419, -0.21384, -0.191256, -0.164803, -0.135724, -0.104817, -0.0725117, -0.0385573, -0.0037286, 0.0308825, 0.0668126, 0.102778, 0.135459, 0.166302, 0.194233, 0.217749, 0.23817, 0.252831, 0.264449, 0.272086, 0.27487, 0.27346, 0.265665, 0.254301, 0.239376, 0.219844, 0.195846, 0.168488, 0.137393, 0.103867, 0.0695678, 0.033984, -0.000156238, -0.0346218, -0.0678978, -0.0992622, -0.130934, -0.158279, -0.184168, -0.208552, -0.225724, -0.239788, -0.248848, -0.251982, -0.251309, -0.245821, -0.237838, -0.224257, -0.205344, -0.18453, -0.159445, -0.130611, -0.100406, -0.0679403, -0.0347117, -0.00101715, 0.0352383, 0.0697805, 0.102933, 0.135169, 0.164305, 0.191134, 0.213796, 0.233196, 0.250029, 0.263132, 0.272763, 0.276325, 0.273514, 0.265775, 0.253712, 0.237029, 0.216982, 0.194876, 0.168322, 0.139089, 0.106883, 0.0723815, 0.0376486, 0.00120259, -0.0346571, -0.0679164, -0.101604, -0.134166};
//############################################################################################################################################
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  float fft_in_buf[2048];
  float fft_out_buf[2048];

  arm_rfft_fast_instance_f32 fft_handler;
  arm_rfft_fast_init_f32(&fft_handler, 2048);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
//############################################################################################################################################


//	for (int i = 0; i < 35; i++) {
//		watcher = c6tone[i];
//		HAL_Delay(1);
//	}

//	float testPeriodForC6 = measurePeriod(c6tone, 4000, 2046);
//	float testPeriodForE6 = measurePeriod(e6tone, 4000, 2046);
//	float testPeriodForG6 = measurePeriod(g6tone, 4000, 2046);



//############################################################################################################################################


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  ITM_Port32(1);
	  int fft_in_ptr = 0;

	  for(int point = 0; point < 2048; point++){
		  int max_index = test_size/sizeof(test[0]);
		  fft_in_buf[fft_in_ptr] = test[point%max_index];
		  fft_in_ptr++;
	  }
	  int frequency = getFrequencies(&fft_handler, 48000, fft_in_buf, fft_out_buf);
	  ITM_Port32(2);

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
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
  RCC_OscInitStruct.PLL.PLLN = 60;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
