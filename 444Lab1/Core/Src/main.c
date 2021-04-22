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
#include <math.h>
#include <time.h>

#define ARM_MATH_CM4
#include <arm_math.h>
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
/* USER CODE BEGIN PFP */






#define TEST_ARRAY {10.4915760032, 10.1349974709, 9.53992591829, 9.60311878706, 10.4858891793, 10.1104642352, 9.51066931906, 9.75755656493, 9.82154078273, 10.2906541933, 10.4861328671, 9.57321181356, 9.70882714139, 10.4359069357, 9.70644021369, 10.2709894039, 10.0823149505, 10.2954563443, 9.57130449017, 9.66832136479, 10.4521677502, 10.4287240667, 10.1833650752, 10.0066049721, 10.3279461634, 10.4767210803, 10.3790964606, 10.1937408814, 10.0318963522, 10.4939180917, 10.2381858895, 9.59703103024, 9.62757986516, 10.1816981174, 9.65703773168, 10.3905666599, 10.0941977598, 9.93515274393, 9.71017053437, 10.0303874259, 10.0173504397, 9.69022731474, 9.73902896102, 9.52524419732, 10.3270730526, 9.54695650657, 10.3573960542, 9.88773266876, 10.1685038683, 10.1683694089, 9.88406620159, 10.3290065898, 10.2547227265, 10.4733422906, 10.0133952458, 10.4205693583, 9.71335255372, 9.89061396699, 10.1652744131, 10.2580948608, 10.3465431058, 9.98446410493, 9.79376005657, 10.202518901, 9.83867150985, 9.89532986869, 10.2885062658, 9.97748768804, 10.0403923759, 10.1538911808, 9.78303667556, 9.72420149909, 9.59117495073, 10.1716116012, 10.2015818969, 9.90650056596, 10.3251329834, 10.4550120431, 10.4925749165, 10.1548177178, 9.60547133785, 10.4644672766, 10.2326496615, 10.2279703226, 10.3535284606, 10.2437410625, 10.3851531317, 9.90784804928, 9.98208344925, 9.52778805729, 9.69323876912, 9.92987312087, 9.73938925207, 9.60543743477, 9.79600805462, 10.4950988486, 10.2814361401, 9.7985283333, 9.6287888922, 10.4491538991, 9.5799256668}

typedef struct kalman_state{
    float q; //process noise covariance
    float r; //measurement noise covariance
    float x; //estimated value
    float p; //estimation error covariance
    float k; // adaptive Kalman filter gain
}kalman_state;

/*
 * Wrapper function for testing
 */
int Kalmanfilter(float* InputArray, float* OutputArray, kalman_state* kstate, int Length);

/*
 * Assembly update function
 * Returns 0 if success
 * Returns -1 if not success (overflow, underflow, division by zero...)
 */
extern int kalmanASS(kalman_state* kstate, float);

/*
 * C update function
 * Returns 0 if success
 * Returns -1 if not success (overflow, underflow, division by zero...)
 */
int kalmanC(kalman_state* kstate, float measurement);

/*
 * Math Functions for statistical analysis
 */
void updateFilter(kalman_state* kstate, float measurement);
float* subtraction(float* differences, float* InputArray, float* OutputArray ,int Length);
float standardDeviation(float* differences, float Length);
float correlation(float* correlationArray, float* InputArray, float* OutputArray, int Length);
float convolution(float* convolutionArray, float* InputArray, float* OutputArray, int Length);
float mean(float* array, int Length);
float summation(float* array, int Length, float mean);
float summation2(float* array, int Length, float mean);




/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int Kalmanfilter(float* InputArray, float* OutputArray, kalman_state* kstate, int Length) {

	/*------------------------------------------------------------------------------------------
	 * Generating outputs with assembly + calculating with C------------------------------------
	 ------------------------------------------------------------------------------------------*/
	int result;
	// Create the output array with the assembly function
	clock_t start = clock(); //Measure time
	for(int position = 0; position < Length; position++){
		result = kalmanASS(kstate, InputArray[position]);
        OutputArray[position] = kstate->x;
    }

	if (result == -1) {
		return result;
	}

	 // Subtraction
	float differences1[Length];
	subtraction(differences1, InputArray, OutputArray, Length);

	// Standard Deviation
	float standardDeviationValue1;
	standardDeviationValue1 = standardDeviation(differences1, Length);

    // Correlation Vector
    float correlationArray1[Length];
    correlation(correlationArray1, InputArray, OutputArray, Length);

    // Convolution Vector
    float convolutionArray1[Length];
    convolution(convolutionArray1, InputArray, OutputArray, Length);

	clock_t end = clock(); //Measure time
	double time_spent1 = ((double)(end - start))/(double)CLOCKS_PER_SEC;
	//------------------------------------------------------------------------------------------



	/*------------------------------------------------------------------------------------------
	 * Generating outputs with C + calculating with C-------------------------------------------
	 -------------------------------------------------------------------------------------------*/
	// Create the output array with the assembly function
	start = clock(); //Measure time
	for(int position = 0; position < Length; position++){
		result = kalmanC(kstate, InputArray[position]);
		OutputArray[position] = kstate->x;
	}

	if (result == -1) {
		return result;
	}

	 // Subtraction
	float differences2[Length];
	subtraction(differences2, InputArray, OutputArray, Length);

	// Standard Deviation
	float standardDeviationValue2;
	standardDeviationValue2 = standardDeviation(differences2, Length);

	// Correlation
	float correlationArray2[Length];
	correlation(correlationArray2, InputArray, OutputArray, Length);

    // Convolution Vector
    float convolutionArray2[Length];
    convolution(convolutionArray2, InputArray, OutputArray, Length);

	end = clock(); //Measure time
	double time_spent2 = ((double)(end - start))/(double)CLOCKS_PER_SEC;
	//------------------------------------------------------------------------------------------



	/*------------------------------------------------------------------------------------------
	 * Generating outputs with assembly + calculating with CMSIS-DSP----------------------------
	 ------------------------------------------------------------------------------------------*/
	// Create the output array with the assembly function
	start = clock(); //Measure time
	for(int position = 0; position < Length; position++){
		int result = kalmanASS(kstate, InputArray[position]);
		OutputArray[position] = kstate->x;
	}
	float differences3[Length];

	//Subtraction
	arm_sub_f32(InputArray, OutputArray, differences3, Length);

	//Mean
	float mean;
	arm_mean_f32(differences3, Length, &mean);

	//Standard Deviation
	float stddev;
	arm_std_f32(differences3, Length, &stddev);

	//Correlation
	float corr[(2*Length - 1)];
	arm_correlate_f32(InputArray, Length, OutputArray, Length, &corr);

	//Convolution
	float conv[(2*Length - 1)];
	arm_conv_f32(InputArray, Length, OutputArray, Length, &conv);


	end = clock(); //Measure time
	double time_spent3 = ((double)(end - start))/(double)CLOCKS_PER_SEC;
	//------------------------------------------------------------------------------------------

    return result;
}

int kalmanC(kalman_state* kstate, float measurement){
	//Typical Kalman Filter update
	kstate->p = kstate->p + kstate->q;
	kstate->k = kstate->p/(kstate->p + kstate->r);
	kstate->x = kstate->x + kstate->k * (measurement - kstate->x);
	kstate->p = (1 - kstate->k) * kstate->p;

	//Checking for NaN's
	if isnan(kstate->p) {
		return -1;
	}
	else if isnan(kstate->k) {
		return -1;
	}
	else if isnan(kstate->x) {
		return -1;
	}
	else if isnan(kstate->r) {
		return -1;
	}
	else {
		return 0;
	}
}

float* subtraction(float* differences, float* InputArray, float* OutputArray ,int Length){
    for(int position = 0; position < Length; position++){
        float difference = abs(InputArray[position] - OutputArray[position]);
        differences[position] = difference;
    }
    return differences;
}

float standardDeviation(float* differences, float Length){
    float meanValue = mean(differences, Length);
    float sum = 0;
    sum = summation2(differences,Length, meanValue);
    float standardDeviation = sqrt(sum/Length);
    return standardDeviation;
}

float correlation(float* correlationArray, float* InputArray, float* OutputArray, int Length){
/*    float correlation = 0;
    float inputMean = 0;
    float outputMean = 0;
    float sum1 = 0;
    float sum2 = 0;
    float sum3 = 0;
    float sum4 = 0;

    inputMean = mean(InputArray, Length);
    outputMean = mean(OutputArray, Length);

    for(int position = 0; position < Length; position++) {
        sum1 += InputArray[position] - inputMean;
        sum2 += OutputArray[position] - outputMean;
        sum3 += pow((InputArray[position] - inputMean), 2);
        sum4 += pow((OutputArray[position] - outputMean), 2);
        correlation = (sum1 * sum2) / sqrt(sum3 * sum4);
        correlationArray[position] = correlation;
    }*/

    float correlation = 0;
    for(int position = 0; position < Length; position++){
    	correlation += InputArray[position]*OutputArray[(Length-1) - position];
        correlationArray[position] = correlation;
    }
    return correlation;
}

float convolution(float* convolutionArray, float* InputArray, float* OutputArray, int Length){
    float convolution = 0;
    for(int position = 0; position < Length; position++){
        convolution += InputArray[position]*OutputArray[position];
        convolutionArray[position] = convolution;
    }
    return convolution;
}

float mean(float* array, int Length){
    float mean = 0;
    for(int position = 0; position < Length; position++){
        mean += array[position]/Length;
    }
    return mean;
}

float summation(float* array, int Length, float mean){
    float sum = 0;
    for(int position = 0; position < Length; position++){
        sum += (array[position]-mean);
    }
    return sum;
};
float summation2(float* array, int Length, float mean){
    float sum = 0;
    for(int position = 0; position < Length; position++){
        sum += pow((array[position]-mean),2);
    }
    return sum;
};



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
  /* USER CODE BEGIN 2 */





  kalman_state  kstate = { 0.1, 0.1, 5, 0.1, 0.0 };
  float InputArray[] = TEST_ARRAY;
  int Length = (int)sizeof(InputArray)/sizeof(float);
  float OutputArray[Length];

  int result = Kalmanfilter(InputArray, OutputArray, &kstate, Length);




  /* USER CODE END 2 */

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
