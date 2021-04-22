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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//########################################################################################################################
//Includes and Defines   #################################################################################################
//########################################################################################################################



#define ITM_Port32(n) (*((volatile unsigned long *) (0xE0000000+4*n)))
#define ARM_MATH_CM4
#define ARM_MATH_CM0
#define pi 3.14159265358979323846
#define ASCII_ESC 27 //For special UART characters

#include "arm_math.h"
#include "arm_const_structs.h"
#include "stdlib.h" //For random number generator



//########################################################################################################################
//########################################################################################################################
//########################################################################################################################
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

osThreadId uartHandle;
osThreadId speakerHandle;
osThreadId checkCycleComplHandle;
osMutexId microphonePermissionsHandle;
/* USER CODE BEGIN PV */
//########################################################################################################################
//Global Variables   #####################################################################################################
//########################################################################################################################


//For Microphone
#define micArraySize 4096
uint32_t micVal = 0;				//Used for viewing raw microphone readings through SWV Trace
uint32_t micArrayIn[micArraySize];	//Buffer that holds the microphone raw data
uint32_t micArrayOut[micArraySize];	//Used for storing the amplified signal of the microphone raw data (sending recording thru DAC Speaker)
uint16_t counter = 0;				//Counter for viewing the raw microphone data in a single variable (micVal) (debugging)
int calibrationVal = 0;				//Saves the DC Offset of the first few microphone readings in this value
int isFirstHalfCycleComplete = 0; 	//Flag for completion of the first half cycle of DMA microphone data recording
int isSecondHalfCycleComplete = 0;	//Flag for second half cycle

//For getFrequency function
int measuredFreq; //USELESS
#define samplingRate 8000					//Chosen sampling rate for the microphone signal - also affects the FFT calculation
#define fftSize 2048						//FFT works with sizes of 2^ -> we like 2048 as a good size -> measures one HALF of the micArrayIn at a time
float fft_in_buf[fftSize];					//FFT input buffer -> sound signal goes into here (do we need this, or could we just send our original through the function?)
float fft_out_buf[fftSize];					//FFT output buffer -> the FFT places its output into this buffer (real, imag)
arm_rfft_fast_instance_f32 fft_handler;		//For initializing the FFT

//For UART
char screen[259];							//Screen to display via UART -> whole thing gets transmitted
char title[70];								//Welcome screen -> will be embedded into screen array above when the time comes
char liveToneAndGoal[70];					//Game screen -> will be embedded into screen array above when the time comes
uint8_t receivedKey;						//Stores the keypress from UART "Press any key to continue..."
int potentialPoints = 0;					//Potential points per round -> gets decremented by Timer 3

#define MAX_SCORE 50
int success = 0; 							//This turns to 1 when the voice freq matches the goal freq
int goalToneFreq = 1000;					//Random goal tone -> starts with 1000
int voiceToneFreq;							//Our measure freq
int goalToneFreqUART;						//Changes the range of possible values for sending to the UART
int voiceToneFreqUART;						//Changes the range of possible values for sending to the UART
int score = 0;								//Keeps track of current score of the user

//For DAC and Speaker
int state = 0;								//State machine: 0 = playing sound out of speaker, 1 = listening to microphone for desired pitch, 3 = game over
uint32_t* tone0;							//The 6 tones in the game stored in memory
uint32_t* tone1;
uint32_t* tone2;
uint32_t* tone3;
uint32_t* tone4;
uint32_t* tone5;
uint32_t* goalWave;							//Points to the chosen tone




//########################################################################################################################
//########################################################################################################################
//########################################################################################################################
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
void startUART(void const * argument);
void startSpeaker(void const * argument);
void checkCycleComplete(void const * argument);

static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//########################################################################################################################
//Microphone + Pitch Functions   ##########################################################################################
//########################################################################################################################



//Sets the ADC to read microphone
void setMicRead() {
	ADC_ChannelConfTypeDef sConfig = { 0 };
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
}


//Sets the ADC to read the potentiometer
void setPotentiometerRead() {
	ADC_ChannelConfTypeDef sConfig = { 0 };
	sConfig.Channel = ADC_CHANNEL_14;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
}


//Calibrates the microphone -> measures the initial DC offset of the microphone readings
void readAvgMicrophoneInput() {
	for (int i = 0; i < 100; i++) {
		//Record the first 100 values read by microphone and take the avg
		calibrationVal = calibrationVal + micArrayIn[i];
	}
	calibrationVal = calibrationVal/100;
}


/*
Use this to calculate the frequencies AND return the highest frequency value.Frequencies will always be HALF the size of the the original FFT input anf FFT output buffers.This is because the arm_rfft_fast_f32() method stores 2 coefficients into the output buffer, namely the real and imaginary components of the result. These 2 values are ADJACENT to each other. Output[i] would correspond to the real value and Output[i+1] would correspond to the imaginary. From there, we simply find the magnitude. Notice that the for loop increments by 2 to accomodate for both real and imaginary components. Use noise to normalize frequency values.
*/
int getFrequencyOfTone(arm_rfft_fast_instance_f32* handler, int sampling_rate ,float* buffer_in, float* buffer_out) {

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



//Sine wave generation function -> returns a pointer to the array stored in mem
//Returns arrays completely ready for sending through DAC to speaker! -> Range is 0-4095
//soundClipSize is the total length of the soundClip that you want to generate (if you want to send to the getFrequency function, use a size of 2048)
//If you want to be able to use the variable soundClipSize, make the for-loop to from 0-soundClipSize, right now the soundClipSize is useless!
uint32_t* sineWaveGenerator(int freq, int soundClipSize) {

	//Sampling rate = 8k
	//Size of array = (sampling rate)/(freq)
	int arraySize = samplingRate/freq;
	uint16_t* sineWave = malloc(arraySize*sizeof(uint16_t));

	for (int i = 0; i < arraySize; i++) {
		sineWave[i] = (uint16_t)((arm_sin_f32(i*2*pi/arraySize) + 1)*(4095/2));
	}

	return sineWave;
}



//########################################################################################################################
//########################################################################################################################
//########################################################################################################################


//########################################################################################################################
//UART Functions   #######################################################################################################
//########################################################################################################################



//Clear screen function
void screenClear() {
	char buffer[5];

	sprintf(buffer, "%c[2J", ASCII_ESC);
	HAL_UART_Transmit(&huart1, &buffer, sizeof(buffer), HAL_MAX_DELAY);
}


//Make cursor invisible
void invisibleCursor() {
	char buffer[6];

	sprintf(buffer, "%c[?25l", ASCII_ESC);
	HAL_UART_Transmit(&huart1, &buffer, sizeof(buffer), HAL_MAX_DELAY);
}

//Print welcome screen
void printWelcomeScreen() {
	sprintf(title, "                         Welcome To Karaoke!                          "); //70 long
	sprintf(screen, "\r//////////////////////////////////////////////////////////////////////////\r\n//%s//\r\n//////////////////////////////////////////////////////////////////////////\r\n\n\nPress any key to continue...", title);
	HAL_UART_Transmit(&huart1, &screen, sizeof(screen), HAL_MAX_DELAY);
}


//Print the game screen
void printGame(int goalToneFreq, int voiceToneFreq, int score) {

	screenClear();

	sprintf(liveToneAndGoal, "                                                                      "); //70 long

	liveToneAndGoal[goalToneFreq] = 'I';
	liveToneAndGoal[voiceToneFreq] = 'I';

	if (score < 100) {
			sprintf(screen, "\r//////////////////////////////////////////////////////////////////////////\r\n//%s//\r\n//////////////////////////////////////////////////////////////////////////\r\n\n\n\t+---\tSCORE:\t%d\t---+        ", liveToneAndGoal, score);
		} else {
			sprintf(screen, "\r//////////////////////////////////////////////////////////////////////////\r\n//%s//\r\n//////////////////////////////////////////////////////////////////////////\r\n\n\n\t+---\tSCORE:\t%d\t---+       ", liveToneAndGoal, score);
		}

	HAL_UART_Transmit(&huart1, &screen, sizeof(screen), HAL_MAX_DELAY);
}


//Print the new desired tone
void printNewDesiredTone(int goalToneFreq, int score) {

	screenClear();

	sprintf(liveToneAndGoal, "                       New Desired Tone: %d Hz                      ", goalToneFreq); //70 long - > New Desired Tone: 1234 Hz

	if (score < 100) {
			sprintf(screen, "\r//////////////////////////////////////////////////////////////////////////\r\n//%s//\r\n//////////////////////////////////////////////////////////////////////////\r\n\n\n\t+---\tSCORE:\t%d\t---+        ", liveToneAndGoal, score);
		} else {
			sprintf(screen, "\r//////////////////////////////////////////////////////////////////////////\r\n//%s//\r\n//////////////////////////////////////////////////////////////////////////\r\n\n\n\t+---\tSCORE:\t%d\t---+       ", liveToneAndGoal, score);
		}

	HAL_UART_Transmit(&huart1, &screen, sizeof(screen), HAL_MAX_DELAY);
}


//Print Game Over Screen
void printGameOver(int score) {

	screenClear();

	sprintf(liveToneAndGoal, "                              YOU WIN!!!                              "); //70 long

	if (score < 100) {
			sprintf(screen, "\r//////////////////////////////////////////////////////////////////////////\r\n//%s//\r\n//////////////////////////////////////////////////////////////////////////\r\n\n\n\t+---\tSCORE:\tGAME OVER\t---+  ", liveToneAndGoal, score);
		} else {
			sprintf(screen, "\r//////////////////////////////////////////////////////////////////////////\r\n//%s//\r\n//////////////////////////////////////////////////////////////////////////\r\n\n\n\t+---\tSCORE:\tGAME OVER\t---+ ", liveToneAndGoal, score);
		}

	HAL_UART_Transmit(&huart1, &screen, sizeof(screen), HAL_MAX_DELAY);
}



//########################################################################################################################
//########################################################################################################################
//########################################################################################################################


//########################################################################################################################
//Callbacks   ############################################################################################################
//########################################################################################################################



////Timer 2 and 3 Callback
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) { -----> This callback was added to the automatically generated callback at bottom
//
//	/*
//	 * Viewing Microphone Raw Reading (Debugging)
//	 */
////	micVal = micArrayIn[counter];
////	++counter;
////	counter = counter%micArraySize;
//
//	/*
//	 * UART Points Decrementer -> uses Timer 3 since Timer 2 is being used as source trigger for ADC in DMA mode
//	 * Frequency is 2Hz -> we start at 10 potential points and decrement by 1 points every 0.5 seconds
//	 */
//	if (htim == &htim3) {
//		if (potentialPoints > 0 && state == 1) potentialPoints = potentialPoints - 1;
//	}
//
//}


//ADC Complete DMA Cycle Callback
//When the full cycle is complete, we want to do calculations on the SECOND half the buffer
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {

	/*
	 * Amplification of raw microphone reading for sendin to DAC Speaker (Debugging)
	 */
//	for (int i = 0; i < 4000; ++i) {
//		micArrayOut[i] = (micArrayIn[i] - calibrationVal + 50) * 41; //41 = 4095/100
//	}

	/*
	 * Trigger the flag for the second half-cycle's completion
	 */
	 if (state == 1) isSecondHalfCycleComplete = 1;

}

//ADC Half DMA Cycle Callback
//When the half cycle is complete, we want to do calculations on the FIRST half of the buffer
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {

	/*
	 * Amplification for sending recording to DAC Speaker
	 */
//	for (int i = 4000; i < micArraySize; ++i) {
//		micArrayOut[i] = (micArrayIn[i] - calibrationVal + 50) * 41; //41 = 4095/100
//	}

	/*
	 * Trigger the flag for the first half-cycle's completion
	 */
	if (state == 1) isFirstHalfCycleComplete = 1;

}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
	/*
	 * Checking the timing of how long it takes to output a complete buffer through the DAC Speaker (Debugging)
	 */
//	ITM_Port32(31) = 33;


}



//########################################################################################################################
//########################################################################################################################
//########################################################################################################################
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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_DAC1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
//########################################################################################################################
//Main Before Loop   #####################################################################################################
//########################################################################################################################


	/*
	 * Basic Initialization
	 */
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED); 	//Calibrate ADC
	HAL_ADC_Start_DMA(&hadc1, micArrayIn, micArraySize); 	//Start ADC in DMA mode
	HAL_TIM_Base_Start_IT(&htim2); 							//Start timer 2 for ADC in DMA mode
	while(micArrayIn[micArraySize-1] == 0); 				//Delay abit so we can read some values through the mic
	readAvgMicrophoneInput(); 								//Read the DC offset of the mic
	//HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, micArrayOut, micArraySize,DAC_ALIGN_12B_R); //Start the DAC speaker for playing back recording

	arm_rfft_fast_init_f32(&fft_handler, fftSize); 			//Initialize the FFT handler (only once) with size 2048!!

	/*
	 * Generating the 6 game tones and storing in memory
	 */
	tone0 = sineWaveGenerator(goalToneFreq, 1000); //The 1000 soundClipSize is useless
	tone1 = sineWaveGenerator(goalToneFreq, 1000);
	tone2 = sineWaveGenerator(goalToneFreq, 1000);
	tone3 = sineWaveGenerator(goalToneFreq, 1000);
	tone4 = sineWaveGenerator(goalToneFreq, 1000);
	tone5 = sineWaveGenerator(goalToneFreq, 1000);

	/*
	 * Game Welcome Screen
	 */
	invisibleCursor();											//Make it so we don't see cursor -> cursor is UGLY
	screenClear(); 												//Start with fresh screen
	printWelcomeScreen();										//Self explanatory - come on!
	HAL_UART_Receive(&huart1, &receivedKey, 1, HAL_MAX_DELAY);	//Wait for key press before starting game
	screenClear();												//Start the game with a fresh screen
	HAL_TIM_Base_Start_IT(&htim3);								//Start Timer 3 for time-based point decrementation -> less time = more points


//########################################################################################################################
//########################################################################################################################
//########################################################################################################################
  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of microphonePermissions */
  osMutexDef(microphonePermissions);
  microphonePermissionsHandle = osMutexCreate(osMutex(microphonePermissions));

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
  /* definition and creation of uart */
  osThreadDef(uart, startUART, osPriorityNormal, 0, 300);
  uartHandle = osThreadCreate(osThread(uart), NULL);

  /* definition and creation of speaker */
  osThreadDef(speaker, startSpeaker, osPriorityIdle, 0, 4000);
  speakerHandle = osThreadCreate(osThread(speaker), NULL);

  /* definition and creation of checkCycleCompl */
  osThreadDef(checkCycleCompl, checkCycleComplete, osPriorityIdle, 0, 4000);
  checkCycleComplHandle = osThreadCreate(osThread(checkCycleCompl), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
//########################################################################################################################
//Main In Loop   #########################################################################################################
//########################################################################################################################



		//If the first half of the buffer is ready for calculation, send it through the getFreq's function
//		if (isFirstHalfCycleComplete) {
//
///*Assigns the input signal values to the FFT input buffer. Make sure to use a modulus operator to wrap back around to the begin of //the array so as to avoid HARD FAULTS. Once the input buffer is full, we can pass it to getfrequencies().
//*/
//			for (int i = 0; i < fftSize; i++){
//				fft_in_buf[i] = micArrayIn[i];
//			}
//
//			voiceToneFreq = getFrequencyOfTone(&fft_handler, samplingRate, fft_in_buf, fft_out_buf);
//			isFirstHalfCycleComplete = 0;
//
//		} else if (isSecondHalfCycleComplete) {
//
//			for (int i = fftSize; i < micArraySize; i++){
//				fft_in_buf[i-fftSize] = micArrayIn[i];
//			}
//
//			voiceToneFreq = getFrequencyOfTone(&fft_handler, samplingRate, fft_in_buf, fft_out_buf);
//			isSecondHalfCycleComplete = 0;
//
//		} else if (state == 0) { //This is the speaker playing state -> transmit the desired tone through the DAC Speaker
//
//			//Print that we are playing through the speaker
//			printNewDesiredTone(goalToneFreq, score);
//
//			//Make the sine wave for the particular frequency
//			uint32_t* goalWave = sineWaveGenerator(goalToneFreq, 1000); //The 1000 soundClipSize is useless
//			int size = samplingRate/goalToneFreq;
//
//			//Turn off microphone so we dont listen to the tone while its playing
//			HAL_ADC_Stop_DMA(&hadc1);
//
//			//Play the sound thru the speaker for awhile
//			HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, goalWave, size, DAC_ALIGN_12B_R);
//			HAL_Delay(1000);
//			HAL_Delay(1000);
//			HAL_Delay(1000);
//			HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
//
//			//Turn mic back on
//			HAL_ADC_Start_DMA(&hadc1, micArrayIn, micArraySize);
//
//			state = 1;
//		}
//
//
//		if (success) {
//			//This generates a random tone between 950-2500Hz
//			//Available frequencies (with 8K sampling rate: 1000Hz, 1143Hz, 1333Hz, 1600Hz, 2000Hz, 2666Hz
//			int goalToneChoice = rand()%6;
//			switch (goalToneChoice) {
//				case 0:
//					goalToneFreq = 1000;
//					break;
//				case 1:
//					goalToneFreq = 1142;
//					break;
//				case 2:
//					goalToneFreq = 1333;
//					break;
//				case 3:
//					goalToneFreq = 1600;
//					break;
//				case 4:
//					goalToneFreq = 2000;
//					break;
//				case 5:
//					goalToneFreq = 2666;
//					break;
//			}
//			//goalToneFreq = rand()%1550 + 950; //No longer just random freq between 950-2500
//			voiceToneFreq = 0;
//			score = score + potentialPoints;
//			potentialPoints = 10;
//			state = 0; //Print new tone on screen + play thru speaker
//			success = 0;
//		  }
//		goalToneFreqUART = (goalToneFreq - 970)/25; //Reason for this is explained below
//
//		//Change the range of the voiceToneFreq from actual frequency to 0-66
//		//Lets say we can whistle from 970-2700 -> # of possible values = 1730
//		//We need to map 1730 possible values to 70 possible UART positions
//		//Each UART position will represent 24.71 frequencies -> we could split this up into different sections maybe?
//		voiceToneFreqUART = (voiceToneFreq - 970)/25;
//		if (voiceToneFreqUART >= 70) voiceToneFreqUART = 69; //Must remain between 0-69 because UART game screen is only 70 characters long
//		if (voiceToneFreqUART < 0) voiceToneFreqUART = 0;
//
//
//		printGame(goalToneFreqUART, voiceToneFreqUART, score);
//		HAL_Delay(11); 										//The UART needs delay or else it acts fkn weird -> min delay = 11ms through trial and error
//
//		if (goalToneFreqUART == voiceToneFreqUART) success = 1;





//########################################################################################################################
//########################################################################################################################
//########################################################################################################################
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* ADC1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(ADC1_IRQn);
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* TIM3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T2_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  /* USER CODE BEGIN DAC1_Init 2 */

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
  htim2.Init.Period = 15000;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 6000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : MICROPHONE_D_Pin */
  GPIO_InitStruct.Pin = MICROPHONE_D_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MICROPHONE_D_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_startUART */
/**
  * @brief  Function implementing the uart thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_startUART */
void startUART(void const * argument)
{
  /* USER CODE BEGIN 5 */
//########################################################################################################################
//Dedicated Task for UART  ###############################################################################################
//########################################################################################################################



  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
    while(1){

		if (success) {
			//This generates a random tone between 950-2500Hz
			//Available frequencies (with 8K sampling rate: 1000Hz, 1143Hz, 1333Hz, 1600Hz, 2000Hz, 2666Hz
			int goalToneChoice = rand()%6;
			switch (goalToneChoice) {
				case 0:
					goalToneFreq = 1000;
					goalWave = tone0;
					break;
				case 1:
					goalToneFreq = 1142;
					goalWave = tone1;
					break;
				case 2:
					goalToneFreq = 1333;
					goalWave = tone2;
					break;
				case 3:
					goalToneFreq = 1600;
					goalWave = tone3;
					break;
				case 4:
					goalToneFreq = 2000;
					goalWave = tone4;
					break;
				case 5:
					goalToneFreq = 2666;
					goalWave = tone5;
					break;
			}
			//goalToneFreq = rand()%1550 + 950; //No longer just random freq between 950-2500
			voiceToneFreq = 0;
			score = score + potentialPoints;
			potentialPoints = 10;
			state = 0; //Print new tone on screen + play thru speaker
			success = 0;
			if (score > MAX_SCORE) state = 2; //End the game when score is reached
		  }
		goalToneFreqUART = (goalToneFreq - 970)/25; //Reason for this is explained below

		//Change the range of the voiceToneFreq from actual frequency to 0-66
		//Lets say we can whistle from 970-2700 -> # of possible values = 1730
		//We need to map 1730 possible values to 70 possible UART positions
		//Each UART position will represent 24.71 frequencies -> we could split this up into different sections maybe?
		voiceToneFreqUART = (voiceToneFreq - 970)/25;
		if (voiceToneFreqUART >= 70) voiceToneFreqUART = 69; //Must remain between 0-69 because UART game screen is only 70 characters long
		if (voiceToneFreqUART < 0) voiceToneFreqUART = 0;

		//Based on state (speaker or microphone use) print one screen or the other
		if (state == 0) {
			//Print that we are playing through the speaker when playing through the speaker
			printNewDesiredTone(goalToneFreq, score);
		} else if (state == 1) {
			printGame(goalToneFreqUART, voiceToneFreqUART, score);
		} else {
			printGameOver(score);
		}

		osDelay(11); 										//The UART needs delay or else it acts fkn weird -> min delay = 11ms through trial and error
		//osMutexRelease(printingPermissionsHandle);
		if (goalToneFreqUART == voiceToneFreqUART) success = 1;



    }
  }



//########################################################################################################################
//########################################################################################################################
//########################################################################################################################
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_startSpeaker */
/**
* @brief Function implementing the speaker thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startSpeaker */
void startSpeaker(void const * argument)
{
  /* USER CODE BEGIN startSpeaker */
//########################################################################################################################
//Dedicated Task for Speaker Output  #####################################################################################
//########################################################################################################################



  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
    while(1){
    	osMutexWait(microphonePermissionsHandle, osWaitForever);
    	if (state == 0) { //This is the speaker playing state -> transmit the desired tone through the DAC Speaker

    	    			//Make the sine wave for the particular frequency

    	    			int size = samplingRate/goalToneFreq;

    	    			//Turn off microphone so we dont listen to the tone while its playing
    	    			HAL_ADC_Stop_DMA(&hadc1);

    	    			//Play the sound thru the speaker for awhile
    	    			HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, goalWave, size, DAC_ALIGN_12B_R);
    	    			osDelay(1000);
    	    			osDelay(1000);
    	    			osDelay(1000);
    	    			//osMutexRelease(printingPermissionsHandle);
    	    			HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);

    	    			//Turn mic back on
    	    			HAL_ADC_Start_DMA(&hadc1, micArrayIn, micArraySize);

    	    			state = 1;
    	    		}
    	osMutexRelease(microphonePermissionsHandle);
    }
  }



//########################################################################################################################
//########################################################################################################################
//########################################################################################################################
  /* USER CODE END startSpeaker */
}

/* USER CODE BEGIN Header_checkCycleComplete */
/**
* @brief Function implementing the checkCycleCompl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_checkCycleComplete */
void checkCycleComplete(void const * argument)
{
  /* USER CODE BEGIN checkCycleComplete */
//########################################################################################################################
//Dedicated Task for Signal Processing  ##################################################################################
//########################################################################################################################



  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
    //If the first half of the buffer is ready for calculation, send it through the getFreq's function
    while(1){
    	osMutexWait(microphonePermissionsHandle, osWaitForever);
    	if (isFirstHalfCycleComplete) {

/*Assigns the input signal values to the FFT input buffer. Make sure to use a modulus operator to wrap back around to the begin of the array so as to avoid HARD FAULTS. Once the input buffer is full, we can pass it to getfrequencies().
*/
    	    			for (int i = 0; i < fftSize; i++){
    	    				fft_in_buf[i] = micArrayIn[i];
    	    			}

    	    			voiceToneFreq = getFrequencyOfTone(&fft_handler, samplingRate, fft_in_buf, fft_out_buf);
    	    			isFirstHalfCycleComplete = 0;

    	    		} else if (isSecondHalfCycleComplete) {

    	    			for (int i = fftSize; i < micArraySize; i++){
    	    				fft_in_buf[i-fftSize] = micArrayIn[i];
    	    			}

    	    			voiceToneFreq = getFrequencyOfTone(&fft_handler, samplingRate, fft_in_buf, fft_out_buf);
    	    			isSecondHalfCycleComplete = 0;

    	    		}
    	osMutexRelease(microphonePermissionsHandle);
    }

  }



//########################################################################################################################
//########################################################################################################################
//########################################################################################################################
  /* USER CODE END checkCycleComplete */
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
	/*
	 * Viewing Microphone Raw Reading (Debugging)
	 */
//	micVal = micArrayIn[counter];
//	++counter;
//	counter = counter%micArraySize;

	/*
	 * UART Points Decrementer -> uses Timer 3 since Timer 2 is being used as source trigger for ADC in DMA mode
	 * Frequency is 2Hz -> we start at 10 potential points and decrement by 1 points every 0.5 seconds
	 */
	if (htim == &htim3) {
		if (potentialPoints > 0 && state == 1) potentialPoints = potentialPoints - 1;
	}
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
	while (1) {
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
