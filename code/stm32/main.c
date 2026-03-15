/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body for EMKW group alpha
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
#include "ssd1306.h"
#include "fonts.h"
#include "stdio.h"

#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE 25  //buffer size for weight display
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// PID controller structure
typedef struct
{
	uint8_t init;	// initiation phase flag
	uint8_t suspend;	// suspend pid loop flag
	float Kp;  // proportional gain
	float Ki;  // integral gain
	float Kd;  // derivative gain
	float M;	// master multiplier
	const uint16_t w;	// setpoint value of adc input
	int32_t u;		// output
	const uint16_t u_min; // minimal output
	const uint16_t u_max; // maximum output
	float ui_old;	// last integral term
	float up;		// proportional term
	float ui;		// integral term
	float ud;		// derivative term
	int32_t e;		// error
	int32_t e_old;	// last error
	const float ta;	// sampling time in s
}PidController;

PidController Pid=
{
		.init = 1, // on @start
		.suspend = 1,	// on @start
		.Kp = 0.02250000,	// to be fine tuned
		.Ki = 2.25000000,	// to be fine tuned
		.Kd = 0.00155000,	// to be fine tuned
		.M = 1.05,	// to be fine tuned
		.w = 575,	// to be fine tuned
		.u = 0,	// not set @startup
		.u_min = 0,	// dutycycle = 0% @TIM2CCR2 = 0
		.u_max = 999,	// dutycycle = 100% @TIM2CCR2 = 999
		.ui_old = 0,	// not set @startup
		.e = 0,	// not set @startup
		.e_old = 0,	// not set @startup
		.ta = 0.00354	//  equals reciprocal pid loop frequency in s
};

// main program logic helpers structure
typedef struct
{
	uint8_t caseNum;	// switch case display management
	uint8_t antiBounceFlag;	// debouncing the buttons
	uint8_t initFlag;	// startup initialisation phase flag
	uint8_t landingFlag;	// soft descending the plate
	uint32_t counter;	// counter for calibration frequency reduction
	uint8_t timeoutCounter;	// counter for timeout
	uint16_t timeoutDuration;	// duration for timeout
	uint8_t averageLevelFlag;	// wanted level archieved flag
}ControllerLogic;

ControllerLogic Logic=
{
		.caseNum = 0,	// not set @startup
		.antiBounceFlag = 0,	// not set @startup
		.initFlag = 0,	// not set @startup
		.landingFlag = 0,	// not set @startup
		.counter = 0,	// not set @startup
		.timeoutCounter = 0,	// not set @startup
		.averageLevelFlag = 0,	// not set @startup
};

// sensor handling structure
typedef struct
{
	int32_t adcValue;	// post processed adc value
	int32_t adcValueRAW;	// raw adc value
	int32_t initialAdcValue;	// adc value @startup with inactive coil
}SensorHandling;

SensorHandling Sensor=
{
		.adcValue = 0,	// not set @startup
		.adcValueRAW = 0,	// not set @startup
		.initialAdcValue = 0	// not set @startup
};

// mass handling and processing structure
typedef struct
{
	uint16_t initialCCR;	// CCR value without mass
	uint16_t currentCCR;	// live CCR value
	const float gain;	// gain for fine tuning offset of the displayed mass
	float dutyCycle;	// live dutycycle in %
	char bufflt[10];
	float buffer[BUFFER_SIZE];	// buffer size processing
	uint8_t bufferIndex;	// buffer index
	uint16_t average;	// averge mass after buffering
	uint16_t lastAverage;	// last average mass
	uint16_t tare;	// tare mass
}MassHandling;

MassHandling Mass=
{
		.initialCCR = 0,	// not set @startup
		.currentCCR = 0,	// not set @startup
		.gain = 0.99f,	// to be fine tuned
		.dutyCycle = 0,	// not set @startup
		.bufflt = {0},	// not set @startup
		.buffer = {0},	// not set @startup
		.bufferIndex = 0,	// not set @startup
		.average = 0,	// not set @startup
		.lastAverage = 0,	// not set @startup
		.tare = 0	// not set @startup
};

// boot screen function
void BootScreen()
{
	SSD1306_Fill(0);	// clear screen on reboot
	SSD1306_GotoXY (6, 22);	// navigate to coordinates on the screen
	SSD1306_Puts ("EMKW ALPHA", &Font_11x18, 1);	// write text
	SSD1306_DrawLine(0, 14, 128, 14, 1);	// draw lines
	SSD1306_DrawLine(0, 45, 128, 45, 1);
	SSD1306_GotoXY (1, 53);	// navigate to coordinates on the screen
	SSD1306_Puts ("BHT BME WS24/25 PT ", &Font_7x10, 1);	// write text
	SSD1306_UpdateScreen();	// refresh screen with new contents
	HAL_Delay(5000);	// delay to keep boot screen for a moment
}

// initiate adc calibration function
void ZeroAdcValue()
{
	Logic.caseNum = 1;	// set display case 1 "calibrating"
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;	// change adc settings to poll value once
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	HAL_ADC_Init(&hadc1);	// initiate adc with new settings
	HAL_ADC_Start(&hadc1);		// start adc
	HAL_ADC_PollForConversion(&hadc1, 1);		// poll for adc value once
	Sensor.initialAdcValue = (HAL_ADC_GetValue(&hadc1) - 50);		// save initial adc value and compensate adc difference between modes
	if (Sensor.initialAdcValue < 0)
	{
		Sensor.initialAdcValue = 0;
	}
	HAL_ADC_Stop(&hadc1);		// stop adc
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;	// change adc settings back to timer controlled interrupt
	hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T4_TRGO;
	HAL_ADC_Init(&hadc1);	// initiate adc with original settings
}

// initiate CCR calibration function
void CalibInit()
{
	Logic.initFlag = 1;	// initiation flag active
	HAL_TIM_Base_Start_IT(&htim6);	// start TIM6 in interrupt mode
}

// initiate landing procedure function
void LandingInit()
{
	Pid.suspend = 1;	//suspend PID controller flag active
	Logic.landingFlag = 1;	//landing flag active
	HAL_TIM_Base_Start_IT(&htim6);  // start TIM6 in interrupt mode
}

// calibrate initial CCR value function in loop
void CCRCalibration()
{
	if (TIM2->CCR2 == 0)
	{
		TIM2->CCR2 = 49;	// set initial dutycycle of 5%
	}

	if (Sensor.adcValue < (Pid.w - 5))
	{
		TIM2->CCR2 = TIM2->CCR2 + 15;	// lift plate or new approach from above the setpoint
	}
	else if (Sensor.adcValue > Pid.w)
	{
		if (++Logic.counter % 5 == 0)  // reduce frequency by 5 for smoothing time
		{
			TIM2->CCR2 = TIM2->CCR2 - 1;  // slow descend to find zero error
		}
	}
	else if (Sensor.adcValue <= (Pid.w + 1) && Sensor.adcValue >= (Pid.w - 1))
	{
		Mass.initialCCR = (uint16_t) TIM2->CCR2;	// write value to CCR register
		HAL_TIM_Base_Stop_IT(&htim6);	// start TIM6 in interrupt mode
		Pid.init = 0;	// end pid initialization
		Logic.initFlag = 0;	// end startup phase
		LandingInit();	// initiate landing procedure
	}
}

// landing function for slow descend in loop after measurement
void PerformLanding()
{
	uint8_t step = 5;
	TIM2->CCR2 = TIM2->CCR2 - step;  // reduce CCR by step
	if (TIM2->CCR2 < step || Sensor.adcValue <= 1)  // if dutycycle is below step or sensor indicates plate is already low
	{
		TIM2->CCR2 = 0;  // make sure pwm is off
		Pid.ui = 0;
		HAL_TIM_Base_Stop_IT(&htim6);  // stop TIM6
		Logic.landingFlag = 0;	// end landing procedure
		Logic.caseNum = 2;	// set new display case "press to start"
	}
}

// process PID terms
void ProcessPIDs()
{
	Pid.e_old = Pid.e;	// set last error
	Pid.e = Pid.w - Sensor.adcValue;	// calculate new error

	Pid.up = Pid.M * Pid.Kp * Pid.e;	// calculate proportional term
	Pid.ud = Pid.M * Pid.Kd / Pid.ta * (Pid.e - Pid.e_old);	// calculate derivative term

	Pid.ui_old = Pid.ui;	// set last integral term
	Pid.ui = Pid.M * Pid.Ki * Pid.ta / 2.0f * (Pid.e_old + Pid.e) + Pid.ui_old;	// calculate integral term

	Pid.u = (uint16_t)(Pid.up + Pid.ui + Pid.ud);	// process output with all calculated terms

	if  (Pid.u <= Pid.u_min)
	{
		Pid.u = Pid.u_min;
	}

	if (Pid.u >= Pid.u_max)	// check if output exceeds max
	{
		Pid.u = Pid.u_max;	// force output limitation
		Pid.ui = 0;	// suspend integral term for next loop
		Pid.ui_old = 0;
		LandingInit();
		Logic.caseNum = 5;
	}
}

// buffer weight values function in loop
void BufferMass()
{
	float mass = (Pid.u - Mass.initialCCR) / 2.0f;	// calculate new mass value
	Mass.buffer[Mass.bufferIndex] = mass;	// write mass to buffer
	Mass.bufferIndex = (Mass.bufferIndex + 1) % BUFFER_SIZE;	// increase buffer index
	float sum;	// temporary mass sum variable
	for (int i = 0; i < BUFFER_SIZE; i++)
	{
		sum += Mass.buffer[i];	// add up mass values
	}
	Mass.average = (uint16_t) (sum / BUFFER_SIZE) * Mass.gain;	// calculate mean of mass values and process with gain
}

// catch cycle timeout depending on weight omn the plate function
void CatchTimeout()
{
	if (Mass.average >= Mass.lastAverage - 1 && Mass.average <= Mass.lastAverage + 1 && !Logic.averageLevelFlag)	// check if plate has reached stable level
	{
		Logic.timeoutDuration = 450 - (10 * Mass.average * (45 - 5) / 375);	// set max duration ~5 sec @u_max, max duration ~45 sec @zero grams
		Logic.averageLevelFlag = 1;	// set flag for timeout active
	}

	if ((Mass.average < Mass.lastAverage - 2 || Mass.average > Mass.lastAverage + 2) && Logic.averageLevelFlag)	// if mass changes while on
	{
		Logic.averageLevelFlag = 0;	// deactivate timeout
	}

	if (Logic.averageLevelFlag && Logic.timeoutCounter >= Logic.timeoutDuration)	// if timeout is active and counter exceeds duration
	{
		HAL_TIM_Base_Stop_IT(&htim7);	// stop Timer for automatic timeout
		LandingInit();	// initiate landing procedure
		Logic.caseNum = 6;	// set case timeout
		Logic.averageLevelFlag = 0;	// reset flag
		Logic.timeoutDuration = 0;	// reset duration
		Logic.timeoutCounter = 0;	// reset counter
	}
	Mass.lastAverage = Mass.average;	// save mass to last mass value
	Logic.timeoutCounter ++;	// encrease counter one step
}

// adc loop callback function
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if (hadc == &hadc1)
	{
		Sensor.adcValueRAW = HAL_ADC_GetValue(&hadc1);	// set raw adc value
		Sensor.adcValue = 0 + (Sensor.initialAdcValue - Sensor.adcValueRAW);	// process raw adc value to start with zero and count up
		if (Sensor.adcValue < 0)
		{
			Sensor.adcValue = 0;
		}
	}
}

// timer callback function
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	// PID loop
	if (htim == &htim4 && !Pid.suspend)	// check if pid loop is active
	{
		ProcessPIDs();	// start pid calculations
		TIM2->CCR2 = Pid.u;	// adjust dutycycle according to pid output
	}

	// user & timeout loop
	if (htim == &htim7)
	{
		Mass.dutyCycle = (TIM2->CCR2+1)/10;	// calculate dutycycle in % for live expressions in debug
		Mass.currentCCR = TIM2->CCR2;	// write CCR value to current for live expressions

		CatchTimeout(); // perform timeout routine
	}

	// calibration loop
	if (htim == &htim6 && Logic.initFlag && Pid.suspend)	// check if initialization active and pid loop inactive
	{
		CCRCalibration();	// CCR calibration
	}

	// landing loop
	if (htim == &htim6 && Logic.landingFlag)	// check if landing is active
	{
		PerformLanding(); // landing
	}
}

// external interrupt trigger callback function
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == BUTN_A_Pin && !Logic.antiBounceFlag)	// check for left button and not bouncing
	{
		if (Pid.suspend && !Logic.landingFlag)	// check if pid loop is inactive and landing not set
		{
			HAL_TIM_Base_Start_IT(&htim7);	// start 1Hz Timer for automatic timeout
			Logic.caseNum = 3;	// set display case 3 "display mass"
			Pid.suspend = 0;	// activate pid loop
		}
		else if (!Pid.suspend)	// check if pid loop is active
		{
			HAL_TIM_Base_Stop_IT(&htim7);	// stop Timer for automatic timeout
			Logic.caseNum = 4;	// set display case 4 "wait"
			LandingInit();	// perform landing
		}
		Logic.antiBounceFlag = 1;	// intercept bouncing
	}

	if (GPIO_Pin == BUTN_B_Pin && !Logic.antiBounceFlag)	// check for right button and not bouncing
	{
		Mass.tare = Mass.average;	// tare weight value
		Logic.antiBounceFlag = 1;	// intercept bouncing
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
	MX_USART2_UART_Init();
	MX_ADC1_Init();
	MX_TIM2_Init();
	MX_TIM4_Init();
	MX_TIM3_Init();
	MX_I2C1_Init();
	MX_TIM8_Init();
	MX_TIM6_Init();
	MX_TIM7_Init();
	/* USER CODE BEGIN 2 */
	SSD1306_Init();	// initiate screen
	BootScreen();	// call boot screen function
	ZeroAdcValue();	// initiate adc calibration
	HAL_ADC_Start_IT(&hadc1);	// start adc
	HAL_TIM_Base_Start_IT(&htim4);	// start adc and pid loop timer
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);		// start pwm output to drive the coil
	CalibInit();	// initiate CCR calibration
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		if (Logic.antiBounceFlag)	// debounce condition
		{
			HAL_Delay(250);	// debounce time window
			Logic.antiBounceFlag = 0; // debouncing done
		}

		BufferMass();	// buffer last mass value
		HAL_Delay(20);	// 50Hz screen and buffer refresh rate

		switch (Logic.caseNum)	// switch between cases
		{
		case 1:	// calibrate
			SSD1306_Fill(0);	// clear screen
			SSD1306_GotoXY (5, 10);	// navigate to coordinates on the screen
			SSD1306_Puts ("calibrating", &Font_11x18, 1);	// write text
			SSD1306_GotoXY (25, 42);	// navigate to coordinates on the screen
			SSD1306_Puts ("please wait", &Font_7x10, 1);	// write text
			SSD1306_UpdateScreen();	// refresh screen with new contents

			break;

		case 2:	// wait fpr user
			SSD1306_Fill(0);	// clear screen
			SSD1306_GotoXY (46, 0);	// navigate to coordinates on the screen
			SSD1306_Puts("press", &Font_7x10, 1);	// write text

			SSD1306_DrawCircle (64, 30, 13, 1);	// draw circle

			SSD1306_GotoXY (35, 52 );	// navigate to coordinates on the screen
			SSD1306_Puts("to start", &Font_7x10, 1);	// write text
			SSD1306_UpdateScreen();	// refresh screen with new contents

			break;

		case 3:	// display mass
			SSD1306_Fill(0);	// clear screen
			SSD1306_GotoXY (40, 0);	// navigate to coordinates on the screen
			SSD1306_Puts("Weight:", &Font_7x10, 1);	// write text

			sprintf (Mass.bufflt, "%d g", (Mass.average - Mass.tare));	// convert mass and check tare
			SSD1306_GotoXY (40, 13);	// navigate to coordinates on the screen
			SSD1306_Puts (Mass.bufflt, &Font_11x18, 1);	// write mass

			SSD1306_DrawLine(0, 36, 128, 36, 1);	// draw line

			SSD1306_GotoXY (8, 40);	// navigate to coordinates on the screen
			SSD1306_Puts ("press     to stop", &Font_7x10, 1);	// write text

			SSD1306_DrawCircle (60, 43, 4, 1);	// draw circle

			SSD1306_GotoXY (8, 53);	// navigate to coordinates on the screen
			SSD1306_Puts ("press     to tare", &Font_7x10, 1);	// write text
			SSD1306_DrawLine(56, 53, 63, 60, 1);	// draw first line for X
			SSD1306_DrawLine(56, 60, 63, 53, 1);	// draw second line for X

			SSD1306_UpdateScreen();	// refresh screen with new contents

			break;

		case 4:	// return to case 2
			SSD1306_Fill(0);	// clear screen
			SSD1306_GotoXY (25, 25);	// navigate to coordinates on the screen
			SSD1306_Puts ("please wait", &Font_7x10, 1);
			SSD1306_UpdateScreen();	// refresh screen with new contents

			break;

		case 5: // overload
			SSD1306_Fill(0);	// clear screen
			SSD1306_GotoXY (20, 25);	// navigate to coordinates on the screen
			SSD1306_Puts ("overload!", &Font_11x18, 1);
			SSD1306_UpdateScreen();	// refresh screen with new contents
			HAL_Delay(1000);

			break;

		case 6:	// timeout
			SSD1306_Fill(0);	// clear screen
			SSD1306_GotoXY (25, 25);	// navigate to coordinates on the screen
			SSD1306_Puts ("timeout!", &Font_11x18, 1);	// write text
			SSD1306_UpdateScreen();	// refresh screen with new contents

			break;
		}
	}
	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */

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
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1
			|RCC_PERIPHCLK_TIM8|RCC_PERIPHCLK_ADC12
			|RCC_PERIPHCLK_TIM2|RCC_PERIPHCLK_TIM34;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
	PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
	PeriphClkInit.Tim8ClockSelection = RCC_TIM8CLK_HCLK;
	PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
	PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
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

	ADC_MultiModeTypeDef multimode = {0};
	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T4_TRGO;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure the ADC multi-mode
	 */
	multimode.Mode = ADC_MODE_INDEPENDENT;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
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
	hi2c1.Init.Timing = 0x0010020A;
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
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 2;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 999;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

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
	htim3.Init.Prescaler = 26;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 9439;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 26;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 9439;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief TIM6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM6_Init(void)
{

	/* USER CODE BEGIN TIM6_Init 0 */

	/* USER CODE END TIM6_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM6_Init 1 */

	/* USER CODE END TIM6_Init 1 */
	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 9999;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 359;
	htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM6_Init 2 */

	/* USER CODE END TIM6_Init 2 */

}

/**
 * @brief TIM7 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM7_Init(void)
{

	/* USER CODE BEGIN TIM7_Init 0 */

	/* USER CODE END TIM7_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM7_Init 1 */

	/* USER CODE END TIM7_Init 1 */
	htim7.Instance = TIM7;
	htim7.Init.Prescaler = 7199;
	htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim7.Init.Period = 999;
	htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM7_Init 2 */

	/* USER CODE END TIM7_Init 2 */

}

/**
 * @brief TIM8 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM8_Init(void)
{

	/* USER CODE BEGIN TIM8_Init 0 */

	/* USER CODE END TIM8_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM8_Init 1 */

	/* USER CODE END TIM8_Init 1 */
	htim8.Instance = TIM8;
	htim8.Init.Prescaler = 999;
	htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim8.Init.Period = 7199;
	htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim8.Init.RepetitionCounter = 0;
	htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM8_Init 2 */

	/* USER CODE END TIM8_Init 2 */

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
	HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_A_GPIO_Port, LED_A_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin : LED_B_Pin */
	GPIO_InitStruct.Pin = LED_B_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_B_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : BUTN_B_Pin */
	GPIO_InitStruct.Pin = BUTN_B_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(BUTN_B_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LED_A_Pin */
	GPIO_InitStruct.Pin = LED_A_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_A_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : BUTN_A_Pin */
	GPIO_InitStruct.Pin = BUTN_A_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(BUTN_A_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
