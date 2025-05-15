/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// Axis variables for code organization
typedef enum AxisNames
{
	X = 0,
	Y = 1,
	Z = 2,
	END

} Axis;

// Axis directions for calculations
typedef enum AxisOrient
{
	Pos = 0,
	Neg = 1

} Dirctn;

// States of the main code
typedef enum MainLoopStates
{
	Init = 0,
	Home = 1,
	Idle = 2,
	Move = 3,

} MainStates;

// GCode decoding variables
typedef enum GcodeFirstByte
{
	Hme = 0,
	Abs = 1,
	Rel = 2,
	Err = 9

} GcodeCMD;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define AXES_NUM 3

#define GCODE_BYTES 8
#define CHECKSUM_BYTES 1

#define ADC_NUM 7

#define HALL_LOW 0x7ff

// Acceleration limitations
#define ACC_START 0
#define ACC_END 150
#define ACC_END_LOW 20

// Offset of the acceleration calculations
#define ACC_OFF 1500

// The number of steps between each acceleration
#define ACC_STEPS 100

// The highest hypothetical position allowed
#define LIM_POS 0x80000000

// Flag masks
#define X_MASK 0x01
#define Y_MASK 0x02
#define Z_MASK 0x04

#define RX_MASK 0x08
#define TX_MASK 0x10

#define HME_MASK 0x20

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

// Acceleration equation
#define GET_DELAY(A) (((ACC_END - AccVal[A]) * 20) + ACC_OFF)

// Direction transformations
#define DIR_X2(dir) (~dir & 0x01)
#define DIR_Z(dir) (~dir & 0x01)

// Timer macros
#define STOP_TIM(axis) 	(AxisTimers[axis]->CR1 &= (~((uint8_t)TIM_CR1_CEN)))
#define START_TIM(axis) (AxisTimers[axis]->CR1 |= TIM_CR1_CEN)
#define RST_TIM(axis) 	(AxisTimers[axis]->CNT = 0)
#define LIM_TIM(axis) 	(AxisTimers[axis]->ARR)

// Movement flag macros
#define SET_MFLAG(axis) (Flags = Flags | AxisFlagMasks[axis])
#define RST_MFLAG(axis) (Flags = Flags & ~AxisFlagMasks[axis])
#define TST_MFLAG(axis) (Flags & AxisFlagMasks[axis])
#define TST_MFLAG_ALL() (Flags & (AxisFlagMasks[X]+AxisFlagMasks[Y]+AxisFlagMasks[Z]))

// Communication flag macros
#define SET_RXFLAG() (Flags = Flags | RX_MASK)
#define RST_RXFLAG() (Flags = Flags & ~RX_MASK)
#define TST_RXFLAG() (Flags & RX_MASK)

#define SET_TXFLAG() (Flags = Flags | TX_MASK)
#define RST_TXFLAG() (Flags = Flags & ~TX_MASK)
#define TST_TXFLAG() (Flags & TX_MASK)

// Homing flag macros
#define SET_HMEFLAG() (Flags = Flags | HME_MASK)
#define RST_HMEFLAG() (Flags = Flags & ~HME_MASK)
#define TST_HMEFLAG() (Flags & HME_MASK)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

/* USER CODE BEGIN PV */

// Flags for macros
const uint8_t AxisFlagMasks[AXES_NUM] = {X_MASK, Y_MASK, Z_MASK};

// Assigning timer addresses for each axis
TIM_TypeDef* AxisTimers[AXES_NUM] = {TIM14, TIM16, TIM17};

// Default values before homing
const uint32_t LimitDefault[AXES_NUM] = {0x12ffff, 0x4ffff, 685900};
float LimitReal[AXES_NUM] = {5702.3, 1530.35, 438.15};
const float MMtoStepDefault[AXES_NUM] = {214, 214, 1565.48442314};

// Buffer for ADC DMA to be mapped
uint16_t ADC_DMA_Buf[ADC_NUM] = {0};

// Assigning sensor addresses from the buffer
const uint16_t* HallPosX2 = 	ADC_DMA_Buf + 1;
const uint16_t* HallPosX = ADC_DMA_Buf + 3;
const uint16_t* HallPosY = 	ADC_DMA_Buf + 5;
const uint16_t* HallPosZ = 	ADC_DMA_Buf + 6;

const uint16_t* HallNegX2 = 	ADC_DMA_Buf + 0;
const uint16_t* HallNegX = ADC_DMA_Buf + 2;
const uint16_t* HallNegY = 	ADC_DMA_Buf + 4;

uint16_t* HallX2[2] = {ADC_DMA_Buf + 1, ADC_DMA_Buf + 0};
uint16_t* HallX[2] = {ADC_DMA_Buf + 3, ADC_DMA_Buf + 2};
uint16_t* HallY[2] = {ADC_DMA_Buf + 5, ADC_DMA_Buf + 4};

// SPI variables
uint8_t RX_Buffer [GCODE_BYTES] = {0};
uint8_t TX_Buffer [CHECKSUM_BYTES] = {0};

volatile uint32_t RXcount = 0;
volatile uint32_t TXcount = 0;

// Positioning variables for each axis
volatile uint32_t 	CurPos[AXES_NUM] = {0};
uint32_t 			ReqPos[AXES_NUM] = {0};
uint32_t 			Stamp[AXES_NUM] = {0};

// Acceleration variables for each axis
Dirctn   	Dir[AXES_NUM] = {0};
uint16_t  	AccVal[AXES_NUM] = {0};

// Limits and conversions for each axis
// These values are theoretical and will be overwritten by homing
uint32_t  	Limit[AXES_NUM] = {LimitDefault[X], LimitDefault[Y], LimitDefault[Z]};
float  		MMtoStep[AXES_NUM] = {MMtoStepDefault[X], MMtoStepDefault[Y], MMtoStepDefault[Z]};

// State machine variables
MainStates MainState;
volatile uint8_t Flags;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM16_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC_Init(void);
/* USER CODE BEGIN PFP */

void GPIOWrite(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
int GPIORead(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIOToggle(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

void Accelerate(Axis A);
void Deccelerate(Axis A);
void SetDirection(Axis A);

void LowAccelerationHandle(Axis A);
void MaxAccelerationHandle(Axis A);

void HomeAxis(Axis A);

void StopCheck(Axis A);
void StopCheckHall(Axis A);
void RequestCheck(Axis A);

uint32_t GetSPICrd(Axis A);
void ParseGCode(void);

void SetPIPinActive(void);
void SetPIPinIdle(void);
int ReadPIPin(void);

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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM17_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  MX_SPI1_Init();
  MX_ADC_Init();
  /* USER CODE BEGIN 2 */

  // initiate DMA mapping
  HAL_ADC_Start_DMA(&hadc, (uint32_t*)ADC_DMA_Buf, ADC_NUM);

  // Setup SPI and allot time for hardware
  HAL_SPI_Receive_IT(&hspi1, RX_Buffer, GCODE_BYTES); //Receiving in Interrupt mode
  HAL_Delay(100);

  // Set all directions to default positive
  GPIOWrite(dirX1_GPIO_Port, dirX1_Pin, Pos);
  GPIOWrite(dirX2_GPIO_Port, dirX2_Pin, DIR_X2(Pos));
  GPIOWrite(dirY_GPIO_Port,  dirY_Pin,  Pos);
  GPIOWrite(dirZ_GPIO_Port,  dirZ_Pin,  Pos);

  // Reset all stepper driving pins to
  GPIOWrite(pulX1_GPIO_Port, pulX1_Pin, GPIO_PIN_RESET);
  GPIOWrite(pulX2_GPIO_Port, pulX2_Pin, GPIO_PIN_RESET);
  GPIOWrite(pulY_GPIO_Port,  pulY_Pin,  GPIO_PIN_RESET);
  GPIOWrite(pulZ_GPIO_Port,  pulZ_Pin,  GPIO_PIN_RESET);

  // Setup all timers in interrupt mode
  HAL_TIM_Base_Start_IT(&htim14);
  HAL_TIM_Base_Start_IT(&htim16);
  HAL_TIM_Base_Start_IT(&htim17);

  // Stop all timers to stop motor movement
  for( Axis A = X; A < END; A++)
	  STOP_TIM(A);

  // Initialize all variables
  for( Axis A = X; A < END; A++)
  {
	  RST_TIM(A);
	  AccVal[A] = ACC_START;
	  LIM_TIM(A) = GET_DELAY(A);
	  CurPos[A] = 0;
	  ReqPos[A] = 0;
	  Dir[A] = Pos;
  }

  MainState = Init;
  Flags = 0x00;

  //SET_HMEFLAG();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  switch(MainState)
	  {
		  case Init:

			  MainState = Init;
			  SetPIPinIdle();

			  // Only exit Init if the PI allows
			  if(ReadPIPin())
			  {
				  if(TST_HMEFLAG())
					  MainState = Home;
				  else
					  MainState = Idle;
			  }

			  break;

		  case Home:

			  MainState = Home;
			  SetPIPinActive();

			  for( Axis A = X; A < END; A++)
				  HomeAxis(A);

			  RST_HMEFLAG();
			  MainState = Idle;

			  break;

		  case Idle:

			  MainState = Idle;
			  SetPIPinIdle();

			  // If SPI received, calculate and send checksum
			  if(TST_RXFLAG())
			  {
					TX_Buffer[0] = 0;

					for(int i = 0; i < GCODE_BYTES; i++)
					  TX_Buffer[0] += RX_Buffer[i];

					HAL_SPI_Transmit_IT(&hspi1, TX_Buffer, CHECKSUM_BYTES); //Sending in Interrupt mode

					RST_RXFLAG();
			  }

			  // If checksum sent, wait for PI confirmation or another RX
			  if(ReadPIPin() && TST_TXFLAG())
			  {
				  RST_TXFLAG();

				  ParseGCode();

				  // Check which, if any, axes need to be moved
				  for( Axis A = X; A < END; A++)
					  RequestCheck(A);

				  // If movement requested, initiate appropriate timers
				  if(TST_MFLAG_ALL())
				  {
					  MainState = Move;
					  SetPIPinActive();

					  for( Axis A = X; A < END; A++)
						  if(TST_MFLAG(A))
							  START_TIM(A);
				  }

			  }

			  if(TST_HMEFLAG())
				  MainState = Home;

			  break;

		  case Move:

			  MainState = Move;

			  // Handle accelerations
			  for( Axis A = X; A < END; A++)
				  if(TST_MFLAG(A))
					  MaxAccelerationHandle(A);

			  // Return to idle once all movement is done
			  if(TST_MFLAG_ALL() == 0)
				  MainState = Idle;

			  break;

		  default:
			  MainState = Idle;
			  break;

	  }

	  // Emergency motor stop
	  if(GPIORead(B1_GPIO_Port,  B1_Pin) == 0)
	  {
		  // Immediately stop all motors
		  for( Axis A = X; A < END; A++)
			  STOP_TIM(A);

		  // Reset all timer variables
		  for( Axis A = X; A < END; A++)
		  {
			  RST_TIM(A);
			  AccVal[A] = ACC_START;
			  LIM_TIM(A) = GET_DELAY(A);
			  ReqPos[A] = CurPos[A];
		  }

		  MainState = Idle;

		  // Blocking delay to hold completely idle during any button bouncing
		  HAL_Delay(100);

		  // Wait for another button press
		  while(GPIORead(B1_GPIO_Port,  B1_Pin));
	  }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR3;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 0;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 4800;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 4800;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 0;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 4800;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, pulX1_Pin|SPI_OUT_Pin|pulZ_Pin|pulY_Pin
                          |dirY_Pin|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(pulX2_GPIO_Port, pulX2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, dirX1_Pin|dirX2_Pin|dirZ_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : pulX1_Pin SPI_OUT_Pin pulZ_Pin pulY_Pin
                           dirY_Pin PB8 */
  GPIO_InitStruct.Pin = pulX1_Pin|SPI_OUT_Pin|pulZ_Pin|pulY_Pin
                          |dirY_Pin|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : pulX2_Pin */
  GPIO_InitStruct.Pin = pulX2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(pulX2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : dirX1_Pin dirX2_Pin dirZ_Pin */
  GPIO_InitStruct.Pin = dirX1_Pin|dirX2_Pin|dirZ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_IN_Pin */
  GPIO_InitStruct.Pin = SPI_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SPI_IN_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief Writes a digital value to GPIO
  * @param GPIOx	The GPIO register
  * @param GPIO_Pin	The register bit
  * @param PinState	The value to write
  * @retval None
  */
void GPIOWrite(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
	  if (PinState)
	    GPIOx->BSRR = (uint32_t)GPIO_Pin;
	  else
	    GPIOx->BRR = (uint32_t)GPIO_Pin;
}

/**
  * @brief Reads a digital value from GPIO
  * @param GPIOx	The GPIO register
  * @param GPIO_Pin	The register bit
  * @retval The GPIO value
  */
int GPIORead(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	  if (GPIOx->IDR & GPIO_Pin)
		  return GPIO_PIN_SET;
	  else
		  return GPIO_PIN_RESET;
}

/**
  * @brief Toggles a GPIO pin
  * @param GPIOx	The GPIO register
  * @param GPIO_Pin	The register bit
  * @retval None
  */
void GPIOToggle(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	  GPIOx->BSRR = ((GPIOx->ODR & GPIO_Pin) << 16U) | (~(GPIOx->ODR) & GPIO_Pin);
}

/**
  * @brief Accelerates if applicable
  * @param A	The axis to apply to
  * @retval None
  */
void Accelerate(Axis A)
{
	if(AccVal[A] >= ACC_END)
		AccVal[A] = ACC_END;
	else
		AccVal[A]++;

	return;
}

/**
  * @brief Deccelerates if applicable
  * @param A	The axis to apply to
  * @retval None
  */
void Deccelerate(Axis A)
{
	if(AccVal[A] <= ACC_START)
		AccVal[A] = ACC_START;
	else
		AccVal[A]--;

	return;
}

/**
  * @brief Sets the direction of the motors
  * @param A	The axis to apply to
  * @retval None
  */
void SetDirection(Axis A)
{
	switch(A)
	{
		case X:
			if(Dir[X])
			{
				GPIOWrite(dirX1_GPIO_Port, dirX1_Pin, Neg);
				GPIOWrite(dirX2_GPIO_Port, dirX2_Pin, DIR_X2(Neg));
			}
			else
			{
				GPIOWrite(dirX1_GPIO_Port, dirX1_Pin, Pos);
				GPIOWrite(dirX2_GPIO_Port, dirX2_Pin, DIR_X2(Pos));
			}
			break;

		case Y:
			if(Dir[Y])
				GPIOWrite(dirY_GPIO_Port,  dirY_Pin,  Neg);
			else
				GPIOWrite(dirY_GPIO_Port,  dirY_Pin,  Pos);
			break;

		case Z:
			if(Dir[Z])
				GPIOWrite(dirZ_GPIO_Port,  dirZ_Pin,  DIR_Z(Neg));
			else
				GPIOWrite(dirZ_GPIO_Port,  dirZ_Pin,  DIR_Z(Pos));
			break;

		default:
			break;
	}
}

/**
  * @brief Determines the type of acceleration needed and calls the appropriate function, accelerates to a smaller value
  * @param A	The axis to apply to
  * @retval None
  */
void LowAccelerationHandle(Axis A)
{
	if(abs((int)CurPos[A] - (int)Stamp[A]) > ACC_STEPS)
	{
		if(abs((int)ReqPos[A] - (int)CurPos[A]) > AccVal[A])
		{
			if(AccVal[A] <= ACC_END_LOW)
				Accelerate(A);
		}
		else
			Deccelerate(A);

	  LIM_TIM(A) =  GET_DELAY(A);
	  Stamp[A] = CurPos[A];
	}
}

/**
  * @brief Determines the type of acceleration needed and calls the appropriate function
  * @param A	The axis to apply to
  * @retval None
  */
void MaxAccelerationHandle(Axis A)
{
	if(abs((int)CurPos[A] - (int)Stamp[A]) > ACC_STEPS)
	{
	  if(abs((int)ReqPos[A] - (int)CurPos[A]) > AccVal[A] * ACC_STEPS)
			Accelerate(A);
		else
			Deccelerate(A);

	  LIM_TIM(A) =  GET_DELAY(A);
	  Stamp[A] = CurPos[A];
	}
}

/**
  * @brief Determines and sets the limit values
  * @param A	The axis to apply to
  * @retval None
  */
void HomeAxis(Axis A)
{
	switch(A)
	{
		case X:

			// Set requested position to reasonably theoretical limit in the positive direction
			CurPos[X] = 0;
			ReqPos[X] = LimitDefault[X];
			SET_MFLAG(X);

			Dir[X] = Pos;
			SetDirection(X);

			// Reset movement values and start timer
			AccVal[X] = ACC_START;
			Stamp[X] = CurPos[X];

			RST_TIM(X);
			LIM_TIM(X) =  GET_DELAY(X);

			START_TIM(X);

			// Wait for Hall sensor to trigger and stop
			while(*HallPosX > HALL_LOW || *HallPosX2 > HALL_LOW)
			{
			  LowAccelerationHandle(X); // Accelerate to a lower than normal because deceleration is impossible to predict

			  if(TST_MFLAG(X))
				  break;
			}

			STOP_TIM(X);

			// Set current position to reasonably theoretical limit and move in the negative direction
			CurPos[X] = LimitDefault[X];
			ReqPos[X] = 0;

			Dir[X] = Neg;
			SetDirection(X);

			// Reset movement values and start timer
			AccVal[X] = ACC_START;
			Stamp[X] = CurPos[X];

			RST_TIM(X);
			LIM_TIM(X) =  GET_DELAY(X);

			START_TIM(X);

			// Wait for Hall sensor to trigger and stop
			while(*HallNegX > HALL_LOW || *HallNegX2 > HALL_LOW)
			{
			  LowAccelerationHandle(X); // Accelerate to a lower than normal because deceleration is impossible to predict

			  if(TST_MFLAG(X))
				  break;
			}


			STOP_TIM(X);

			// Calculate the exact limit and ratio and set position to 0
			Limit[X] = LimitDefault[X]; - CurPos[X];
			MMtoStep[X] = Limit[X] / LimitReal[X];

			CurPos[X] = 0;

			RST_MFLAG(X);

			break;

		case Y:

			// Set requested position to reasonably theoretical limit in the positive direction
			CurPos[Y] = 0;
			ReqPos[Y] = LimitDefault[Y];
			SET_MFLAG(Y);

			Dir[Y] = Pos;
			SetDirection(Y);

			// Reset movement values and start timer
			AccVal[Y] = ACC_START;
			Stamp[Y] = CurPos[Y];

			RST_TIM(Y);
			LIM_TIM(Y) =  GET_DELAY(Y);

			START_TIM(Y);

			// Wait for Hall sensor to trigger and stop
			while(*HallPosY > HALL_LOW)
			{
			  LowAccelerationHandle(Y); // Accelerate to a lower than normal because deceleration is impossible to predict

			  if(TST_MFLAG(X))
				  break;
			}

			STOP_TIM(Y);

			// Set current position to reasonably theoretical limit and move in the negative direction
			CurPos[Y] = LimitDefault[Y];
			ReqPos[Y] = 0;

			Dir[Y] = Neg;
			SetDirection(Y);

			// Reset movement values and start timer
			AccVal[Y] = ACC_START;
			Stamp[Y] = CurPos[Y];

			RST_TIM(Y);
			LIM_TIM(Y) =  GET_DELAY(Y);

			START_TIM(Y);

			// Wait for Hall sensor to trigger and stop
			while(*HallNegY > HALL_LOW)
			{
			  LowAccelerationHandle(Y);  // Accelerate to a lower than normal because deceleration is impossible to predict

			  if(TST_MFLAG(Y))
				  break;
			}



			STOP_TIM(Y);

			// Calculate the exact limit and ratio and set position to 0
			Limit[Y] = LimitDefault[Y] - CurPos[Y];

			MMtoStep[Y] = Limit[Y] / LimitReal[Y];

			CurPos[Y] = 0;

			RST_MFLAG(Y);

			break;

		case Z:

			// Set current position to reasonably theoretical limit and move in the negative direction
			CurPos[Z] = LimitDefault[Z];
			ReqPos[Z] = 0;
			SET_MFLAG(Z);

			Dir[Z] = Neg;
			SetDirection(Z);

			// Reset movement values and start timer
			AccVal[Z] = ACC_START;
			Stamp[Z] = CurPos[Z];

			RST_TIM(Z);
			LIM_TIM(Z) =  GET_DELAY(Z);

			START_TIM(Z);

			// Wait for Hall sensor to trigger and stop
			while(*HallPosZ > HALL_LOW)
			{
			  LowAccelerationHandle(Z); // Accelerate to a lower than normal because deceleration is impossible to predict

			  if(TST_MFLAG(Z))
				  break;
			}

			STOP_TIM(Z);

			Limit[Z] = LimitDefault[Z];
			MMtoStep[Z] = Limit[Z] / LimitReal[Z];

			// Set position to 0
			CurPos[Z] = 0;

			RST_MFLAG(Z);

			break;

		default:
			break;
	}
}

/**
  * @brief Checks if the requested value has been reached and stops movement if so
  * @param A	The axis to apply to
  * @retval None
  */
void StopCheck(Axis A)
{
	// Direction determines the logical operation that determines if we've met or passed the requested position
	if(Dir[A])
	{
	  if(CurPos[A] <= ReqPos[A])
	  {
		  STOP_TIM(A);
		  AccVal[A] = 0;
		  RST_MFLAG(A);
	  }
	}
	else
	{
	  if(CurPos[A] >= ReqPos[A])
	  {
		  STOP_TIM(A);
		  AccVal[A] = 0;
		  RST_MFLAG(A);
	  }
	}
}

/**
  * @brief Checks if the Hall sensor has been reached and stops movement if so
  * @param A	The axis to apply to
  * @retval None
  */
void StopCheckHall(Axis A)
{
	switch(A)
	{
		case X:
			if(*HallX[Dir[A]] < HALL_LOW  && *HallX2[Dir[X]] < HALL_LOW )
			{
				  STOP_TIM(A);
				  AccVal[A] = 0;
				  RST_MFLAG(A);
			}
			break;

		case Y:
			if(*HallY[Dir[A]] < HALL_LOW )
			{
				  STOP_TIM(A);
				  AccVal[A] = 0;
				  RST_MFLAG(A);
			}
			break;

		case Z:
			if(*HallPosZ < HALL_LOW && Dir[A] == Pos)
			{
				  STOP_TIM(A);
				  AccVal[A] = 0;
				  RST_MFLAG(A);
			}
			break;
	}
}

/**
  * @brief Checks if the requested value has changed and sets flags if so
  * @param A	The axis to apply to
  * @retval None
  */
void RequestCheck(Axis A)
{
	// If the position is off by more than one half a MM, set movement flags
	if(abs((int)CurPos[A] - (int)ReqPos[A]) > (MMtoStep[A] / 2))
	{
	  if(CurPos[A] > ReqPos[A])
		  Dir[A] = Neg;
	  else
		  Dir[A] = Pos;

	  SetDirection(A);

	  AccVal[A] = ACC_START;
	  Stamp[A] = CurPos[A];

	  SET_MFLAG(A);

	  RST_TIM(A);
	  LIM_TIM(A) =  GET_DELAY(A);
	}
}

/**
  * @brief Converts the SPI mm values to step values
  * @param A	The axis to apply to
  * @retval The step value
  */
uint32_t GetSPICrd(Axis A)
{
	return (uint32_t)(((RX_Buffer[(2*A) + 2] & 0x7) << 8) + RX_Buffer[(2*A) + 3]) * MMtoStep[A];
}

/**
  * @brief Determines the GCode operation and executes it
  * @param None
  * @retval None
  */
void ParseGCode(void)
{
	switch((GcodeCMD)RX_Buffer[0])
	{
		case Hme:
			SET_HMEFLAG();
			break;

		case Abs:

			ReqPos[X] = GetSPICrd(X);
			ReqPos[Y] = GetSPICrd(Y);
			ReqPos[Z] = GetSPICrd(Z);

			break;

		case Rel:

			if(RX_Buffer[2] & 0x80)
				ReqPos[X] -= GetSPICrd(X);
			else
				ReqPos[X] += GetSPICrd(X);

			if(RX_Buffer[4] & 0x80)
				ReqPos[Y] -= GetSPICrd(Y);
			else
				ReqPos[Y] += GetSPICrd(Y);


			if(RX_Buffer[6] & 0x80)
				ReqPos[Z] -= GetSPICrd(Z);
			else
				ReqPos[Z] += GetSPICrd(Z);


			break;

		case Err:
			HAL_NVIC_SystemReset();
			break;

		default:
			break;

	}

	// Assure that requested position did not overflow or go out of range
	for(Axis A = X; A < END; A++)
		if(ReqPos[A] & LIM_POS)
			ReqPos[A] = 0;
		else if (ReqPos[A] > Limit[A])
			ReqPos[A] = Limit[A];
}

/**
  * @brief Sets the PI GPIO indicator to low to communicate activity
  * @param None
  * @retval None
  */
void SetPIPinActive(void)
{
	GPIOWrite(SPI_OUT_GPIO_Port,  SPI_OUT_Pin,  GPIO_PIN_RESET);
}

/**
  * @brief Sets the PI GPIO indicator to high to communicate inactivity
  * @param None
  * @retval None
  */
void SetPIPinIdle(void)
{
	GPIOWrite(SPI_OUT_GPIO_Port,  SPI_OUT_Pin,  GPIO_PIN_SET);
}

/**
  * @brief Reads the PI GPIO indicator
  * @param None
  * @retval None
  */
int ReadPIPin(void)
{
	return GPIORead(SPI_IN_GPIO_Port,  SPI_IN_Pin);
}

/*
 *
 * 		Interrupt functions
 *
 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// no switch here because of compile reduction :(

	// Check the timer instance and toggle the appropriate axis
	// Additionally increment and evaluate the current position

	if(htim->Instance == AxisTimers[X])
	{
		StopCheckHall(X);

		if(*(HallX[Dir[X]]) > HALL_LOW )
			GPIOToggle(pulX1_GPIO_Port, pulX1_Pin);

		if(*(HallX2[Dir[X]]) > HALL_LOW )
			GPIOToggle(pulX2_GPIO_Port, pulX2_Pin);

		if(Dir[X])
			CurPos[X]--;
		else
			CurPos[X]++;


		if(CurPos[X] & LIM_POS)
			CurPos[X] = 0;

		StopCheck(X);

		return;
	}
	else if(htim->Instance == AxisTimers[Y])
	{
		StopCheckHall(Y);

		GPIOToggle(pulY_GPIO_Port,  pulY_Pin);

		if(Dir[Y])
			CurPos[Y]--;
		else
			CurPos[Y]++;

		if(CurPos[Y] & LIM_POS)
			CurPos[Y] = 0;

		StopCheck(Y);

		return;
	}
	else if(htim->Instance == AxisTimers[Z])
	{
		StopCheckHall(Z);

		GPIOToggle(pulZ_GPIO_Port,  pulZ_Pin);

		if(Dir[Z])
			CurPos[Z]--;
		else
			CurPos[Z]++;

		if(CurPos[Z] & LIM_POS)
			CurPos[Z] = 0;

		StopCheck(Z);

		return;
	}

}

// This is called when SPI transmit is done
void HAL_SPI_TxCpltCallback (SPI_HandleTypeDef * hspi)
{
	SetPIPinActive();

	TXcount++;
	SET_TXFLAG();

	// Call for another RX handler only after sending a TX response to avoid overwhelming
	HAL_SPI_Receive_IT(&hspi1, RX_Buffer, GCODE_BYTES);

	SetPIPinIdle();
}

// This is called when SPI receive is done
void HAL_SPI_RxCpltCallback (SPI_HandleTypeDef * hspi)
{
	SetPIPinActive();

	RXcount++;
	SET_RXFLAG();

	SetPIPinIdle();
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
