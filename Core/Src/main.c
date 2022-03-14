/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "semphr.h"

#include "oled.h"
#include "stdlib.h"
#include "ICM20948.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for encoder */
osThreadId_t encoderHandle;
const osThreadAttr_t encoder_attributes = {
  .name = "encoder",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for display */
osThreadId_t displayHandle;
const osThreadAttr_t display_attributes = {
  .name = "display",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for gyroscope */
osThreadId_t gyroscopeHandle;
const osThreadAttr_t gyroscope_attributes = {
  .name = "gyroscope",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC2_Init(void);
void StartDefaultTask(void *argument);
void Encoder(void *argument);
void Display(void *argument);
void Gyroscope(void *argument);

/* USER CODE BEGIN PFP */
void StartDefaultTask(void *argument);
void Encoder(void *argument);
void Display(void *argument);
void Gyroscope(void *argument);
char Obstacle_Straight_Move();
char Straight_Move(bool, int);
char Turning(bool, bool, int);
char Tight_Turn(bool, bool);
char Spot_Rotate(bool);
char U_Turn(uint8_t);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static SemaphoreHandle_t Mutex;
uint8_t aRxBuffer[20];
static float YAW = 0.0;

axises my_gyro;
axises my_accel;
axises my_mag;

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
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
	OLED_Init();
	HAL_UART_Receive_IT(&huart3, (uint8_t*) aRxBuffer, 10);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add Mutexes, ... */
	Mutex = xSemaphoreCreateMutex();
	if (Mutex != NULL) {
//		HAL_UART_Transmit(&huart3,(uint8_t *) 'J', 1, 0xFFFF);
	}
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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of encoder */
  encoderHandle = osThreadNew(Encoder, NULL, &encoder_attributes);

  /* creation of display */
  displayHandle = osThreadNew(Display, NULL, &display_attributes);

  /* creation of gyroscope */
  gyroscopeHandle = osThreadNew(Gyroscope, NULL, &gyroscope_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 160;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 7199;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |ICM_INT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : OLED_SCL_Pin OLED_SDA_Pin OLED_RST_Pin OLED_DC_Pin
                           ICM_INT_Pin */
  GPIO_InitStruct.Pin = OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |ICM_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : AIN2_Pin AIN1_Pin BIN1_Pin BIN2_Pin */
  GPIO_InitStruct.Pin = AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	/* Prevent unused argument(s) compilation warning*/
	UNUSED(huart);

//	HAL_UART_Transmit(&huart3, (uint8_t*) aRxBuffer, 10, 0xFFFF);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */

char Straight_Move(bool forward, int steps) {

	uint16_t rightPwmVal = 1400;
	uint16_t leftPwmVal = 1400;

	// right
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);

	// left
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

	int16_t cnt1, cnt2;
	int target;
	float diff;
	int reorient;
	int center = 143;

	float yaw = 0;
	yaw = YAW;

	htim1.Instance->CCR4 = center;	//center
	// Reset Timer Val
//	__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
//	TIM3->CNT = 0;
	osDelay(500);
	cnt1 = __HAL_TIM_GET_COUNTER(&htim3);

	for (;;) {
		// Lounge
		htim1.Instance->CCR4 = center;	//center
		if (forward == true) {
			// corridor
//			htim1.Instance->CCR4 = center;	//center
			HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);

			diff = abs(abs(YAW) - abs(yaw));
			// if tilt to right
			if (YAW < yaw)
				diff = 0 - diff;
			reorient = 4;
			if (diff >= 0.5)
			{
				// If tilt to right
				// turn right
				htim1.Instance->CCR4 = center + reorient;
				osDelay(10);
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal+20); // Modify the comparison value for the duty cycle
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal+300); // Modify the comparison value for the duty cycle
			}
			else if (diff <= -0.5) {
				// If tilt to left
				// turn right
				htim1.Instance->CCR4 = center + reorient;
				osDelay(10);
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal+300); // Modify the comparison value for the duty cycle
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal+20); // Modify the comparison value for the duty cycle

			} else {
				htim1.Instance->CCR4 = center;
				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,rightPwmVal); // Modify the comparison value for the duty cycle
				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,leftPwmVal); // Modify the comparison value for the duty cycle
			}
		} else {
//			// corridor
//			htim1.Instance->CCR4 = center;	//center
			HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
			diff = abs(abs(YAW) - abs(yaw));
			// if tilt to right
			if (YAW > yaw)
				diff = 0 - diff;
			reorient = 4;
			if (diff <= -0.5)
			{
				// If tilt to right
				// turn left
				htim1.Instance->CCR4 = center + reorient;
				osDelay(10);
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal-20); // Modify the comparison value for the duty cycle
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal); // Modify the comparison value for the duty cycle
			}
			else if (diff >= 0.5) {
				// If tilt to left
				// turn right
				htim1.Instance->CCR4 = center + reorient;
				osDelay(10);
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal-40); // Modify the comparison value for the duty cycle
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal+20); // Modify the comparison value for the duty cycle

			} else {
				htim1.Instance->CCR4 = center;
				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,leftPwmVal); // Modify the comparison value for the duty cycle
				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,rightPwmVal); // Modify the comparison value for the duty cycle
			}
		}

		if (forward == true) {
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,rightPwmVal); // Modify the comparison value for the duty cycle
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,leftPwmVal); // Modify the comparison value for the duty cycle
		} else {
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,leftPwmVal); // Modify the comparison value for the duty cycle
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,rightPwmVal); // Modify the comparison value for the duty cycle
		}

//		osDelay(10);

		if (forward == true)
			target = 663*steps;
		else
			target = 675*steps;
//		target = 5000;
		cnt2 = __HAL_TIM_GET_COUNTER(&htim3);

		if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3)) {
//			if (cnt1 > cnt2){
				if ((cnt1-cnt2) >= target)
					break;
//			}
//			else{
//				if ((cnt1 + 65535 - cnt2) >= target)
//					break;
//			}
//			else {
//				// if less than half the distance
//				if ((cnt1-cnt2) < target/2 && leftPwmVal < 4000) {
//					rightPwmVal += 60;
//					leftPwmVal += 60;
//				}
//				else if ((cnt1-cnt2) >= target/2 && leftPwmVal > 800) {
//					rightPwmVal -= 300;
//					leftPwmVal -= 300;
//				}
//
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,rightPwmVal); // Modify the comparison value for the duty cycle
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,leftPwmVal); // Modify the comparison value for the duty cycle
//			}
		}
		else {
//			if (cnt2 > cnt1){
				if ((cnt2-cnt1) >= target)
					break;
//			} else {
//				if ((cnt2 + 65535 -cnt1) >= target)
//					break;
//			}
//			else {
//				// if less than half the distance
//				if ((cnt2-cnt1) < target/2 && rightPwmVal < 4000) {
//					rightPwmVal += 60;
//					leftPwmVal += 60;
//				}
//				else if ((cnt2-cnt1) >= target/2 && rightPwmVal > 800) {
//					rightPwmVal -= 300;
//					leftPwmVal -= 300;
//				}
//
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,rightPwmVal); // Modify the comparison value for the duty cycle
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,leftPwmVal); // Modify the comparison value for the duty cycle
//			}
		}
	}

	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0); // Modify the comparison value for the duty cycle
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0); // Modify the comparison value for the duty cycle

	// Reset Timer Val
	__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
	TIM3->CNT = 0;
	return 'R';
}


// For turning with encoder
char Turning(bool forward, bool left, int angle) {
	uint16_t pwmVal = 1000;
	uint8_t buf[20];
	uint16_t cnt;
	float yaw = 0;
	int target;

	// Reset Timer Val
	__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
	TIM3->CNT = 0;

	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);

	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);


// For TR
//	if (left == true) {
//		htim1.Instance->CCR4 = 100; // Turning Left
//		if (forward == true)
//			target = 2490;
//		else
//			target = 2510;
//	} else {
//		htim1.Instance->CCR4 = 230; // Turning Right
//		if (forward == true)
//			target = 4250;
//		else
//			target = 4230;
//	}

// For lounge
	if (left == true) {
		htim1.Instance->CCR4 = 100; // Turning Left
		if (forward == true)
			target = 2940;
		else
			target = 2960;
	} else {
		htim1.Instance->CCR4 = 240; // Turning Right
		if (forward == true)
			target = 4210;
		else
			target = 4280;
	}



	xSemaphoreTake(Mutex, portMAX_DELAY);
	if (yaw == 0)
		yaw = YAW;
	xSemaphoreGive(Mutex);

	if (forward == true) {
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
	}


	osDelay(500);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal); // Modify the comparison value for the duty cycle
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal); // Modify the comparison value for the duty cycle
	osDelay(10); // Move on top

	for (;;) {


		// for 90deg left turn
		cnt = __HAL_TIM_GET_COUNTER(&htim3);

		if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3)) {
			if (65535-cnt >= target && cnt!=0) {
				break;
			}
//			else {
//				// if less than half the distance
//				if (cnt > target/2 && pwmVal < 2000)
//					pwmVal += 40;
//				else if (cnt < target/2 && pwmVal > 1200)
//					pwmVal -= 40;
//
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal); // Modify the comparison value for the duty cycle
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal); // Modify the comparison value for the duty cycle
//			}
		}
		else {
			if (cnt >= target) {
				break;
			}
//			else {
//				// if less than half the distance
//				if (cnt < target/2 && pwmVal < 2000)
//					pwmVal += 40;
//				else if (cnt > target/2 && pwmVal > 1200)
//					pwmVal -= 40;
//
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal); // Modify the comparison value for the duty cycle
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal); // Modify the comparison value for the duty cycle
//			}
		}

	}
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0); // Modify the comparison value for the duty cycle
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0); // Modify the comparison value for the duty cycle
	osDelay(10);
	htim1.Instance->CCR4 = 143; // Forward

	// Reset Timer Val
	__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
	TIM3->CNT = 0;
	return 'R';
}

// For turning with encoder
char Tight_Turn(bool forward, bool left) {
	uint16_t pwmVal = 1800;
	uint8_t buf[20];
	uint16_t cnt;
	float yaw = 0;
	int target;
	int targetFront1;
	int targetBack;
	int targetFront2;

//	// Reset Timer Val
//	__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
//	TIM3->CNT = 0;

	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);

	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

	yaw = YAW;

// For TR
//	if (left == true) {
//		htim1.Instance->CCR4 = 100; // Turning Left
//		if (forward == true)
//			target = 2490;
//		else
//			target = 2510;
//	} else {
//		htim1.Instance->CCR4 = 230; // Turning Right
//		if (forward == true)
//			target = 4250;
//		else
//			target = 4230;
//	}

//// For lounge
//	if (forward == true){
//		if (left == true) {
//			htim1.Instance->CCR4 = 93; // Turning Left 2940
//			targetFront1 = 980;
//			targetBack = 1380;
//			targetFront2 = 800;
//		} else {
//			htim1.Instance->CCR4 = 240; // Turning Right 4210
//			targetFront1 = 1554;
//			targetBack = 1184;
//			targetFront2 = 810;
//		}
//	} else {
//		if (left == true) {
//			htim1.Instance->CCR4 = 93; // A Turning Left 2940
//			targetFront1 = 400;
//			targetBack = 2000;
//			targetFront2 = 710;
//		} else {
//			htim1.Instance->CCR4 = 240; // D Turning Right 4210
//			targetFront1 = 700;
//			targetBack = 1550;
//			targetFront2 = 610;
//		}
//	}

	// For corridor
		if (forward == true){
			if (left == true) {
				htim1.Instance->CCR4 = 93; // Turning Left 2940
				targetFront1 = 1110;
				targetBack = 1430;
				targetFront2 = 850;
			} else {
				htim1.Instance->CCR4 = 240; // Turning Right 4210
				targetFront1 = 1454;
				targetBack = 934;
				targetFront2 = 1090;
			}
		} else {
			if (left == true) {
				htim1.Instance->CCR4 = 93; // A Turning Left 2940
				targetFront1 = 200;
				targetBack = 1600;
				targetFront2 = 810;
			} else {
				htim1.Instance->CCR4 = 240; // D Turning Right 4210
				targetFront1 = 700;
				targetBack = 950;
				targetFront2 = 910;
			}
		}
	if (forward == true) {
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
	}


	osDelay(500);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal); // Modify the comparison value for the duty cycle
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal); // Modify the comparison value for the duty cycle
	osDelay(10); // Move on top

	// Reset Timer Val
	__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
	TIM3->CNT = 0;

	for (;;) {
		// for 90deg left turn
		cnt = __HAL_TIM_GET_COUNTER(&htim3);

		if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3)) {
			if (65535-cnt >= targetFront1 && cnt!=0) {
				break;
			}
		}
		else {
			if (cnt >= targetFront1) {
				break;
			}
		}

	}

	// Stop
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0); // Modify the comparison value for the duty cycle
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0); // Modify the comparison value for the duty cycle


//	__HAL_TIM_SET_COUNTER(&htim3,0);

	osDelay(100);

	// Back

	if (left == true) {
		htim1.Instance->CCR4 = 240; // Turning Left 2940
	} else {
		htim1.Instance->CCR4 = 93; // Turning Right 4210
	}

	if (forward == true) {
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
	}
	osDelay(500);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal); // Modify the comparison value for the duty cycle
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal); // Modify the comparison value for the duty cycle
	osDelay(10); // Move on top
	// Reset Timer Val
	__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
	TIM3->CNT = 0;

	for (;;) {
		// for 90deg left turn
		cnt = __HAL_TIM_GET_COUNTER(&htim3);

		if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3)) {
			if (65535-cnt >= targetBack && cnt!=0) {
				break;
			}
		}
		else {
			if (cnt >= targetBack) {
				break;
			}
		}

	}

	// Front2
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0); // Modify the comparison value for the duty cycle
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0); // Modify the comparison value for the duty cycle


//	__HAL_TIM_SET_COUNTER(&htim3,0);

	osDelay(100);

	if (left == true) {
		htim1.Instance->CCR4 = 93; // Turning Left 2940
	} else {
		htim1.Instance->CCR4 = 240; // Turning Right 4210
	}

	if (forward == true) {
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
	}
	osDelay(500);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal); // Modify the comparison value for the duty cycle
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal); // Modify the comparison value for the duty cycle
	osDelay(10); // Move on top

	// Reset Timer Val
	__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
	TIM3->CNT = 0;

	for (;;) {
		// for 90deg left turn
		cnt = __HAL_TIM_GET_COUNTER(&htim3);

		if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3)) {
			if (65535-cnt >= targetFront2 && cnt!=0) {
				break;
			}
		}
		else {
			if (cnt >= targetFront2) {
				break;
			}
		}

	}


	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0); // Modify the comparison value for the duty cycle
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0); // Modify the comparison value for the duty cycle
	osDelay(10);
	htim1.Instance->CCR4 = 143; // Forward

	// cnt declaration

	int16_t cnt1, cnt2;
	// Reset Timer Val
	__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
	TIM3->CNT = 0;

	cnt1 = __HAL_TIM_GET_COUNTER(&htim3);

	// Backwards correction
	for (;;) {
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
		osDelay(10);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal); // Modify the comparison value for the duty cycle
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal); // Modify the comparison value for the duty cycle
		if (forward == true){
			if (left == true)
				target = 917;
			else
				target = 900;
		} else {
			if (left == true)
				target = 400;
			else
				target = 1000;
		}
		cnt2 = __HAL_TIM_GET_COUNTER(&htim3);

		if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3)) {
			if ((cnt1-cnt2) >= target) {
				break;
			}
		}
		else {
			if ((cnt2-cnt1) >= target) {
				break;
			}
		}
	}
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0); // Modify the comparison value for the duty cycle
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0); // Modify the comparison value for the duty cycle

//	if ((YAW == 0.00) || (yaw == 0.00) || !(isdigit(YAW))) {
//		RESET_GYRO = true;
//		osDelay(5000);
//		HAL_UART_Transmit(&huart3,(uint8_t *)'N', 1, 0xFFFF);
//	}
//	else {
//		HAL_UART_Transmit(&huart3,(uint8_t *)'Y', 1, 0xFFFF);

		osDelay(500);

		// angle correction
		float diff;
		float left_target = 89;
		float right_target = 88;
		float threshold = 1;
		if (left == true) {
			diff = YAW-yaw;
			if (diff > left_target+threshold) {
				for (;;) {

					htim1.Instance->CCR4 = 152; // Turning Right 4210

					osDelay(200);

					HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
					__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal); // Modify the comparison value for the duty cycle
					__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal); // Modify the comparison value for the duty cycle

					osDelay(200);

					__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0); // Modify the comparison value for the duty cycle
					__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0); // Modify the comparison value for the duty cycle

					htim1.Instance->CCR4 = 140;

					osDelay(200);

					HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
					__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal); // Modify the comparison value for the duty cycle
					__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal); // Modify the comparison value for the duty cycle

					osDelay(200);

					__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0); // Modify the comparison value for the duty cycle
					__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0); // Modify the comparison value for the duty cycle

					diff = YAW-yaw;
					if (diff <= left_target+threshold)
						break;
					if (YAW == 0.00)
						break;
					osDelay(100);
				}

			}
			else if (diff < left_target-threshold) {
				for (;;) {

					htim1.Instance->CCR4 = 140;

					osDelay(200);

					HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
					__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal); // Modify the comparison value for the duty cycle
					__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal); // Modify the comparison value for the duty cycle

					osDelay(200);

					__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0); // Modify the comparison value for the duty cycle
					__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0); // Modify the comparison value for the duty cycle

					htim1.Instance->CCR4 = 152; // Turning Right 4210

					osDelay(200);

					HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
					__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal); // Modify the comparison value for the duty cycle
					__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal); // Modify the comparison value for the duty cycle

					osDelay(200);

					__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0); // Modify the comparison value for the duty cycle
					__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0); // Modify the comparison value for the duty cycle

					diff = YAW-yaw;
					if (diff >= left_target-threshold)
						break;
					if (YAW == 0.00)
						break;
					osDelay(100);
				}

			}
		}
		// If right turn
		else {
			diff = yaw-YAW;
			if (diff < right_target-threshold) {
				for (;;) {

					htim1.Instance->CCR4 = 155; // Turning Right 4210

					osDelay(200);

					HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
					__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal); // Modify the comparison value for the duty cycle
					__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal); // Modify the comparison value for the duty cycle

					osDelay(200);

					__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0); // Modify the comparison value for the duty cycle
					__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0); // Modify the comparison value for the duty cycle

					htim1.Instance->CCR4 = 138;

					osDelay(200);

					HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
					__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal); // Modify the comparison value for the duty cycle
					__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal); // Modify the comparison value for the duty cycle

					osDelay(200);

					__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0); // Modify the comparison value for the duty cycle
					__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0); // Modify the comparison value for the duty cycle

					diff = yaw-YAW;
					if (diff >= right_target-threshold)
						break;
					if (YAW == 0.00)
						break;
					osDelay(100);
				}

			}
			else if (diff > right_target+threshold) {
				for (;;) {

					htim1.Instance->CCR4 = 138;

					osDelay(200);

					HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
					__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal); // Modify the comparison value for the duty cycle
					__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal); // Modify the comparison value for the duty cycle

					osDelay(200);

					__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0); // Modify the comparison value for the duty cycle
					__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0); // Modify the comparison value for the duty cycle

					htim1.Instance->CCR4 = 155; // Turning Right 4210

					osDelay(200);

					HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
					__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal); // Modify the comparison value for the duty cycle
					__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal); // Modify the comparison value for the duty cycle

					osDelay(200);

					__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0); // Modify the comparison value for the duty cycle
					__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0); // Modify the comparison value for the duty cycle

					diff = yaw-YAW;
					if (diff <= right_target+threshold)
						break;
					if (YAW == 0.00)
						break;
					osDelay(100);
				}

			}
//		}
	}

	htim1.Instance->CCR4 = 146;
	// Reset Timer Val
	__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
	TIM3->CNT = 0;
	return 'R';
}

char U_Turn(uint8_t dist) {
	uint16_t leftPwmVal = 1200;
	uint16_t rightPwmVal = 1200;
	int steps = dist-'0';
	int target;
	uint8_t buf[20];
	int16_t cnt1, cnt2;
	float diff = 0;
	bool stopTurning = false;

	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);

	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

	int reorient;
	int center = 143;

	float yaw = 0;
	yaw = YAW;

	htim1.Instance->CCR4 = center;	//center
	// Reset Timer Val
//	__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
//	TIM3->CNT = 0;
	osDelay(500);
	cnt1 = __HAL_TIM_GET_COUNTER(&htim3);

	for (;;) {
		// Lounge
		htim1.Instance->CCR4 = center;	//center
		// corridor
//			htim1.Instance->CCR4 = center;	//center
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);

		diff = abs(abs(YAW) - abs(yaw));
		// if tilt to right
		if (YAW < yaw)
			diff = 0 - diff;
		reorient = 4;
		if (diff >= 0.5)
		{
			// If tilt to right
			// turn right
			htim1.Instance->CCR4 = center + reorient;
			osDelay(10);
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal+20); // Modify the comparison value for the duty cycle
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal+300); // Modify the comparison value for the duty cycle
		}
		else if (diff <= -0.5) {
			// If tilt to left
			// turn right
			htim1.Instance->CCR4 = center + reorient;
			osDelay(10);
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal+300); // Modify the comparison value for the duty cycle
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal+20); // Modify the comparison value for the duty cycle

		}

		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,rightPwmVal); // Modify the comparison value for the duty cycle
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,leftPwmVal); // Modify the comparison value for the duty cycle

		target = 663*steps;
//		target = 5000;

		cnt2 = __HAL_TIM_GET_COUNTER(&htim3);

		if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3)) {
//			if (cnt1 > cnt2){
				if ((cnt1-cnt2) >= target)
					break;
//			}
//			else{
//				if ((cnt1 + 65535 - cnt2) >= target)
//					break;
//			}
//			else {
//				// if less than half the distance
//				if ((cnt1-cnt2) < target/2 && leftPwmVal < 4000) {
//					rightPwmVal += 60;
//					leftPwmVal += 60;
//				}
//				else if ((cnt1-cnt2) >= target/2 && leftPwmVal > 800) {
//					rightPwmVal -= 300;
//					leftPwmVal -= 300;
//				}
//
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,rightPwmVal); // Modify the comparison value for the duty cycle
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,leftPwmVal); // Modify the comparison value for the duty cycle
//			}
		}
		else {
//			if (cnt2 > cnt1){
				if ((cnt2-cnt1) >= target)
					break;
//			} else {
//				if ((cnt2 + 65535 -cnt1) >= target)
//					break;
//			}
//			else {
//				// if less than half the distance
//				if ((cnt2-cnt1) < target/2 && rightPwmVal < 4000) {
//					rightPwmVal += 60;
//					leftPwmVal += 60;
//				}
//				else if ((cnt2-cnt1) >= target/2 && rightPwmVal > 800) {
//					rightPwmVal -= 300;
//					leftPwmVal -= 300;
//				}
//
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,rightPwmVal); // Modify the comparison value for the duty cycle
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,leftPwmVal); // Modify the comparison value for the duty cycle
//			}
		}
	}

	yaw = YAW;

	htim1.Instance->CCR4 = 240;	//right

	for(;;) {
		if (yaw - YAW >= 88.0)
			break;
	}

	htim1.Instance->CCR4 = center;	//right

//	// Reset Timer Val
//	__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
//	TIM3->CNT = 0;

	// reading side IR
	while (IR(hadc1) != 0) {

	};

	int obsDist = __HAL_TIM_GET_COUNTER(&htim3);

	yaw = YAW;

	for(;;) {
			if (YAW - yaw >= 176.0)
				break;
	}

//	// Reset Timer Val
//	__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
//	TIM3->CNT = 0;
	int currVal = __HAL_TIM_GET_COUNTER(&htim3);

	for (;;) {
		// Lounge
		htim1.Instance->CCR4 = center;	//center
		// corridor
//			htim1.Instance->CCR4 = center;	//center
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);

		diff = abs(abs(YAW) - abs(yaw));
		// if tilt to right
		if (YAW < yaw)
			diff = 0 - diff;
		reorient = 4;
		if (diff >= 0.5)
		{
			// If tilt to right
			// turn right
			htim1.Instance->CCR4 = center + reorient;
			osDelay(10);
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal+20); // Modify the comparison value for the duty cycle
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal+300); // Modify the comparison value for the duty cycle
		}
		else if (diff <= -0.5) {
			// If tilt to left
			// turn right
			htim1.Instance->CCR4 = center + reorient;
			osDelay(10);
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal+300); // Modify the comparison value for the duty cycle
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal+20); // Modify the comparison value for the duty cycle

		}

		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,rightPwmVal); // Modify the comparison value for the duty cycle
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,leftPwmVal); // Modify the comparison value for the duty cycle

		target = 663*6 + currVal;
//		target = 5000;

		cnt2 = __HAL_TIM_GET_COUNTER(&htim3);

		if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3)) {
//			if (cnt1 > cnt2){
				if ((cnt1-cnt2) >= target)
					break;
//			}
//			else{
//				if ((cnt1 + 65535 - cnt2) >= target)
//					break;
//			}
//			else {
//				// if less than half the distance
//				if ((cnt1-cnt2) < target/2 && leftPwmVal < 4000) {
//					rightPwmVal += 60;
//					leftPwmVal += 60;
//				}
//				else if ((cnt1-cnt2) >= target/2 && leftPwmVal > 800) {
//					rightPwmVal -= 300;
//					leftPwmVal -= 300;
//				}
//
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,rightPwmVal); // Modify the comparison value for the duty cycle
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,leftPwmVal); // Modify the comparison value for the duty cycle
//			}
		}
		else {
//			if (cnt2 > cnt1){
				if ((cnt2-cnt1) >= target)
					break;
//			} else {
//				if ((cnt2 + 65535 -cnt1) >= target)
//					break;
//			}
//			else {
//				// if less than half the distance
//				if ((cnt2-cnt1) < target/2 && rightPwmVal < 4000) {
//					rightPwmVal += 60;
//					leftPwmVal += 60;
//				}
//				else if ((cnt2-cnt1) >= target/2 && rightPwmVal > 800) {
//					rightPwmVal -= 300;
//					leftPwmVal -= 300;
//				}
//
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,rightPwmVal); // Modify the comparison value for the duty cycle
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,leftPwmVal); // Modify the comparison value for the duty cycle
//			}
		}
	}

	yaw = YAW;

	htim1.Instance->CCR4 = 240;	//right

	for(;;) {
				if (YAW - yaw >= 176.0)
					break;
		}

	htim1.Instance->CCR4 = center;	//right

//	// Reset Timer Val
//	__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
//	TIM3->CNT = 0;

	currVal = __HAL_TIM_GET_COUNTER(&htim3);

	for (;;) {
		// Lounge
		htim1.Instance->CCR4 = center;	//center
		// corridor
//			htim1.Instance->CCR4 = center;	//center
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);

		diff = abs(abs(YAW) - abs(yaw));
		// if tilt to right
		if (YAW < yaw)
			diff = 0 - diff;
		reorient = 4;
		if (diff >= 0.5)
		{
			// If tilt to right
			// turn right
			htim1.Instance->CCR4 = center + reorient;
			osDelay(10);
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal+20); // Modify the comparison value for the duty cycle
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal+300); // Modify the comparison value for the duty cycle
		}
		else if (diff <= -0.5) {
			// If tilt to left
			// turn right
			htim1.Instance->CCR4 = center + reorient;
			osDelay(10);
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal+300); // Modify the comparison value for the duty cycle
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal+20); // Modify the comparison value for the duty cycle

		}

		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,rightPwmVal); // Modify the comparison value for the duty cycle
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,leftPwmVal); // Modify the comparison value for the duty cycle

		target = currVal + (663*6) - obsDist;
//		target = 5000;

		cnt2 = __HAL_TIM_GET_COUNTER(&htim3);

		if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3)) {
//			if (cnt1 > cnt2){
				if ((cnt1-cnt2) >= target)
					break;
//			}
//			else{
//				if ((cnt1 + 65535 - cnt2) >= target)
//					break;
//			}
//			else {
//				// if less than half the distance
//				if ((cnt1-cnt2) < target/2 && leftPwmVal < 4000) {
//					rightPwmVal += 60;
//					leftPwmVal += 60;
//				}
//				else if ((cnt1-cnt2) >= target/2 && leftPwmVal > 800) {
//					rightPwmVal -= 300;
//					leftPwmVal -= 300;
//				}
//
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,rightPwmVal); // Modify the comparison value for the duty cycle
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,leftPwmVal); // Modify the comparison value for the duty cycle
//			}
		}
		else {
//			if (cnt2 > cnt1){
				if ((cnt2-cnt1) >= target)
					break;
//			} else {
//				if ((cnt2 + 65535 -cnt1) >= target)
//					break;
//			}
//			else {
//				// if less than half the distance
//				if ((cnt2-cnt1) < target/2 && rightPwmVal < 4000) {
//					rightPwmVal += 60;
//					leftPwmVal += 60;
//				}
//				else if ((cnt2-cnt1) >= target/2 && rightPwmVal > 800) {
//					rightPwmVal -= 300;
//					leftPwmVal -= 300;
//				}
//
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,rightPwmVal); // Modify the comparison value for the duty cycle
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,leftPwmVal); // Modify the comparison value for the duty cycle
//			}
		}
	}

	yaw = YAW;

	htim1.Instance->CCR4 = 240;	//right

	for(;;) {
				if (yaw - YAW >= 88.0)
					break;
		}

	htim1.Instance->CCR4 = center;	//right



//	// Reset Timer Val
//	__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
//	TIM3->CNT = 0;

	currVal = __HAL_TIM_GET_COUNTER(&htim3);

	for (;;) {
		// Lounge
		htim1.Instance->CCR4 = center;	//center
		// corridor
//			htim1.Instance->CCR4 = center;	//center
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);

		diff = abs(abs(YAW) - abs(yaw));
		// if tilt to right
		if (YAW < yaw)
			diff = 0 - diff;
		reorient = 4;
		if (diff >= 0.5)
		{
			// If tilt to right
			// turn right
			htim1.Instance->CCR4 = center + reorient;
			osDelay(10);
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal+20); // Modify the comparison value for the duty cycle
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal+300); // Modify the comparison value for the duty cycle
		}
		else if (diff <= -0.5) {
			// If tilt to left
			// turn right
			htim1.Instance->CCR4 = center + reorient;
			osDelay(10);
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal+300); // Modify the comparison value for the duty cycle
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal+20); // Modify the comparison value for the duty cycle

		}

		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,rightPwmVal); // Modify the comparison value for the duty cycle
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,leftPwmVal); // Modify the comparison value for the duty cycle

		target = currVal + 663*steps;
//		target = 5000;

		cnt2 = __HAL_TIM_GET_COUNTER(&htim3);

		if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3)) {
//			if (cnt1 > cnt2){
				if ((cnt1-cnt2) >= target)
					break;
//			}
//			else{
//				if ((cnt1 + 65535 - cnt2) >= target)
//					break;
//			}
//			else {
//				// if less than half the distance
//				if ((cnt1-cnt2) < target/2 && leftPwmVal < 4000) {
//					rightPwmVal += 60;
//					leftPwmVal += 60;
//				}
//				else if ((cnt1-cnt2) >= target/2 && leftPwmVal > 800) {
//					rightPwmVal -= 300;
//					leftPwmVal -= 300;
//				}
//
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,rightPwmVal); // Modify the comparison value for the duty cycle
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,leftPwmVal); // Modify the comparison value for the duty cycle
//			}
		}
		else {
//			if (cnt2 > cnt1){
				if ((cnt2-cnt1) >= target)
					break;
//			} else {
//				if ((cnt2 + 65535 -cnt1) >= target)
//					break;
//			}
//			else {
//				// if less than half the distance
//				if ((cnt2-cnt1) < target/2 && rightPwmVal < 4000) {
//					rightPwmVal += 60;
//					leftPwmVal += 60;
//				}
//				else if ((cnt2-cnt1) >= target/2 && rightPwmVal > 800) {
//					rightPwmVal -= 300;
//					leftPwmVal -= 300;
//				}
//
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,rightPwmVal); // Modify the comparison value for the duty cycle
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,leftPwmVal); // Modify the comparison value for the duty cycle
//			}
		}
	}


	return 'R';
}

char Spot_Rotate(bool left) {

	uint16_t pwmVal = 1200;

	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);

	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

	int16_t cnt;

	for (;;) {
		htim1.Instance->CCR4 = 146;	//center

		if (left == true) {
			HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
		} else {
			HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
		}

		osDelay(10);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal); // Modify the comparison value for the duty cycle
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal); // Modify the comparison value for the duty cycle

		cnt = __HAL_TIM_GET_COUNTER(&htim2);

		if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)) {
			if (cnt <= -322) {
				break;
			}
		}
		else {
			if (cnt >= 322) {
				break;
			}
		}
	}
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0); // Modify the comparison value for the duty cycle
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0); // Modify the comparison value for the duty cycle

	// Reset Timer Val
	__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
	TIM2->CNT = 0;
	return 'R';
}

//IR sensor function
int IR(ADC_HandleTypeDef adcVal){
	uint16_t raw = 0;
	uint8_t buf[10];
	int voltage;
	int distance;
//	int raw = 0;

	///////IR Sensor Code////////////
	//Set GPIO pin to high
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);

	for (int i = 0; i < 20; i++) {
		//Get ADC value
		HAL_ADC_Start(&adcVal);
		HAL_ADC_PollForConversion(&adcVal, HAL_MAX_DELAY);
		// Get voltage reading
		raw += HAL_ADC_GetValue(&adcVal);
		//Set GPIO pin to low
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
	}

	voltage = (int)raw/20;

	if (voltage <= 2960 && voltage > 2500)			//10
		distance = 10;
	else if (voltage <= 2500 && voltage > 1845)		//15
		distance = 15;
	else if (voltage <= 1845 && voltage > 1460)		//20
		distance = 20;
	else if (voltage <= 1460 && voltage > 1275)		//25
		distance = 25;
	else if (voltage <= 1275 && voltage > 1135)		//30
		distance = 30;
	else if (voltage <= 1135 && voltage > 1020)		//35
		distance = 35;
	else if (voltage <= 1020 && voltage > 925)		//40
		distance = 40;
	else if (voltage <= 925 && voltage > 815)		//45
		distance = 45;
	else if (voltage <= 815 && voltage > 710)		//50
		distance = 50;
	// Nothing within 50cm
	else if (voltage <= 710)
		distance = 0;
	// Too close
	else
		distance = 5;

	// Trasmit to RPi
//	sprintf((char*) buf, "%d", distance);
//	HAL_UART_Transmit(&huart3, buf, strlen((char*) buf), HAL_MAX_DELAY);


//	return voltage;
	return distance;
}

/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	uint8_t receiveBuffer[4];
	uint8_t ch;
	uint8_t char2;
	uint8_t display_buf[20];
	bool have_var = false;
	bool two_digits = false;
	int count = 1;
	int var;
	int var2;

	/* Infinite loop */
	for (;;) {

			int irSide = IR(hadc1);
			int irFront = IR(hadc1);
			sprintf(display_buf, "%5d %5d", irSide, irFront);
			OLED_ShowString(10, 20, display_buf);
//
//			__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
//			TIM3->CNT = 0;

		HAL_UART_Receive_IT(&huart3, (uint8_t*) aRxBuffer, 1);
		memcpy(receiveBuffer, aRxBuffer, sizeof(char));
		switch (receiveBuffer[0]) {
		case 'W':
//					osDelay(100);
			if (have_var) {
				if (two_digits) {
					var = (var * 10) + var2;
				}
				ch = Straight_Move(true, var);
				have_var = false;
			} else {
				ch = Straight_Move(true, 1);
			}
			// reset var
			two_digits = false;
			count = 1;
			ch = 'R';
			HAL_UART_Transmit(&huart3, (uint8_t*) &ch, 1, 0xFFFF);
			break;
		case 'S':
//					osDelay(100);
			if (have_var) {
				if (two_digits) {
					var = (var * 10) + var2;
				}
				ch = Straight_Move(false, var);
				have_var = false;
			} else {
				ch = Straight_Move(false, 1);
			}
			// reset var
			two_digits = false;
			count = 1;
			ch = 'R';
			HAL_UART_Transmit(&huart3, (uint8_t*) &ch, 1, 0xFFFF);
			break;
		case 'Q':
			//				ch = Turning(true, true, 90);
			//				HAL_UART_Transmit(&huart3,(uint8_t *)&ch, 1, 0xFFFF);
			//				break;
//					osDelay(100);
			if (have_var) {
				if (two_digits) {
					var = (var * 10) + var2;
				}
//						ch = Turning(true, true, var);
				ch = Tight_Turn(true, true);
				have_var = false;
			} else {
//						ch = Turning(true, true, 9);
				ch = Tight_Turn(true, true);
			}
			// reset var
			two_digits = false;
			count = 1;
			ch = 'R';
			HAL_UART_Transmit(&huart3, (uint8_t*) &ch, 1, 0xFFFF);
			break;
		case 'E':
			//				ch = Turning(true, false, 9);
			//				HAL_UART_Transmit(&huart3,(uint8_t *)&ch, 1, 0xFFFF);
			//				break;
//					osDelay(100);
			if (have_var) {
				if (two_digits) {
					var = (var * 10) + var2;
				}
//						ch = Turning(true, false, var);
				ch = Tight_Turn(true, false);
				have_var = false;
			} else {
//						ch = Turning(true, false, 9);
				ch = Tight_Turn(true, false);
			}
			// reset var
			two_digits = false;
			count = 1;
			ch = 'R';
			HAL_UART_Transmit(&huart3, (uint8_t*) &ch, 1, 0xFFFF);
			break;
		case 'A':
//					ch = Turning(false, true, 9);
			osDelay(100);
			ch = Tight_Turn(false, true);
			HAL_UART_Transmit(&huart3, (uint8_t*) &ch, 1, 0xFFFF);
			break;
		case 'D':
//					ch = Turning(false, false, 9);
			osDelay(100);
			ch = Tight_Turn(false, false);
			HAL_UART_Transmit(&huart3, (uint8_t*) &ch, 1, 0xFFFF);
			break;
		case 'O':
			IR(hadc1);
			break;
		case 'U':
//					ch = U_Turn();
			HAL_UART_Transmit(&huart3, (uint8_t*) &ch, 1, 0xFFFF);
			break;
		case '0':
		case '1':
		case '2':
		case '3':
		case '4':
		case '5':
		case '6':
		case '7':
		case '8':
		case '9':
//					ch = '1';
//					HAL_UART_Transmit(&huart3,(uint8_t *)&ch, 1, 0xFFFF);
//					have_var = true;
//					// if second digit
//					if (count == 2) {
//						ch = '2';
//						HAL_UART_Transmit(&huart3,(uint8_t *)&ch, 1, 0xFFFF);
//						var2 = receiveBuffer[0]-'0';
//						two_digits = true;
//						count = 1;
//					}
//					// if first digit
//					else {
//						var = receiveBuffer[0]-'0';
//						count++;
//					}

				U_Turn(receiveBuffer[0]);

				break;
			default:
				break;
		}
		memmove(aRxBuffer, aRxBuffer+1, sizeof(aRxBuffer)-1);
//		HAL_UART_Transmit(&huart3,(uint8_t *)&ch, 1, 0xFFFF); // acknowledge
//		if (ch < 'Z')
//			ch++;
//		else
//			ch = 'A';

		osDelay(100);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Encoder */
/**
 * @brief Function implementing the encoder thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Encoder */
void Encoder(void *argument)
{
  /* USER CODE BEGIN Encoder */
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

	int cnt, cnt1, cnt2, diff;
	uint32_t tick;

	cnt1 = __HAL_TIM_GET_COUNTER(&htim3);
	tick = HAL_GetTick();
	uint8_t hello[20];
	uint16_t dir;
	/* Infinite loop */

	for (;;) {
		if (HAL_GetTick() - tick > 1000L) {
			cnt2 = __HAL_TIM_GET_COUNTER(&htim3);
			if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3)) {
				if (cnt2 < cnt1)
					diff = cnt1 - cnt2;
				else
					diff = (65535 - cnt2) + cnt1;
			} else {
				if (cnt2 > cnt1)
					diff = cnt2 - cnt1;
				else
					diff = (65535 - cnt1) + cnt2;
			}

			sprintf(hello, "Speed:%5d\0", diff);
//			OLED_ShowString(10, 20, hello);
			dir = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3);
			sprintf(hello, "Dir:%5d\0", dir);
//			OLED_ShowString(10, 30, hello);
			cnt = __HAL_TIM_GET_COUNTER(&htim3);
			sprintf(hello, "%5d\0", cnt);
			OLED_ShowString(10, 30, hello);
			cnt1 = __HAL_TIM_GET_COUNTER(&htim3);
			tick = HAL_GetTick();
		}
		osDelay(1);
	}
  /* USER CODE END Encoder */
}

/* USER CODE BEGIN Header_Display */
/**
 * @brief Function implementing the display thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Display */
void Display(void *argument)
{
  /* USER CODE BEGIN Display */
	uint8_t display_buf[20] = "Hello World!\0";
	/* Infinite loop */
	for (;;) {
		sprintf(display_buf, "%s\0", aRxBuffer);
		OLED_ShowString(10, 10, display_buf);

		// IR display
//		sprintf(display_buf, "%hu\0", IR_READ);
//		OLED_ShowString(10, 40, display_buf);

		sprintf(display_buf, "%5.2f", YAW);
		OLED_ShowString(10, 40, display_buf);

		OLED_Refresh_Gram();
		osDelay(1000);
	}
  /* USER CODE END Display */
}

/* USER CODE BEGIN Header_Gyroscope */
/**
 * @brief Function implementing the gyroscope thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Gyroscope */
void Gyroscope(void *argument)
{
  /* USER CODE BEGIN Gyroscope */
	uint8_t buf[20];
	float offset;
//	float pitch, roll, yaw;
//	float Xh, Yh;
	ICM20948_Init();
	AK09916_Init();

	// gyro cali
	uint32_t diff, tick;
	for (int i = 1; i <= 100; i++){
		tick = HAL_GetTick();
		ICM20948_Gyro_Read_dps(&my_gyro);
		osDelay(10);
		offset += ((HAL_GetTick()-tick)*(my_gyro.z))/1000;
	}
	offset /= 100;

	for (;;) {
		tick = HAL_GetTick();

		ICM20948_Gyro_Read_dps(&my_gyro);
//		ICM20948_Accel_Read_g(&my_accel);
//		AK09916_Mag_Read_uT(&my_mag);

//		pitch = atan2f(my_accel.y, (sqrtf((my_accel.x * my_accel.x) + (my_accel.z * my_accel.z))));
//		roll = atan2f(-my_accel.x, (sqrtf((my_accel.y * my_accel.y) + (my_accel.z * my_accel.z))));
//		Yh = (my_mag.y * cosf(roll)) - (my_mag.z * sinf(roll));
//		Xh = (my_mag.x * cosf(pitch)) + (my_mag.y * sinf(roll) * sinf(pitch)) + (my_mag.z * cosf(roll) * sinf(pitch));
//		YAW = atan2f(Yh, Xh);
//		YAW = 180 * atanf(my_accel.z/sqrtf(my_accel.x*my_accel.x + my_accel.y*my_accel.y))/M_PI;
//		sprintf(buf, "Pitch:%5.2f", pitch);
//		OLED_ShowString(10, 20, buf);
//		sprintf(buf, "Roll: %5.2f", roll);
//		OLED_ShowString(10, 30, buf);
//		sprintf(buf, "Yaw:  %5.2f", YAW);
//		OLED_ShowString(10, 40, buf);
		osDelay(10);
		YAW += ((HAL_GetTick()-tick)*(my_gyro.z))/1000-offset;

//		sprintf(buf, "X:%5.2f", my_accel.x);
//		OLED_ShowString(10, 20, buf);
//		sprintf(buf, "Gyro:%5.2f", my_gyro.z);
//		OLED_ShowString(10, 30, buf);
//		sprintf(buf, "%5.2f", YAW);
//		OLED_ShowString(10, 40, buf);
	}
  /* USER CODE END Gyroscope */
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

