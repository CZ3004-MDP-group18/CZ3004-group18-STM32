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

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for encoder */
osThreadId_t encoderHandle;
const osThreadAttr_t encoder_attributes = {
  .name = "encoder",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for display */
osThreadId_t displayHandle;
const osThreadAttr_t display_attributes = {
  .name = "display",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for gyroscope */
osThreadId_t gyroscopeHandle;
const osThreadAttr_t gyroscope_attributes = {
  .name = "gyroscope",
  .stack_size = 128 * 4,
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
char Spot_Rotate(bool);
char U_Turn();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static SemaphoreHandle_t Mutex;
uint8_t aRxBuffer[20];
static float YAW = 0.0;
static int IR_READ = -1;

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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**I2C1 GPIO Configuration
  PB8   ------> I2C1_SCL
  PB9   ------> I2C1_SDA
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8|LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  /** I2C Initialization
  */
  LL_I2C_DisableOwnAddress2(I2C1);
  LL_I2C_DisableGeneralCall(I2C1);
  LL_I2C_EnableClockStretching(I2C1);
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.ClockSpeed = 100000;
  I2C_InitStruct.DutyCycle = LL_I2C_DUTYCYCLE_2;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C1, &I2C_InitStruct);
  LL_I2C_SetOwnAddress2(I2C1, 0);
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

//IR sensor function
int IR(ADC_HandleTypeDef hadc1){
	uint16_t raw;
//	uint8_t msg[10];
	int voltage;
	int distance;
	double temp;

	///////IR Sensor Code////////////
	//Set GPIO pin to high
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);

	//Get ADC value
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	raw = HAL_ADC_GetValue(&hadc1);

	//Set GPIO pin to low
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

//	//Convert to string and display on OLED
	voltage = (int)raw;

	if (voltage <= 2960 && voltage > 2500)
		distance = 1;
	else if (voltage <= 2500 && voltage > 1845)
		distance = 2;
	else if (voltage <= 1845 && voltage > 1460)
		distance = 3;
	else if (voltage <= 1460 && voltage > 1275)
		distance = 4;
	else if (voltage <= 1275 && voltage > 1135)
		distance = 5;
	else if (voltage <= 1135 && voltage > 1020)
		distance = 6;
	else if (voltage <= 1020 && voltage > 925)
		distance = 7;
	else if (voltage <= 925 && voltage > 815)
		distance = 8;
	else if (voltage <= 815 && voltage > 710)
		distance = 9;
	// Nothing within 50cm
	else if (voltage <= 710)
		distance = 0;
	// Too close
	else
		distance = -1;

	IR_READ = distance;
//	IR_READ = voltage;

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
		bool have_var = false;
		bool two_digits = false;
		int count = 1;
		int var;
		int var2;

		/* Infinite loop */
		for (;;) {

			IR(hadc1);

			HAL_UART_Receive_IT(&huart3, (uint8_t*) aRxBuffer, 1);
			memcpy(receiveBuffer, aRxBuffer, sizeof(char));
			switch(receiveBuffer[0]){
				case 'W':
					if (have_var){
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
					HAL_UART_Transmit(&huart3,(uint8_t *)&ch, 1, 0xFFFF);
					break;
				case 'S':
					if (have_var){
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
					HAL_UART_Transmit(&huart3,(uint8_t *)&ch, 1, 0xFFFF);
					break;
				case 'Q':
	//				ch = Turning(true, true, 90);
	//				HAL_UART_Transmit(&huart3,(uint8_t *)&ch, 1, 0xFFFF);
	//				break;

					if (have_var){
						if (two_digits) {
							var = (var * 10) + var2;
						}
						ch = Turning(true, true, var);
						have_var = false;
					} else {
						ch = Turning(true, true, 9);
					}
					// reset var
					two_digits = false;
					count = 1;
					ch = 'R';
					HAL_UART_Transmit(&huart3,(uint8_t *)&ch, 1, 0xFFFF);
					break;
				case 'E':
	//				ch = Turning(true, false, 9);
	//				HAL_UART_Transmit(&huart3,(uint8_t *)&ch, 1, 0xFFFF);
	//				break;

					if (have_var){
						if (two_digits) {
							var = (var * 10) + var2;
						}
						ch = Turning(true, false, var);
						have_var = false;
					} else {
						ch = Turning(true, false, 9);
					}
					// reset var
					two_digits = false;
					count = 1;
					ch = 'R';
					HAL_UART_Transmit(&huart3,(uint8_t *)&ch, 1, 0xFFFF);
					break;
				case 'A':
					ch = Turning(false, true, 9);
					HAL_UART_Transmit(&huart3,(uint8_t *)&ch, 1, 0xFFFF);
					break;
				case 'D':
					ch = Turning(false, false, 9);
					HAL_UART_Transmit(&huart3,(uint8_t *)&ch, 1, 0xFFFF);
					break;
	//			case 8:
	//				HAL_UART_Receive_IT(&huart3, (uint8_t*) aRxBuffer, 1);
	//				if (aRxBuffer[0] == 'W')
	//					Straight_Move(false, 8);
				case 'Z':
					ch = Obstacle_Straight_Move();
					HAL_UART_Transmit(&huart3,(uint8_t *)&ch, 1, 0xFFFF);
					break;
				case 'O':
					ch = Spot_Rotate(true);
					HAL_UART_Transmit(&huart3,(uint8_t *)&ch, 1, 0xFFFF);
					break;
				case 'U':
					ch = U_Turn();
					HAL_UART_Transmit(&huart3,(uint8_t *)&ch, 1, 0xFFFF);
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
					ch = '1';
					HAL_UART_Transmit(&huart3,(uint8_t *)&ch, 1, 0xFFFF);
					have_var = true;
					// if second digit
					if (count == 2) {
						ch = '2';
						HAL_UART_Transmit(&huart3,(uint8_t *)&ch, 1, 0xFFFF);
						var2 = receiveBuffer[0]-'0';
						two_digits = true;
						count = 1;
					}
					// if first digit
					else {
						var = receiveBuffer[0]-'0';
						count++;
					}

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
		/* USER CODE END Display */
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
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

	int cnt1, cnt2, diff;
	uint32_t tick;

	cnt1 = __HAL_TIM_GET_COUNTER(&htim2);
	tick = HAL_GetTick();
	uint8_t hello[20];
	uint16_t dir;
	/* Infinite loop */

	for (;;) {
		if (HAL_GetTick() - tick > 1000L) {
			cnt2 = __HAL_TIM_GET_COUNTER(&htim2);
			if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)) {
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
			OLED_ShowString(10, 20, hello);
//			HAL_UART_Transmit(&huart3, hello, strlen((char*)hello), HAL_MAX_DELAY);
			dir = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2);
			sprintf(hello, "Dir:%5d\0", dir);
			OLED_ShowString(10, 30, hello);
			cnt1 = __HAL_TIM_GET_COUNTER(&htim2);
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
		sprintf(display_buf, "%hu\r\n", IR_READ);
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
	int16_t magdata_int[3];
	float magdata[3];
//	uint8_t buf[200];
	uint8_t ch;

	LL_mDelay(1000);
	printf("start\r\n");
	init_ICM20948(&icm20948, I2C1);
	printf("%.2X\r\n", ICM_who_am_i(&icm20948));
	init_AK09916(&icm20948);
	printf("%.2X\r\n", AK_Company_ID(&icm20948));
	printf("%.2X\r\n", AK_Device_ID(&icm20948));
	ICM_Gyrocali(&icm20948);
	ICM_Angcali(&icm20948);

	ch = 'R';
	HAL_UART_Transmit(&huart3,(uint8_t *)&ch, 1, 0xFFFF);

	/* Infinite loop */
	for (;;) {
		osDelay(100);
		ICM_ComplementaryFilter(&icm20948);
		AK_ReadData(&icm20948, magdata, magdata_int);

//		sprintf((char*) buf, "ptc:%.2f rol:%.2f yaw:%.2f\n\r", icm20948.pitch,
//				icm20948.roll, icm20948.yaw);
//		HAL_UART_Transmit(&huart3, buf, strlen((char*) buf), HAL_MAX_DELAY);

		// write, make sure that other tasks are not reading
		xSemaphoreTake(Mutex, portMAX_DELAY);
		YAW = icm20948.yaw;
		xSemaphoreGive(Mutex);
	}
  /* USER CODE END Gyroscope */
}

//IR(hadc1);
char Obstacle_Straight_Move() {

	uint16_t pwmVal = 1200;

	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);

	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

//	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

//	int16_t cnt;

	int dist = IR(hadc1);

	for (;;) {
		htim1.Instance->CCR4 = 148;	//center

		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);

		osDelay(10);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal); // Modify the comparison value for the duty cycle
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal); // Modify the comparison value for the duty cycle

		if (IR(hadc1) < 1300)
			break;
	}
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0); // Modify the comparison value for the duty cycle
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0); // Modify the comparison value for the duty cycle

	return 'R';
}

char Straight_Move(bool forward, int steps) {

	uint16_t pwmVal = 1200;

	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);

	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

	int16_t cnt1, cnt2;
	int target;
	cnt1 = __HAL_TIM_GET_COUNTER(&htim3);
	for (;;) {
		htim1.Instance->CCR4 = 148;	//center

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

		osDelay(10);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal); // Modify the comparison value for the duty cycle
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal); // Modify the comparison value for the duty cycle

		target = 370*steps - 48;
		cnt2 = __HAL_TIM_GET_COUNTER(&htim3);

		if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3)) {
			if ((cnt1-cnt2) >= target) {
				break;
			}
			else {
				// if less than half the distance
//				if (cnt > target/2 && pwmVal < 2000)
//					pwmVal += 40;
//				else if (cnt < target/2 && pwmVal > 1200)
//					pwmVal -= 40;

				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal); // Modify the comparison value for the duty cycle
				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal); // Modify the comparison value for the duty cycle
			}
		}
		else {
			if ((cnt2-cnt1) >= target) {
				break;
			}
			else {
				// if less than half the distance
//				if (cnt < target/2 && pwmVal < 2000)
//					pwmVal += 40;
//				else if (cnt > target/2 && pwmVal > 1200)
//					pwmVal -= 40;

				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal); // Modify the comparison value for the duty cycle
				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal); // Modify the comparison value for the duty cycle
			}
		}
	}
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0); // Modify the comparison value for the duty cycle
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0); // Modify the comparison value for the duty cycle

	// Reset Timer Val
	__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
	TIM2->CNT = 0;
	return 'R';
}


// For turning with gyroscope
//char Turning(bool forward, bool left, int angle) {
//	uint16_t pwmVal = 1200;
//	uint8_t buf[20];
//	int16_t cnt;
//	float yaw = 0;
//	float diff = 0;
//	bool stopTurning = false;
//
//	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
//
//	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
//
//	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
//
//	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
//
//	for (;;) {
//		xSemaphoreTake(Mutex, portMAX_DELAY);
//		if (yaw == 0)
//			yaw = YAW;
//		xSemaphoreGive(Mutex);
//
//		if (left == true) {
//			htim1.Instance->CCR4 = 100; // Turning Left
//		} else {
//			htim1.Instance->CCR4 = 235; // Turning Right
//		}
//
//		if (forward == true) {
//			HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
//			HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
//			HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
//			HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
//		} else {
//			HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
//			HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
//			HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
//			HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
//		}
//
////		// for 90deg left turn
////		if (left == true) {
////			xSemaphoreTake(Mutex, portMAX_DELAY);
////			diff = yaw - YAW;
////			xSemaphoreGive(Mutex);
////		} else {
////			xSemaphoreTake(Mutex, portMAX_DELAY);
////			diff = YAW - yaw;
////			xSemaphoreGive(Mutex);
////		}
////
////		osDelay(10);
////		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal); // Modify the comparison value for the duty cycle
////		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal); // Modify the comparison value for the duty cycle
////
////		// Separate branching so that we don't carry on moving while
////		// semaphore is not taken
////		while (stopTurning == false) {
////			if (left == true) {
////				if (diff >= 1.68) {
////					// 1.68 is the gyroscope value for 90deg left turn
////					stopTurning = true;
////				}
////			}
////			else {
////				// for 90deg right turn
////				if (diff >= 1.69) {
////					// 1.69 is the gyroscope value for 90deg right turn
////					stopTurning = true;
////				}
////			}
////		}
////		// Reach break when while loop condition is not met
////		break;
////	}
////	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0); // Modify the comparison value for the duty cycle
////	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0); // Modify the comparison value for the duty cycle
////	htim1.Instance->CCR4 = 148;
////	return 'R';
//
//		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal); // Modify the comparison value for the duty cycle
//		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal); // Modify the comparison value for the duty cycle
//		osDelay(10); // Move on top
//
//		// for 90deg left turn
//
//		if (((left == true) && (forward == true)) || ((left == false) && (forward == false))) {
//			xSemaphoreTake(Mutex, portMAX_DELAY);
//			diff = yaw - YAW;
//			xSemaphoreGive(Mutex);
//			if (diff >= (1.73/90)*(angle*10)) {
//				// 1.68 is the gyroscope value for 90deg left turn
//				htim1.Instance->CCR4 = 148;
////				cnt = __HAL_TIM_GET_COUNTER(&htim2);
////				sprintf(buf, "%d\0", cnt);
////				HAL_UART_Transmit(&huart3, buf, sizeof(buf), 0xFFFF); // acknowledge
//
//				break;
//			}
//		} else {
//			xSemaphoreTake(Mutex, portMAX_DELAY);
//			diff = YAW - yaw;
//			xSemaphoreGive(Mutex);
//			// for 90deg right turn
//			if (diff >= (1.73/90)*(angle*10)) {
//				// 1.69 is the gyroscope value for 90deg right turn
//				htim1.Instance->CCR4 = 148;
////				cnt = __HAL_TIM_GET_COUNTER(&htim2);
////				sprintf(buf, "%d\0", cnt);
////				HAL_UART_Transmit(&huart3, buf, sizeof(buf), 0xFFFF); // acknowledge
//
//				break;
//			}
//		}
//
//	}
//	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0); // Modify the comparison value for the duty cycle
//	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0); // Modify the comparison value for the duty cycle
//	osDelay(10);
//
//	// Reset Timer Val
//	__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
//	TIM2->CNT = 0;
//	return 'R';
//}


// For turning with encoder
char Turning(bool forward, bool left, int angle) {
	uint16_t pwmVal = 1200;
	uint8_t buf[20];
	int16_t cnt;
	float yaw = 0;
	int target;
	bool stopTurning = false;

	// Reset Timer Val
	__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
	TIM2->CNT = 0;

	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);

	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

	if (left == true) {
		htim1.Instance->CCR4 = 100; // Turning Left
		if (forward == true)
			target = 1825;
		else
			target = 1835;
	} else {
		htim1.Instance->CCR4 = 230; // Turning Right
		if (forward == true)
			target = 1825;
		else
			target = 1835;
	}

	for (;;) {
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

		// for 90deg left turn
		cnt = __HAL_TIM_GET_COUNTER(&htim3);

		if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3)) {
			if (cnt <= 0-target) {
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
	htim1.Instance->CCR4 = 148; // Forward

	// Reset Timer Val
	__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
	TIM2->CNT = 0;
	return 'R';
}

char U_Turn() {
	uint16_t pwmVal = 1200;
	uint8_t buf[20];
	int16_t cnt;
	float yaw = 0;
	float diff = 0;
	bool stopTurning = false;

	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);

	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

	for (;;) {
		xSemaphoreTake(Mutex, portMAX_DELAY);
		if (yaw == 0)
			yaw = YAW;
		xSemaphoreGive(Mutex);

		htim1.Instance->CCR4 = 100; // Turning Left


		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);

		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal); // Modify the comparison value for the duty cycle
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal); // Modify the comparison value for the duty cycle
		osDelay(10); // Move on top

		// for 180deg left turn
		xSemaphoreTake(Mutex, portMAX_DELAY);
		diff = yaw - YAW;
		xSemaphoreGive(Mutex);
		if (diff >= 3.40) {
			// 1.68 is the gyroscope value for 90deg left turn
			htim1.Instance->CCR4 = 148;

			break;
		}

	}
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0); // Modify the comparison value for the duty cycle
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0); // Modify the comparison value for the duty cycle
	osDelay(10);

	// Reset Timer Val
	__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
	TIM2->CNT = 0;
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
		htim1.Instance->CCR4 = 148;	//center

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

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
