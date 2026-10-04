/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
/*
 * MCP2515-over-SPI2 driver, same library used by the rest of the rover's
 * CAN nodes. Provides uCAN_MSG, dSTANDARD_CAN_MSG_ID_2_0B, and
 * CANSPI_Initialize/Transmit/Receive().
 */
#include "CANSPI.h"
#include "MCP2515.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*
 * ---- CAN arbitration IDs ----
 * Same scheme as the other STM32 nodes: (node << 5) | msg, 5-bit msg type.
 * msg-type values are the wrist node's set.
 */
#define gripper 10

#define CAN_FAIL 10000

#define SENSOR_DATA   0b00010  /* SEND QUAD/CURRENT DATAS */
#define SENSORS_CHECK 0b11111  /* SENSOR CHECK AT BEGINNING */

#define HEARTBEAT     0b00000  /* HEARTBEAT RECEIVED */
#define HEARTBEATED   0b00001  /* HEARTBEAT SENT */

#define GRIPPER_PWM   0b10010  /* GRIPPER COMMAND TO BE RECEIVED */
#define START_NODE    0b10100  /* NODE START COMMAND */

#define MAKE_ARBITRATION_ID(node, msg) (((node) << 5) | (msg))

#define gripper_command_msg  MAKE_ARBITRATION_ID(gripper, GRIPPER_PWM)
#define sensor_data_msg      MAKE_ARBITRATION_ID(gripper, SENSOR_DATA)
#define heartbeat_msg        MAKE_ARBITRATION_ID(gripper, HEARTBEAT)
#define heartbeated_msg      MAKE_ARBITRATION_ID(gripper, HEARTBEATED)
#define start_node_msg       MAKE_ARBITRATION_ID(gripper, START_NODE)
#define sensor_check_msg     MAKE_ARBITRATION_ID(gripper, SENSORS_CHECK)

/* Watchdog: if this many main-loop iterations pass with no CAN frame
 * received at all, assume the bus/supervisor is gone, zero the outputs,
 * and require a fresh start_node handshake before actuating again. */
#define CAN_FAILED_THRESHOLD       CAN_FAIL

/* Sensor telemetry (GripperEncoders equivalent) broadcast period. */
#define SENSOR_BROADCAST_PERIOD_MS 50U

/* PWM: TIM5 CH3 (PA2, N20) as configured in the .ioc:
 * 72MHz APB1 timer clock, PSC=71 -> 1MHz (1us/tick), ARR=999 -> 1kHz.
 * Servo (PA3) runs on TIM9 CH2 (AF3), set up in Servo_Init(): 72MHz APB2
 * timer clock, PSC=71 -> 1us/tick, ARR=19999 -> 50Hz, pulse 1000-2000us.
 */
#define PWM_PERIOD_TICKS           1000U  /* ARR+1 */
#define SERVO_MIN_US               1000U
#define SERVO_MAX_US               2000U
#define SERVO_PERIOD_TICKS         20000U /* ARR+1, 50Hz at 1us/tick */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */
/*
 * Gripper CAN protocol (matches gripperrr.py's GripperPositionCommand /
 * GripperEncoders exactly):
 *
 * ROS -> STM32 (gripper_command_msg, dlc=2 or 3):
 *   data0 = dcm1  N20 motor command, 0-255 raw byte  -> TIM5_CH3 (PA2)
 *   data1 = dcm2  Servo command,     0-255 raw byte  -> TIM9_CH2 (PA3), 1-2ms pulse at 50Hz
 *   data2 = N20 direction, optional (dlc>=3): 0 = forward, non-zero = reverse -> DIR1 (PA4).
 *           With dlc=2 the direction stays forward (0).
 *
 * STM32 -> ROS (sensor_data_msg, dlc=5, every SENSOR_BROADCAST_PERIOD_MS):
 *   data0 = quad_1     TIM2 quadrature encoder (PA0/PA1), low byte of count
 *   data1 = quad_2     TIM3 quadrature encoder (PA6/PA7), low byte of count
 *   data2 = acs_1      ADC1 ch8 (PB0) current sensor, low byte (12-bit)
 *   data3 = acs_2      ADC1 ch9 (PB1) current sensor, low byte (12-bit)
 *   data4 = up_check   always 0 - no discrete status/limit-switch bits
 *                      are defined for this node
 */
volatile uint8_t dcm1 = 0;    /* N20 motor command, 0-255 */
volatile uint8_t dcm2 = 128;  /* Servo command, 0-255, ~center on boot */

volatile uint16_t adc_buf[2];

uCAN_MSG rx;
uCAN_MSG tx_heartbeat;
uCAN_MSG tx_sensor;

volatile bool beat_pls = false;
volatile uint8_t heartbeater = 0;

volatile bool start_node = false;
volatile bool start_node_init = false;
volatile uint32_t CAN_failed_counter = 0;

bool CAN_checker = false;
uint32_t lastSensorBroadcastTick = 0;

TIM_HandleTypeDef htim9;       /* servo, TIM9 CH2 on PA3 */
static uint8_t n20_dir = 0;    /* current N20 direction, mirrors PA4 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
static void CAN_Heartbeat(void);
static void CAN_Send_SensorData(void);
static void CAN_Send_SensorCheck(void);
static uint16_t cast_to_arbid(uint16_t id);
static void delay_us(uint16_t us);
static void ApplyGripperCommand(uint8_t motorByte, uint8_t servoByte, uint8_t dir);
static void Servo_Init(void);
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);                 /* free-running us tick for delay_us() */

  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);   /* PWM_N20  */
  Servo_Init();                               /* PWM_SERVO on TIM9 CH2 (PA3), 50Hz */
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  /* Boot-time CAN init, with a PC13 (onboard BlackPill LED) blink so a
   * failure is visible without a debugger attached. */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  HAL_Delay(100);

  CAN_checker = CANSPI_Initialize();

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  HAL_Delay(100);

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, 2);
  __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE);

  lastSensorBroadcastTick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* ---- CAN watchdog: silent bus/supervisor -> fail safe ---- */
    if (CAN_failed_counter >= CAN_FAILED_THRESHOLD)
    {
      __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
      __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
      n20_dir = 0;
      dcm1 = 0;
      start_node = false;
    }

    /* ---- start_node handshake ----
     * Block here (blinking PC13) until the supervisor sends start_node_msg,
     * or answer sensor_check_msg while waiting. */
    if (!start_node)
    {
      HAL_Delay(500);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
      HAL_Delay(100);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
      HAL_Delay(100);
      while (!CANSPI_Receive(&rx)) {}
      if (cast_to_arbid(rx.frame.id) == start_node_msg)
      {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_Delay(100);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_Delay(100);
        start_node = true;
        start_node_init = true;
        delay_us(50);
      }
      else if (cast_to_arbid(rx.frame.id) == sensor_check_msg)
      {
        CAN_Send_SensorCheck();
      }
      else if (start_node_init)
      {
        start_node = true;
      }
    }

    /* ---- periodic sensor telemetry (GripperEncoders source) ---- */
    if (start_node && (HAL_GetTick() - lastSensorBroadcastTick) >= SENSOR_BROADCAST_PERIOD_MS)
    {
      lastSensorBroadcastTick = HAL_GetTick();
      CAN_Send_SensorData();
    }

    /* ---- normal-operation CAN receive dispatch ---- */
    if (CANSPI_Receive(&rx))
    {
      CAN_failed_counter = 0;

      if (start_node && cast_to_arbid(rx.frame.id) == gripper_command_msg)
      {
        ApplyGripperCommand(rx.frame.data0, rx.frame.data1,
                            (rx.frame.dlc >= 3 && rx.frame.data2) ? 1 : 0);
      }
      else if (cast_to_arbid(rx.frame.id) == heartbeat_msg)
      {
        /* Echo a heartbeat derived from the supervisor's ping. */
        heartbeater = rx.frame.data0 + rx.frame.data1;
        beat_pls = true;
        CAN_Heartbeat();
      }
    }

    CAN_failed_counter++;
    delay_us(100);
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 2;
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
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
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
  htim2.Init.Period = 4999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
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
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 71;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|SPI_debug_Pin|I2C_debug_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DIR1_Pin|DIR2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CAN_CS_Pin|IMU_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 SPI_debug_Pin I2C_debug_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_13|SPI_debug_Pin|I2C_debug_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR1_Pin DIR2_Pin */
  GPIO_InitStruct.Pin = DIR1_Pin|DIR2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : CAN_CS_Pin IMU_RST_Pin */
  GPIO_InitStruct.Pin = CAN_CS_Pin|IMU_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : IMU_INT_Pin */
  GPIO_InitStruct.Pin = IMU_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IMU_INT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  Sets up the servo output: TIM9 CH2 on PA3 (AF3), 50Hz, 1us/tick.
  *         Runs after MX_TIM5_Init(), so it takes PA3 over from TIM5 CH4.
  *         Pulse starts at 0 (no signal) until the first command.
  */
static void Servo_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_TIM9_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 71;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = SERVO_PERIOD_TICKS - 1U;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF3_TIM9;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  if (HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  Applies an inbound gripper_command_msg. motorByte/servoByte are
  *         raw 0-255 CAN payload bytes (gripperrr.py's dcm1/dcm2): the N20
  *         byte is scaled to 0..ARR on TIM5 CH3, the servo byte to a
  *         SERVO_MIN_US..SERVO_MAX_US pulse on TIM9 CH2. dir is the N20
  *         direction (0 forward, 1 reverse); on a change the PWM is zeroed
  *         first so the motor never reverses under duty.
  */
static void ApplyGripperCommand(uint8_t motorByte, uint8_t servoByte, uint8_t dir)
{
  dcm1 = motorByte;
  dcm2 = servoByte;

  if (dir != n20_dir)
  {
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
    n20_dir = dir;
  }

  uint32_t n20Compare = ((uint32_t)dcm1 * (PWM_PERIOD_TICKS - 1)) / 255U;
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, n20Compare);

  uint32_t servoUs = SERVO_MIN_US + ((uint32_t)dcm2 * (SERVO_MAX_US - SERVO_MIN_US)) / 255U;
  __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, servoUs);
}

/**
  * @brief  Broadcasts current encoder/current-sense readings as a
  *         sensor_data_msg CAN frame (GripperEncoders source).
  */
static void CAN_Send_SensorData(void)
{
  uint32_t quad1 = __HAL_TIM_GET_COUNTER(&htim2);
  uint32_t quad2 = __HAL_TIM_GET_COUNTER(&htim3);

  tx_sensor.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
  tx_sensor.frame.id = sensor_data_msg;
  tx_sensor.frame.dlc = 5;
  tx_sensor.frame.data0 = (uint8_t)(quad1 & 0xFFU);
  tx_sensor.frame.data1 = (uint8_t)(quad2 & 0xFFU);
  tx_sensor.frame.data2 = (uint8_t)(adc_buf[0] & 0xFFU);
  tx_sensor.frame.data3 = (uint8_t)(adc_buf[1] & 0xFFU);
  tx_sensor.frame.data4 = 0;                 /* up_check - no status bits defined */
  CANSPI_Transmit(&tx_sensor);
}

/**
  * @brief  Replies to a sensor_check_msg received while still waiting on
  *         the start_node handshake, by reporting current sensor values.
  */
static void CAN_Send_SensorCheck(void)
{
  CAN_Send_SensorData();
}

/**
  * @brief  Sends the current heartbeat frame if one is pending.
  */
static void CAN_Heartbeat(void)
{
  if (beat_pls)
  {
    tx_heartbeat.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
    tx_heartbeat.frame.id = heartbeated_msg;
    tx_heartbeat.frame.dlc = 1;
    tx_heartbeat.frame.data0 = heartbeater;
    beat_pls = false;
    CANSPI_Transmit(&tx_heartbeat);
  }
}

/**
  * @brief  Masks a composed id down to the 11-bit standard CAN ID range.
  */
static uint16_t cast_to_arbid(uint16_t id)
{
  return (id & 0x7FFU);
}

/**
  * @brief  Busy-wait microsecond delay using TIM1 as a free-running
  *         1us-tick counter.
  */
static void delay_us(uint16_t us)
{
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  while (__HAL_TIM_GET_COUNTER(&htim1) < us) {}
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
