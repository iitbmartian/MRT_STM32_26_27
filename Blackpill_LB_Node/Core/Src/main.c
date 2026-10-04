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
#include "CANSPI.h"
#include "MCP2515.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define pitch 5
#define Vsup 5
#define KE 0.0004

#define CAN_FAIL 10000

/* ---- CAN node identifiers (bits [10:5] of the 11-bit standard ID) ---- */
#define RPI          0
#define FR           1
#define FL           2
#define BR           3
#define BL           4
#define LINEAR_BASE  5   /* this node */

/* ---- CAN message-type identifiers (bits [4:0]) ---- */
#define HEARTBEAT      0b00010  /* Wheel_Node value */
#define HEARTBEATED    0b00110  /* Wheel_Node value */
#define POSN_SEND      0b00000  /* was 0b00010 == HEARTBEAT; Wheel's MOTOR_DATA slot */
#define PWM_RECEIVE    0b00011
#define PWM_INTERRUPT  0b01010  /* Wheel_Node value */
#define START_NODE     0b10100  /* mirrors Wheel_Node's value for the same semantic message */

#define BUSOFF_CHECK_PERIOD_MS 200

#define MAKE_ARBITRATION_ID(node, msg) (((node) << 5) | (msg))

#define CPR (1000*4)
#define POS_UNITS_PER_MM 100   /* position sent in 0.01 mm: int16 -> +-327.67 mm */
#define pi 3.141592
#define dt 0.0001
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
uCAN_MSG tx_linearbase;
uCAN_MSG tx_heartbeat;
uCAN_MSG tx_pwminterrupt;

uCAN_MSG rx;

uint16_t CAN_failed_counter = 0;
uint16_t counter = 0;
uint8_t timer = 0;

bool beat_pls = false;
uint8_t heartbeater = 0;

bool start_node = false;
uint16_t curr_loc = 0;  //this is the value of quadrature

volatile uint16_t adc_buf[2] = {0, 0}; /* [0]=ADC_CHANNEL_8/PB0 = real motor current, sent over CAN.
                                  [1]=ADC_CHANNEL_9/PB1 is vestigial leftover config copied from
                                  Wheel_Node's 2-motor setup — DMA fills it but it is never sent. */


static const uint16_t msg_linearbase = MAKE_ARBITRATION_ID(LINEAR_BASE, POSN_SEND);
static const uint16_t start_node_msg = MAKE_ARBITRATION_ID(LINEAR_BASE, START_NODE);
static const uint16_t msg_pwmreceive = MAKE_ARBITRATION_ID(LINEAR_BASE, PWM_RECEIVE);

static const uint16_t heartbeat_msg = MAKE_ARBITRATION_ID(LINEAR_BASE, HEARTBEAT);
static const uint16_t heartbeated_msg = MAKE_ARBITRATION_ID(LINEAR_BASE, HEARTBEATED);

static const uint16_t pwminterrupted_msg = MAKE_ARBITRATION_ID(LINEAR_BASE, PWM_INTERRUPT);
static uint32_t last_bussoff_check_tick = 0;

volatile uint8_t interrupted_left = 0;
volatile uint8_t interrupted_right = 0;
volatile bool left_edge_pending = false;
volatile bool right_edge_pending = false;
volatile uint8_t cur_dir = 0;       /* last commanded direction: 0 = left, 1 = right */
volatile uint32_t tx_drop_count = 0;
int32_t pos_counts = 0;             /* signed accumulated 4x encoder counts (TIM3) */
uint16_t last_cnt = 0;
volatile bool CAN_checker = false;
volatile bool start_node_init = 0;

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
void CAN_LinearBase(void){
	/* mm = counts * pitch / CPR (pitch in mm per rev, CPR = 4x counts per rev) */
	int32_t pos_units = (int32_t)((int64_t)pos_counts * pitch * POS_UNITS_PER_MM / CPR);
	if (pos_units > 32767) pos_units = 32767;
	if (pos_units < -32768) pos_units = -32768;
	curr_loc = (uint16_t)(int16_t)pos_units;
	tx_linearbase.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
	tx_linearbase.frame.id = msg_linearbase;
	tx_linearbase.frame.dlc = 7;
	tx_linearbase.frame.data0 = curr_loc & 0xFF;         //position LSB
	tx_linearbase.frame.data1 = (curr_loc >> 8) & 0xFF;  //position MSB
	tx_linearbase.frame.data2 = adc_buf[0] & 0xFF;        //current LSB, raw ADC counts
	tx_linearbase.frame.data3 = (adc_buf[0] >> 8) & 0xFF; //current MSB, raw ADC counts
	tx_linearbase.frame.data4 = adc_buf[1] & 0xFF;        //current 2 LSB, raw ADC counts
	tx_linearbase.frame.data5 = (adc_buf[1] >> 8) & 0xFF; //current 2 MSB, raw ADC counts
	tx_linearbase.frame.data6 = (interrupted_left ? 0x01 : 0) | (interrupted_right ? 0x02 : 0); //limit switches: bit0 left, bit1 right
	if (!CANSPI_Transmit(&tx_linearbase)) tx_drop_count++;
}



void delay_us (uint16_t us){
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
}


void LB_PWM(uint8_t dir, uint8_t pwm){
	dir = (dir != 0) ? 1 : 0;
	if (dir != cur_dir){
		/* direction change: PWM 0 -> DIR -> new PWM, never reverse at duty */
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
		cur_dir = dir;
	}
	/* dir 0 is stopped by the left switch, dir 1 by the right one; the other way stays free */
	if ((dir == 0) ? interrupted_left : interrupted_right){
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
		return;
	}
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, pwm);   /* ARR is 255, so pwm 0..255 is 0..100% */
	/* a limit ISR may have fired between the check and the write above */
	if ((dir == 0) ? interrupted_left : interrupted_right){
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
	}
}

void CAN_Heartbeat(void){
    if (beat_pls){
		tx_heartbeat.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
		tx_heartbeat.frame.id = heartbeated_msg;
		tx_heartbeat.frame.dlc = 1;
		tx_heartbeat.frame.data0 = heartbeater;
		beat_pls = false;
		if (!CANSPI_Transmit(&tx_heartbeat)) tx_drop_count++;
    }
}

void CAN_PWMInterrupt(){
	tx_pwminterrupt.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
	tx_pwminterrupt.frame.id = pwminterrupted_msg;
	tx_pwminterrupt.frame.dlc = 2;
	tx_pwminterrupt.frame.data0 = interrupted_left;
	tx_pwminterrupt.frame.data1 = interrupted_right;
	if (!CANSPI_Transmit(&tx_pwminterrupt)) tx_drop_count++;
}

void handle_rx(void){
	if (rx.frame.idType != dSTANDARD_CAN_MSG_ID_2_0B) return;
	if (rx.frame.id == msg_pwmreceive){
		/* data0 = direction, data1 = 8-bit magnitude (same layout as Wheel_Node) */
		if (rx.frame.dlc >= 2) LB_PWM(rx.frame.data0, rx.frame.data1);
	}
	else if (rx.frame.id == heartbeat_msg){
		heartbeater = rx.frame.data0 + rx.frame.data1;
		beat_pls = true;
		CAN_Heartbeat();
	}
}

void can_health_check(void){
	if (HAL_GetTick() - last_bussoff_check_tick >= BUSOFF_CHECK_PERIOD_MS){
		last_bussoff_check_tick = HAL_GetTick();
		if (!CAN_checker || CANSPI_isBussOff()){
			CAN_checker = false;
			CAN_checker = CANSPI_Initialize();
		}
	}
}




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
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,0);
  HAL_Delay(100);
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,1);
  HAL_Delay(100);

  HAL_Delay(1000);
  CAN_checker = CANSPI_Initialize();
  HAL_Delay(1000);

  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,0);
  HAL_Delay(100);
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,1);
  HAL_Delay(100);

  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,0);
  HAL_Delay(100);
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,1);
  HAL_Delay(100);

  HAL_TIM_Base_Start(&htim1);

  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
//  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);   /* TIM3 is the one read for position */

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, 2);
  __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE);

  /* limit switches: read the real state once (carriage may power up on a switch),
     then enable the EXTI lines only now that CAN/SPI are initialised */
  interrupted_left  = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2)  == GPIO_PIN_SET) ? 1 : 0;
  interrupted_right = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_SET) ? 1 : 0;
  if (interrupted_left || interrupted_right) CAN_PWMInterrupt();
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_2 | GPIO_PIN_10);
  HAL_NVIC_ClearPendingIRQ(EXTI2_IRQn);
  HAL_NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);


//  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,1);
//  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,1);
//  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 150);
//  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 150);
//
//  HAL_Delay(2000);
//
//  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,0);
//  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,0);
//  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
//  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0);
//
//  HAL_Delay(1000);

  /* Independent watchdog: LSI ~32 kHz / 64 = 500 Hz, reload 500 -> ~1 s.
     Fed from the main loop (and the start-wait loop); a hang resets the MCU, which drops PWM. */
  DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;   /* freeze while halted in the debugger */
  IWDG->KR  = 0x5555;
  IWDG->PR  = 4;
  IWDG->RLR = 500;
  IWDG->KR  = 0xAAAA;
  IWDG->KR  = 0xCCCC;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

      IWDG->KR = 0xAAAA;   /* feed watchdog */

      // TTL disabled for now (loop-pass based, to be redone later):
      if (CAN_failed_counter>=CAN_FAIL){
    	  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
    	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,0);
    	  start_node = false;
      }

      can_health_check();

      if (!start_node){
    	  while(!CANSPI_Receive(&rx)){ IWDG->KR = 0xAAAA; can_health_check(); }
    	  CAN_failed_counter = 0;
    	  if (rx.frame.id == start_node_msg){
    		  start_node = true;
    		  start_node_init = true;
    		  delay_us(50);
    	  }
    	  else if (start_node_init){
    		  start_node = true;
    		  handle_rx();   /* the frame that woke the node is processed, not dropped */
    	  }
      }

  	  if(timer==10){
  		  timer = 0;
  		  CAN_LinearBase();
  	  }
  	  if (counter==1000){
  		  HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
  		  counter = 0;
  	  }

  	  if (left_edge_pending){
  		  left_edge_pending = false;
  		  CAN_PWMInterrupt();
  	  }
  	  if (right_edge_pending){
  		  right_edge_pending = false;
  		  CAN_PWMInterrupt();
  	  }

  	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, CAN_checker ? GPIO_PIN_SET : GPIO_PIN_RESET); // SPI_debug: follows CAN health

  	  if (CANSPI_Receive(&rx)){
  		  CAN_failed_counter=0;
  		  handle_rx();
  	  }

  	  //live position: signed accumulation of the 4x encoder count (wrap-safe, 16-bit counter)
  	  {
  		  uint16_t now_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  		  pos_counts += (int16_t)(now_cnt - last_cnt);
  		  last_cnt = now_cnt;
  	  }
  	  /* TODO(velocity): when the velocity command is defined, dead-reckon position from it here
  	     (pos += v*dt using a measured dt) and re-sync on limit hits. */

  	  timer++;
  	  counter++;
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
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 6;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 6;
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
  htim5.Init.Period = 255;
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

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
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
  /* EXTI2 enabled in main() after CANSPI_Initialize() */

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  /* EXTI15_10 enabled in main() after CANSPI_Initialize() */

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	/* ISR does no SPI: update the flag, stop the motor if it is driving into this switch,
	   and let the main loop send the CAN frame. */
	if (GPIO_Pin==GPIO_PIN_2){
		interrupted_left = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_SET) ? 1 : 0;
		if (interrupted_left && cur_dir == 0) __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
		left_edge_pending = true;
	}
	else if(GPIO_Pin==GPIO_PIN_10){
		interrupted_right = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_SET) ? 1 : 0;
		if (interrupted_right && cur_dir == 1) __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
		right_edge_pending = true;
	}
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
  TIM5->CCR3 = 0;                              /* motor off */
  GPIOA->BSRR = (uint32_t)GPIO_PIN_4 << 16U;   /* DIR low */
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
