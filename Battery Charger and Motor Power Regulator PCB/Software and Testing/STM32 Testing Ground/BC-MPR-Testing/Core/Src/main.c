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
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// BQ25756 I2C and register addresses
#define BQ25756_ADDR 			0x6B << 1

#define BQ25756_CHG_VLIM 			0x00
#define BQ25756_CHG_ILIM			0x02
#define BQ25756_INP_DPM_ILIM		0x06
#define BQ25756_INP_DPM_VLIM		0x08
#define BQ25756_INP_REV_ILIM		0x0A
#define BQ25756_INP_REV_VLIM		0x0C
#define BQ25756_PCHG_ILIM			0x10
#define BQ25756_CHG_ITERM			0x12
#define BQ25756_PCHG_TERM_CTRL		0x14
#define BQ25756_TIM_CTRL			0x15
#define BQ25756_3STG_CHG_CTRL		0x16
#define BQ25756_CHGR_CTRL			0x17
#define BQ25756_PIN_CTRL			0x18
#define BQ25756_PWR_REV_CTRL		0x19
#define BQ25756_MPPT_CTRL			0x1A
#define BQ25756_TS_CHG_THR_CTRL		0x1B
#define BQ25756_TS_CHG_RGN_CTRL		0x1C
#define BQ25756_TS_REV_THR_CTRL		0x1D
#define BQ25756_REV_UV_CTRL			0x1E
#define BQ25756_VAC_MPP_DET			0x1F
#define BQ25756_CHGR_STAT1			0x21
#define BQ25756_CHGR_STAT2			0x22
#define BQ25756_CHGR_STAT3			0x23
#define BQ25756_FLT_STAT			0x24
#define BQ25756_CHGR_FLAG1			0x25
#define BQ25756_CHGR_FLAG2			0x26
#define BQ25756_FLT_FLAG			0x27
#define BQ25756_CHGR_MSK1			0x28
#define BQ25756_CHGR_MSK2			0x29
#define BQ25756_FLT_MSK				0x2A
#define BQ25756_ADC_CTRL			0x2B
#define BQ25756_ADC_CH_CTRL			0x2C
#define BQ25756_IAC_ADC				0x2D
#define BQ25756_IBAT_ADC			0x2F
#define BQ25756_VAC_ADC				0x31
#define BQ25756_VBAT_ADC			0x33
#define BQ25756_TS_ADC				0x37
#define BQ25756_VFB_ADC				0x39
#define BQ25756_GDRV_STR_CTRL		0x3B
#define BQ25756_GDRV_DT_CTRL		0x3C
#define BQ25756_PART_INFO			0x3D
#define BQ25756_REV_IBAT_DSG		0x62


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t currentDutyCycle = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
uint16_t ReadThrottle();
uint8_t GetHallSector();
void SetCommutation(uint8_t sector, uint8_t duty);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *data, int len) {
    // Redirect printf to UART
    HAL_UART_Transmit(&huart2, (uint8_t*)data, len, HAL_MAX_DELAY);
    return len;
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
  // Arrays for I2C communication
  uint8_t txData[3] = {0};
  uint8_t rxData[16] = {0};

  // Variables for storing read values from ADCs
  int16_t busCurrentADC = 0;
  int16_t battCurrentADC = 0;
  uint16_t busVoltageADC = 0;
  uint16_t battVoltageADC = 0;

  // 32-bit variables to avoid overflow when converting bit-step of ADCs to actual values
  int32_t busCurrent32 = 0;
  int32_t battCurrent32 = 0;
  uint32_t busVoltage32 = 0;
  uint32_t battVoltage32 = 0;

  uint8_t chgStatus1 = 0;
  uint8_t chgStatus2 = 0;
  uint8_t chgStatus3 = 0;
  uint8_t fltStatus = 0;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  /********* Configuring the BQ25756 registers *********/
  // Charge voltage limit: FB limit set to 1.532V
  txData[0] = BQ25756_CHG_VLIM;
  txData[1] = 0x0E;
  txData[2] = 0x00;
  HAL_I2C_Master_Transmit(&hi2c1, BQ25756_ADDR, txData, 3, HAL_MAX_DELAY);

  // Charge current limit: Set to 15A (6000mA with 5mΩ resistor = 15000mA with 2mΩ resistor)
  txData[0] = BQ25756_CHG_ILIM;
  txData[1] = 0xE0;
  txData[2] = 0x01;
  HAL_I2C_Master_Transmit(&hi2c1, BQ25756_ADDR, txData, 3, HAL_MAX_DELAY);

  // Input current DPM limit: Set to 25A (10000mA with 5mΩ resistor = 25000mA with 2mΩ resistor)
  txData[0] = BQ25756_INP_DPM_ILIM;
  txData[1] = 0x20;
  txData[2] = 0x03;
  HAL_I2C_Master_Transmit(&hi2c1, BQ25756_ADDR, txData, 3, HAL_MAX_DELAY);

  // Input voltage DPM limit: Set to 48V
  txData[0] = BQ25756_INP_DPM_VLIM;
  txData[1] = 0x80;
  txData[2] = 0x25;
  HAL_I2C_Master_Transmit(&hi2c1, BQ25756_ADDR, txData, 3, HAL_MAX_DELAY);

  // Reverse mode input current limit: Set to 36A (14400mA with 5mΩ resistor = 36000mA with 2mΩ resistor)
  txData[0] = BQ25756_INP_REV_ILIM;
  txData[1] = 0x80;
  txData[2] = 0x04;
  HAL_I2C_Master_Transmit(&hi2c1, BQ25756_ADDR, txData, 3, HAL_MAX_DELAY);

  // Reverse mode input voltage regulation: Set to 36V
  txData[0] = BQ25756_INP_REV_VLIM;
  txData[1] = 0x20;
  txData[2] = 0x1C;
  HAL_I2C_Master_Transmit(&hi2c1, BQ25756_ADDR, txData, 3, HAL_MAX_DELAY);

  // Precharge current limit: Set to 3A (1200mA with 5mΩ resistor = 3000mA with 2mΩ resistor)
  txData[0] = BQ25756_PCHG_ILIM;
  txData[1] = 0x60;
  txData[2] = 0x00;
  HAL_I2C_Master_Transmit(&hi2c1, BQ25756_ADDR, txData, 3, HAL_MAX_DELAY);

  // Timer Control: Disable watchdog
  txData[0] = BQ25756_TIM_CTRL;
  txData[1] = 0x0D;
  HAL_I2C_Master_Transmit(&hi2c1, BQ25756_ADDR, txData, 2, HAL_MAX_DELAY);

  // Charger Control: Disable charging
  txData[0] = BQ25756_CHGR_CTRL;
  txData[1] = 0xD0;
  HAL_I2C_Master_Transmit(&hi2c1, BQ25756_ADDR, txData, 2, HAL_MAX_DELAY);

  // Pin configurations: Disable ICHG, ILIM_HIZ pins
  txData[0] = BQ25756_PIN_CTRL;
  txData[1] = 0x00;
  HAL_I2C_Master_Transmit(&hi2c1, BQ25756_ADDR, txData, 2, HAL_MAX_DELAY);

  // TS charging threshold control: Set thresholds as -10ºC COLD, 5ºC COOL, 50ºC WARM, 60ºC HOT
  txData[0] = BQ25756_TS_CHG_THR_CTRL;
  txData[1] = 0xA0;
  HAL_I2C_Master_Transmit(&hi2c1, BQ25756_ADDR, txData, 2, HAL_MAX_DELAY);

  // TS charging region behavior: Ignore TS pin for charging control
  txData[0] = BQ25756_TS_CHG_RGN_CTRL;
  txData[1] = 0x5A;
  HAL_I2C_Master_Transmit(&hi2c1, BQ25756_ADDR, txData, 2, HAL_MAX_DELAY);

  // Reverse undervoltage control: Fix UVP at 3.3V
  txData[0] = BQ25756_REV_UV_CTRL;
  txData[1] = 0x20;
//  HAL_I2C_Master_Transmit(&hi2c1, BQ25756_ADDR, txData, 2, HAL_MAX_DELAY);

  // ADC control: Enable ADC in one-shot mode, 15-bit effective resolution (24ms sample time)
  txData[0] = BQ25756_ADC_CTRL;
  txData[1] = 0xC0;
  HAL_I2C_Master_Transmit(&hi2c1, BQ25756_ADDR, txData, 2, HAL_MAX_DELAY);

  // ADC channel control: Enable all ADC channels
  txData[0] = BQ25756_ADC_CH_CTRL;
  txData[1] = 0x00;
  HAL_I2C_Master_Transmit(&hi2c1, BQ25756_ADDR, txData, 2, HAL_MAX_DELAY);

  // Reverse mode battery discharge: Disable fast transient response
  txData[0] = BQ25756_REV_IBAT_DSG;
  txData[1] = 0x00;
  HAL_I2C_Master_Transmit(&hi2c1, BQ25756_ADDR, txData, 2, HAL_MAX_DELAY);

  // Power path and reverse mode control: Enable reverse mode (regulate VAC)
  txData[0] = BQ25756_PWR_REV_CTRL;
  txData[1] = 0x01; // TODO: Check if enabling IAC load changes anything
  HAL_I2C_Master_Transmit(&hi2c1, BQ25756_ADDR, txData, 2, HAL_MAX_DELAY);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // 1 second delay between all ADC reads
	  HAL_Delay(1000);

	  // Reset watchdog, trigger ADC readings
	  txData[0] = BQ25756_CHGR_CTRL;
	  txData[1] = 0xE9;
//	  HAL_I2C_Master_Transmit(&hi2c1, BQ25756_ADDR, txData, 2, HAL_MAX_DELAY);

	  txData[0] = BQ25756_ADC_CTRL;
	  txData[1] = 0xC0;
	  HAL_I2C_Master_Transmit(&hi2c1, BQ25756_ADDR, txData, 2, HAL_MAX_DELAY);
//
//	  txData[0] = BQ25756_ADC_CH_CTRL;
//	  txData[1] = 0x00;
//	  HAL_I2C_Master_Transmit(&hi2c1, BQ25756_ADDR, txData, 2, HAL_MAX_DELAY);

	  HAL_Delay(200); // Give time for ADC conversion to finish

	  // Read all reported ADC readings
	  txData[0] = BQ25756_IAC_ADC;
	  HAL_I2C_Master_Transmit(&hi2c1, BQ25756_ADDR, txData, 1, HAL_MAX_DELAY);
	  HAL_I2C_Master_Receive(&hi2c1, BQ25756_ADDR, rxData, 8, HAL_MAX_DELAY);

	  // Convert ADC readings based on bit step
	  // IBUS: 0.8mA bit-step with 5mΩ resistor = 2mA bit-step with 2mΩ resistor
	  busCurrentADC = (rxData[0] | (rxData[1] << 8));
	  busCurrent32 = 0;
	  for (int i = 0; i < 2; i++) {busCurrent32 += busCurrentADC;}

	  // IBAT: 2mA bit-step with 5mΩ resistor = 5mA bit-step with 2mΩ resistor
	  battCurrentADC = (rxData[2] | (rxData[3] << 8));
	  battCurrent32 = 0;
	  for (int i = 0; i < 5; i++) {battCurrent32 += battCurrentADC;}

	  // VBUS: 2mV bit-step
	  busVoltageADC = (rxData[4] | (rxData[5] << 8));
	  busVoltage32 = 0;
	  for (int i = 0; i < 2; i++) {busVoltage32 += busVoltageADC;}
	  // VBAT: 2mV bit-step
	  battVoltageADC = (rxData[6] | (rxData[7] << 8)); // 2mV bit-step
	  battVoltage32 = 0;
	  for (int i = 0; i < 2; i++) {battVoltage32 += battVoltageADC;}

	  // Read all statuses and faults
	  txData[0] = BQ25756_CHGR_STAT1;
	  HAL_I2C_Master_Transmit(&hi2c1, BQ25756_ADDR, txData, 1, HAL_MAX_DELAY);
	  HAL_I2C_Master_Receive(&hi2c1, BQ25756_ADDR, rxData, 4, HAL_MAX_DELAY);

	  // Store readings
	  chgStatus1 = rxData[0];
	  chgStatus2 = rxData[1];
	  chgStatus3 = rxData[2];
	  fltStatus = rxData[3];

	  // Broadcast ADC readings to UART
	  printf("\n*********************** BQ25756 ADC READINGS ***********************\r\n");
	  printf("I_BUS: %ld mA \t", busCurrent32);
	  printf("I_BATT: %ld mA \t", battCurrent32);
	  printf("V_BUS: %ld mV \t", busVoltage32);
	  printf("V_BATT: %ld mV\r\n", battVoltage32);

	  // Broadcast current status
	  printf("Current BQ25756 status: ");
	  if (chgStatus1 & (1 << 6)) {
		  printf("In input current regulation mode; ");
	  }
	  if (chgStatus1 & (1 << 5)) {
		  printf("In input voltage regulation mode; ");
	  }
	  if (chgStatus2 & (1 << 7)) {
		  printf("Input power good; ");
	  }
	  if (chgStatus3 & (1 << 3)) {
		  printf("CV Timer expired; ");
	  }
	  if (chgStatus3 & (1 << 2)) {
		  printf("Converter Reverse Mode On;");
	  }
	  printf("\r\n");

	  // Broadcast any detected faults
	  if (fltStatus & (1 << 1)) {
		  printf("Detected fault: DRV_SUP not OK\r\n");
	  }
	  if (fltStatus & (1 << 2)) {
		  printf("Detected fault: Charge safety timer expired\r\n");
	  }
	  if (fltStatus & (1 << 3)) {
		  printf("Detected fault: Thermal shutdown active\r\n");
	  }
	  if (fltStatus & (1 << 4)) {
		  printf("Detected fault: Battery overvoltage fault\r\n");
	  }
	  if (fltStatus & (1 << 5)) {
		  printf("Detected fault: Battery overcurrent fault\r\n");
	  }
	  if (fltStatus & (1 << 6)) {
		  printf("Detected fault: Input overvoltage fault\r\n");
	  }
	  if (fltStatus & (1 << 7)) {
		  printf("Detected fault: Input undervoltage fault\r\n");
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
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
  hi2c1.Init.Timing = 0x00100D14;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
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
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PC1 PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * Defining ISR for change in state in any of the hall effect sensors
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    // Respond to any Hall sensor pin change
    if (GPIO_Pin == GPIO_PIN_1 || GPIO_Pin == GPIO_PIN_2 || GPIO_Pin == GPIO_PIN_3) {
        uint8_t sector = GetHallSector();
        SetCommutation(sector, currentDutyCycle);
    }
}

/**
 * Reads the ADC input for the throttle
 * @return Value between 0-255, representing throttle
 */
uint16_t ReadThrottle() {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    return HAL_ADC_GetValue(&hadc1);
}

/**
 * Reads the hall effect sensor inputs
 * @return 3-bit integer representing, from MSB to LSB, hall A, B, C
 */
uint8_t GetHallSector() {
    uint8_t hallA = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1);
    uint8_t hallB = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2);
    uint8_t hallC = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3);
    return (hallA << 2) | (hallB << 1) | hallC;
}

/**
 * Sets the gate driver outputs based on the hall effect sensors and duty cycle
 * @param sector 3-bit integer representing, from MSB to LSB, hall A, B, C
 * @param duty Duty cycle to use, from 0% to 100%
 */
void SetCommutation(uint8_t sector, uint8_t duty) {
	// Clamp duty to between 20 and 100
	if (duty < 20) duty = 20;
	if (duty > 100) duty = 100;
	// Calculate PWM COMPARE value based on TIM1 settings
    uint32_t ARR = __HAL_TIM_GET_AUTORELOAD(&htim1);
    uint32_t compare = (ARR * duty) / 100;

    // Set duty cycle for all PWM channels
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, compare);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, compare);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, compare);

    // Enable/disable appropriate gate inputs based on sensor inputs
    // TIM1 CH1, CH2, CH3 driving phases A, B, C respectively
    switch (sector) {
        case 0b101: // Sector 1: A+ B-
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  		// A+ PWM
            HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1); 	// A- floating
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);   		// B+ floating
            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);	// B- PWM
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);   		// C+ floating
            HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3); 	// C- floating
            break;
        case 0b100: // Sector 2: A+ C-
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  		// A+ PWM
            HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1); 	// A- floating
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);   		// B+ floating
            HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);		// B- floating
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);   		// C+ floating
            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3); 	// C- PWM
            break;
        case 0b110: // Sector 3: B+ C-
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);  		// A+ floating
            HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1); 	// A- floating
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);		// B+ PWM
            HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);		// B- floating
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);   		// C+ floating
            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3); 	// C- PWM
            break;
        case 0b010: // Sector 4: B+ A-
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);  		// A+ floating
            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1); 	// A- PWM
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);   	// B+ PWM
            HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);		// B- floating
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);   		// C+ floating
            HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3); 	// C- floating
            break;
        case 0b011: // Sector 5: C+ A-
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);		// A+ floating
            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);	// A- PWM
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);		// B+ floating
            HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);		// B- floating
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);		// C+ PWM
            HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3); 	// C- floating
            break;
        case 0b001: // Sector 6: C+ B-
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);  		// A+ floating
            HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1); 	// A- floating
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);   		// B+ floating
            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);	// B- PWM
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);		// C+ PWM
            HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3); 	// C- floating
            break;
        default:
            // Invalid state: disable all gate inputs
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
            HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
            HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
            HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
            break;
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
