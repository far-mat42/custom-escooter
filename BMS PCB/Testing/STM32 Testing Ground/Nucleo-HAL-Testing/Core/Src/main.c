/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32l4xx.h"
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

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
/* Function Prototypes -------------------------------------------------------*/
void initClocks(void);
void configGPIO(void);
void configSPI(void);
void configTIM1(void);
void delay(uint16_t milliseconds);
void toggleLED(void);
uint8_t readSPI(uint8_t addr);
uint8_t writeSPI(uint8_t addr, uint8_t tx_data);
void UART_Receive(uint8_t *pData, uint16_t len);
void UART_Transmit(uint8_t *pData, uint16_t len);
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

  uint8_t txData[2],rxData[2]; // Buffers for storing TX/RX data
  HAL_StatusTypeDef hal_status; // Status indicator for SPI transaction
  uint8_t msg[] = "Hello world!";

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  initClocks();
//  configGPIO();
//  configSPI();

  // SPI addresses
  uint8_t addr[4] = {37, 6, 22, 57};
  uint8_t data[4] = {197, 5, 73, 41};

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  UART_Transmit(msg, sizeof(msg));
	  HAL_Delay(1000);
	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);

	  // Loop through all addresses and write all 4 data bytes
//	  for (int i = 0; i < 4; i++) {
//		  for (int j = 0; j < 4; j++) {
//			  // Load the address and data into the TX buffer
//			  txData[0] = addr[i];
//			  txData[1] = data[j];
//
////			  // Initiate the SPI transaction
////			  hal_status = HAL_SPI_TransmitReceive(&hspi1, txData, rxData, 2, 0x0000FFFF);
////			  if (hal_status == HAL_OK) {
////				  txData[1] = data[j]; // Placeholder, we really just do nothing
////			  }
//			  writeSPI(addr[i], data[j]);
//			  HAL_Delay(1000); // Delay a second between transactions
//		  }
//	  }

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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}


/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */
	__HAL_RCC_USART1_CLK_ENABLE();
  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
  GPIO_InitStruct.Pin = GPIO_PIN_6; // TX pin for USART1
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_7;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void initClocks(void) {
	RCC->AHB2ENR |= 0x00000003; // Enable AHB2 peripheral clock for GPIOA and GPIOB
	RCC->APB2ENR |= 0x00005800; // Enable APB2 peripheral clock for SPI1 and TIM1
}

void configGPIO(void) {
	// Resetting registers to be set later to ensure they are in a known state
	GPIOA->MODER &= (~(0x0000FC00));
	GPIOB->MODER &= (~(0x000000C3));
	GPIOA->AFR[0] &= (~(0xFFF00000));
	GPIOB->AFR[0] &= (~(0x0000000F));

	GPIOA->MODER |= 0x0000A800; // Set PA5-7 to alternate function mode
	GPIOB->MODER |= 0x0000A042; // Set PB0, PB6-7 to alternate function mode, PB3 to general output mode
	GPIOB->OTYPER &= (~(0x00000008)); // Set PB3 to push-pull mode

	GPIOA->AFR[0] |= 0x55500000; // Set PA5-7 to AF5 (SPI1)
	GPIOB->AFR[0] |= 0x77000005; // Set PB0 to AF5 (SPI1), PB6-7 to AF7 (USART1)
}

void configSPI(void) {
	//SPI1->CR1 &= (~(0x0040)); // Disable SPI
	SPI1->CR1 &= (~(0x0003)); // Resetting CPOL and CPHA
	SPI1->CR1 &= (~(0x8400)); // Set to full duplex mode
	SPI1->CR1 &= (~(0x0080)); // Set to MSB first
	SPI1->CR1 &= (~(0x0038)); // Reset baud rate register to known state
	SPI1->CR1 &= (~(0x2000)); // Disable CRC calculation
	SPI1->CR1 &= (~(0x0200)); // Disable software slave management (hardware mode only)
	SPI1->CR1 |= 0x0010; // Set clock frequency to fPCLK/8 (250kHz)
	SPI1->CR1 |= 0x0004; // Set to master configuration

	SPI1->CR2 &= (~(0x1000)); // RXNE event triggers when FIFO level equals 16 bits (performing 16-bit transactions w/ AFE)
	SPI1->CR2 |= 0x0F00; // Set data length for SPI transfers to 16-bit
	SPI1->CR2 &= (~(0x00F0)); // Mask interrupts & set frame format to Motorola mode
	SPI1->CR2 |= 0x0004; // Enable slave select output
	SPI1->CR2 &= (~(0x0003)); // Disable DMA requests
}

uint8_t readSPI(uint8_t addr) {
	uint8_t rx_data = 0;

	SPI1->CR1 |= 0x0040; // Enable SPI, also pulls CS low

	// Shift in address & dummy byte (0x00) into data frame
	SPI1->DR = (uint16_t)(addr << 8);
	SPI1->DR |= 0x8000; // Set the R/W bit high to indicate a read

	// Tight poll until SPI not busy and RX buffer is not empty
	while ( ((SPI1->SR) & 0x0080) || (!((SPI1->SR) & 0x0001)) ) {}

	rx_data = (uint8_t)SPI1->DR; // Store data from SPI RX buffer

	SPI1->CR1 &= (~(0x0040)); // Disable SPI, also pulls CS high

	return rx_data;
}

uint8_t writeSPI(uint8_t addr, uint8_t tx_data) {
	uint16_t rx_data = 0; // Variable for storing slave's response to write

	SPI1->CR1 |= 0x0040; // Enable SPI, also pulls CS low

	// Shift in address & tx_data into data frame
	SPI1->DR = (uint16_t)(addr << 8);
	SPI1->DR |= (uint16_t)(tx_data);
	SPI1->DR &= (~(0x8000)); // Set the R/W bit low to indicate a write
	// Tight poll until SPI not busy and TX buffer is empty
//	while ( ((SPI1->SR) & 0x0080) || (!((SPI1->SR) & 0x0002)) ) {}
	while ( ((SPI1->SR) & 0x0080) ) {}

	rx_data = SPI1->DR; // Store slave's response to write

	SPI1->CR1 &= (~(0x0040)); // Disable SPI, also pulls CS high

	return rx_data;
}

void UART_Receive(uint8_t *pData, uint16_t len) {
    if (HAL_UART_Receive(&huart1, pData, len, HAL_MAX_DELAY) != HAL_OK) {
        // Reception error
        Error_Handler();
    }
}

void UART_Transmit(uint8_t *pData, uint16_t len) {
    if (HAL_UART_Transmit(&huart1, pData, len, HAL_MAX_DELAY) != HAL_OK) {
        // Transmission error
        Error_Handler();
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
