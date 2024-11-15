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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// CRC Polynomial
#define CRC_POLY 0x07 // x^8 + x^2 + x + 1
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
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void SystemClock_Config(void);

void SPI_Init(void);
void SPI_Select(void);
void SPI_Deselect(void);
void SPI_Transmit(uint8_t data);
uint8_t SPI_Receive(void);
uint8_t SPI_CalculateCRC(uint8_t *data, uint32_t size);

void TIM1_Init(void);
void TIM1_UP_TIM16_IRQHandler(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Configure the system clock
void SystemClock_Config(void) {
	RCC->CR |= RCC_CR_HSION; // Enable HSI clock
	while (!(RCC->CR & RCC_CR_HSIRDY)); // Wait until HSI clock is ready

	RCC->CR |= RCC_CR_MSIPLLEN; // Enable MSI PLL
	while (!(RCC->CR & RCC_CR_MSIRDY)); // Wait until MSI clock is ready

	RCC->CFGR &= ~(RCC_CFGR_SW); // Reset SW bits
	RCC->CFGR |= RCC_CFGR_SW_MSI; // Select MSI as system clock

	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_MSI); // Wait until MSI is used as system clock
}

// Initialize SPI peripheral
void SPI_Init(void) {
    // Enable GPIO clocks
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;

    // Configure GPIO pins for SPI functionality
    GPIOA->MODER &= ~(GPIO_MODER_MODE5_Msk | GPIO_MODER_MODE6_Msk | GPIO_MODER_MODE7_Msk);
    GPIOA->MODER |= (GPIO_MODER_MODE5_1 | GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1);
    GPIOA->AFR[0] |= (5 << GPIO_AFRL_AFSEL5_Pos) | (5 << GPIO_AFRL_AFSEL6_Pos) | (5 << GPIO_AFRL_AFSEL7_Pos);

    // Configure GPIO pin for chip select
    GPIOB->MODER &= ~GPIO_MODER_MODE0_Msk;
    GPIOB->MODER |= GPIO_MODER_MODE0_0; // Output mode

    // Enable SPI peripheral clock
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    // Configure SPI1 settings
    SPI1->CR1 = 0;
    SPI1->CR1 |= (SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI);
    SPI1->CR1 |= (SPI_CR1_BR_1 | SPI_CR1_BR_2); // Assuming PCLK/128
//    SPI1->CR1 |= 0x0020; // Assuming PCLK/32
    SPI1->CR1 |= SPI_CR1_SPE;
}

// Select SPI chip
void SPI_Select(void) {
    GPIOB->ODR &= ~GPIO_ODR_OD0;
}

// Deselect SPI chip
void SPI_Deselect(void) {
    GPIOB->ODR |= GPIO_ODR_OD0;
}

// Transmit data over SPI
void SPI_Transmit(uint8_t data) {
    // Wait until transmit buffer is empty
    while (!(SPI1->SR & SPI_SR_TXE));

    // Write data to transmit buffer
    *((__IO uint8_t*)&SPI1->DR) = data;

    // Wait until transmission is complete
    while (SPI1->SR & SPI_SR_BSY);
}

// Receive data over SPI
uint8_t SPI_Receive(void) {
    // Wait until receive buffer is not empty
    while (!(SPI1->SR & SPI_SR_RXNE));

    // Read data from receive buffer
    return *((__IO uint8_t*)&SPI1->DR);
}

// Calculate CRC
uint8_t SPI_CalculateCRC(uint8_t *data, uint32_t size) {
    uint8_t crc = 0;

    for (uint32_t i = 0; i < size; i++) {
        crc ^= data[i];

        for (uint32_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ CRC_POLY;
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

// Initialize TIM1 peripheral
void TIM1_Init(void) {
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; // Enable TIM1 clock

	TIM1->PSC = 4000 - 1; // Assuming 4MHz clock, 4000 cycles for 1ms
	TIM1->ARR = 1000 - 1; // Generate interrupt every 2000ms (2s)

	TIM1->DIER |= TIM_DIER_UIE; // Enable update interrupt

	NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 0); // Set TIM1 interrupt priority
	NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn); // Enable TIM1 interrupt

	TIM1->CR1 |= TIM_CR1_CEN; // Enable TIM1
}

// IRQ handler for TIM1
void TIM1_UP_TIM16_IRQHandler(void) {
	// Check if UIF flag is set
	if (TIM1->SR & TIM_SR_UIF) {
		// Select SPI chip
		SPI_Select();

		// Example data to transmit
		uint8_t data[] = {0x12, 0x34}; // Example data to transmit, change as needed

		// Calculate CRC for the data
		uint8_t crc = SPI_CalculateCRC(data, sizeof(data));

		// Transmit data over SPI
		for (int i = 0; i < sizeof(data); i++) {
			SPI_Transmit(data[i]);
		}

		// Transmit CRC over SPI
		SPI_Transmit(crc);

		// Deselect SPI chip
		SPI_Deselect();

		TIM1->SR &= ~TIM_SR_UIF; // Clear UIF flag
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
  SPI_Init();
//  TIM1_Init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // Blink LED to indicate a successful transmit
	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
	  HAL_Delay(500);
	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
	  HAL_Delay(500);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
//void SystemClock_Config(void)
//{
//  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//
//  /** Configure the main internal regulator output voltage
//  */
//  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Configure LSE Drive Capability
//  */
//  HAL_PWR_EnableBkUpAccess();
//  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
//
//  /** Initializes the RCC Oscillators according to the specified parameters
//  * in the RCC_OscInitTypeDef structure.
//  */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
//  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
//  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
//  RCC_OscInitStruct.MSICalibrationValue = 0;
//  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Initializes the CPU, AHB and APB buses clocks
//  */
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
//
//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Enable MSI Auto calibration
//  */
//  HAL_RCCEx_EnableMSIPLLMode();
//}

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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
