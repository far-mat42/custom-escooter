/** Early-May code for getting SPI to transmit from the Nucleo
#include "stm32l4xx.h"

// CRC Polynomial
#define CRC_POLY 0x07 // x^8 + x^2 + x + 1

// Function prototypes
void SystemClock_Config(void);

void SPI_Init(void);
void SPI_Select(void);
void SPI_Deselect(void);
void SPI_Transmit(uint8_t data);
uint8_t SPI_Receive(void);
uint8_t SPI_CalculateCRC(uint8_t *data, uint32_t size);

void TIM1_Init(void);
void TIM1_UP_TIM16_IRQHandler(void);

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

int main(void) {
	// Initialize system clock
	SystemClock_Config();
    // Initialize peripherals
    SPI_Init();
    TIM1_Init();

    while (1) {
        // Select SPI chip
//        SPI_Select();
//
//        // Example data to transmit
//        uint8_t data[] = {0x12, 0x34}; // Example data to transmit, change as needed
//
//        // Calculate CRC for the data
//        uint8_t crc = SPI_CalculateCRC(data, sizeof(data));
//
//        // Transmit data over SPI
//        for (int i = 0; i < sizeof(data); i++) {
//            SPI_Transmit(data[i]);
//        }
//
//        // Transmit CRC over SPI
//        SPI_Transmit(crc);
//
//        // Deselect SPI chip
//        SPI_Deselect();
    }
}

**/

/** May 23 - Code for sending CONFIG_UPDATE command to AFE writing directly to registers
#include "stm32l4xx.h"

void SystemClock_Config(void);
void GPIO_Init(void);
void SPI1_Init(void);
uint8_t crc8(uint8_t *data, size_t len);
void EnterConfigUpdateMode(void);
void SPI1_Transmit(uint8_t *data, size_t len);

int main(void)
{
    // System initialization
    SystemClock_Config();
    GPIO_Init();
    SPI1_Init();

    while (1)
    {
    	// Enter Config Update Mode
    	EnterConfigUpdateMode();
    	for (int i = 0; i < 400000; i++) {

    	}
    }
}

void SystemClock_Config(void)
{
    // System Clock Configuration here
}

void GPIO_Init(void)
{
    // Enable GPIO clocks
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;

    // Configure PB0 as SPI1_NSS (software controlled, push-pull)
    GPIOB->MODER &= ~GPIO_MODER_MODE0;
    GPIOB->MODER |= GPIO_MODER_MODE0_0; // General purpose output mode
    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR0; // High speed
    GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD0; // No pull-up, no pull-down
    GPIOB->OTYPER &= ~GPIO_OTYPER_OT0; // Push-pull mode
    GPIOB->ODR |= GPIO_ODR_OD0; // Set NSS high

    // Configure PA5 (SPI1_SCK), PA6 (SPI1_MISO), PA7 (SPI1_MOSI)
    GPIOA->MODER &= ~(GPIO_MODER_MODE5 | GPIO_MODER_MODE6 | GPIO_MODER_MODE7);
    GPIOA->MODER |= (GPIO_MODER_MODE5_1 | GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1); // Alternate function mode
    GPIOA->AFR[0] |= (0x5 << GPIO_AFRL_AFSEL5_Pos) | (0x5 << GPIO_AFRL_AFSEL6_Pos) | (0x5 << GPIO_AFRL_AFSEL7_Pos); // AF5 for SPI1
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR5 | GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR7; // High speed
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD5 | GPIO_PUPDR_PUPD6 | GPIO_PUPDR_PUPD7); // No pull-up, no pull-down
}

void SPI1_Init(void)
{
    // Enable SPI1 clock
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    // Configure SPI1
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_BR_2 | SPI_CR1_BR_0 | SPI_CR1_SSI | SPI_CR1_SSM;
    SPI1->CR2 = SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0 | SPI_CR2_FRXTH;
    SPI1->CRCPR = 7; // CRC Polynomial

    // Enable SPI1
    SPI1->CR1 |= SPI_CR1_SPE;
}

uint8_t crc8(uint8_t *data, size_t len)
{
    uint8_t crc = 0x00;
    while (len--)
    {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; ++i)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x07; // Polynomial 0x07
            else
                crc <<= 1;
        }
    }
    return crc;
}

void EnterConfigUpdateMode(void)
{
    uint8_t command[] = { 0x00, 0x90 };
    uint8_t crc = crc8(command, 2);
    uint8_t txData[] = { command[0], command[1], crc };

    // Pull NSS low
    GPIOB->ODR &= ~GPIO_ODR_OD0;

    // Transmit data
    SPI1_Transmit(txData, sizeof(txData));

    // Pull NSS high
    GPIOB->ODR |= GPIO_ODR_OD0;
}

void SPI1_Transmit(uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; ++i)
    {
        // Wait until TXE (Transmit buffer empty) flag is set
        while (!(SPI1->SR & SPI_SR_TXE));
        // Send data
        SPI1->DR = data[i];
        // Wait until RXNE (Receive buffer not empty) flag is set
//        while (!(SPI1->SR & SPI_SR_RXNE));
        // Read data to clear RXNE flag
//        (void)SPI1->DR;
    }

    // Wait until not busy
    while (SPI1->SR & SPI_SR_BSY);

    // Clear overrun flag by reading DR and SR
    (void)SPI1->DR;
    (void)SPI1->SR;
}
**/

#include "stm32l4xx_hal.h"

// Function prototypes
void SystemClock_Config(void);
void GPIO_Init(void);
void SPI1_Init(void);
uint8_t crc8(uint8_t *data, size_t len);
void EnterConfigUpdateMode(void);
void ReadManufacturingStatusRegister(void);
void SPI1_Receive(uint8_t *data, size_t len);
void SPI1_Transmit(uint8_t *data, size_t len);

// Global SPI handle
SPI_HandleTypeDef hspi1;

int main(void)
{
    // HAL initialization
    HAL_Init();

    // System clock configuration
    SystemClock_Config();

    // Initialize GPIO and SPI
    GPIO_Init();
    SPI1_Init();

    while (1)
    {
    	// Enter Config Update Mode
    	ReadManufacturingStatusRegister();

    	for (int i = 0; i < 400000; i++) {

    	}
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    // Configure the main internal regulator output voltage
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    // Initializes the RCC Oscillators according to the specified parameters
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    // Initializes the CPU, AHB and APB buses clocks
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);
}

void GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Configure PB0 as SPI1_NSS (software controlled, push-pull)
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); // Set NSS high

    // Configure PA5 (SPI1_SCK), PA6 (SPI1_MISO), PA7 (SPI1_MOSI)
    GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void SPI1_Init(void)
{
    // Enable SPI1 clock
    __HAL_RCC_SPI1_CLK_ENABLE();

    // Configure SPI1
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 7;

    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        // Initialization error
        while (1);
    }
}

uint8_t crc8(uint8_t *data, size_t len)
{
    uint8_t crc = 0x00;
    while (len--)
    {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; ++i)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x07; // Polynomial 0x07
            else
                crc <<= 1;
        }
    }
    return crc;
}

void EnterConfigUpdateMode(void)
{
    uint8_t command[] = { 0x00, 0x20 };
    uint8_t crc = crc8(command, 2);
    uint8_t txData[] = { command[0], command[1], crc };

    // Pull NSS low
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

    // Transmit data
    if (HAL_SPI_Transmit(&hspi1, txData, sizeof(txData), HAL_MAX_DELAY) != HAL_OK)
    {
        // Transmission error
        while (1);
    }

    // Pull NSS high
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
}

void ReadManufacturingStatusRegister(void)
{
    uint8_t command[] = { 0x00, 0x57 }; // Command to read Manufacturing Status Register
    uint8_t crc = crc8(command, 2);
    uint8_t txData[] = { command[0], command[1], crc };
    uint8_t rxData[3] = {0};

    // Pull NSS low
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

    // Transmit command
    HAL_SPI_Transmit(&hspi1, txData, sizeof(txData), HAL_MAX_DELAY);

    // Receive response
    HAL_SPI_Receive(&hspi1, rxData, sizeof(rxData), HAL_MAX_DELAY);

    // Pull NSS high
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

    // Process the received data
    uint8_t received_crc = rxData[2];
    uint8_t calculated_crc = crc8(rxData, 2);

    if (received_crc == calculated_crc)
    {
        // CRC is valid, process the received data
        uint8_t manufacturing_status = rxData[1];
        // Use the manufacturing_status as needed
    }
    else
    {
        // CRC is invalid, handle the error
    }
}

void SPI1_Transmit(uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; ++i)
    {
        // Wait until TXE (Transmit buffer empty) flag is set
        while (!(SPI1->SR & SPI_SR_TXE));
        // Send data
        SPI1->DR = data[i];
        // Wait until RXNE (Receive buffer not empty) flag is set
//        while (!(SPI1->SR & SPI_SR_RXNE));
        // Read data to clear RXNE flag
        (void)SPI1->DR;
    }

    // Wait until not busy
    while (SPI1->SR & SPI_SR_BSY);

    // Clear overrun flag by reading DR and SR
    (void)SPI1->DR;
    (void)SPI1->SR;
}

void SPI1_Receive(uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; ++i)
    {
        // Wait until TXE (Transmit buffer empty) flag is set
        while (!(SPI1->SR & SPI_SR_TXE));
        // Send dummy data to generate clock for receiving
        SPI1->DR = 0xFF;
        // Wait until RXNE (Receive buffer not empty) flag is set
//        while (!(SPI1->SR & SPI_SR_RXNE));
        // Read received data
        data[i] = SPI1->DR;
    }

    // Wait until not busy
    while (SPI1->SR & SPI_SR_BSY);
}