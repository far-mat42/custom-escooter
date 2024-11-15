/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_uart.h"

UART_HandleTypeDef huart1;

/* Function Prototypes -------------------------------------------------------*/
void initClocks(void);
void configGPIO(void);
void configSPI(void);
void configTIM1(void);
void delay(uint16_t milliseconds);
void toggleLED(void);
uint8_t readSPI(uint8_t addr);
uint8_t writeSPI(uint8_t addr, uint8_t tx_data);
void SPI1_Receive(uint8_t *data, size_t len);
void SPI1_Transmit(uint8_t *data, size_t len);
void configUART(void);
void UART_Receive(uint8_t *pData, uint16_t len);
void UART_Transmit(uint8_t *pData, uint16_t len);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	// Initializing registers for GPIO, clocks and SPI
	initClocks();
	configGPIO();
	configSPI();
	configTIM1();

	uint8_t msg[] = "Hello world!";

	// Main loop
	while (1) {
		UART_Transmit(msg, sizeof(msg) - 1);
		HAL_Delay(1000);
		toggleLED();
	}
}

/* Function definitions ------------------------------------------------------*/
void initClocks(void) {
	RCC->AHB2ENR |= 0x00000003; // Enable AHB2 peripheral clock for GPIOA and GPIOB
	RCC->APB2ENR |= 0x00005800; // Enable APB2 peripheral clock for SPI1 and TIM1 and USART1
}

void configGPIO(void) {
	// Resetting registers to be set later to ensure they are in a known state
	GPIOA->MODER &= (~(0x0000FC00));
	GPIOB->MODER &= (~(0x000000C3));
	GPIOA->AFR[0] &= (~(0xFFF00000));
	GPIOB->AFR[0] &= (~(0x0000000F));

	GPIOA->MODER |= 0x0000A800; // Set PA5-7 to alternate function mode
	GPIOB->MODER |= 0x00009042; // Set PB0, PB6-7 to alternate function mode, PB3 to general output mode
	GPIOB->OTYPER &= (~(0x00000008)); // Set PB3 to push-pull mode

	GPIOA->AFR[0] |= 0x55500000; // Set PA5-7 to AF5 (SPI1)
	GPIOB->AFR[0] |= 0x33000005; // Set PB0 to AF5 (SPI1), PB6-7 to AF3 (USART1)
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

void configTIM1(void) {
	TIM1->PSC = 0x0027; // Set prescaler to 40, dividing 4MHz input frequency down to 1kHz
	TIM1->ARR = (uint32_t)1000000; // Set auto-reload value to 1 million
	TIM1->CNT &= (~(0x0000FFFF)); // Set timer's initial count to 0
//	TIM1->CR1 |= 0x0001; // Enable TIM1
}

void configUART(void) {
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 9600;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart1) != HAL_OK) {
        // Initialization error
        Error_Handler();
    }
}

void delay(uint16_t milliseconds) {
//	uint32_t counter = 0; // Counts how much time elapsed between cycles
//	uint32_t goalCount = milliseconds*1000; // Sets the goal based on requested delay time in ms
//	// Variables to calculate time elapsed between cycles
//	uint32_t prevCount = 0;
//	uint32_t currCount = 0;
//
//	prevCount = TIM1->CNT; // Get current count of TIM1
//
//	// Loop until the goal count is reached
//	while (counter < goalCount) {
//		currCount = TIM1->CNT; // Get newest count
//
//		// Handling for case where auto-reload reached
//		if (currCount < prevCount) {
//			counter += (1000000 - prevCount) + currCount;
//		}
//		// Default case; add difference between prev and current counts
//		else {
//			counter += currCount - prevCount;
//		}
//		prevCount = currCount; // Previous count is assigned value of current count for next iteration
//	}

	TIM1->CNT &= (~(0x0000FFFF)); // Reset initial count value to 0
	TIM1->CR1 |= 0x0001; // Enable the timer
	while (TIM1->CNT <= milliseconds) {} // Tight poll until timer count exceeds value in milliseconds
	TIM1->CR1 &= (~(0x0001)); // Disable the timer
}

void toggleLED(void) {
	GPIOB->ODR ^= (0x00000008); // Toggles PB3 (LED GPIO)
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
	while ( ((SPI1->SR) & 0x0080) || (!((SPI1->SR) & 0x0002)) ) {}
//	while ( ((SPI1->SR) & 0x0080) ) {}

	rx_data = SPI1->DR; // Store slave's response to write

	SPI1->CR1 &= (~(0x0040)); // Disable SPI, also pulls CS high

	return rx_data;
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

void UART_Transmit(uint8_t *pData, uint16_t len) {
    if (HAL_UART_Transmit(&huart1, pData, len, HAL_MAX_DELAY) != HAL_OK) {
        // Transmission error
        Error_Handler();
    }
}

void UART_Receive(uint8_t *pData, uint16_t len) {
    if (HAL_UART_Receive(&huart1, pData, len, HAL_MAX_DELAY) != HAL_OK) {
        // Reception error
        Error_Handler();
    }
}

void Error_Handler(void) {
    while (1) {
        // Stay in an infinite loop for debugging
    }
}
