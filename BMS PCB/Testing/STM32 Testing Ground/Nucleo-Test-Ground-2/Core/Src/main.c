#include "stm32l4xx_hal.h"
#include "bq76952.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <time.h>

// Function prototypes
// Initializing peripherals
void SystemClock_Config(void);
void GPIO_Init(void);
void SPI1_Init(void);
void USART1_Init(void);
void TIM1_Init(void);
void TIM2_Init(void);
void ADC1_Init(void);
void RTC_Init(void);

uint8_t crc8(uint8_t *data, size_t len);

// Functions to handle communication with the AFE
void DirectCmdRead(uint8_t cmd, uint8_t *returnData, uint8_t len);
void DirectCmdWrite(uint8_t cmd, uint8_t *writeData, uint8_t len);
void SubCmdNoData(uint16_t cmd);
void SubCmdReadData(uint16_t cmd, uint8_t *returnData, uint8_t len);
void RAMRegisterRead(uint16_t addr, uint8_t *returnData, uint8_t len);
void RAMRegisterWrite(uint16_t addr, uint8_t *writeData, uint8_t len);
void RAMRegisterInit(void);

// Helper functions that do the handling for verifying the AFE received a SPI command
void AFETransmitReadCmd(uint8_t *txBytes, uint8_t *rxBytes, uint8_t arrSize);
void AFETransmitWriteCmd(uint8_t *txBytes, uint8_t *rxBytes, uint8_t arrSize);

// Helper functions to transmit the data from the AFE to the UART lines
void TransmitLogAndTimestamp(void);
void TransmitCellPackVoltages(uint16_t *volts, uint8_t len);
void TransmitADCReadings(uint32_t *counts, uint8_t len);
void TransmitCurrentReading(int16_t current);
void TransmitTemperatures(int16_t *temps, uint8_t len);
void TransmitSafetyStatusA(void);
void TransmitSafetyStatusB(void);

// Other miscellaneous helper functions
int16_t T4_Acquire(void);
time_t RTCToUnixTimestamp(RTC_DateTypeDef *date, RTC_TimeTypeDef *time);
void GetDateTime(char *datetimeStr, size_t size);

void Error_Handler(void);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

// Global defines
#define T4_BETA 3435.0
#define T4_R0 10000.0
#define T4_PU_R 17800.0
#define T4_PU_V 3.3
#define V_REF 2.048

// Global handles
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart1;
TIM_HandleTypeDef htim1;
ADC_HandleTypeDef hadc1;
RTC_HandleTypeDef hrtc;

// Global variables - used for ISRs to raise flags
bool logDataFlag = 0;
bool logAlertsFlag = 0;

int main(void) {
    // HAL initialization
    HAL_Init();

    // System clock configuration
    SystemClock_Config();

    // Initialize GPIO, SPI, UART, TIM1, ADC1
    GPIO_Init();
    SPI1_Init();
    USART1_Init();
    TIM1_Init();
    TIM2_Init();
    ADC1_Init();
    RTC_Init();

    // Start the logging timer
    TIM1->CR1 |= TIM_CR1_CEN;

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // Turn off heartbeat

    // Blank array to store data from any commands that receive data
    uint8_t readData[32] = {0};

    uint16_t ctrlStatus = 0;
    uint16_t cellVolt = 0;
    uint16_t cellVolts[17] = {0};
    int16_t cellGains[16] = {12000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 11900};
    uint32_t MCUTemperature = 0;
    int16_t AFETemperature = 0;
    int16_t temperatures[4] = {0};
    int16_t currentRead = 0;
    uint8_t fetStatus = 0;
    uint8_t cmdAddr = 0;

    uint8_t writeData[32] = {0};

    // Read battery status register and manufacturing status register
	DirectCmdRead(0x12, readData, 2);
	SubCmdReadData(0x0057, readData, 2);
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
	HAL_Delay(250);
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
	HAL_Delay(750);

	// Disable SLEEP mode and disable FET_TEST mode if already in FET_TEST
	SubCmdNoData(0x009A);
	if (!(readData[0] & (1 << 4))) SubCmdNoData(0x0022);

	// Read battery status register and manufacturing status register
	DirectCmdRead(0x12, readData, 2);
	SubCmdReadData(0x0057, readData, 2);

	// Enter CONFIG_UPDATE mode
	SubCmdNoData(0x0090);
	// Wait for Battery Status to confirm transition to CONFIG_UPDATE mode
	do {
		DirectCmdRead(0x12, readData, 2);
	} while (!(readData[0] & 0x01));

	// Program configuration for all AFE registers
	RAMRegisterInit();

	/**
	 * Set calibration gain values for all cell voltages
	 */
	for (int i = 0; i < 16; i++) {
		format_int16(writeData, cellGains[i]);
		RAMRegisterWrite(CAL_GAIN_CL1 + i*2, writeData, 2);
	}

	// Exit CONFIG_UPDATE mode, disable SLEEP mode, read manufacturing status register again
	SubCmdNoData(0x0092);
	SubCmdNoData(0x009A);
	SubCmdReadData(0x0057, readData, 2);
	// Read battery status register
	DirectCmdRead(0x12, readData, 2);

	// Blink status LED a few times to indicate setup is complete
	for (int i = 0; i < 10; i++) {
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
		HAL_Delay(100);
	}

	// Clear bits in the alarm registers
	writeData[0] = 0xFF;
	writeData[1] = 0xFE;
	DirectCmdWrite(0xE2, writeData, 2);

    while (1) {
    	// Read the control status register
//    	DirectCmdRead(0x02, readData, 2);
//    	ctrlStatus = (readData[0]) + (readData[1] << 8);

    	// Check if flag to log data was raised
    	if (logDataFlag) {
    		logDataFlag = false; // Clear the flag
    		//TODO: Only read measurements if FULLSCAN bit of Alarm Status is set, then clear bit after reading measurements
    		// Enable status LED to indicate data being logged
    		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
    		// Read the cell voltage for all 16 cells and then the pack voltage
			for (int i = 0; i < 17; i++) {
				cmdAddr = 0x14 + 2*i;
				DirectCmdRead(cmdAddr, readData, 2);
				// Combine the 2 8-bit cell voltage bytes into a single 16-byte variable
				cellVolt = (readData[0]) + (readData[1] << 8);
				cellVolts[i] = cellVolt;
			}

			// Read the AFETemperature measured at TS1-3
			for (int i = 0; i < 3; i++) {
				cmdAddr = 0x70 + 2*i;
				DirectCmdRead(cmdAddr, readData, 2);
				AFETemperature = (readData[0]) + (readData[1] << 8);
				temperatures[i] = AFETemperature;
			}
			// Calculate temperature measured by MCU's ADC
			MCUTemperature = T4_Acquire();
			temperatures[3] = MCUTemperature;

			// Read the CC2 current and FET status
			DirectCmdRead(0x3A, readData, 2);
			currentRead = (readData[0]) + (readData[1] << 8);
			DirectCmdRead(0x7F, readData, 1);
			fetStatus = readData[0];

			// Transmit logging information
			TransmitLogAndTimestamp();
			TransmitCellPackVoltages(cellVolts, 17);
			TransmitCurrentReading(currentRead);
			TransmitTemperatures(temperatures, 4);

			// Disable status LED to indicate data finished being logged
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
    	}

    	// Check if AFE Alarm Status indicates one of the safety status bits has been set
    	if (logAlertsFlag) {
    		logAlertsFlag = false;
			// Read the Alarm Status register to figure out what's causing the alert
    		DirectCmdRead(0x62, readData, 2);
    		writeData[0] = 0x00;
    		writeData[1] = 0x00;
    		// Check each bit and determine where to look for the cause of the alert
    		// Safety status B/C
    		if (readData[1] & (1 << 7)) {
    			TransmitSafetyStatusB();
    			// No need to check safety status C as no protections there are enabled
    			writeData[1] |= (1 << 7);
    		}
    		// Safety status A
    		if (readData[1] & (1 << 6)) {
    			TransmitSafetyStatusA();
    			writeData[1] |= (1 << 6);
    		}
    		// Permanent failure
    		if (readData[1] & (1 << 5)) {
    			// If there's a permanent failure, continuously transmit a distress signal
    			uint8_t msg[] = "Permanent failure! All BMS operations halted, requesting attention...";
    			while (1) {
    				HAL_Delay(10000);
    				HAL_UART_Transmit(&huart1, msg, sizeof(msg) - 1, HAL_MAX_DELAY);
    			}
    		}
    		// Clear the bits for the received safety statuses, as well as the masked safety alerts
    		writeData[1] |= 0x18;
//    		writeData[0] = 0xFF;
//    		writeData[1] = 0xFE;
    		DirectCmdWrite(0xE2, writeData, 2);
    	}
    }
}

/**
 * Configures the clocks for the STM32
 */
void SystemClock_Config(void) {
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

/**
 * Initializes all GPIO pins
 */
void GPIO_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Configure PB0 as SPI1_NSS (software controlled, open-drain)
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); // Set NSS high

    // Configure PB5 as a GPIO (push-pull, no pull-up)
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Configure PA5 (SPI1_SCK), PA6 (SPI1_MISO), PA7 (SPI1_MOSI)
    GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Configure PB6 (UART_TX), PB7 (UART_RX)
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1; // Alternate function for USART1
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	// Configure PA4 (AFE ALERT pin) as an external interrupt
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);

	// Configure PA1 (T4 pin) as an analog input
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

// Initializes the SPI1 peripheral in master mode
void SPI1_Init(void) {
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

/**
 * Initializes the USART1 peripheral in UART TX/RX mode
 */
void USART1_Init(void) {
	// Enable USART1 clock
	__HAL_RCC_USART1_CLK_ENABLE();

	// Configure UART peripheral
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 9600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;

	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		// Initialization error
		while (1);
	}
}

/**
 * Initializes the TIM1 peripheral with interrupts enabled
 */
void TIM1_Init(void) {
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; // Enable TIM1 clock

	TIM1->PSC = 4000 - 1; // Given 4MHz clock, 4000 cycles for 1ms
	TIM1->ARR = 5000 - 1; // Generate interrupt every 5000ms (5s)

	TIM1->DIER |= TIM_DIER_UIE; // Enable update interrupt

	NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 1); // Set TIM1 interrupt priority
	NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn); // Enable TIM1 interrupt

	TIM1->CR1 |= TIM_CR1_CEN; // Enable TIM1
}

/**
 * Initializes the TIM2 peripheral with interrupts enabled
 */
void TIM2_Init(void) {
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN; // Enable TIM2 clock

	TIM2->PSC = 4000 - 1; // Given 4MHz clock, 4000 cycles for 1ms
	TIM2->ARR = 500 - 1; // Generate interrupt every 500ms (half second)

	TIM2->DIER |= TIM_DIER_UIE; // Enable update interrupt

	NVIC_SetPriority(TIM2_IRQn, 0); // Set TIM2 interrupt priority
	NVIC_EnableIRQ(TIM2_IRQn); // Enable TIM2 interrupt

	TIM2->CR1 |= TIM_CR1_CEN; // Enable TIM2
}

/**
 * Defining the ISR for the TIM1 timer
 */
void TIM1_UP_TIM16_IRQHandler(void) {
	// Check if UIF flag is set for TIM1
	if (TIM1->SR & TIM_SR_UIF) {
		logDataFlag = true; // Raise a flag to log data from the AFE
		TIM1->SR &= ~TIM_SR_UIF; // Clear UIF flag
	}
}

void TIM2_IRQHandler(void) {
	// Check if UIF flag is set for TIM2
    if (TIM2->SR & TIM_SR_UIF) {
    	logAlertsFlag = true; // Raise a flag to check for safety alerts from the AFE
        TIM2->SR &= ~TIM_SR_UIF; // Clear the interrupt flag
    }
}

/**
 * Initializes the ADC1 peripheral in 12-bit resolution
 */
void ADC1_Init(void) {
    ADC_ChannelConfTypeDef sConfig = {0};

    // Enable the clock for ADC1
    __HAL_RCC_ADC_CLK_ENABLE();

    // Configure the ADC peripheral
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;  // 12-bit resolution
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;  // Right data alignment
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;  // Single channel
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;  // End of conversion flag after each conversion
    hadc1.Init.ContinuousConvMode = DISABLE;  // Single conversion mode
    hadc1.Init.NbrOfConversion = 1;  // Single conversion
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;  // Start conversion by software
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;

    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        // Initialization error
        Error_Handler();
    }

    // Configure the ADC regular channel (PA1 = ADC_CHANNEL_6)
    sConfig.Channel = ADC_CHANNEL_6;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;  // Sample time (adjust as necessary)

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        // Channel configuration error
        Error_Handler();
    }
}

/**
  * Initializes the RTC peripheral
  */
void RTC_Init(void) {
	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef sDate = {0};

	// Initialize RTC
	hrtc.Instance = RTC;
	hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
	hrtc.Init.AsynchPrediv = 127;
	hrtc.Init.SynchPrediv = 255;
	hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
	hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
	hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
	hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
	if (HAL_RTC_Init(&hrtc) != HAL_OK) {
		Error_Handler();
	}

	// Set initial time
	sTime.Hours = 0x09;
	sTime.Minutes = 0x32;
	sTime.Seconds = 0x00;
	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
	{
		Error_Handler();
	}

	// Set initial date
	sDate.WeekDay = RTC_WEEKDAY_TUESDAY;
	sDate.Month = RTC_MONTH_DECEMBER;
	sDate.Date = 0x10;
	sDate.Year = 0x24; // Year 2024
	if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * Calculates a CRC value according to the polynomial x^8 + x^2 + x + 1
 * @param data Pointer to an array storing the data bytes that will be transmitted
 * @param len Number of bytes that will be transmitted
 * @return The calculated CRC byte
 */
uint8_t crc8(uint8_t *data, size_t len) {
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

/**
 * Sends a direct command to the AFE and reads the data that is output
 * @param cmd The address byte for the command
 * @param returnData Pointer to the 8-bit integer array for storing the read data
 * @param len Number of bytes to read from the AFE. The function automatically increments the address byte based on this value
 */
void DirectCmdRead(uint8_t cmd, uint8_t *returnData, uint8_t len) {
	uint8_t rxData[3] = {0};
	uint8_t txData[3] = {0};
	uint8_t fullCmd[] = { cmd, 0xFF }; // Data byte doesn't matter since it's a read, just use 0xFF
	uint8_t crcLower = 0;
//	bool commReceived = false;

	// Increment the command address based on the data length given
	for (int i = 0; i < len; i++)
	{
		fullCmd[0] = cmd + i; // Increment the address
		crcLower = crc8(fullCmd, 2); // Recalculate the CRC
		// Construct the TX data for the SPI transaction
		txData[0] = fullCmd[0];
		txData[1] = fullCmd[1];
		txData[2] = crcLower;

		AFETransmitReadCmd(txData, rxData, sizeof(txData));
		returnData[i] = rxData[1]; // Save data byte received from last transaction
	}
}

/**
 * Sends a direct command to the AFE and writes the provided data to it
 * @param cmd The address byte for the command
 * @param writeData Pointer to the 8-bit integer array containing the data to write
 * @param len Number of bytes to write to the AFE. The function automatically increments the address byte based on this value
 */
void DirectCmdWrite(uint8_t cmd, uint8_t *writeData, uint8_t len) {
	uint8_t rxData[3] = {0};
	uint8_t txData[3] = {0};
	uint8_t fullCmd[2] = {0};
	uint8_t crcLower = 0;

	// Increment the command address based on the data length given
	for (int i = 0; i < len; i++) {
		fullCmd[0] = cmd + i;
		fullCmd[1] = writeData[i];
		crcLower = crc8(fullCmd, 2);
		// Construct the TX data for the SPI transaction
		txData[0] = fullCmd[0];
		txData[1] = fullCmd[1];
		txData[2] = crcLower;

		AFETransmitWriteCmd(txData, rxData, sizeof(txData));
	}
}

/**
 * Sends a sub-command to the AFE, no data is written or read
 * @param cmd The upper and lower address bytes for the sub-command
 */
void SubCmdNoData(uint16_t cmd) {
	uint8_t rxData[3] = {0};
	uint8_t commandLowerAddr[] = { LOWER_ADDR_REG_WRITE, ((uint8_t)(cmd & 0xFF)) };
	uint8_t crcLower = crc8(commandLowerAddr, 2);
	uint8_t commandUpperAddr[] = { UPPER_ADDR_REG_WRITE, ((uint8_t)(cmd >> 8)) };
	uint8_t crcUpper = crc8(commandUpperAddr, 2);

	// Keep writing the command until MISO reflects command was received
	// Starting with lower byte
	uint8_t txData[] = { commandLowerAddr[0], commandLowerAddr[1], crcLower };
	AFETransmitWriteCmd(txData, rxData, sizeof(txData));

	// Continue to upper byte
	txData[0] = commandUpperAddr[0];
	txData[1] = commandUpperAddr[1];
	txData[2] = crcUpper;
	AFETransmitWriteCmd(txData, rxData, sizeof(txData));
}

/**
 * Sends a sub-command to the AFE and reads the data that is output
 * @param cmd The upper and lower address bytes for the sub-command
 * @param returnData Pointer to the 8-bit integer array for storing the read data
 * @param len Number of bytes to read from the AFE's 32-byte data buffer
 */
void SubCmdReadData(uint16_t cmd, uint8_t *returnData, uint8_t len) {
	uint8_t rxData[3] = {0};
	uint8_t commandLowerAddr[] = { LOWER_ADDR_REG_WRITE, ((uint8_t)(cmd & 0xFF)) };
	uint8_t crcLower = crc8(commandLowerAddr, 2);
	uint8_t commandUpperAddr[] = { UPPER_ADDR_REG_WRITE, ((uint8_t)(cmd >> 8)) };
	uint8_t crcUpper = crc8(commandUpperAddr, 2);

	// Keep writing the command until MISO reflects command was received
	// Starting with lower byte
	uint8_t txData[] = { commandLowerAddr[0], commandLowerAddr[1], crcLower };
	AFETransmitWriteCmd(txData, rxData, sizeof(txData));

	// Continue to upper byte
	txData[0] = commandUpperAddr[0];
	txData[1] = commandUpperAddr[1];
	txData[2] = crcUpper;
	AFETransmitWriteCmd(txData, rxData, sizeof(txData));

	// Read each byte based on the data length given in parameters
	uint8_t readData[2] = {0};
	for (int i = 0; i < len; i++) {
		readData[0] = READ_DATA_BUFF_LSB + i;
		readData[1] = 0xFF;

		txData[0] = readData[0];
		txData[1] = readData[1];
		txData[2] = crc8(readData, 2);

		AFETransmitReadCmd(txData, rxData, sizeof(txData));

		returnData[i] = rxData[1]; // Save data byte received from last transaction
	}
}

/**
 * Reads the value stored in one of the AFE's RAM registers
 * Might remove this function, it's exactly the same as the SubCmd read data function
 * @param addr The register address
 * @param returnData Pointer to the 8-bit integer array for storing the read data
 * @param len Number of bytes to read from the AFE's 32-byte data buffer
 */
void RAMRegisterRead(uint16_t addr, uint8_t *returnData, uint8_t len) {
	// Preparing the SPI transaction to send to tell the AFE a RAM register read is happening
	uint8_t rxData[3] = {0};
	uint8_t commandLowerAddr[] = { LOWER_ADDR_REG_WRITE, ((uint8_t)(addr & 0xFF)) };
	uint8_t crcLower = crc8(commandLowerAddr, 2);
	uint8_t commandUpperAddr[] = { UPPER_ADDR_REG_WRITE, ((uint8_t)(addr >> 8)) };
	uint8_t crcUpper = crc8(commandUpperAddr, 2);

	// Keep writing the command until MISO reflects command was received
	// Starting with lower byte
	uint8_t txData[] = { commandLowerAddr[0], commandLowerAddr[1], crcLower };
	AFETransmitWriteCmd(txData, rxData, sizeof(txData));

	// Continue to upper byte
	txData[0] = commandUpperAddr[0];
	txData[1] = commandUpperAddr[1];
	txData[2] = crcUpper;
	AFETransmitWriteCmd(txData, rxData, sizeof(txData));

	// Read each byte based on the data length given in parameters
	uint8_t readData[2] = {0};
	for (int i = 0; i < len; i++) {
		readData[0] = READ_DATA_BUFF_LSB + i;
		readData[1] = 0xFF;

		txData[0] = readData[0];
		txData[1] = readData[1];
		txData[2] = crc8(readData, 2);

		AFETransmitReadCmd(txData, rxData, sizeof(txData));

		returnData[i] = rxData[1]; // Save data byte received from last transaction
	}
}

/**
 * Writes the given value in one of the AFE's RAM registers
 * @param addr The register address
 * @param writeData Pointer to the 8-bit integer array for the data to write to the register
 * @param len Number of bytes to write to the AFE's 32-byte data buffer
 */
void RAMRegisterWrite(uint16_t addr, uint8_t *writeData, uint8_t len) {
	uint8_t rxData[3] = {0};
	uint8_t lowerAddr[] = { LOWER_ADDR_REG_WRITE, ((uint8_t)(addr & 0xFF)) };
	uint8_t crcLower = crc8(lowerAddr, 2);
	uint8_t upperAddr[] = { UPPER_ADDR_REG_WRITE, ((uint8_t)(addr >> 8)) };
	uint8_t crcUpper = crc8(upperAddr, 2);

	// Keep writing the register address until MISO reflects command was received
	// Starting with lower byte
	uint8_t txData[] = { lowerAddr[0], lowerAddr[1], crcLower };
	AFETransmitWriteCmd(txData, rxData, sizeof(txData));

	// Continue to upper byte
	txData[0] = upperAddr[0];
	txData[1] = upperAddr[1];
	txData[2] = crcUpper;
	AFETransmitWriteCmd(txData, rxData, sizeof(txData));

	// Write the data provided to the AFE's 32-byte data buffer
	uint8_t writeBytes[2] = {0};
	for (int i = 0; i < len; i++)
	{
		// Increment data buffer address and include the next address byte
		writeBytes[0] = WRITE_DATA_BUFF_LSB + i;
		writeBytes[1] = writeData[i];

		txData[0] = writeBytes[0];
		txData[1] = writeBytes[1];
		txData[2] = crc8(writeBytes, 2); // Recalculate CRC

		AFETransmitWriteCmd(txData, rxData, sizeof(txData));
	}

	// Calculate the check-sum and write it to the AFE's checksum register
	uint8_t checkSum = 0;
	for (int i = 0; i < len; i++) {
		checkSum += writeData[i];
	}
	checkSum += lowerAddr[1];
	checkSum += upperAddr[1];
	checkSum = ~(checkSum);

	writeBytes[0] = WRITE_CHECKSUM_ADDR;
	writeBytes[1] = checkSum;

	txData[0] = writeBytes[0];
	txData[1] = writeBytes[1];
	txData[2] = crc8(writeBytes, 2); // Recalculate CRC

	AFETransmitWriteCmd(txData, rxData, sizeof(txData));

	// Write the data length to the AFE's data length register
	writeBytes[0] = WRITE_DATALEN_ADDR;
	writeBytes[1] = len + 4; // Length of data buffer, plus upper and lower address bytes, plus checksum and data length bytes

	txData[0] = writeBytes[0];
	txData[1] = writeBytes[1];
	txData[2] = crc8(writeBytes, 2); // Recalculate CRC

	AFETransmitWriteCmd(txData, rxData, sizeof(txData));
}

/**
 * Programs all the relevant AFE registers. To be used whenever BMS powers on or AFE enters SHUTDOWN
 */
void RAMRegisterInit(void) {
	// Initializing buffer for writing data to AFE
	uint8_t writeData[32] = {0};

	/**
	 * Configuration settings registers
	 */
	// Configure TS pins
	writeData[0] = 0x07; // TS1 & TS2: Thermistor temperature, for cell temperature protection
	RAMRegisterWrite(SET_CONF_TS1_CFG, writeData, 1);
	RAMRegisterWrite(SET_CONF_TS2_CFG, writeData, 1);
	writeData[0] = 0x0F; // TS3: Thermistor temperature, for FET temperature protection
	RAMRegisterWrite(SET_CONF_TS3_CFG, writeData, 1);
	// Configure ALERT pin
	writeData[0] = 0x2A;
	RAMRegisterWrite(SET_CONF_ALERT_CFG, writeData, 1);
	// Configure DA
	writeData[0] = 0x06;
	RAMRegisterWrite(SET_CONF_DA_CFG, writeData, 1);

	/**
	 * Protection settings registers
	 */
	writeData[0] = 0xFC;
	RAMRegisterWrite(SET_PROT_ENPROT_A, writeData, 1); // Enables SCD, OCD1, OCC, COV, CUV protection
	writeData[0] = 0xF7;
	RAMRegisterWrite(SET_PROT_ENPROT_B, writeData, 1); // Enables OTF, OTINT, OTD, OTC, and all UT protection
	writeData[0] = 0x00;
	RAMRegisterWrite(SET_PROT_ENPROT_C, writeData, 1); // Disables all special/latch protections
	writeData[0] = 0x98;
	RAMRegisterWrite(SET_PROT_CHGFET_PROT_A, writeData, 1); // SCD, OCC, and COV disable CHG FET
	writeData[0] = 0xD4;
	RAMRegisterWrite(SET_PROT_CHGFET_PROT_B, writeData, 1); // OTF, OTINT, OTC, and UTINT disable CHG FET
	writeData[0] = 0x00;
	RAMRegisterWrite(SET_PROT_CHGFET_PROT_C, writeData, 1); // Type C protections are disabled anyways
	writeData[0] = 0xE4;
	RAMRegisterWrite(SET_PROT_DSGFET_PROT_A, writeData, 1); // SCD, OCD1, OCD2, and CUV disable DSG FET
	writeData[0] = 0xE4;
	RAMRegisterWrite(SET_PROT_DSGFET_PROT_B, writeData, 1); // OTF, OTINT, OTD, and UTINT disable DSG FET
	writeData[0] = 0x00;
	RAMRegisterWrite(SET_PROT_DSGFET_PROT_C, writeData, 1); // Type C protections are disabled anyways

	/**
	 * FET settings registers
	 */
	writeData[0] = 0x1E;
	RAMRegisterWrite(SET_FET_OPTIONS, writeData, 1); // Enable PDSG, disable body diode protection, enable CHG FET in SLEEP
	format_uint16(writeData, 0x06A4);
	RAMRegisterWrite(SET_FET_PCHG_STRT_V, writeData, 2); // Min. cell voltage below 1700mV activates PCHG mode
	format_uint16(writeData, 0x06D6);
	RAMRegisterWrite(SET_FET_PCHG_STP_V, writeData, 2); // Min. cell voltage above 1750mV deactivates PCHG mode
	writeData[0] = 0x64;
	RAMRegisterWrite(SET_FET_PDSG_TO, writeData, 1); // PDSG timeout after 1000ms, enables DSG FET after
	writeData[0] = 0x64;
	RAMRegisterWrite(SET_FET_PDSG_STP_DLT, writeData, 1); // Exit PDSG and enable DSG FET when LD equals VBAT+ minus 1000mV

	/**
	 * Misc. settings
	 */
	// Setting MFG Status Init to disable FET Test commands
	format_uint16(writeData, 0x0050);
	RAMRegisterWrite(SET_MFG_STATUS_INIT, writeData, 2);
	// Setting DSG threshold to 100mA and CHG threshold to 50mA
	format_uint16(writeData, 0x000A);
	RAMRegisterWrite(SET_CURRTH_DSG_CURRTH, writeData, 2);
	format_uint16(writeData, 0x0005);
	RAMRegisterWrite(SET_CURRTH_CHG_CURRTH, writeData, 2);

	/**
	 * Cell balancing settings registers
	 */
	writeData[0] = 0x0F;
	RAMRegisterWrite(SET_CLBCFG_CONFIG, writeData, 1); // Exits SLEEP to perform balancing, allow balancing while charging and in relax mode
	writeData[0] = 0x0A;
	RAMRegisterWrite(SET_CLBCFG_CB_INTRVL, writeData, 1); // Recalculates which cells to balance every 10 seconds
	writeData[0] = 0x08;
	RAMRegisterWrite(SET_CLBCFG_CB_MAX_CLS, writeData, 1); // Allows up to 8 cells to be balanced at once
	// Min. cell voltage must be at least 2500mV for cell balancing to occur while charging or in relax mode
	format_uint16(writeData, 0x09C4);
	RAMRegisterWrite(SET_CLBCFG_CHG_MIN_V, writeData, 2);
	RAMRegisterWrite(SET_CLBCFG_RLX_MIN_V, writeData, 2);

	/**
	 * Power registers
	 */
	format_uint16(writeData, 0x0960);
	RAMRegisterWrite(PWR_SHDN_BATT_V, writeData, 2); // If pack voltage falls below 24000mV, AFE enters SHUTDOWN mode
	format_uint16(writeData, 0x000A);
	RAMRegisterWrite(PWR_SLP_CURR, writeData, 2); // Current above 10mA will cause device to exit SLEEP mode
	format_uint16(writeData, 0x0960);
	RAMRegisterWrite(PWR_SLP_CHG_V_THLD, writeData, 2); // If pack voltage falls below 24000mV, SLEEP mode is blocked when charger detected

	/**
	 * Protections registers (testing only)
	 */
	writeData[0] = 0x1A;
	RAMRegisterWrite(PROT_CUV_THLD, writeData, 1); // CUV triggered at 1.265V, cleared above 1.3662V
	writeData[0] = 0x23;
	RAMRegisterWrite(PROT_COV_THLD, writeData, 1); // COV triggered at 1.771V, cleared below 1.6698V

	/**
	 * Protections registers
	 */
//	writeData[0] = 0x23;
//	RAMRegisterWrite(PROT_CUV_THLD, writeData, 1); // CUV triggered at 1.771V, cleared above 1.8732V
//	writeData[0] = 0x37;
//	RAMRegisterWrite(PROT_COV_THLD, writeData, 1); // COV triggered at 2.783V, cleared below 2.6818V
	writeData[0] = 0x08;
	RAMRegisterWrite(PROT_OCC_THLD, writeData, 1); // OCC triggered at 16A
	writeData[0] = 0x15;
	RAMRegisterWrite(PROT_OCD1_THLD, writeData, 1); // OCD1 triggered at 42A
	writeData[0] = 0x64;
	RAMRegisterWrite(PROT_OCD1_DLY, writeData, 1); // OCD1 triggered after 340ms delay
	writeData[0] = 0x19;
	RAMRegisterWrite(PROT_OCD2_THLD, writeData, 1); // OCD2 triggered at 50A
	writeData[0] = 0x1C;
	RAMRegisterWrite(PROT_OCD2_DLY, writeData, 1); // OCD2 triggered after 100ms delay
	writeData[0] = 0x03;
	RAMRegisterWrite(PROT_SCD_THLD, writeData, 1); // SCD triggered at 60A
	RAMRegisterWrite(PROT_SCD_DLY, writeData, 1); // SCD triggered after 30µs delay
	writeData[0] = 0xEC;
	RAMRegisterWrite(PROT_UTD_THLD, writeData, 1); // UTD triggered at -20ºC
	RAMRegisterWrite(PROT_UTC_THLD, writeData, 1); // UTC triggered at -20ºC
	writeData[0] = 0xF1;
	RAMRegisterWrite(PROT_UTD_RCVR, writeData, 1); // UTD cleared above -15ºC
	RAMRegisterWrite(PROT_UTC_RCVR, writeData, 1); // UTC cleared above -15ºC
}

/**
 * Handles the proper SPI communication procedure with the AFE for a SPI read command
 * @param txBytes Pointer to array containing the data to transmit
 * @param rxBytes Pointer to array containing the data to be received
 * @param arrSize Number of bytes that will be transmitted/received
 */
void AFETransmitReadCmd(uint8_t *txBytes, uint8_t *rxBytes, uint8_t arrSize) {
	// Continuously transmit the SPI transaction until the AFE has received it
	bool commReceived = false;
	uint8_t readBytes[2] = {0};
	uint8_t crcReceived = 0;
	while (!commReceived)
	{
		// Pull NSS low
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
		// Transmit data and receive AFE's response
		HAL_SPI_TransmitReceive(&hspi1, txBytes, rxBytes, arrSize, HAL_MAX_DELAY);

		// For read command, confirm the AFE received the command by checking the address and CRC bytes
//		if (txBytes[0] == rxBytes[0]) commReceived = true;
		readBytes[0] = rxBytes[0];
		readBytes[1] = rxBytes[1];
		crcReceived = crc8(readBytes, 2);
		if (txBytes[0] == rxBytes[0] && crcReceived == rxBytes[2]) commReceived = true;

		// Pull NSS high and wait for transaction to be processed
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
		HAL_Delay(1);
	}

}

/**
 * Handles the proper SPI communication procedure with the AFE for a SPI write command
 * @param txBytes Pointer to array containing the data to transmit
 * @param rxBytes Pointer to array containing the data to be received
 * @param arrSize Number of bytes that will be transmitted/received
 */
void AFETransmitWriteCmd(uint8_t *txBytes, uint8_t *rxBytes, uint8_t arrSize) {
	// Continuously transmit the SPI transaction until the AFE has received it
	bool commReceived = false;
	while (!commReceived)
	{
		// Pull NSS low
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
		// Transmit data and receive AFE's response
		HAL_SPI_TransmitReceive(&hspi1, txBytes, rxBytes, arrSize, HAL_MAX_DELAY);

		// For write command, confirm the AFE received the command by checking every single byte
		commReceived = true;
		for (int i = 0; i < arrSize; i++)
		{
			if (txBytes[i] != rxBytes[i]) commReceived = false; // If any mismatch occurs, flag it and retransmit
		}

		// Pull NSS high and wait for transaction to be processed
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
		HAL_Delay(1);
	}
}

/**
 * Helper function to begin transmission of logging information with the current timestamp
 */
void TransmitLogAndTimestamp(void) {
	char buffer[512] = {0}; // Initialize buffer to store message
	char temp[48]; // Temporary buffer for each line

	// Preparing a heading for the data log
	snprintf(temp, sizeof(temp), "**************************************\n\r");
	strncat(buffer, temp, sizeof(buffer) - strlen(buffer) - 1);
	snprintf(temp, sizeof(temp), "************ BMS DATA LOG ************\n\r");
	strncat(buffer, temp, sizeof(buffer) - strlen(buffer) - 1);
	snprintf(temp, sizeof(temp), "**************************************\n\r");
	strncat(buffer, temp, sizeof(buffer) - strlen(buffer) - 1);

	// Logging the timestamp
	snprintf(temp, sizeof(temp), "Timestamp: ");
	strncat(buffer, temp, sizeof(buffer) - strlen(buffer) - 1);
	GetDateTime(temp, sizeof(temp)); // Getting the current date and time
	strncat(buffer, temp, sizeof(buffer) - strlen(buffer) - 1);
	snprintf(temp, sizeof(temp), "\n\n\r");
	strncat(buffer, temp, sizeof(buffer) - strlen(buffer) - 1);

	// Transmit the final message over UART
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

/**
 * Helper function to transmit all the cell voltage readings over UART
 * @param volts Array containing the cell and pack voltage readings
 * @param len Length of the provided array
 */
void TransmitCellPackVoltages(uint16_t *volts, uint8_t len) {
	char buffer[1024] = {0}; // Initialize buffer to store message
	char temp[32]; // Temporary buffer for each line

	uint8_t lines = 0;
	uint8_t entriesPerLine = 5;

	// Preparing a heading for the current reading
	snprintf(temp, sizeof(temp), "******** VOLTAGE ********\n\r");
	strncat(buffer, temp, sizeof(buffer) - strlen(buffer) - 1);

	// Calculate how many lines to make based on number of voltage readings (8 readings per line max.)
	if (len % entriesPerLine == 0) lines = len / entriesPerLine;
	else lines = len / entriesPerLine + 1;

	for (int i = 0; i < lines; i++) {
		// First write out the cell numbers
		for (int j = 1; j <= entriesPerLine; j++) {
			if ((i*entriesPerLine + j) > len) break; // Break once number of voltage readings has been reached
			// Different header for the last entry (pack voltage)
			if ((i*entriesPerLine + j) == len) {
				snprintf(temp, sizeof(temp), "VBAT\t\t");
				strncat(buffer, temp, sizeof(buffer) - strlen(buffer) - 1);
			}
			else {
				snprintf(temp, sizeof(temp), "CV%d\t\t", (i*entriesPerLine + j));
				strncat(buffer, temp, sizeof(buffer) - strlen(buffer) - 1);
			}
		}
		snprintf(temp, sizeof(temp), "\n\r");
		strncat(buffer, temp, sizeof(buffer) - strlen(buffer) - 1);
		// On the following line, write out the cell voltage readings
		for (int j = 1; j <= entriesPerLine; j++) {
			if ((i*entriesPerLine + j) > len) break; // Break once number of voltage readings has been reached
			// Alternate handling for data for pack voltage to write it in volts instead of mV
			if ((i*entriesPerLine + j) == len) {
				snprintf(temp, sizeof(temp), "%d.%dV\t\t", volts[i*entriesPerLine + j - 1] / 100, volts[i*entriesPerLine + j - 1] % 100);
				strncat(buffer, temp, sizeof(buffer) - strlen(buffer) - 1);
			}
			else {
				snprintf(temp, sizeof(temp), "%dmV\t\t", volts[i*entriesPerLine + j - 1]);
				strncat(buffer, temp, sizeof(buffer) - strlen(buffer) - 1);
			}
		}
		snprintf(temp, sizeof(temp), "\n\n\r");
		strncat(buffer, temp, sizeof(buffer) - strlen(buffer) - 1);
	}

	// Transmit the final message over UART
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

void TransmitADCReadings(uint32_t *counts, uint8_t len) {
	char buffer[1024] = {0}; // Initialize buffer to store message
	char temp[32]; // Temporary buffer for each line

	for (int i = 1; i <= len; i++) {
		// Format the data into a single line
		snprintf(temp, sizeof(temp), "CV%d: %lu mV\n\r", i, counts[i-1]);
		// Append the formatted data to the buffer
		strncat(buffer, temp, sizeof(buffer) - strlen(buffer) - 1);
	}

	// Transmit the final message over UART
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

/**
 * Helper function to transmit the battery current reading over UART
 * @param current CC2 current reading - negative for discharging, positive for charging
 */
void TransmitCurrentReading(int16_t current) {
	char buffer[128] = {0}; // Initialize buffer to store message
	char temp[32] = {0}; // Temporary buffer for each line

	// Preparing a heading for the current reading
	snprintf(temp, sizeof(temp), "******** CURRENT ********\n\r");
	strncat(buffer, temp, sizeof(buffer) - strlen(buffer) - 1);

	// Determine if charging or discharging based on current value
	if (current > 0) {
		snprintf(temp, sizeof(temp), "CC2: Charging at %d mA\n\n\r", current*10);
	}
	else {
		snprintf(temp, sizeof(temp), "CC2: Discharging at %d mA\n\n\r", current*(-10));
	}
	strncat(buffer, temp, sizeof(buffer) - strlen(buffer) - 1);
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

/**
 * Helper function to transmit all the temperature readings over UART
 * @param temps Array containing the temperature readings
 * @param len Length of the provided array
 */
void TransmitTemperatures(int16_t *temps, uint8_t len) {
	char buffer[1024] = {0}; // Initialize buffer to store message
	char temp[32]; // Temporary buffer for each line

	// Separate variables for integer part and decimal part of temperature reading
	int16_t degC = 0;
	int16_t deg_int = 0;
	int16_t deg_dec = 0;

	// Preparing a heading for the temperature readings
	snprintf(temp, sizeof(temp), "******** TEMPERATURE ********\n\r");
	strncat(buffer, temp, sizeof(buffer) - strlen(buffer) - 1);

	// Tabulating the temperature sensors
	for (int i = 1; i <= len; i++) {
		snprintf(temp, sizeof(temp), "TS%d\t\t", i);
		strncat(buffer, temp, sizeof(buffer) - strlen(buffer) - 1);
	}

	snprintf(temp, sizeof(temp), "\n\r");
	strncat(buffer, temp, sizeof(buffer) - strlen(buffer) - 1);

	for (int i = 1; i <= len; i++) {
		// Convert from Kelvin into Celsius
		degC = temps[i-1] - 2731;
		deg_int = degC / 10;
		deg_dec = degC % 10;
		// If temperature is negative, keep the decimal part positive
		if (degC < 0 && deg_dec != 0) deg_dec = abs(deg_dec);

		snprintf(temp, sizeof(temp), "%d.%dºC\t\t", deg_int, deg_dec);
		strncat(buffer, temp, sizeof(buffer) - strlen(buffer) - 1);
	}

	snprintf(temp, sizeof(temp), "\n\n\r");
	strncat(buffer, temp, sizeof(buffer) - strlen(buffer) - 1);

	// Transmit the final message over UART
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

/**
 * Helper function to transmit UART messages corresponding to any faults detected in the
 * Safety Status A register
 */
void TransmitSafetyStatusA(void) {
	// Read the bits of the Safety Status A register and transmit the appropriate message
	// if the corresponding fault was triggered
	uint8_t statusA[1] = {0};
	DirectCmdRead(0x03, statusA, 1);

	// Short Circuit Discharge
	if (statusA[0] & (1 << 7)) {
		uint8_t msg[] = "SCD fault triggered! Discharging will be disabled for a moment...\n\r";
		HAL_UART_Transmit(&huart1, msg, sizeof(msg) - 1, HAL_MAX_DELAY);
	}
	// Overcurrent in Discharge 1st Tier
	if (statusA[0] & (1 << 5)) {
		uint8_t msg[] = "OCD1 fault triggered! Discharging will be disabled for a moment...\n\r";
		HAL_UART_Transmit(&huart1, msg, sizeof(msg) - 1, HAL_MAX_DELAY);
	}
	// Overcurrent in Charge
	if (statusA[0] & (1 << 4)) {
		uint8_t msg[] = "OCC fault triggered! Charging will be disabled for a moment...\n\r";
		HAL_UART_Transmit(&huart1, msg, sizeof(msg) - 1, HAL_MAX_DELAY);
	}
	// Cell Overvoltage
	if (statusA[0] & (1 << 3)) {
		uint8_t msg[] = "COV fault triggered! Charging will be disabled until voltage drops sufficiently.\n\r";
		HAL_UART_Transmit(&huart1, msg, sizeof(msg) - 1, HAL_MAX_DELAY);
	}
	// Cell Undervoltage
	if (statusA[0] & (1 << 2)) {
		uint8_t msg[] = "CUV fault triggered! Discharging will be disabled until voltage rises sufficiently.\n\r";
		HAL_UART_Transmit(&huart1, msg, sizeof(msg) - 1, HAL_MAX_DELAY);
	}
}

/**
 * Helper function to transmit UART messages corresponding to any faults detected in the
 * Safety Status B register
 */
void TransmitSafetyStatusB(void) {
	// Read the bits of the Safety Status B register and transmit the appropriate message
	// if the corresponding fault was triggered
	uint8_t statusB[1] = {0};
	DirectCmdRead(0x05, statusB, 1);

	// FET Overtemperature
	if (statusB[0] & (1 << 7)) {
		uint8_t msg[] = "OTF fault triggered! Discharging will be disabled for a moment...\n\r";
		HAL_UART_Transmit(&huart1, msg, sizeof(msg) - 1, HAL_MAX_DELAY);
	}
	// Internal Overtemperature
	if (statusB[0] & (1 << 6)) {
		uint8_t msg[] = "OTINT fault triggered! All AFE operations will be disabled for a moment...\n\r";
		HAL_UART_Transmit(&huart1, msg, sizeof(msg) - 1, HAL_MAX_DELAY);
	}
	// Overtemperature in Discharge
	if (statusB[0] & (1 << 5)) {
		uint8_t msg[] = "OTD fault triggered! Discharging will be disabled for a moment...\n\r";
		HAL_UART_Transmit(&huart1, msg, sizeof(msg) - 1, HAL_MAX_DELAY);
	}
	// Overtemperature in Charge
	if (statusB[0] & (1 << 4)) {
		uint8_t msg[] = "OTC fault triggered! Charging will be disabled for a moment...\n\r";
		HAL_UART_Transmit(&huart1, msg, sizeof(msg) - 1, HAL_MAX_DELAY);
	}
	// Internal Undertemperature
	if (statusB[0] & (1 << 2)) {
		uint8_t msg[] = "UTINT fault triggered! All AFE operations will be disabled for a moment...\n\r";
		HAL_UART_Transmit(&huart1, msg, sizeof(msg) - 1, HAL_MAX_DELAY);
	}
	// Undertemperature in Discharge
	if (statusB[0] & (1 << 1)) {
		uint8_t msg[] = "UTD fault triggered! No operational changes, but prolonged operation is not advised.\n\r";
		HAL_UART_Transmit(&huart1, msg, sizeof(msg) - 1, HAL_MAX_DELAY);
	}
	// Undertemperature in Charge
	if (statusB[0] & 0x01) {
		uint8_t msg[] = "UTC fault triggered! No operational changes, but prolonged operation is not advised.\n\r";
		HAL_UART_Transmit(&huart1, msg, sizeof(msg) - 1, HAL_MAX_DELAY);
	}
}

/**
 * Measures the thermistor connected to the STM32's ADC and converts the reading to a temperature
 * @return Temperature of the thermistor in degrees Celsius
 */
int16_t T4_Acquire(void) {
    uint32_t adcValue = 0;
    float T4_volt = 0.0;
    float T4_res = 0.0;
    float T4_temp = 0.0;
    int16_t T4 = 0;

    // Start ADC conversion
    if (HAL_ADC_Start(&hadc1) != HAL_OK) {
        // Start error
        Error_Handler();
    }

    // Poll for end of conversion
    if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
        // Read the ADC conversion result (12-bit resolution)
        adcValue = HAL_ADC_GetValue(&hadc1);

        // Convert ADC value to voltage (divide by max. ADC counts, multiply by VREF (3.3V))
        T4_volt = ((float)adcValue / 4095.0) * V_REF;
        // Calculate thermistor resistance based on voltage (voltage divider rearranged)
        T4_res = T4_PU_R * (T4_volt / (T4_PU_V - T4_volt));
        // Using thermistor's beta value, calculate the temperature
        T4_temp = 1.0 / ((1.0 / 298.15) + (log(T4_res / T4_R0)) / T4_BETA);
        // Convert value in Kelvin to 16-bit integer like the other temperature measurements (units of 0.1K)
        T4 = (int16_t)(T4_temp * 10.0);
    }

    // Stop the ADC conversion
    HAL_ADC_Stop(&hadc1);

    return T4;
}

time_t RTCToUnixTimestamp(RTC_DateTypeDef *date, RTC_TimeTypeDef *time) {
	struct tm tm_time;

	// Populate the tm structure
	tm_time.tm_year = date->Year + 100; // Years since 1900
	tm_time.tm_mon  = date->Month - 1;  // Months since January
	tm_time.tm_mday = date->Date;       // Day of the month
	tm_time.tm_hour = time->Hours;
	tm_time.tm_min  = time->Minutes;
	tm_time.tm_sec  = time->Seconds;
	tm_time.tm_isdst = -1;              // No daylight saving time

	// Convert to UNIX timestamp
	return mktime(&tm_time);
}

/**
 * Helper function to get the current date and time and format it into a string
 * @param datetimeStr String to store the date & time in
 * @param size Size of the string (char array)
 */
void GetDateTime(char *datetimeStr, size_t size) {
	time_t now;
	struct tm *timeinfo;

	// Get the current time
	time(&now);

	// Convert the time to local time structure
	timeinfo = localtime(&now);

	// Format the time as a string
	strftime(datetimeStr, size, "%Y-%m-%d %H:%M:%S", timeinfo);
}

/**
 * Implementation of _gettimeofday using the STM32's RTC
 */
int _gettimeofday(struct timeval *tv, void *tzvp) {
    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;

    // Get the current RTC time and date
    if (HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK || HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
        return -1; // Error reading RTC
    }

    // Convert to UNIX timestamp
    time_t timestamp = RTCToUnixTimestamp(&sDate, &sTime);

    // Populate the timeval structure
    if (tv) {
        tv->tv_sec = timestamp; // Seconds since the Unix epoch
        tv->tv_usec = 0;       // Microseconds (RTC typically doesn't support this)
    }

    return 0;
}

/**
 * Error handler if a UART transmission error occurs
 * TODO: Implement this
 */
void Error_Handler(void) {
    // Stay in an infinite loop to allow for debugging
    while (1);
}

/**
 * Callback function for handling an interrupt from a GPIO pin
 * @param GPIO_Pin The GPIO pin number where an interrupt was received
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	uint8_t msg[] = "Interrupt! ";
	HAL_UART_Transmit(&huart1, msg, sizeof(msg) - 1, HAL_MAX_DELAY);

	switch (GPIO_Pin) {
	case GPIO_PIN_4:
		logAlertsFlag = true;
	}
}
