/*
 * bq76952.c
 *  Source file for implementing helper functions to communicate with the BQ76952 AFE
 *  Created on: Jul 27, 2024
 *      Author: Farris Matar
 */

#include "bq76952.h"

/**
 * Implementation for functions to format data types into the AFE's data buffer
 */

/**
 * Formats an unsigned 16-bit integer into the AFE's data buffer (little-endian byte order)
 * @param dataArr 8-bit integer array for storing the bytes to be transmitted to the AFE
 * @param data 16-bit integer to re-format
 */
void format_uint16(uint8_t *dataArr, uint16_t data) {
	dataArr[0] = ((uint8_t)(data & 0xFF));
	dataArr[1] = ((uint8_t)(data >> 8));
}

/**
 * Formats an unsigned 32-bit integer into the AFE's data buffer (little-endian byte order)
 * @param dataArr 8-bit integer array for storing the bytes to be transmitted to the AFE
 * @param data 32-bit integer to re-format
 */
void format_uint32(uint8_t *dataArr, uint32_t data) {
	dataArr[0] = ((uint8_t)(data & 0xFF));
	dataArr[1] = ((uint8_t)((data >> 8) & 0xFF));
	dataArr[2] = ((uint8_t)((data >> 16) & 0xFF));
	dataArr[3] = ((uint8_t)(data >> 24));
}

/**
 * Formats a signed 16-bit integer into the AFE's data buffer (little-endian byte order, 2's complement)
 * @param dataArr 8-bit integer array for storing the bytes to be transmitted to the AFE
 * @param data 16-bit integer to re-format
 */
void format_int16(uint8_t *dataArr, int data) {
	if (data < 0) format_uint16(dataArr, (~(abs(data)) + 1));
	else format_uint16(dataArr, data);
}

/**
 * Formats a signed 32-bit integer into the AFE's data buffer (little-endian byte order, 2's complement)
 * @param dataArr 8-bit integer array for storing the bytes to be transmitted to the AFE
 * @param data 32-bit integer to re-format
 */
void format_int32(uint8_t *dataArr, int data) {
	if (data < 0) format_uint32(dataArr, (~(abs(data)) + 1));
	else format_uint32(dataArr, data);
}

