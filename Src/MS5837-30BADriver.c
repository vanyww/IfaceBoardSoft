/*
 * MS5837-30BADriver.c
 *
 *  Created on: 27 мая 2018 г.
 *      Author: Satty
 */

#include "MS5837-30BADriver.h"
#include "i2c.h"

#define MS5837_ADDR 0x76
#define MS5837_READ_MASK 0x00
#define MS5837_WRITE_MASK 0x01

#define MS5837_RESET_COMMAND		0x1E
#define MS5837_READ_ADC_COMMAND		0x00
#define MS5837_PROM_READ_ADDRESS_0 	0xA0
#define MS5837_PROM_READ_ADDRESS_1 	0xA2
#define MS5837_PROM_READ_ADDRESS_2 	0xA4
#define MS5837_PROM_READ_ADDRESS_3 	0xA6
#define MS5837_PROM_READ_ADDRESS_4 	0xA8
#define MS5837_PROM_READ_ADDRESS_5 	0xAA
#define MS5837_PROM_READ_ADDRESS_6 	0xAC
#define MS5837_PROM_READ_ADDRESS_7 	0xAE

#define MS5837_CONVERSION_OSR_MASK 0x0F

#define MS5837_CONVERSION_TIME_OSR_256  1
#define MS5837_CONVERSION_TIME_OSR_512  2
#define MS5837_CONVERSION_TIME_OSR_1024 3
#define MS5837_CONVERSION_TIME_OSR_2048 5
#define MS5837_CONVERSION_TIME_OSR_4096 9
#define MS5837_CONVERSION_TIME_OSR_8192 17

#define MS5837_CRC_INDEX 								0
#define MS5837_PRESSURE_SENSITIVITY_INDEX 				1
#define MS5837_PRESSURE_OFFSET_INDEX 					2
#define MS5837_TEMP_COEFF_OF_PRESSURE_SENSITIVITY_INDEX 3
#define MS5837_TEMP_COEFF_OF_PRESSURE_OFFSET_INDEX 	    4
#define MS5837_REFERENCE_TEMPERATURE_INDEX 				5
#define MS5837_TEMP_COEFF_OF_TEMPERATURE_INDEX 			6
#define MS5837_SUBSIDIARY_VALUE_INDEX		 			7

#define MS5837_COEFFICIENT_NUMBER 7

#define MS5837_COMMAND_SIZE 1
#define MS5837_PROM_DATA_SIZE 2
#define MS5837_ADC_DATA_SIZE 3

#define MS5837_TIMEOUT_MS 1000

static enum MS5837ErrorCode MS5837ReadEEPROMCoeffs(void);
static uint16_t CRC4(uint16_t *n_prom);
static enum MS5837ErrorCode m_connectionStatus = ANY_ERROR; //false

static uint16_t m_eepromCoeffs[MS5837_COEFFICIENT_NUMBER + 1];
static uint32_t m_conversionTime[6] = { MS5837_CONVERSION_TIME_OSR_256,
										MS5837_CONVERSION_TIME_OSR_512,
										MS5837_CONVERSION_TIME_OSR_1024,
										MS5837_CONVERSION_TIME_OSR_2048,
										MS5837_CONVERSION_TIME_OSR_4096,
										MS5837_CONVERSION_TIME_OSR_8192 };

static uint16_t CRC4(uint16_t *n_prom) {
	uint16_t n_rem = 0;

	for (uint8_t cnt = 0; cnt < 16; cnt++)
	{
		if (cnt % 2 == 1) n_rem ^= n_prom[cnt >> 1] & 0x00FF;
		else			  n_rem ^= n_prom[cnt >> 1] >> 8;

		for (uint8_t n_bit = 8; n_bit > 0; n_bit--)
			if (n_rem & 0x8000) n_rem = (n_rem << 1) ^ 0x3000;
			else 				  n_rem = n_rem << 1;
	}

	n_rem = n_rem >> 12 & 0x000F;
	return n_rem ^ 0x00;
}

static enum MS5837ErrorCode MS5837ReadEEPROMCoeffs(void) {
	uint8_t buffer[MS5837_PROM_DATA_SIZE];

	for (uint8_t i = 0; i < MS5837_COEFFICIENT_NUMBER; i++) {
		if (HAL_I2C_Mem_Read(&hi2c1, MS5837_ADDR << 1 | MS5837_READ_MASK,
				MS5837_PROM_READ_ADDRESS_0 + (i << 1), MS5837_COMMAND_SIZE,
				buffer, MS5837_PROM_DATA_SIZE, MS5837_TIMEOUT_MS))
			return MS5837_ANY_ERROR;
		m_eepromCoeffs[i] = (uint16_t)(buffer[0] << 8 | buffer[1]);
	}

	uint8_t prom_crc = (m_eepromCoeffs[MS5837_CRC_INDEX] & 0xF000) >> 12;
	m_eepromCoeffs[MS5837_CRC_INDEX] = m_eepromCoeffs[MS5837_CRC_INDEX] & 0x0FFF;
	m_eepromCoeffs[MS5837_SUBSIDIARY_VALUE_INDEX] = 0;

	if (CRC4(m_eepromCoeffs) != prom_crc)
		return MS5837_CRC_ERROR;

	return m_connectionStatus = MS5837_ALL_OK;
}

enum MS5837ErrorCode MS5837Init(void) {
	if (MS5837Reset())
		return MS5837_ANY_ERROR;
	//10ms like in python driver version
	HAL_Delay(10);

	MS5837ReadEEPROMCoeffs();
	return MS5837_ALL_OK;
}

enum MS5837ErrorCode MS5837IsConnected(void) {
	return m_connectionStatus;
}

enum MS5837ErrorCode MS5837Reset(void) {
	uint8_t command = MS5837_RESET_COMMAND;

	if (HAL_I2C_Master_Transmit(&hi2c1, MS5837_ADDR << 1 | MS5837_WRITE_MASK,
			&command, MS5837_COMMAND_SIZE, MS5837_TIMEOUT_MS))
		return MS5837_ANY_ERROR;

	return MS5837_ALL_OK;
}

enum MS5837ErrorCode MS5837ReadTemperature(enum MS5837D2OSRCommand osrD2, float *temperature) {
	uint8_t buffer[MS5837_ADC_DATA_SIZE];
	if (HAL_I2C_Master_Transmit(&hi2c1, MS5837_ADDR << 1 | MS5837_WRITE_MASK,
			&osrD2, MS5837_COMMAND_SIZE, MS5837_TIMEOUT_MS))
		return MS5837_ANY_ERROR;

	HAL_Delay(m_conversionTime[(osrD2 & MS5837_CONVERSION_OSR_MASK) / 2]);

	if (HAL_I2C_Mem_Read(&hi2c1, MS5837_ADDR << 1 | MS5837_READ_MASK,
			MS5837_READ_ADC_COMMAND, MS5837_COMMAND_SIZE,
			buffer, MS5837_ADC_DATA_SIZE, MS5837_TIMEOUT_MS))
		return MS5837_ANY_ERROR;

	uint32_t adc_temperature = buffer[0] << 16 | buffer[1] << 8 | buffer[2];

	if (adc_temperature == 0)
		return MS5837_I2C_TRANSFER_ERROR;

	int32_t dT = (int32_t)adc_temperature -
			((int32_t)m_eepromCoeffs[MS5837_REFERENCE_TEMPERATURE_INDEX] << 8);

	int32_t TEMP = 2000	+
			(int32_t)(dT * ((float)m_eepromCoeffs[MS5837_TEMP_COEFF_OF_TEMPERATURE_INDEX] / 8388608));

	int64_t T2 = (TEMP < 2000) ?
				 (float)(3 * (int64_t)dT  * (int64_t)dT) / 8589934592 :
				 (float)(2 * (int64_t)dT  * (int64_t)dT) / 137438953472;

	*temperature = ((float)TEMP  - T2) / 100;

	return MS5837_ALL_OK;
}


enum MS5837ErrorCode MS5837ReadTemperatureAndPressure(enum MS5837D1OSRCommand osrD1,
													  enum MS5837D2OSRCommand osrD2,
													  float *temperature,
													  float *pressure) {
	uint8_t buffer[MS5837_ADC_DATA_SIZE];
		if (HAL_I2C_Master_Transmit(&hi2c1, MS5837_ADDR << 1 | MS5837_WRITE_MASK,
				&osrD2, MS5837_COMMAND_SIZE, MS5837_TIMEOUT_MS))
			return MS5837_ANY_ERROR;

		HAL_Delay(m_conversionTime[(osrD2 & MS5837_CONVERSION_OSR_MASK) / 2]);

		if (HAL_I2C_Mem_Read(&hi2c1, MS5837_ADDR << 1 | MS5837_READ_MASK,
				MS5837_READ_ADC_COMMAND, MS5837_COMMAND_SIZE,
				buffer, MS5837_ADC_DATA_SIZE, MS5837_TIMEOUT_MS))
			return MS5837_ANY_ERROR;

		uint32_t adc_temperature = buffer[0] << 16 | buffer[1] << 8 | buffer[2];

		if (HAL_I2C_Master_Transmit(&hi2c1, MS5837_ADDR << 1 | MS5837_WRITE_MASK,
				&osrD1, MS5837_COMMAND_SIZE, MS5837_TIMEOUT_MS))
			return MS5837_ANY_ERROR;

		HAL_Delay(m_conversionTime[(osrD1 & MS5837_CONVERSION_OSR_MASK) / 2]);

		if (HAL_I2C_Mem_Read(&hi2c1, MS5837_ADDR << 1 | MS5837_READ_MASK,
				MS5837_READ_ADC_COMMAND, MS5837_COMMAND_SIZE,
				buffer, MS5837_ADC_DATA_SIZE, MS5837_TIMEOUT_MS))
			return MS5837_ANY_ERROR;

		uint32_t adc_pressure = buffer[0] << 16 | buffer[1] << 8 | buffer[2];

		if (adc_temperature == 0 || adc_pressure == 0)
			return MS5837_I2C_TRANSFER_ERROR;

		int32_t dT = (int32_t)adc_temperature - ((int32_t)m_eepromCoeffs[MS5837_REFERENCE_TEMPERATURE_INDEX] << 8);

		int32_t TEMP = 2000	+ (int32_t)(dT * ((float)m_eepromCoeffs[MS5837_TEMP_COEFF_OF_TEMPERATURE_INDEX] / 8388608));

		int64_t T2, SENS, OFF, OFF2, SENS2;

		if (TEMP < 2000)
		{
			T2 = (float)(3 * (int64_t)dT  * (int64_t)dT) / 8589934592;
			OFF2 = (float)(3 * ((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000)) / 2;
			SENS2 = (float)(5 * ((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000)) / 8;

			if (TEMP < -1500)
			{
				OFF2 += 7 * ((int64_t)TEMP + 1500) * ((int64_t)TEMP + 1500);
				SENS2 += 4 * ((int64_t)TEMP + 1500) * ((int64_t)TEMP + 1500);
			}
		}
		else
		{
			T2 = (float)(2 * (int64_t)dT  * (int64_t)dT) / 137438953472;
			OFF2 = (float)((int64_t)TEMP + 1500) * ((int64_t)TEMP + 1500) / 16;
			SENS2 = 0 ;
		}

		OFF = ((int64_t)m_eepromCoeffs[MS5837_PRESSURE_OFFSET_INDEX] << 16) +
				(((int64_t)m_eepromCoeffs[MS5837_TEMP_COEFF_OF_PRESSURE_OFFSET_INDEX] * dT) >> 7);
		OFF -= OFF2;

		SENS = ((int64_t)m_eepromCoeffs[MS5837_PRESSURE_SENSITIVITY_INDEX] << 15) +
				(((int64_t)m_eepromCoeffs[MS5837_TEMP_COEFF_OF_PRESSURE_SENSITIVITY_INDEX] * dT) >> 8);
		SENS -= SENS2;

		int32_t P = (((adc_pressure * SENS) >> 21) - OFF) >> 13;

		*temperature = (float)(TEMP  - T2) / 100;
		*pressure = (float)P / 10;

		return MS5837_ALL_OK;
}
