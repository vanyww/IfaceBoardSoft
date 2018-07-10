/*
 * MS5837-30BADriver.h
 *
 *  Created on: 27 мая 2018 г.
 *      Author: Satty
 */

#ifndef MS5837_30BADRIVER_H_
#define MS5837_30BADRIVER_H_

#include "defs.h"

enum MS5837D1OSRCommand {
	MS5837_D1_OSR_256 = 0x40,
	MS5837_D1_OSR_512 = 0x42,
	MS5837_D1_OSR_1024 = 0x44,
	MS5837_D1_OSR_2048 = 0x46,
	MS5837_D1_OSR_4096 = 0x48,
	MS5837_D1_OSR_8192 = 0x4A
};

enum MS5837D2OSRCommand {
	MS5837_D2_OSR_256 = 0x50,
	MS5837_D2_OSR_512 = 0x52,
	MS5837_D2_OSR_1024 = 0x54,
	MS5837_D2_OSR_2048 = 0x56,
	MS5837_D2_OSR_4096 = 0x58,
	MS5837_D2_OSR_8192 = 0x5A
};

enum MS5837ErrorCode {
	MS5837_ALL_OK = ALL_OK,

	MS5837_I2C_TRANSFER_ERROR,
	MS5837_CRC_ERROR,

	MS5837_ANY_ERROR = ANY_ERROR
};

enum MS5837ErrorCode MS5837Init(void);
enum MS5837ErrorCode MS5837Reset(void);
enum MS5837ErrorCode MS5837IsConnected(void);
enum MS5837ErrorCode MS5837ReadTemperature(enum MS5837D2OSRCommand osr, float *temperature);
enum MS5837ErrorCode MS5837ReadTemperatureAndPressure(enum MS5837D1OSRCommand osrD1,
													  enum MS5837D2OSRCommand osrD2,
													  float *temperature,
													  float *pressure);

#endif /* PERIPHERALS_INC_MS5837_30BA_H_ */
