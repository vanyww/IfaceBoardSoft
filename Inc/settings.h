/*
 * settings.h
 *
 *  Created on: 6 ���. 2018 �.
 *      Author: Satty
 */

#ifndef SETTINGS_H_
#define SETTINGS_H_

#include "stdint.h"
#include "stm32f0xx.h"
#include "peripheral_drivers.h"
#include "stm32f0xx_hal_flash_ex.h"

#define USER_DATA_FIRST_PAGE 0x800F800

struct DevicesStruct {
	uint16_t IsUsed;

	struct BESCDriverHandle BESCDrivers[BESC_DRIVERS_MAX];
	uint16_t BESCDriversCounter;

	struct BCSDriverHandle BCSDrivers[BCS_DRIVERS_MAX];
	uint16_t BCSDriversCounter;

	struct LEDDriverHandle LEDDrivers[LED_DRIVERS_MAX];
	uint16_t LEDDriversCounter;
};

struct ParamsStruct {
	const uint64_t F_BaudRate;
	const uint16_t F_SlaveId;
	const struct DevicesStruct F_Devices;
};

extern const union _ParamsUnion{
	const struct ParamsStruct Params;
	const uint16_t Bytes[sizeof(struct ParamsStruct) / sizeof(uint16_t)];
} ParamsUnion __attribute__((__section__(".user_data")));

void StartChangeFlashParam();
void EndChangeFlashParam(void *changedParamAddress, uint32_t paramSize);
void LoadDeviceConfig(void);

#endif /* SETTINGS_H_ */
