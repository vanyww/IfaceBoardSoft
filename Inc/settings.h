/*
 * settings.h
 *
 *  Created on: 6 апр. 2018 г.
 *      Author: Satty
 */

#ifndef SETTINGS_H_
#define SETTINGS_H_

#include "stdint.h"
#include "stm32f0xx.h"
#include "stm32f0xx_hal_flash_ex.h"

#define USER_DATA_FIRST_PAGE 0x800F800

struct ParamsStruct {
	const uint64_t F_BaudRate;
	const uint16_t F_SlaveId;
};

extern const union _ParamsUnion{
	const struct ParamsStruct Params;
	const uint16_t Bytes[sizeof(struct ParamsStruct) / sizeof(uint16_t)];
} ParamsUnion __attribute__((__section__(".user_data")));

extern const struct ParamsStruct *PParams;

void StartChangeFlashParam();
void EndChangeFlashParam(void *changedParamAddress, uint32_t paramSize);

#endif /* SETTINGS_H_ */
