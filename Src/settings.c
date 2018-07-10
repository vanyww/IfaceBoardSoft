/*
 * settings.h
 *
 *  Created on: 8 апр. 2018 г.
 *      Author: Vanyw
 */

#include "settings.h"
#include "string.h"

#define MAX16BIT 0xFFFF

const union _ParamsUnion ParamsUnion;
const struct ParamsStruct *PParams = &ParamsUnion.Params;

uint16_t m_paramsBuffer[sizeof(struct ParamsStruct) / sizeof(uint16_t)];

uint32_t m_pageError = 0;
FLASH_EraseInitTypeDef m_eraseInitStruct = {.TypeErase = FLASH_TYPEERASE_PAGES,
											.PageAddress = USER_DATA_FIRST_PAGE,
											.NbPages = 1};

void BufferParams();
void UnbufferParams(uint32_t changedAddress, uint32_t changedSize);

void BufferParams() {
	memcpy(m_paramsBuffer, ParamsUnion.Bytes, sizeof(struct ParamsStruct));
}

void UnbufferParams(uint32_t changedAddress, uint32_t changedSize) {
	uint32_t currentAddress = USER_DATA_FIRST_PAGE;

	for(uint8_t i = 0; i < sizeof(struct ParamsStruct) / sizeof(uint16_t); i++, currentAddress += sizeof(uint16_t)) {
		if(currentAddress == (uint32_t)changedAddress) {
			currentAddress += changedSize;
			i += changedSize / sizeof(uint16_t) - 1;
			continue;
		}

		if(m_paramsBuffer[i] == MAX16BIT)
			return;

		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, currentAddress, m_paramsBuffer[i]);
	}
}

void StartChangeFlashParam() {
	BufferParams();
	HAL_FLASH_Unlock();

	HAL_FLASHEx_Erase(&m_eraseInitStruct, &m_pageError);
}

void EndChangeFlashParam(void *changedParamAddress, uint32_t changedSize) {
	UnbufferParams((uint32_t)changedParamAddress, changedSize);
	HAL_FLASH_Lock();
}
