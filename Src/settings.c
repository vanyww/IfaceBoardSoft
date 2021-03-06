/*
 * settings.h
 *
 *  Created on: 8 ���. 2018 �.
 *      Author: Vanyw
 */

#include "settings.h"
#include "string.h"

#define MAX16BIT 0xFFFF

volatile const union _ParamsUnion ParamsUnion;

uint16_t m_paramsBuffer[sizeof(struct ParamsStruct) / sizeof(uint16_t) + 1];

uint32_t m_pageError = 0;
FLASH_EraseInitTypeDef m_eraseInitStruct = { .TypeErase = FLASH_TYPEERASE_PAGES,
		.PageAddress = USER_DATA_PAGE, .NbPages = 1 };

void BufferParams();
void UnbufferParams(uint32_t changedAddress, uint32_t changedSize);

void BufferParams() {
	volmemcpy(m_paramsBuffer, ParamsUnion.Bytes, sizeof(struct ParamsStruct));
}

void UnbufferParams(uint32_t changedAddress, uint32_t changedSize) {
	uint32_t currentAddress = (uint32_t)&ParamsUnion;

	for (uint8_t i = 0;
			i < sizeof(struct ParamsStruct) / sizeof(uint16_t)
							+ sizeof(struct ParamsStruct) % sizeof(uint16_t);
			i++, currentAddress += sizeof(uint16_t)) {
		if (currentAddress == (uint32_t) changedAddress) {
			currentAddress += changedSize - sizeof(uint16_t);
			i += changedSize / sizeof(uint16_t) - 1;
			continue;
		}

		if (m_paramsBuffer[i] == MAX16BIT)
			continue;

		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, currentAddress,
				m_paramsBuffer[i]);
	}
}

void StartChangeFlashParam() {
	BufferParams();
	HAL_FLASH_Unlock();

	HAL_FLASHEx_Erase(&m_eraseInitStruct, &m_pageError);
}

void EndChangeFlashParam(void *changedParamAddress, uint32_t changedSize) {
	UnbufferParams((uint32_t) changedParamAddress, changedSize);
	HAL_FLASH_Lock();
}

void InitializeFlash(void) {
	  HAL_FLASH_Unlock();
	  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
	  HAL_FLASH_Lock();
}

