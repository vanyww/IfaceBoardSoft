/*
 * commands.c
 *
 *  Created on: 6 апр. 2018 г.
 *      Author: Satty
 */

#include "MS5837-30BADriver.h"
#include "commands.h"
#include "string.h"
#include "settings.h"
#include "usart.h"

#define RESULT_BUFFER_LENGTH 10
#define ADD_2_2(value) ((value) + (value) % 2)

struct BESCDriverHandle BESCDrivers[BESC_DRIVERS_MAX];
uint8_t BESCDriversCounter = 0;

struct BCSDriverHandle *TIM1BCS;
struct BCSDriverHandle BCSDrivers[BCS_DRIVERS_MAX];
uint8_t BCSDriversCounter = 0;

struct LEDDriverHandle LEDDrivers[LED_DRIVERS_MAX];
uint8_t LEDDriversCounter = 0;

static uint8_t ResultBuffer[RESULT_BUFFER_LENGTH] = {0};

void SaveDeviceConfig(void) {
	StartChangeFlashParam();

	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
			(uint32_t) &ParamsUnion.Params.F_Devices.IsUsed, (uint16_t) 0x1);

	for (uint8_t i = 0;
			i < BESC_DRIVERS_MAX * sizeof(struct BESCDriverHandle) / 2
							+ BESC_DRIVERS_MAX * sizeof(struct BESCDriverHandle)
									% 2; i++)
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
				(uint32_t) &ParamsUnion.Params.F_Devices.BESCDrivers
						+ i * sizeof(uint16_t), ((uint16_t *) &BESCDrivers)[i]);

	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
			(uint32_t) &ParamsUnion.Params.F_Devices.BESCDriversCounter,
			(uint16_t) BESCDriversCounter);

	for (uint8_t i = 0;
			i < BCS_DRIVERS_MAX * sizeof(struct BCSDriverHandle) / 2
							+ BCS_DRIVERS_MAX * sizeof(struct BCSDriverHandle)
									% 2; i++)
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
				(uint32_t) &ParamsUnion.Params.F_Devices.BCSDrivers
						+ i * sizeof(uint16_t), ((uint16_t *) &BCSDrivers)[i]);

	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
			(uint32_t) &ParamsUnion.Params.F_Devices.BCSDriversCounter,
			(uint16_t) BCSDriversCounter);

	for (uint8_t i = 0;
			i < LED_DRIVERS_MAX * sizeof(struct LEDDriverHandle) / 2
							+ LED_DRIVERS_MAX * sizeof(struct LEDDriverHandle)
									% 2; i++)
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
				(uint32_t) &ParamsUnion.Params.F_Devices.LEDDrivers
						+ i * sizeof(uint16_t), ((uint16_t *) &LEDDrivers)[i]);

	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
			(uint32_t) &ParamsUnion.Params.F_Devices.LEDDriversCounter,
			(uint16_t) LEDDriversCounter);

	EndChangeFlashParam((void *) &ParamsUnion.Params.F_Devices,
			sizeof(ParamsUnion.Params.F_Devices));
}

uint8_t LoadDeviceConfig(void) {
	if (!ParamsUnion.Params.F_Devices.IsUsed)
		return ANY_ERROR;

	BESCDriversCounter = ParamsUnion.Params.F_Devices.BESCDriversCounter;
	BCSDriversCounter = ParamsUnion.Params.F_Devices.BCSDriversCounter;
	LEDDriversCounter = ParamsUnion.Params.F_Devices.LEDDriversCounter;

	volmemcpy(BESCDrivers, ParamsUnion.Params.F_Devices.BESCDrivers,
			sizeof(struct BESCDriverHandle) * BESC_DRIVERS_MAX);
	volmemcpy(BCSDrivers, ParamsUnion.Params.F_Devices.BCSDrivers,
			sizeof(struct BCSDriverHandle) * BCS_DRIVERS_MAX);
	volmemcpy(LEDDrivers, ParamsUnion.Params.F_Devices.LEDDrivers,
			sizeof(struct LEDDriverHandle) * LED_DRIVERS_MAX);

	return ALL_OK;
}

void ResetDeviceConfig(void) {
	StartChangeFlashParam();

	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
			(uint32_t) &ParamsUnion.Params.F_Devices.IsUsed, (uint16_t) 0x0);

	EndChangeFlashParam((void *) &ParamsUnion.Params.F_Devices.IsUsed,
			sizeof(ParamsUnion.Params.F_Devices.IsUsed));
}

void SetDeviceConfiguration(void) {
	LoadDeviceConfig();

	for(uint8_t i = 0; i < BESCDriversCounter; i++)
		ReinitializeBESCDriver(&BESCDrivers[i]);

	for(uint8_t i = 0; i < BCSDriversCounter; i++)
		ReinitializeBCSDriver(&BCSDrivers[i]);

	for(uint8_t i = 0; i < LEDDriversCounter; i++)
		ReinitializeLEDDriver(&LEDDrivers[i]);
}

void LoadUARTBaud(void) {
	uint32_t flashUARTBaud;

	flashUARTBaud = ParamsUnion.Params.F_USART1BaudRate;
	if(flashUARTBaud != 0) {
		huart1.Init.BaudRate = flashUARTBaud;
		HAL_UART_Init(&huart1);
	}

	flashUARTBaud = ParamsUnion.Params.F_USART2BaudRate;
	if(flashUARTBaud != 0) {
		huart2.Init.BaudRate = flashUARTBaud;
		HAL_UART_Init(&huart2);
	}
}

//
uint8_t C_R_Ping(struct ModbusRecvMessage *msg, uint8_t *result,
		uint8_t *resultLength) {

	*resultLength = 0;
	return ALL_OK;
}
//

//
uint8_t C_W_ChangeBaudRate(struct ModbusRecvMessage *msg, uint16_t *writtenDataLength) {
	if (msg->DataLength != ADD_2_2(sizeof(uint8_t) + sizeof(uint32_t)))
		return ANY_ERROR;

	uint8_t usartId = msg->Data[0];
	uint32_t newBaudRate;

	memcpy(&newBaudRate, &msg->Data[sizeof(usartId)], sizeof(newBaudRate));

	StartChangeFlashParam();

	switch(usartId) {
	case 1:
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
				(uint32_t) &ParamsUnion.Params.F_USART1BaudRate, newBaudRate);
		EndChangeFlashParam((void *) &ParamsUnion.Params.F_USART1BaudRate,
				sizeof(ParamsUnion.Params.F_USART1BaudRate));

		huart1.Init.BaudRate = newBaudRate;
		HAL_UART_Init(&huart1);
		break;

	case 2:
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
				(uint32_t) &ParamsUnion.Params.F_USART2BaudRate, newBaudRate);
		EndChangeFlashParam((void *) &ParamsUnion.Params.F_USART2BaudRate,
				sizeof(ParamsUnion.Params.F_USART2BaudRate));


		huart2.Init.BaudRate =  newBaudRate;
		HAL_UART_Init(&huart2);
		break;
	}

	*writtenDataLength = ADD_2_2(sizeof(uint8_t) + sizeof(uint32_t));

	return ALL_OK;
}
//

//
uint8_t C_W_Reset(struct ModbusRecvMessage *msg, uint16_t *writtenDataLength) {
	if (msg->DataLength != 0)
		return ANY_ERROR;

	NVIC_SystemReset();

	*writtenDataLength = 0;

	return ALL_OK;
}
//

//
uint8_t C_W_ChangeSlaveId(struct ModbusRecvMessage *msg, uint16_t *writtenDataLength) {
	if (msg->DataLength != ADD_2_2(sizeof(uint8_t)))
		return ANY_ERROR;

	uint8_t newId = msg->Data[0];

	if (!newId)
		return ANY_ERROR;

	StartChangeFlashParam();

	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
			(uint32_t) &ParamsUnion.Params.F_SlaveId, (uint16_t) newId);

	EndChangeFlashParam((void *) &ParamsUnion.Params.F_SlaveId,
			sizeof(ParamsUnion.Params.F_SlaveId));

	SlaveId = newId;
	*writtenDataLength = ADD_2_2(sizeof(uint8_t));

	return ALL_OK;
}
//

//
uint8_t C_W_SaveDeviceConfiguration(struct ModbusRecvMessage *msg, uint16_t *writtenDataLength) {
	if (msg->DataLength != 0)
		return ANY_ERROR;

	SaveDeviceConfig();

	*writtenDataLength = 0;

	return ALL_OK;
}
//

//
uint8_t C_W_ResetDeviceConfiguration(struct ModbusRecvMessage *msg, uint16_t *writtenDataLength) {
	if (msg->DataLength != 0)
		return ANY_ERROR;

	ResetDeviceConfig();

	*writtenDataLength = 0;

	return ALL_OK;
}
//

//
uint8_t C_W_InitializeBESCDevice(struct ModbusRecvMessage *msg, uint16_t *writtenDataLength) {
	if (msg->DataLength != ADD_2_2(sizeof(enum TIMChannel)))
		return ANY_ERROR;

	if (InitializeBESCDriver((enum TIMChannel)msg->Data[0],
			&BESCDrivers[BESCDriversCounter]))
		return ANY_ERROR;


	ResultBuffer[0] = BESCDriversCounter++;

	*writtenDataLength = ADD_2_2(sizeof(enum TIMChannel));

	return ALL_OK;
}
//

//
uint8_t C_W_InitializeLEDDevice(struct ModbusRecvMessage *msg, uint16_t *writtenDataLength) {
	if (msg->DataLength != ADD_2_2(sizeof(enum TIMChannel)))
		return ANY_ERROR;

	if (InitializeLEDDriver((enum TIMChannel) msg->Data[0],
			&LEDDrivers[LEDDriversCounter]))
		return ANY_ERROR;

	ResultBuffer[0] = LEDDriversCounter++;

	*writtenDataLength = ADD_2_2(sizeof(enum TIMChannel));

	return ALL_OK;
}
//

//
uint8_t C_W_InitializeBCSDevice(struct ModbusRecvMessage *msg, uint16_t *writtenDataLength) {
	if (msg->DataLength
			!= ADD_2_2(sizeof(enum TIMChannel) + 3 * sizeof(enum GPIOChannel)))
		return ANY_ERROR;

	uint8_t *data = msg->Data;

	enum TIMChannel timChannel = (enum TIMChannel) data[0];
	enum GPIOChannel directionChannel = (enum GPIOChannel) data[1];
	enum GPIOChannel disableChannel = (enum GPIOChannel) data[2];
	enum GPIOChannel terminalChannel = (enum GPIOChannel) data[3];

	if (directionChannel == disableChannel || disableChannel == terminalChannel)
		return ANY_ERROR;

	struct BCSDriverHandle *handle = &BCSDrivers[BCSDriversCounter];
	if (InitializeBCSDriver(timChannel, directionChannel, disableChannel,
			terminalChannel, handle))
		return ANY_ERROR;

	if (handle->PWMTIM->ComplementTIM->Instance == TIM1)
		TIM1BCS = handle;

	ResultBuffer[0] = BCSDriversCounter++;

	*writtenDataLength = ADD_2_2(sizeof(enum TIMChannel) + 3 * sizeof(enum GPIOChannel));

	return ALL_OK;
}
//

//
uint8_t C_W_BESCChangeSpeed(struct ModbusRecvMessage *msg, uint16_t *writtenDataLength) {
	if (msg->DataLength != ADD_2_2(sizeof(uint8_t) + sizeof(uint8_t)))
		return ANY_ERROR;

	uint8_t bescDriverId = msg->Data[0];

	if (bescDriverId >= BESCDriversCounter)
		return ANY_ERROR;

	if (BESCDriverChangeSpeed(&BESCDrivers[bescDriverId],
			&msg->Data[sizeof(bescDriverId)]))
		return ANY_ERROR;

	*writtenDataLength = ADD_2_2(sizeof(uint8_t) + sizeof(uint8_t));

	return ALL_OK;
}
//

//
uint8_t C_W_LEDChangeBrightness(struct ModbusRecvMessage *msg, uint16_t *writtenDataLength) {
	if (msg->DataLength != ADD_2_2(sizeof(uint8_t) + sizeof(uint8_t)))
		return ANY_ERROR;

	uint8_t ledDriverId = msg->Data[0];

	if (ledDriverId >= LEDDriversCounter)
		return ANY_ERROR;

	if (LEDDriverChangeBrightness(&LEDDrivers[ledDriverId],
			&msg->Data[sizeof(ledDriverId)]))
		return ANY_ERROR;

	*writtenDataLength = ADD_2_2(sizeof(uint8_t) + sizeof(uint8_t));

	return ALL_OK;
}
//

//
uint8_t C_W_BCS_ChangeSpeed(struct ModbusRecvMessage *msg, uint16_t *writtenDataLength) {
	if (msg->DataLength != ADD_2_2(sizeof(uint8_t) + sizeof(uint8_t)))
		return ANY_ERROR;

	uint8_t bcsDriverId = msg->Data[0];

	if (bcsDriverId >= BCSDriversCounter)
		return ANY_ERROR;

	if (BCSDriverChangeSpeed(&BCSDrivers[bcsDriverId],
			&msg->Data[sizeof(bcsDriverId)]))
		return ANY_ERROR;

	*writtenDataLength = ADD_2_2(sizeof(uint8_t) + sizeof(uint8_t));

	return ALL_OK;
}
//

//
uint8_t C_W_BCS_Move(struct ModbusRecvMessage *msg, uint16_t *writtenDataLength) {
	if (msg->DataLength != ADD_2_2(2 * sizeof(uint8_t) + sizeof(uint16_t)))
		return ANY_ERROR;

	uint8_t bcsDriverId = msg->Data[0];

	if (bcsDriverId >= BCSDriversCounter)
		return ANY_ERROR;

	if (BCSDriverMove(&BCSDrivers[bcsDriverId], &msg->Data[sizeof(bcsDriverId)]))
		return ANY_ERROR;

	*writtenDataLength = ADD_2_2(2 * sizeof(uint8_t) + sizeof(uint16_t));

	return ALL_OK;
}
//

//
uint8_t C_W_BCS_Stop(struct ModbusRecvMessage *msg, uint16_t *writtenDataLength) {
	if (msg->DataLength != ADD_2_2(sizeof(uint8_t)))
		return ANY_ERROR;

	uint8_t bcsDriverId = msg->Data[0];

	if (bcsDriverId >= BCSDriversCounter)
		return ANY_ERROR;

	if (BCSDriverStop(&BCSDrivers[bcsDriverId]))
		return ANY_ERROR;

	*writtenDataLength = ADD_2_2(sizeof(uint8_t));

	return ALL_OK;
}
//

//
uint8_t C_W_BCS_MoveToEnd(struct ModbusRecvMessage *msg, uint16_t *writtenDataLength) {
	if (msg->DataLength != ADD_2_2(2 * sizeof(uint8_t)))
		return ANY_ERROR;

	uint8_t bcsDriverId = msg->Data[0];

	if (bcsDriverId >= BCSDriversCounter)
		return ANY_ERROR;

	if (BCSDriverMoveToEnd(&BCSDrivers[bcsDriverId],
			&msg->Data[sizeof(bcsDriverId)]))
		return ANY_ERROR;

	*writtenDataLength = ADD_2_2(2 * sizeof(uint8_t));

	return ALL_OK;
}
//

//
uint8_t C_R_MS5837_CheckConnection(struct ModbusRecvMessage *msg, uint8_t *result,
		uint8_t *resultLength) {
	enum MS5837ErrorCode status = MS5837IsConnected();

	result[0] = 0;
	result[1] = status;

	*resultLength = sizeof(uint16_t);

	return ALL_OK;
}

uint8_t C_W_MS5837_ReadTemp(struct ModbusRecvMessage *msg, uint16_t *writtenDataLength) {
	if (msg->DataLength != ADD_2_2(sizeof(enum MS5837D2OSRCommand)))
		return ANY_ERROR;

	enum MS5837D2OSRCommand d2osr = msg->Data[0];
	float temp;

	if (MS5837ReadTemperature(d2osr, &temp))
		return ANY_ERROR;

	memcpy(ResultBuffer, &temp, sizeof(temp));

	*writtenDataLength = ADD_2_2(sizeof(enum MS5837D2OSRCommand));

	return ALL_OK;
}

uint8_t C_W_MS5837_ReadTempAndPress(struct ModbusRecvMessage *msg, uint16_t *writtenDataLength) {
	if (msg->DataLength
			!= ADD_2_2(sizeof(enum MS5837D2OSRCommand)
					+ sizeof(enum MS5837D1OSRCommand)))
		return ANY_ERROR;

	enum MS5837D1OSRCommand d1osr = msg->Data[0], d2osr = msg->Data[1];
	float temp, press;

	if (MS5837ReadTemperatureAndPressure(d1osr, d2osr, &temp, &press))
		return ANY_ERROR;

	memcpy(ResultBuffer, &temp, sizeof(float));
	memcpy(ResultBuffer + sizeof(float), &press, sizeof(float));

	*writtenDataLength = ADD_2_2(sizeof(enum MS5837D2OSRCommand)
				+ sizeof(enum MS5837D1OSRCommand));

	return ALL_OK;
}

uint8_t C_W_MS5837_ReadPress(struct ModbusRecvMessage *msg, uint16_t *writtenDataLength) {
	if (msg->DataLength
			!= ADD_2_2(sizeof(enum MS5837D2OSRCommand)
					+ sizeof(enum MS5837D1OSRCommand)))
		return ANY_ERROR;

	enum MS5837D1OSRCommand d1osr = msg->Data[0], d2osr = msg->Data[1];
	float press, temp;

	if (MS5837ReadTemperatureAndPressure(d1osr, d2osr, &temp, &press))
		return ANY_ERROR;

	memcpy(ResultBuffer, &press, sizeof(float));

	*writtenDataLength = ADD_2_2(sizeof(enum MS5837D2OSRCommand)
			+ sizeof(enum MS5837D1OSRCommand));

	return ALL_OK;
}
//

//
uint8_t C_R_ReadResultBuffer(struct ModbusRecvMessage *msg, uint8_t *result,
		uint8_t *resultLength) {
	if(msg->RegistersCount > RESULT_BUFFER_LENGTH)
		return ANY_ERROR;

	uint8_t readSize = msg->RegistersCount * sizeof(uint16_t);
	memcpy(result, ResultBuffer, readSize);
	*resultLength = readSize;

	return ALL_OK;
}
//

void InitializeCommands() {
	InitializeModbus();
    InitializeDriversWorker();
	MS5837Init();
	InitializeFlash();

	//System
	AddCommand(0x00, &C_R_Ping, NULL);
	AddCommand(0x01, NULL, &C_W_ChangeSlaveId);
	AddCommand(0x02, NULL, &C_W_SaveDeviceConfiguration);
	AddCommand(0x03, NULL, &C_W_ResetDeviceConfiguration);
	AddCommand(0x04, NULL, &C_W_Reset);
	AddCommand(0x05, NULL, &C_W_ChangeBaudRate);
	AddCommand(0x06, &C_R_ReadResultBuffer, NULL);

	//BESC
	AddCommand(0x10, NULL, &C_W_InitializeBESCDevice);
	AddCommand(0x11, NULL, &C_W_BESCChangeSpeed);

	//LED
	AddCommand(0x20, NULL, &C_W_InitializeLEDDevice);
	AddCommand(0x21, NULL, &C_W_LEDChangeBrightness);

	//BCS
	AddCommand(0x30, NULL, &C_W_InitializeBCSDevice);
	AddCommand(0x31, NULL, &C_W_BCS_ChangeSpeed);
	AddCommand(0x32, NULL, &C_W_BCS_Move);
	AddCommand(0x33, NULL, &C_W_BCS_MoveToEnd);
	AddCommand(0x34, NULL, &C_W_BCS_Stop);

	//MS5837
	AddCommand(0x40, &C_R_MS5837_CheckConnection, NULL);
	AddCommand(0x41, NULL, &C_W_MS5837_ReadTemp);
	AddCommand(0x42, NULL, &C_W_MS5837_ReadTempAndPress);
	AddCommand(0x43, NULL, &C_W_MS5837_ReadPress);
}
