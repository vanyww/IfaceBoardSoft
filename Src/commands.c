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

struct BESCDriverHandle BESCDrivers[BESC_DRIVERS_MAX];
uint8_t BESCDriversCounter = 0;

struct BCSDriverHandle *TIM1BCS;
struct BCSDriverHandle BCSDrivers[BCS_DRIVERS_MAX];
uint8_t BCSDriversCounter = 0;

struct LEDDriverHandle LEDDrivers[LED_DRIVERS_MAX];
uint8_t LEDDriversCounter = 0;

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

void LoadDeviceConfig(void) {
	if (!ParamsUnion.Params.F_Devices.IsUsed)
		return;

	BESCDriversCounter = ParamsUnion.Params.F_Devices.BESCDriversCounter;
	BCSDriversCounter = ParamsUnion.Params.F_Devices.BCSDriversCounter;
	LEDDriversCounter = ParamsUnion.Params.F_Devices.LEDDriversCounter;

	memcpy(BESCDrivers, ParamsUnion.Params.F_Devices.BESCDrivers,
			sizeof(struct BESCDriverHandle) * BESC_DRIVERS_MAX);
	memcpy(BCSDrivers, ParamsUnion.Params.F_Devices.BCSDrivers,
			sizeof(struct BCSDriverHandle) * BCS_DRIVERS_MAX);
	memcpy(LEDDrivers, ParamsUnion.Params.F_Devices.LEDDrivers,
			sizeof(struct LEDDriverHandle) * LED_DRIVERS_MAX);
}

void ResetDeviceConfig(void) {
	StartChangeFlashParam();

	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
			(uint32_t) &ParamsUnion.Params.F_Devices.IsUsed, (uint16_t) 0x0);

	EndChangeFlashParam((void *) &ParamsUnion.Params.F_Devices.IsUsed,
			sizeof(ParamsUnion.Params.F_Devices.IsUsed));
}

//
uint8_t C_R_Ping(struct ModbusRecvMessage *msg, uint8_t *result,
		uint8_t *resultLength) {
	*resultLength = 0;

	return ALL_OK;
}
//

//
uint8_t C_W_ChangeSlaveId(struct ModbusRecvMessage *msg) {
	if (msg->DataLength != sizeof(uint8_t))
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

	return ALL_OK;
}
//

//
uint8_t C_W_SaveDeviceConfiguration(struct ModbusRecvMessage *msg) {
	if (msg->DataLength != 0)
		return ANY_ERROR;

	SaveDeviceConfig();

	return ALL_OK;
}
//

//
uint8_t C_W_ResetDeviceConfiguration(struct ModbusRecvMessage *msg) {
	if (msg->DataLength != 0)
		return ANY_ERROR;

	ResetDeviceConfig();

	return ALL_OK;
}
//

//
uint8_t C_W_LoadDeviceConfiguration(struct ModbusRecvMessage *msg) {
	if (msg->DataLength != 0)
		return ANY_ERROR;

	LoadDeviceConfig();

	return ALL_OK;
}
//

//
uint8_t C_R_InitializeBESCDevice(struct ModbusRecvMessage *msg, uint8_t *result,
		uint8_t *resultLength) {
	if (msg->DataLength != sizeof(enum TIMChannel))
		return ANY_ERROR;

	if (InitializeBESCDriver(*(enum TIMChannel *) msg->Data,
			&BESCDrivers[BESCDriversCounter]))
		return ANY_ERROR;

	*result = BESCDriversCounter++;
	*resultLength = sizeof(BESCDriversCounter);

	return ALL_OK;
}
//

//
uint8_t C_R_InitializeLEDDevice(struct ModbusRecvMessage *msg, uint8_t *result,
		uint8_t *resultLength) {
	if (msg->DataLength != sizeof(enum TIMChannel))
		return ANY_ERROR;

	if (InitializeLEDDriver(*(enum TIMChannel *) msg->Data,
			&LEDDrivers[LEDDriversCounter]))
		return ANY_ERROR;

	*result = LEDDriversCounter++;
	*resultLength = sizeof(LEDDriversCounter);

	return ALL_OK;
}
//

//
uint8_t C_R_InitializeBCSDevice(struct ModbusRecvMessage *msg, uint8_t *result,
		uint8_t *resultLength) {
	if (msg->DataLength
			!= sizeof(enum TIMChannel) + 3 * sizeof(enum GPIOChannel))
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

	*result = BCSDriversCounter++;
	*resultLength = sizeof(BCSDriversCounter);

	return ALL_OK;
}
//

//
uint8_t C_W_BESCChangeSpeed(struct ModbusRecvMessage *msg) {
	if (msg->DataLength != sizeof(uint8_t) + sizeof(double))
		return ANY_ERROR;

	uint8_t bescDriverId = msg->Data[0];

	if (bescDriverId >= BESCDriversCounter)
		return ANY_ERROR;

	if (BESCDriverChangeSpeed(&BESCDrivers[bescDriverId],
			&msg->Data[sizeof(uint8_t)]))
		return ANY_ERROR;

	return ALL_OK;
}
//

//
uint8_t C_W_LEDChangeBrightness(struct ModbusRecvMessage *msg) {
	if (msg->DataLength != sizeof(uint8_t) + sizeof(double))
		return ANY_ERROR;

	uint8_t ledDriverId = msg->Data[0];

	if (ledDriverId >= LEDDriversCounter)
		return ANY_ERROR;

	if (LEDDriverChangeBrightness(&LEDDrivers[ledDriverId],
			&msg->Data[sizeof(uint8_t)]))
		return ANY_ERROR;

	return ALL_OK;
}
//

//
uint8_t C_W_BCS_ChangeSpeed(struct ModbusRecvMessage *msg) {
	if (msg->DataLength != sizeof(uint8_t) + sizeof(double))
		return ANY_ERROR;

	uint8_t bcsDriverId = msg->Data[0];

	if (bcsDriverId >= BCSDriversCounter)
		return ANY_ERROR;

	if (BCSDriverChangeSpeed(&BCSDrivers[bcsDriverId],
			&msg->Data[sizeof(uint8_t)]))
		return ANY_ERROR;

	return ALL_OK;
}
//

//
uint8_t C_W_BCS_Move(struct ModbusRecvMessage *msg) {
	if (msg->DataLength != 2 * sizeof(uint8_t) + sizeof(uint16_t))
		return ANY_ERROR;

	uint8_t bcsDriverId = msg->Data[0];

	if (bcsDriverId >= BCSDriversCounter)
		return ANY_ERROR;

	if (BCSDriverMove(&BCSDrivers[bcsDriverId], &msg->Data[sizeof(uint8_t)]))
		return ANY_ERROR;

	return ALL_OK;
}
//

//
uint8_t C_W_BCS_Stop(struct ModbusRecvMessage *msg) {
	if (msg->DataLength != sizeof(uint8_t))
		return ANY_ERROR;

	uint8_t bcsDriverId = msg->Data[0];

	if (bcsDriverId >= BCSDriversCounter)
		return ANY_ERROR;

	if (BCSDriverStop(&BCSDrivers[bcsDriverId]))
		return ANY_ERROR;

	return ALL_OK;
}
//

//
uint8_t C_W_BCS_MoveToEnd(struct ModbusRecvMessage *msg) {
	if (msg->DataLength != 2 * sizeof(uint8_t))
		return ANY_ERROR;

	uint8_t bcsDriverId = msg->Data[0];

	if (bcsDriverId >= BCSDriversCounter)
		return ANY_ERROR;

	if (BCSDriverMoveToEnd(&BCSDrivers[bcsDriverId],
			&msg->Data[sizeof(uint8_t)]))
		return ANY_ERROR;

	return ALL_OK;
}
//

//
uint8_t C_R_MS5837_CheckConnection(struct ModbusRecvMessage *msg,
		uint8_t *result, uint8_t *resultLength) {
	enum MS5837ErrorCode status = MS5837IsConnected();
	*resultLength = sizeof(status);
	*result = status;

	return ALL_OK;
}

uint8_t C_R_MS5837_ReadTemp(struct ModbusRecvMessage *msg, uint8_t *result,
		uint8_t *resultLength) {
	if (msg->DataLength != sizeof(enum MS5837D2OSRCommand))
		return ANY_ERROR;

	enum MS5837D2OSRCommand d2osr = msg->Data[0];
	float temp;

	if (MS5837ReadTemperature(d2osr, &temp))
		return ANY_ERROR;

	*resultLength = sizeof(temp);
	memcpy(result, &temp, sizeof(float));

	return ALL_OK;
}

uint8_t C_R_MS5837_ReadTempAndPress(struct ModbusRecvMessage *msg,
		uint8_t *result, uint8_t *resultLength) {
	if (msg->DataLength
			!= sizeof(enum MS5837D2OSRCommand)
					+ sizeof(enum MS5837D1OSRCommand))
		return ANY_ERROR;

	enum MS5837D1OSRCommand d1osr = msg->Data[0], d2osr = msg->Data[1];
	float temp, press;

	if (MS5837ReadTemperatureAndPressure(d1osr, d2osr, &temp, &press))
		return ANY_ERROR;

	*resultLength = sizeof(temp) + sizeof(press);
	memcpy(result, &temp, sizeof(float));
	memcpy(result + sizeof(float), &press, sizeof(float));

	return ALL_OK;
}

uint8_t C_R_MS5837_ReadPress(struct ModbusRecvMessage *msg, uint8_t *result,
		uint8_t *resultLength) {
	if (msg->DataLength
			!= sizeof(enum MS5837D2OSRCommand)
					+ sizeof(enum MS5837D1OSRCommand))
		return ANY_ERROR;

	enum MS5837D1OSRCommand d1osr = msg->Data[0], d2osr = msg->Data[1];
	float press, temp;

	if (MS5837ReadTemperatureAndPressure(d1osr, d2osr, &temp, &press))
		return ANY_ERROR;

	*resultLength = sizeof(press);
	memcpy(result, &press, sizeof(float));

	return ALL_OK;
}
//

void InitializeCommands() {
	InitializeModbus();
	InitializeDriversWorker();
	MS5837Init();

	AddCommand(0x00, &C_R_Ping, NULL);
	AddCommand(0x01, NULL, &C_W_ChangeSlaveId);
	AddCommand(0x02, NULL, &C_W_SaveDeviceConfiguration);
	AddCommand(0x03, NULL, &C_W_ResetDeviceConfiguration);

	AddCommand(0x10, &C_R_InitializeBESCDevice, NULL);
	AddCommand(0x11, NULL, &C_W_BESCChangeSpeed);

	AddCommand(0x20, &C_R_InitializeLEDDevice, NULL);
	AddCommand(0x21, NULL, &C_W_LEDChangeBrightness);

	AddCommand(0x30, &C_R_InitializeBCSDevice, NULL);
	AddCommand(0x31, NULL, &C_W_BCS_ChangeSpeed);
	AddCommand(0x32, NULL, &C_W_BCS_Move);
	AddCommand(0x33, NULL, &C_W_BCS_MoveToEnd);
	AddCommand(0x34, NULL, &C_W_BCS_Stop);

	AddCommand(0x40, &C_R_MS5837_CheckConnection, NULL);
	AddCommand(0x41, &C_R_MS5837_ReadTemp, NULL);
	AddCommand(0x42, &C_R_MS5837_ReadTempAndPress, NULL);
	AddCommand(0x43, &C_R_MS5837_ReadPress, NULL);
}
