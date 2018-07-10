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

struct BESCDriverHandle BESCDrivers[4];
uint8_t BESCDriversCounter = 0;

struct BCSDriverHandle *TIM1BCS;
struct BCSDriverHandle BCSDrivers[1];
uint8_t BCSDriversCounter = 0;

struct LEDDriverHandle LEDDrivers[4];
uint8_t LEDDriversCounter = 0;

//
uint8_t C_R_Ping(struct ModbusRecvMessage *msg, uint8_t *result,
		uint8_t *resultLength) {
	*resultLength = 0;

	return ALL_OK;
}
//

//
uint8_t C_W_ChangeSlaveId(struct ModbusRecvMessage *msg) {
	if(msg->DataLength != sizeof(uint8_t))
		return ANY_ERROR;

	uint8_t newId = msg->Data[0];

	if(newId == 0)
		return ANY_ERROR;

	StartChangeFlashParam();

	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (uint32_t)&ParamsUnion.Params.F_SlaveId, (uint16_t)newId);

	EndChangeFlashParam((void *)&ParamsUnion.Params.F_SlaveId, sizeof(uint16_t));

	SlaveId = newId;

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
	*resultLength = 1;

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
	*resultLength = 1;

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

	//
	if (handle->PWMTIM->ComplementTIM->Instance == TIM1)
		TIM1BCS = handle;
	//

	*result = BCSDriversCounter++;
	*resultLength = 1;

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
			&(msg->Data[sizeof(uint8_t)])))
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
			&(msg->Data[sizeof(uint8_t)])))
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
			&(msg->Data[sizeof(uint8_t)])))
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

	if (BCSDriverMove(&BCSDrivers[bcsDriverId], &(msg->Data[sizeof(uint8_t)])))
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
			&(msg->Data[sizeof(uint8_t)])))
		return ANY_ERROR;

	return ALL_OK;
}
//

//
uint8_t C_R_MS5837_CheckConnection(struct ModbusRecvMessage *msg,
		uint8_t *result, uint8_t *resultLength) {
	*resultLength = 1;
	*result = MS5837IsConnected();

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

	*resultLength = sizeof(float);
	memcpy(result, &temp, sizeof(float));

	return ALL_OK;
}

uint8_t C_R_MS5837_ReadTempAndPress(struct ModbusRecvMessage *msg,
		uint8_t *result, uint8_t *resultLength) {
	if (msg->DataLength
			!= sizeof(enum MS5837D2OSRCommand)
					+ sizeof(enum MS5837D1OSRCommand))
		return ANY_ERROR;

	enum MS5837D1OSRCommand d1osr = msg->Data[0];
	enum MS5837D2OSRCommand d2osr = msg->Data[1];
	float temp;
	float press;

	if (MS5837ReadTemperatureAndPressure(d1osr, d2osr, &temp, &press))
		return ANY_ERROR;

	*resultLength = 2*sizeof(float);
	memcpy(result, &temp, sizeof(float));
	memcpy(result + sizeof(float), &press, sizeof(float));

	return ALL_OK;
}
//

void InitializeCommands() {
	InitializeModbus();
	InitializeDriversWorker();
	MS5837Init();

	AddCommand(0x00, &C_R_Ping, NULL);
	AddCommand(0x01, NULL, &C_W_ChangeSlaveId);

	AddCommand(0x10, &C_R_InitializeBESCDevice, NULL);
	AddCommand(0x11, NULL, &C_W_BESCChangeSpeed);

	AddCommand(0x20, &C_R_InitializeLEDDevice, NULL);
	AddCommand(0x21, NULL, &C_W_LEDChangeBrightness);

	AddCommand(0x30, &C_R_InitializeBCSDevice, NULL);
	AddCommand(0x31, NULL, &C_W_BCS_ChangeSpeed);
	AddCommand(0x32, NULL, &C_W_BCS_Move);
	AddCommand(0x33, NULL, &C_W_BCS_MoveToEnd);

	AddCommand(0x40, &C_R_MS5837_CheckConnection, NULL);
	AddCommand(0x41, &C_R_MS5837_ReadTemp, NULL);
	AddCommand(0x42, &C_R_MS5837_ReadTempAndPress, NULL);
}
