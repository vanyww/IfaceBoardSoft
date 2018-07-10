/*
 * modbus.c
 *
 *  Created on: 14 мар. 2018 г.
 *      Author: Satty
 */

#include "crc16.h"
#include "modbus.h"
#include "settings.h"

#define FUNCTIONS_NUMBER 2

//Request data indexes
#define RQ_SLAVE_ID_INDEX 0
#define RQ_FUNCTION_INDEX 1
#define RQ_COMMAND_INDEX 2
#define RQ_DATA_INDEX 4
#define RQ_CRC_INDEX -2

//Reply data indexes
#define RP_SLAVE_ID_INDEX 0
#define RP_FUNCTION_INDEX 1

//Reply [WRITE] data indexes
#define RP_W_COMMAND_INDEX 2
#define RP_W_DATA_INDEX 4

//Reply [ERROR] data indexes
#define RP_E_ERROR_CODE_INDEX 2

#define CRC_LENGTH 2
#define SLAVE_ID_LENGTH 1
#define FUNCTION_LENGTH 1
#define ADDRESS_LENGTH 2
#define RP_W_NO_CRC_NO_DATA_LENGTH 4
#define RP_R_NO_CRC_NO_DATA_LENGTH 3

#define ERROR_FUNCTION_BIT 0x80

static const uint16_t m_defaultSlaveId = 1;
uint16_t SlaveId;

static const uint8_t m_funcEnumValues[] = { 0x03, 0x06 };

static uint8_t IsFunctionCode(uint8_t value);

static uint8_t IsFunctionCode(uint8_t value) {
	for (uint8_t i = 0; i < FUNCTIONS_NUMBER; i++)
		if (m_funcEnumValues[i] == value)
			return ALL_OK;

	return ANY_ERROR;
}

void EncodeReplyWrite(struct ModbusRecvMessage *mbusMessage, uint8_t *rawReply,
		uint8_t *rawReplyLength) {
	rawReply[RP_SLAVE_ID_INDEX] = mbusMessage->Slave;
	rawReply[RP_FUNCTION_INDEX] = mbusMessage->Function;
	rawReply[RP_W_COMMAND_INDEX] = mbusMessage->Command.Bytes[0];
	rawReply[RP_W_COMMAND_INDEX + 1] = mbusMessage->Command.Bytes[1];

	for (uint16_t i = 0; i < mbusMessage->DataLength; i++)
		rawReply[RP_W_DATA_INDEX + i] = mbusMessage->Data[i];
	*rawReplyLength = RP_W_NO_CRC_NO_DATA_LENGTH + mbusMessage->DataLength;

	uint16_t crc16 = CRC16(rawReply, *rawReplyLength);

	rawReply[*rawReplyLength] = (uint8_t) crc16;
	rawReply[*rawReplyLength + 1] = (uint8_t) (crc16 >> 8);

	*rawReplyLength += CRC_LENGTH;
}

void EncodeReplyRead(struct ModbusRecvMessage *mbusMessage, uint8_t *resultData,
		uint8_t commandResultLength, uint8_t *resultDataLength) {
	resultData[RP_SLAVE_ID_INDEX] = mbusMessage->Slave;
	resultData[RP_FUNCTION_INDEX] = mbusMessage->Function;
	resultData[RP_R_DATA_COUNT_INDEX] = commandResultLength;
	*resultDataLength = RP_R_NO_CRC_NO_DATA_LENGTH + commandResultLength;

	uint16_t crc16 = CRC16(resultData, *resultDataLength);

	resultData[*resultDataLength] = (uint8_t) crc16;
	resultData[*resultDataLength + 1] = (uint8_t) (crc16 >> 8);

	*resultDataLength += CRC_LENGTH;
}

enum ModbusErrorCode DecodeRequest(uint8_t *rawRequest, uint8_t requestLength,
		struct ModbusRecvMessage *mbusMessage) {
	mbusMessage->Slave = rawRequest[RQ_SLAVE_ID_INDEX];
	if (mbusMessage->Slave != SlaveId)
		return MBS_ILLEGAL_SLAVE;

	if (requestLength < MIN_MESSAGE_SIZE || requestLength > MAX_MESSAGE_SIZE)
		return MBS_ILLEGAl_DATA;

	if (CRC16(rawRequest, requestLength - CRC_LENGTH) !=
	BYTES_2_SHORT(rawRequest[requestLength + RQ_CRC_INDEX],
			rawRequest[requestLength + RQ_CRC_INDEX + 1]))
		return MBS_ILLEGAL_CRC;

	mbusMessage->Function = rawRequest[RQ_FUNCTION_INDEX];
	if (IsFunctionCode(mbusMessage->Function))
		return MBS_ILLEGAL_FUNCTION;

	mbusMessage->Data = rawRequest + RQ_DATA_INDEX;
	mbusMessage->DataLength = requestLength - CRC_LENGTH - SLAVE_ID_LENGTH
			- FUNCTION_LENGTH - ADDRESS_LENGTH;
	mbusMessage->Command.Bytes[0] = rawRequest[RQ_COMMAND_INDEX];
	mbusMessage->Command.Bytes[1] = rawRequest[RQ_COMMAND_INDEX + 1];

	return MBS_ALL_OK;
}

//no error handling
enum ModbusErrorCode ProcessError(enum ModbusErrorCode error,
		uint8_t *sendBuffer, uint8_t *resultLength) {
	return error;
}

void InitializeModbus(void) {
	uint16_t flashSlaveId = ParamsUnion.Params.F_SlaveId;

	if (flashSlaveId == 0) {
		SlaveId = m_defaultSlaveId;
		return;
	}

	SlaveId = (uint8_t) flashSlaveId;
}
