/*
 * modbus.c
 *
 *  Created on: 14 мар. 2018 г.
 *      Author: Satty
 */

#include "crc16.h"
#include "modbus.h"
#include "settings.h"

//Request data indexes
#define RQ_SLAVE_ID_INDEX 0
#define RQ_FUNCTION_INDEX 1
#define RQ_COMMAND_INDEX 2
#define RQ_REGISTERS_NUMBER_INDEX 4
#define RQ_CRC_NEGINDEX -2

#define RQ_W_DATA_COUNT_SIZE 1
#define RQ_W_DATA_COUNT_INDEX 6
#define RQ_W_DATA_INDEX 7

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
#define RQ_REGISTERS_COUNT_LENGTH 2

#define ERROR_FUNCTION_BIT 0x80

static const uint16_t m_defaultSlaveId = 1;
uint16_t SlaveId;

enum ModbusErrorCode DecodeRequestWrite(uint8_t *rawRequest,
		uint8_t requestLength, struct ModbusRecvMessage *mbusMessage);
enum ModbusErrorCode DecodeRequestRead(uint8_t *rawRequest,
		uint8_t requestLength, struct ModbusRecvMessage *mbusMessage);

void EncodeReplyWrite(struct ModbusRecvMessage *mbusMessage,
		uint16_t writtenDataLength, uint8_t *rawReply, uint8_t *rawReplyLength) {
	rawReply[RP_SLAVE_ID_INDEX] = mbusMessage->Slave;
	rawReply[RP_FUNCTION_INDEX] = mbusMessage->Function;
	rawReply[RP_W_COMMAND_INDEX] = mbusMessage->Command.Bytes[0];
	rawReply[RP_W_COMMAND_INDEX + 1] = mbusMessage->Command.Bytes[1];

	uint16_t writtenRegistersCount = writtenDataLength / sizeof(uint16_t);

	rawReply[RP_W_DATA_INDEX] = (uint8_t) writtenRegistersCount >> 8;
	rawReply[RP_W_DATA_INDEX + 1] = (uint8_t) writtenRegistersCount;


	*rawReplyLength = RP_W_NO_CRC_NO_DATA_LENGTH + 2;

	uint16_t crc16 = CRC16(rawReply, *rawReplyLength);

	rawReply[*rawReplyLength] = (uint8_t) (crc16 >> 8);
	rawReply[*rawReplyLength + 1] = (uint8_t) crc16;

	*rawReplyLength += CRC_LENGTH;
}

void EncodeReplyRead(struct ModbusRecvMessage *mbusMessage, uint8_t *resultData,
		uint8_t commandResultLength, uint8_t *resultDataLength) {
	resultData[RP_SLAVE_ID_INDEX] = mbusMessage->Slave;
	resultData[RP_FUNCTION_INDEX] = mbusMessage->Function;
	resultData[RP_R_DATA_COUNT_INDEX] = commandResultLength;
	*resultDataLength = RP_R_NO_CRC_NO_DATA_LENGTH + commandResultLength;

	uint16_t crc16 = CRC16(resultData, *resultDataLength);

	resultData[*resultDataLength] = (uint8_t) (crc16 >> 8);
	resultData[*resultDataLength + 1] = (uint8_t) crc16;

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
	BYTES_2_SHORT(rawRequest[requestLength + RQ_CRC_NEGINDEX + 1],
			rawRequest[requestLength + RQ_CRC_NEGINDEX]))
		return MBS_ILLEGAL_CRC;

	mbusMessage->Function = rawRequest[RQ_FUNCTION_INDEX];

	mbusMessage->RegistersCount = BYTES_2_SHORT(
			rawRequest[RQ_REGISTERS_NUMBER_INDEX + 1],
			rawRequest[RQ_REGISTERS_NUMBER_INDEX]);

	mbusMessage->Command.Bytes[0] = rawRequest[RQ_COMMAND_INDEX + 1];
	mbusMessage->Command.Bytes[1] = rawRequest[RQ_COMMAND_INDEX];

	switch (mbusMessage->Function) {
	case READ:
		return MBS_ALL_OK;
	case WRITE:
		return DecodeRequestWrite(rawRequest, requestLength, mbusMessage);
	default:
		return MBS_ILLEGAL_FUNCTION;
	}
}

enum ModbusErrorCode DecodeRequestWrite(uint8_t *rawRequest,
		uint8_t requestLength, struct ModbusRecvMessage *mbusMessage) {
	mbusMessage->DataLength = requestLength - ADDRESS_LENGTH - SLAVE_ID_LENGTH
			- FUNCTION_LENGTH - CRC_LENGTH - RQ_W_DATA_COUNT_SIZE - RQ_REGISTERS_COUNT_LENGTH;

	if (mbusMessage->DataLength != rawRequest[RQ_W_DATA_COUNT_INDEX]
			|| mbusMessage->DataLength / 2 + mbusMessage->DataLength % 2
					!= mbusMessage->RegistersCount)
		return MBS_ILLEGAl_DATA;

	mbusMessage->Data = &rawRequest[RQ_W_DATA_INDEX];

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
