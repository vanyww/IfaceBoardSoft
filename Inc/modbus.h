/*
 * modbus.h
 *
 *  Created on: 14 мар. 2018 г.
 *      Author: Satty
 */

#ifndef MODBUS_H_
#define MODBUS_H_

#include "defs.h"
#include "stdint.h"

#define MAX_MESSAGE_SIZE 0xFF
#define MIN_MESSAGE_SIZE 4

//Reply [READ] data indexes
#define RP_R_DATA_COUNT_INDEX 2
#define RP_DATA_COUNT_INDEX 3

enum ModbusErrorCode {
	MBS_ALL_OK = ALL_OK,

	MBS_ILLEGAL_FUNCTION,
	MBS_ILLEGAL_ADDRESS,
	MBS_ILLEGAl_DATA,
	MBS_SLAVE_ERROR,

	MBS_ILLEGAL_CRC,
	MBS_ILLEGAL_SLAVE
};

enum FunctionCode {
	READ = 0x03, WRITE = 0x10
};

union CommandUnion {
	uint16_t CommandCode;
	uint8_t Bytes[sizeof(uint16_t)];
};

struct ModbusRecvMessage {
	uint8_t Slave;
	enum FunctionCode Function;
	union CommandUnion Command;
	uint16_t RegistersCount;
	uint8_t DataLength;
	uint8_t *Data;
};

void EncodeReplyWrite(struct ModbusRecvMessage *mbusMessage,
		uint16_t writtenDataLength, uint8_t *rawReply, uint8_t *rawReplyLength);

void EncodeReplyRead(struct ModbusRecvMessage *mbusMessage, uint8_t *resultData,
		uint8_t commandResultLength, uint8_t *resultDataLength);

enum ModbusErrorCode DecodeRequest(uint8_t *rawRequest, uint8_t requestLength,
		struct ModbusRecvMessage *mbusMessage);

enum ModbusErrorCode ProcessError(enum ModbusErrorCode error,
		uint8_t *sendBuffer, uint8_t *resultLength);

void InitializeModbus(void);

extern uint16_t SlaveId;

#endif /* MODBUS_H_ */
