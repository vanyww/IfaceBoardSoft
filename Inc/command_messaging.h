/*
 * command_messaging.h
 *
 *  Created on: 6 апр. 2018 г.
 *      Author: Satty
 */

#ifndef COMMAND_MESSAGING_H_
#define COMMAND_MESSAGING_H_

#include "modbus.h"
#include "stddef.h"

//max is 0xFF
#define COMMANDS_NUMBER 30

enum CommandErrorCode {
	CMD_ALL_OK = ALL_OK,

	CMD_NO_COMMAND = MBS_ILLEGAL_ADDRESS,
	CMD_NO_SUBCOMMAND = MBS_ILLEGAL_FUNCTION,

	CMD_ANY_ERROR = ANY_ERROR
};

typedef uint8_t (*WriteCommandFunction)(struct ModbusRecvMessage *msg,
										uint16_t *writtenDataLength);

typedef uint8_t (*ReadCommandFunction)(struct ModbusRecvMessage *msg,
									   uint8_t *result,
									   uint8_t *resultLength);

struct Command {
	WriteCommandFunction WriteFunction;
	ReadCommandFunction ReadFunction;
	uint16_t CommandCode;
};

enum CommandErrorCode ProcessCommand(uint8_t *recvBuffer,
								     uint8_t requestLength,
									 uint8_t *sendBuffer,
									 uint8_t *resultLength);

enum CommandErrorCode  AddCommand(uint16_t commandCode,
							      ReadCommandFunction readFunction,
								  WriteCommandFunction writeFunction);

#endif /* COMMAND_MESSAGING_H_ */
