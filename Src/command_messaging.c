/*
 * command_messaging.c
 *
 *  Created on: 6 апр. 2018 г.
 *      Author: Satty
 */

#include "command_messaging.h"

static uint16_t m_registeredCommandsCount = 0;
static struct Command m_commands[COMMANDS_NUMBER] = {{.CommandCode=0,
												   .ReadFunction = NULL,
												   .WriteFunction = NULL}};
static struct ModbusRecvMessage m_message;

uint8_t FindCommand(uint16_t commandCode, struct Command **command);

uint8_t FindCommand(uint16_t commandCode, struct Command **command) {
	for(uint8_t i = 0; i < m_registeredCommandsCount; i++)
		if(m_commands[i].CommandCode == commandCode) {
			*command = &m_commands[i];
			return ALL_OK;
		}

	return ANY_ERROR;
}

enum CommandErrorCode ProcessCommand(uint8_t *recvBuffer,
								     uint8_t requestLength,
									 uint8_t *sendBuffer,
									 uint8_t *resultLength) {
	{
		enum ModbusErrorCode errorCode;
		if((errorCode = DecodeRequest(recvBuffer, requestLength, &m_message)))
			return ProcessError(errorCode, sendBuffer, resultLength);
	}

	struct Command *calledCommand;

	if(FindCommand(m_message.Command.CommandCode, &calledCommand))
		return CMD_NO_COMMAND;

	switch(m_message.Function) {
		case READ: {
			if(calledCommand->ReadFunction == NULL)
				return CMD_NO_SUBCOMMAND;

			uint8_t commandResultLength;
			if(calledCommand->ReadFunction(&m_message,
								 	       sendBuffer + RP_DATA_COUNT_INDEX,
										   &commandResultLength))
				return CMD_ANY_ERROR;
			EncodeReplyRead(&m_message, sendBuffer, commandResultLength, resultLength);
			return CMD_ALL_OK;
		}

		case WRITE:
			if(calledCommand->WriteFunction == NULL)
				return CMD_NO_SUBCOMMAND;

			if(calledCommand->WriteFunction(&m_message))
				return CMD_ANY_ERROR;
			EncodeReplyWrite(&m_message, sendBuffer, resultLength);
			return CMD_ALL_OK;
	}

	return CMD_ANY_ERROR;
}

enum CommandErrorCode AddCommand(uint16_t commandCode,
				   	   	   	     ReadCommandFunction readFunction,
								 WriteCommandFunction writeFunction) {
	if(m_registeredCommandsCount >= COMMANDS_NUMBER)
		return CMD_ANY_ERROR;

	if(readFunction == NULL && writeFunction == NULL)
		return CMD_ANY_ERROR;

	struct Command *nextCommand = &m_commands[m_registeredCommandsCount];

	nextCommand->CommandCode = commandCode;
	nextCommand->ReadFunction = readFunction;
	nextCommand->WriteFunction = writeFunction;

	m_registeredCommandsCount++;

	return CMD_ALL_OK;
}
