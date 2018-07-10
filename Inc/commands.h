/*
 * commands.h
 *
 *  Created on: 25 мая 2018 г.
 *      Author: Vanyw
 */

#ifndef COMMANDS_H_
#define COMMANDS_H_

#include "peripheral_drivers.h"
#include "command_messaging.h"

void InitializeCommands();

extern struct BESCDriverHandle BESCDrivers[4];
extern uint8_t BESCDriversCounter;

extern struct BCSDriverHandle *TIM1BCS;
extern struct BCSDriverHandle BCSDrivers[1];
extern uint8_t BCSDriversCounter;

extern struct LEDDriverHandle LEDDrivers[4];
extern uint8_t LEDDriversCounter;

#endif /* COMMANDS_H_ */
