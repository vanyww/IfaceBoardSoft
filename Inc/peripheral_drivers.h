/*
 * peripheral_drivers.h
 *
 *  Created on: 24 θών. 2018 γ.
 *      Author: Satty
 */

#ifndef PERIPHERAL_DRIVERS_H_
#define PERIPHERAL_DRIVERS_H_

#include "stdint.h"
#include "defs.h"
#include "gpio.h"
#include "tim.h"

#define BESC_DRIVERS_MAX 4
#define LED_DRIVERS_MAX 4
#define SLED_DRIVERS_MAX 4
#define BCS_DRIVERS_MAX 1

enum ChannelState {
	OCCUPIED, FREE
};

enum TIMChannel {
	TIMCh1 = 0, TIMCh2 = 1, TIMCh3 = 2, TIMCh4 = 3
};

enum GPIOChannel {
	GPIOCh1 = 0, GPIOCh2 = 1, GPIOCh3 = 2, GPIOCh4 = 3, GPIOCh5 = 4, GPIOCh6 = 5
};

enum Direction {
	FORWARD = 0, BACKWARD = 1
};

struct TIMChannelHandle {
	TIM_HandleTypeDef *Tim;
	uint32_t Channel;
	TIM_HandleTypeDef *ComplementTIM;
	enum ChannelState State;
};

struct GPIOChannelHandle {
	GPIO_TypeDef *GPIO;
	uint16_t Pin;
	IRQn_Type EXTIId;
	enum ChannelState State;
};

struct BESCDriverHandle {
	struct TIMChannelHandle *PWMTIM;
};

struct LEDDriverHandle {
	struct TIMChannelHandle *PWMTIM;
};

struct SLEDDriverHandle {
	struct TIMChannelHandle *PWMTIM;
};

struct BCSDriverHandle {
	struct TIMChannelHandle *PWMTIM;
	struct GPIOChannelHandle *DirectionGPIO;
	struct GPIOChannelHandle *DisableGPIO;
	struct GPIOChannelHandle *TerminalGPIO;
	enum Direction CurrentDirection;
};

void InitializeDriversWorker(void);

uint8_t ChangeUSARTBaudRate(uint8_t usartId, uint32_t baud);

uint8_t InitializeBESCDriver(enum TIMChannel pwmTIMCh,
		struct BESCDriverHandle *handle);
uint8_t InitializeLEDDriver(enum TIMChannel pwmTIMCh,
		struct LEDDriverHandle *handle);
uint8_t InitializeSLEDDriver(enum TIMChannel pwmTIMCh,
		struct SLEDDriverHandle *handle);
uint8_t InitializeBCSDriver(enum TIMChannel pwmTIMCh,
		enum GPIOChannel directionGPIOCh, enum GPIOChannel disableGPIOCh,
		enum GPIOChannel terminalGPIOCh, struct BCSDriverHandle *handle);

uint8_t BESCDriverChangeSpeed(struct BESCDriverHandle *handle, uint8_t *data);

uint8_t LEDDriverChangeBrightness(struct LEDDriverHandle *handle, uint8_t *data);
uint8_t SLEDDriverChangeBrightness(struct SLEDDriverHandle *handle, uint8_t *data);

uint8_t BCSDriverChangeSpeed(struct BCSDriverHandle *handle, uint8_t *data);
uint8_t BCSDriverMove(struct BCSDriverHandle *handle, uint8_t *data);
uint8_t BCSDriverMoveToEnd(struct BCSDriverHandle *handle, uint8_t *data);
uint8_t BCSDriverStop(struct BCSDriverHandle *handle);

uint8_t ReinitializeBESCDriver(struct BESCDriverHandle *handle);
uint8_t ReinitializeBCSDriver(struct BCSDriverHandle *handle);
uint8_t ReinitializeLEDDriver(struct LEDDriverHandle *handle);
uint8_t ReinitializeSLEDDriver(struct SLEDDriverHandle *handle);

uint8_t DeinitializeBSCDriver(struct BCSDriverHandle *handle);
#endif /* PERIPHERAL_DRIVERS_H_ */
