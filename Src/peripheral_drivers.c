/*
 * peripheral_drivers.c
 *
 *  Created on: 24 ���. 2018 �.
 *      Author: Satty
 */

#include "math.h"
#include "peripheral_drivers.h"
#include "string.h"
#include "stdlib.h"
#include "limits.h"
#include "usart.h"

#define TIM_PWM_PSC (10 - 1)

//300Hz - frequency, coz has the best duty cycle values
//servo means type of control signals used for BESC and SLED
#define SERVO_PWM_FREQUENCY 300
#define SERVO_PWM_PERIOD (1.0 / SERVO_PWM_FREQUENCY)

#define STOPPED_DC (1.5e-3 / SERVO_PWM_PERIOD)
#define MAX_FORWARD_DC (1.9e-3 / SERVO_PWM_PERIOD)
#define MAX_REVESE_DC (1.1e-3 / SERVO_PWM_PERIOD)
#define DEADBAND_DC (0.025e-3 / SERVO_PWM_PERIOD)
#define MAX_VAR_DC (MAX_FORWARD_DC - STOPPED_DC - DEADBAND_DC)

//800Hz - frequency, coz has the best brightness control
#define LED_PWM_FREQUENCY 800

#define BCS_TIM_CH_INEDX_1 0
#define BCS_TIM_CH_INEDX_2 1
#define BCS_MAX_FREQUENCY 11970
#define BCS_MIN_FREQUECNY 1000
#define BCS_MAX_STEPS (USHRT_MAX / 2)

const uint8_t TIMChannelHandlesNumber = 4;

static struct TIMChannelHandle m_TIMChannels[] = {
		{ &htim2, TIM_CHANNEL_1, &htim1, FREE },
		{ &htim2, TIM_CHANNEL_2, &htim1, FREE },
		{ &htim3, TIM_CHANNEL_1, NULL, FREE },
		{ &htim3, TIM_CHANNEL_2, NULL, FREE } };

static struct GPIOChannelHandle m_GPIOChannels[] = {
		{ GPIOA, GPIO_PIN_8, EXTI4_15_IRQn, FREE },
		{ GPIOA, GPIO_PIN_11, EXTI4_15_IRQn, FREE },
		{ GPIOB, GPIO_PIN_2, EXTI2_3_IRQn, FREE },
		{ GPIOB, GPIO_PIN_8, EXTI4_15_IRQn, FREE },
		{ GPIOF, GPIO_PIN_0, EXTI0_1_IRQn, FREE },
		{ GPIOF, GPIO_PIN_1, EXTI0_1_IRQn, FREE } };

static uint32_t m_BESCDriverARR, m_LEDDriverARR, m_BCSDriverMaxARR,
		m_BCSDriverMinARR, m_SLEDDriverARR;

void InitializeDriversWorker(void) {
	m_SLEDDriverARR =
	m_BESCDriverARR = SystemCoreClock / ((TIM_PWM_PSC + 1) * SERVO_PWM_FREQUENCY)
			- 1;
	m_LEDDriverARR = SystemCoreClock / ((TIM_PWM_PSC + 1) * LED_PWM_FREQUENCY)
			- 1;
	m_BCSDriverMinARR = SystemCoreClock
			/ ((TIM_PWM_PSC + 1) * BCS_MAX_FREQUENCY) - 1;
	m_BCSDriverMaxARR = SystemCoreClock
			/ ((TIM_PWM_PSC + 1) * BCS_MIN_FREQUECNY) - 1;
}

uint8_t ReinitializeBESCDriver(struct BESCDriverHandle *handle) {
	struct TIMChannelHandle *tim = handle->PWMTIM;

	__HAL_TIM_SET_COMPARE(tim->Tim, tim->Channel,
			m_BESCDriverARR * (1.0 - STOPPED_DC));
	__HAL_TIM_SET_PRESCALER(tim->Tim, TIM_PWM_PSC);
	__HAL_TIM_SET_AUTORELOAD(tim->Tim, m_BESCDriverARR);

	return ALL_OK;
}

uint8_t ReinitializeBCSDriver(struct BCSDriverHandle *handle) {
	struct TIMChannelHandle *timCh = handle->PWMTIM;
	struct GPIOChannelHandle *direction = handle->DirectionGPIO;
	struct GPIOChannelHandle *disable = handle->DisableGPIO;
	struct GPIOChannelHandle *terminal = handle->TerminalGPIO;

	__HAL_TIM_SET_COMPARE(timCh->Tim, timCh->Channel, m_BCSDriverMaxARR / 2);
	__HAL_TIM_SET_AUTORELOAD(timCh->Tim, m_BCSDriverMaxARR);
	__HAL_TIM_SET_PRESCALER(timCh->Tim, TIM_PWM_PSC);

	GPIO_InitTypeDef GPIOInitStruct = { .Pin = direction->Pin, .Mode =
			GPIO_MODE_OUTPUT_PP, .Pull = GPIO_NOPULL, .Speed =
					GPIO_SPEED_FREQ_LOW };
	HAL_GPIO_Init(direction->GPIO, &GPIOInitStruct);

	GPIOInitStruct.Pin = disable->Pin;
	HAL_GPIO_Init(disable->GPIO, &GPIOInitStruct);

	GPIOInitStruct.Pin = terminal->Pin;
	GPIOInitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIOInitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(terminal->GPIO, &GPIOInitStruct);
	__HAL_GPIO_EXTI_CLEAR_IT(terminal->Pin);
	HAL_NVIC_SetPriority(terminal->EXTIId, 0, 0);
	HAL_NVIC_EnableIRQ(terminal->EXTIId);

	__HAL_TIM_SET_COUNTER(timCh->ComplementTIM, 0);

	TIM_SlaveConfigTypeDef sSlaveConfig;
	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_GATED;
	sSlaveConfig.InputTrigger = TIM_TS_ITR0;
	HAL_TIM_SlaveConfigSynchronization(timCh->Tim, &sSlaveConfig);

	return ALL_OK;
}

uint8_t ReinitializeLEDDriver(struct LEDDriverHandle *handle) {
	struct TIMChannelHandle *tim = handle->PWMTIM;

	__HAL_TIM_SET_COMPARE(tim->Tim, tim->Channel, m_LEDDriverARR);
	__HAL_TIM_SET_PRESCALER(tim->Tim, TIM_PWM_PSC);
	__HAL_TIM_SET_AUTORELOAD(tim->Tim, m_LEDDriverARR);

	return ALL_OK;
}

uint8_t ReinitializeSLEDDriver(struct SLEDDriverHandle *handle) {
	struct TIMChannelHandle *tim = handle->PWMTIM;

	__HAL_TIM_SET_COMPARE(tim->Tim, tim->Channel, m_SLEDDriverARR);
	__HAL_TIM_SET_PRESCALER(tim->Tim, TIM_PWM_PSC);
	__HAL_TIM_SET_AUTORELOAD(tim->Tim, m_SLEDDriverARR);

	return ALL_OK;
}

uint8_t DeinitializeBSCDriver(struct BCSDriverHandle *handle) {
	HAL_NVIC_DisableIRQ(handle->TerminalGPIO->EXTIId);

	TIM_SlaveConfigTypeDef sSlaveConfig;
	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
	if (HAL_TIM_SlaveConfigSynchronization(handle->PWMTIM->Tim, &sSlaveConfig) != HAL_OK)
		return ANY_ERROR;

	__HAL_TIM_SET_COUNTER(handle->PWMTIM->ComplementTIM, 0);

	return ALL_OK;
}

uint8_t InitializeBESCDriver(enum TIMChannel pwmTIMCh,
		struct BESCDriverHandle *handle) {
	struct TIMChannelHandle *tim = &m_TIMChannels[pwmTIMCh];

	if (tim->State == OCCUPIED)
		return ANY_ERROR;

	__HAL_TIM_SET_COMPARE(tim->Tim, tim->Channel,
			m_BESCDriverARR * (1.0 - STOPPED_DC));
	__HAL_TIM_SET_PRESCALER(tim->Tim, TIM_PWM_PSC);
	__HAL_TIM_SET_AUTORELOAD(tim->Tim, m_BESCDriverARR);

	tim->State = OCCUPIED;

	handle->PWMTIM = tim;

	return ALL_OK;
}

uint8_t InitializeLEDDriver(enum TIMChannel pwmTIMCh,
		struct LEDDriverHandle *handle) {
	struct TIMChannelHandle *tim = &m_TIMChannels[pwmTIMCh];

	if (tim->State == OCCUPIED)
		return ANY_ERROR;

	__HAL_TIM_SET_COMPARE(tim->Tim, tim->Channel, m_LEDDriverARR);
	__HAL_TIM_SET_PRESCALER(tim->Tim, TIM_PWM_PSC);
	__HAL_TIM_SET_AUTORELOAD(tim->Tim, m_LEDDriverARR);

	tim->State = OCCUPIED;

	handle->PWMTIM = tim;

	return ALL_OK;
}

uint8_t InitializeSLEDDriver(enum TIMChannel pwmTIMCh,
		struct SLEDDriverHandle *handle) {
	struct TIMChannelHandle *tim = &m_TIMChannels[pwmTIMCh];

	if (tim->State == OCCUPIED)
		return ANY_ERROR;

	__HAL_TIM_SET_COMPARE(tim->Tim, tim->Channel, m_SLEDDriverARR);
	__HAL_TIM_SET_PRESCALER(tim->Tim, TIM_PWM_PSC);
	__HAL_TIM_SET_AUTORELOAD(tim->Tim, m_SLEDDriverARR);

	tim->State = OCCUPIED;

	handle->PWMTIM = tim;

	return ALL_OK;
}

uint8_t InitializeBCSDriver(enum TIMChannel pwmTIMCh,
		enum GPIOChannel directionGPIOCh, enum GPIOChannel disableGPIOCh,
		enum GPIOChannel terminalGPIOCh, struct BCSDriverHandle *handle) {
	struct TIMChannelHandle *timCh = &m_TIMChannels[pwmTIMCh];
	struct GPIOChannelHandle *direction = &m_GPIOChannels[directionGPIOCh];
	struct GPIOChannelHandle *disable = &m_GPIOChannels[disableGPIOCh];
	struct GPIOChannelHandle *terminal = &m_GPIOChannels[terminalGPIOCh];

	if (timCh->State == OCCUPIED || direction->State == OCCUPIED
			|| disable->State == OCCUPIED || terminal->State == OCCUPIED)
		return ANY_ERROR;

	__HAL_TIM_SET_COMPARE(timCh->Tim, timCh->Channel, m_BCSDriverMaxARR / 2);
	__HAL_TIM_SET_AUTORELOAD(timCh->Tim, m_BCSDriverMaxARR);
	__HAL_TIM_SET_PRESCALER(timCh->Tim, TIM_PWM_PSC);

	GPIO_InitTypeDef GPIOInitStruct = { .Pin = direction->Pin, .Mode =
	GPIO_MODE_OUTPUT_PP, .Pull = GPIO_NOPULL, .Speed =
	GPIO_SPEED_FREQ_LOW };
	HAL_GPIO_Init(direction->GPIO, &GPIOInitStruct);

	GPIOInitStruct.Pin = disable->Pin;
	HAL_GPIO_Init(disable->GPIO, &GPIOInitStruct);

	GPIOInitStruct.Pin = terminal->Pin;
	GPIOInitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIOInitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(terminal->GPIO, &GPIOInitStruct);
	__HAL_GPIO_EXTI_CLEAR_IT(terminal->Pin);
	HAL_NVIC_SetPriority(terminal->EXTIId, 0, 0);
	HAL_NVIC_EnableIRQ(terminal->EXTIId);

	__HAL_TIM_SET_COUNTER(timCh->ComplementTIM, 0);

	TIM_SlaveConfigTypeDef sSlaveConfig;
	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_GATED;
	sSlaveConfig.InputTrigger = TIM_TS_ITR0;
	if (HAL_TIM_SlaveConfigSynchronization(timCh->Tim, &sSlaveConfig) != HAL_OK)
		return ANY_ERROR;

	for (uint8_t i = 0; i < TIMChannelHandlesNumber; i++)
		if (timCh->Tim == m_TIMChannels[i].Tim)
			m_TIMChannels[i].State = OCCUPIED;
	disable->State = OCCUPIED;
	direction->State = OCCUPIED;
	terminal->State = OCCUPIED;

	handle->PWMTIM = timCh;
	handle->DisableGPIO = disable;
	handle->DirectionGPIO = direction;
	handle->TerminalGPIO = terminal;

	return ALL_OK;
}

uint8_t BESCDriverChangeSpeed(struct BESCDriverHandle *handle, uint8_t *data) {
	int8_t newSpeed = data[0];

	float newDutyCycle = (newSpeed == 0) ? STOPPED_DC : (STOPPED_DC +
						 	 MAX_VAR_DC * ((float)newSpeed / 0x7F) +
							 DEADBAND_DC * ((newSpeed > 0) ? 1 : -1));

	__HAL_TIM_SET_COMPARE(handle->PWMTIM->Tim, handle->PWMTIM->Channel,
			round(m_BESCDriverARR * (1.0 - newDutyCycle)));

	return ALL_OK;
}

uint8_t LEDDriverChangeBrightness(struct LEDDriverHandle *handle, uint8_t *data) {
	uint8_t newBrightness = data[0];

	__HAL_TIM_SET_COMPARE(handle->PWMTIM->Tim, handle->PWMTIM->Channel,
			round(m_LEDDriverARR * ((float)newBrightness / UINT8_MAX)));

	return ALL_OK;
}

uint8_t SLEDDriverChangeBrightness(struct SLEDDriverHandle *handle, uint8_t *data) {
	uint8_t newBrightness = data[0];

	float newDutyCycle = MAX_FORWARD_DC * ((float)newBrightness / UINT8_MAX);

	__HAL_TIM_SET_COMPARE(handle->PWMTIM->Tim, handle->PWMTIM->Channel,
			round(m_SLEDDriverARR * (1.0 - newDutyCycle)));

	return ALL_OK;
}

uint8_t BCSDriverChangeSpeed(struct BCSDriverHandle *handle, uint8_t *data) {
	uint8_t newSpeed = data[0];

	uint32_t newARR = m_BCSDriverMaxARR
			- round(((float)newSpeed / UINT8_MAX) * (m_BCSDriverMaxARR - m_BCSDriverMinARR));

	__HAL_TIM_SET_COUNTER(handle->PWMTIM->Tim, 0);
	__HAL_TIM_SET_AUTORELOAD(handle->PWMTIM->Tim, newARR);
	__HAL_TIM_SET_COMPARE(handle->PWMTIM->Tim, handle->PWMTIM->Channel,
			newARR / 2);

	return ALL_OK;
}

uint8_t BCSDriverMove(struct BCSDriverHandle *handle, uint8_t *data) {
	enum Direction direction = (enum Direction) data[0];
	uint16_t steps;
	memcpy(&steps, &data[1], sizeof(int16_t));

	if(steps > BCS_MAX_STEPS) return ANY_ERROR;

	steps = steps * 2 - 1;

	if (!(direction == FORWARD || direction == BACKWARD)) return ANY_ERROR;

	uint16_t doneSteps = __HAL_TIM_GET_COUNTER(handle->PWMTIM->ComplementTIM);

	if(doneSteps != 0) {
		__HAL_TIM_DISABLE(handle->PWMTIM->ComplementTIM);

		doneSteps = __HAL_TIM_GET_COUNTER(handle->PWMTIM->ComplementTIM);
		int32_t leftSteps = __HAL_TIM_GET_AUTORELOAD(handle->PWMTIM->ComplementTIM) - doneSteps;

		if(handle->CurrentDirection == direction){
			if(leftSteps > USHRT_MAX - steps)
				return ANY_ERROR;

			steps += leftSteps;
		}
		else {
			leftSteps -= steps;
			if(leftSteps < 0) {
				HAL_GPIO_WritePin(handle->DirectionGPIO->GPIO, handle->DirectionGPIO->Pin,
						(direction) ? GPIO_PIN_SET : GPIO_PIN_RESET);
				handle->CurrentDirection = direction;
			}

			steps = (uint16_t)abs(leftSteps);
		}

		__HAL_TIM_SET_AUTORELOAD(handle->PWMTIM->ComplementTIM, steps);
		__HAL_TIM_SET_COUNTER(handle->PWMTIM->ComplementTIM, 0);
		__HAL_TIM_ENABLE(handle->PWMTIM->ComplementTIM);
		return ALL_OK;
	}

	__HAL_TIM_SET_AUTORELOAD(handle->PWMTIM->ComplementTIM, steps);
	HAL_GPIO_WritePin(handle->DisableGPIO->GPIO, handle->DisableGPIO->Pin,
			GPIO_PIN_SET);
	HAL_GPIO_WritePin(handle->DirectionGPIO->GPIO, handle->DirectionGPIO->Pin,
							(direction) ? GPIO_PIN_SET : GPIO_PIN_RESET);

	__HAL_TIM_ENABLE(handle->PWMTIM->ComplementTIM);
	handle->CurrentDirection = direction;

	return ALL_OK;
}

uint8_t BCSDriverStop(struct BCSDriverHandle *handle) {

	__HAL_TIM_DISABLE(handle->PWMTIM->ComplementTIM);
	HAL_GPIO_WritePin(handle->DisableGPIO->GPIO, handle->DisableGPIO->Pin,
			GPIO_PIN_RESET);
	__HAL_TIM_SET_COUNTER(handle->PWMTIM->ComplementTIM, 0);

	return ALL_OK;
}

uint8_t BCSDriverMoveToEnd(struct BCSDriverHandle *handle, uint8_t *data) {
	enum Direction direction = (enum Direction) data[0];

	if (!(direction == FORWARD || direction == BACKWARD))
		return ANY_ERROR;

	HAL_GPIO_WritePin(handle->DirectionGPIO->GPIO, handle->DirectionGPIO->Pin,
			(direction) ? GPIO_PIN_SET : GPIO_PIN_RESET);

	__HAL_TIM_SET_AUTORELOAD(handle->PWMTIM->ComplementTIM, UINT16_MAX);

	HAL_GPIO_WritePin(handle->DisableGPIO->GPIO, handle->DisableGPIO->Pin,
			GPIO_PIN_SET);

	__HAL_TIM_ENABLE(handle->PWMTIM->ComplementTIM);

	return ALL_OK;
}
