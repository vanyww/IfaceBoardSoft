/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "commands.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

static uint8_t m_USART1dmaRxBuffer[USART_RX_BUFFER_LENGTH],
			   m_USART1RxBuffer[MAX_MESSAGE_SIZE],
			   m_USART1TxBuffer[MAX_MESSAGE_SIZE],
			   m_USART1RxBufferLength = 0,

			   m_USART2dmaRxBuffer[USART_RX_BUFFER_LENGTH],
			   m_USART2RxBuffer[MAX_MESSAGE_SIZE],
			   m_USART2TxBuffer[MAX_MESSAGE_SIZE],
			   m_USART2RxBufferLength = 0;

enum _RXOverflow {
	NORMAL = 0,
	OVERFLOW = 1
} m_USART1Overflow = NORMAL,
  m_USART2Overflow = NORMAL;

/* Private variables ---------------------------------------------------------*/
#if defined   (  __GNUC__  )
__IO uint32_t VectorTable[48] __attribute__((section(".RAMVectorTable")));
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		if (!m_USART1RxBufferLength)
			__HAL_TIM_ENABLE(&htim6);

		__HAL_TIM_SET_COUNTER(&htim6, 0);
		m_USART1RxBuffer[m_USART1RxBufferLength++] = m_USART1dmaRxBuffer[0];

		if(m_USART1RxBufferLength == MAX_MESSAGE_SIZE) {
			m_USART1Overflow = OVERFLOW;
			m_USART1RxBufferLength = 0;
			return;
		}

		return;
	}

	if (huart->Instance == USART2) {
		if (!m_USART2RxBufferLength)
			__HAL_TIM_ENABLE(&htim16);

		__HAL_TIM_SET_COUNTER(&htim16, 0);
		m_USART2RxBuffer[m_USART2RxBufferLength++] = m_USART2dmaRxBuffer[0];

		if(m_USART2RxBufferLength == MAX_MESSAGE_SIZE) {
			m_USART2Overflow = OVERFLOW;
			m_USART2RxBufferLength = 0;
			return;
		}

		return;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		if (!m_USART1RxBufferLength)
			__HAL_TIM_ENABLE(&htim6);

		__HAL_TIM_SET_COUNTER(&htim6, 0);
		m_USART1RxBuffer[m_USART1RxBufferLength++] = m_USART1dmaRxBuffer[1];

		if(m_USART1RxBufferLength == MAX_MESSAGE_SIZE) {
			m_USART1Overflow = OVERFLOW;
			m_USART1RxBufferLength = 0;
			return;
		}

		return;
	}

	if (huart->Instance == USART2) {
		if (!m_USART2RxBufferLength)
			__HAL_TIM_ENABLE(&htim16);

		__HAL_TIM_SET_COUNTER(&htim16, 0);
		m_USART2RxBuffer[m_USART2RxBufferLength++] = m_USART2dmaRxBuffer[1];

		if(m_USART2RxBufferLength == MAX_MESSAGE_SIZE) {
			m_USART2Overflow = OVERFLOW;
			m_USART2RxBufferLength = 0;
			return;
		}

		return;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t pin) {
	if (pin == TIM1BCS->TerminalGPIO->Pin) {
		__HAL_TIM_DISABLE(TIM1BCS->PWMTIM->ComplementTIM);
		HAL_GPIO_WritePin(TIM1BCS->DisableGPIO->GPIO, TIM1BCS->DisableGPIO->Pin,
				GPIO_PIN_RESET);
		__HAL_TIM_SET_COUNTER(TIM1BCS->PWMTIM->ComplementTIM, 0);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
	if (htim->Instance == TIM14) {
		__HAL_IWDG_RELOAD_COUNTER(&hiwdg);
		return;
	}

	if (htim->Instance == TIM6) {
		if(m_USART1Overflow == OVERFLOW) {
			m_USART1RxBufferLength = 0;
			m_USART1Overflow = NORMAL;
			return;
		}

		uint8_t resultLength;

		if (ProcessCommand(m_USART1RxBuffer, m_USART1RxBufferLength, m_USART1TxBuffer,
				&resultLength)) {
			m_USART1RxBufferLength = 0;
			return;
		}

		HAL_UART_Transmit_DMA(&huart1, m_USART1TxBuffer, resultLength);
		m_USART1RxBufferLength = 0;
		return;
	}

	if (htim->Instance == TIM16) {
		if(m_USART2Overflow == OVERFLOW) {
			m_USART2RxBufferLength = 0;
			m_USART2Overflow = NORMAL;
			return;
		}

		uint8_t resultLength;

		if (ProcessCommand(m_USART2RxBuffer, m_USART2RxBufferLength, m_USART2TxBuffer,
				&resultLength)) {
			m_USART2RxBufferLength = 0;
			return;
		}

		HAL_UART_Transmit_DMA(&huart2, m_USART2TxBuffer, resultLength);
		m_USART2RxBufferLength = 0;
		return;
	}

	if (htim->Instance == TIM1) {
		HAL_GPIO_WritePin(TIM1BCS->DisableGPIO->GPIO, TIM1BCS->DisableGPIO->Pin,
				GPIO_PIN_RESET);
		return;
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  for(uint8_t i = 0; i < 48; i++)
  {
    VectorTable[i] = *(__IO uint32_t *)(0x8003000 + (i << 2));
  }

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_SYSCFG_REMAPMEMORY_SRAM();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM14_Init();
  MX_IWDG_Init();
  MX_TIM16_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  LoadUARTBaud();
  LoadMS5837OSR();
  InitializeCommands();
  SetDeviceConfiguration();

  __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
  __HAL_TIM_ENABLE_IT(&htim6, TIM_IT_UPDATE);

  __HAL_TIM_CLEAR_FLAG(&htim16, TIM_FLAG_UPDATE);
  __HAL_TIM_ENABLE_IT(&htim16, TIM_IT_UPDATE);

  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);

  __HAL_TIM_CLEAR_FLAG(&htim14, TIM_FLAG_UPDATE);
  __HAL_TIM_ENABLE_IT(&htim14, TIM_IT_UPDATE);
  __HAL_TIM_ENABLE(&htim14);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  HAL_UART_Receive_DMA(&huart1, (uint8_t *) m_USART1dmaRxBuffer, 2);
  HAL_UART_Receive_DMA(&huart2, (uint8_t *) m_USART2dmaRxBuffer, 2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  /* TIM6_DAC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  /* TIM14_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM14_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM14_IRQn);
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* DMA1_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);
  /* TIM16_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM16_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(TIM16_IRQn);
  /* TIM1_BRK_UP_TRG_COM_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
