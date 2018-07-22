/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
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
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

/* USER CODE BEGIN 0 */

extern uint16_t SPI_rx[1]; //ADC Readings
extern uint16_t SPI_tx[1]; //ADC Readings

extern UART_HandleTypeDef huart2;
extern SPI_HandleTypeDef hspi1;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim5;

static uint16_t old_spi_value = 0;
static uint16_t new_spi_value = 0;
static uint16_t ack = 0;

extern uint16_t duty_cycle_LED;

extern uint16_t duty_cycle_channel_A;
extern uint16_t duty_cycle_channel_B;

extern uint16_t enable_channel_A;
extern uint16_t enable_channel_B;

extern uint16_t enable_channels;

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles DMA2 stream0 global interrupt.
*/
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_rx);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/**
* @brief This function handles DMA2 stream3 global interrupt.
*/
void DMA2_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream3_IRQn 0 */

  /* USER CODE END DMA2_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_tx);
  /* USER CODE BEGIN DMA2_Stream3_IRQn 1 */

  /* USER CODE END DMA2_Stream3_IRQn 1 */
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi){

	old_spi_value = new_spi_value;
	new_spi_value = SPI_rx[0];

	if (new_spi_value>12){

		switch(old_spi_value){

		case 1:
			duty_cycle_channel_A = new_spi_value << 16;
			ack = new_spi_value;
			break;
		case 2:
			duty_cycle_channel_A = duty_cycle_channel_A | new_spi_value ;
			ack = new_spi_value;
			break;
		case 3:
			duty_cycle_channel_B = new_spi_value << 16;
			ack = new_spi_value;
			break;
		case 4:
			duty_cycle_channel_B = duty_cycle_channel_B | new_spi_value ;
			ack = new_spi_value;
			break;
		case 5:
			duty_cycle_LED = new_spi_value;
			ack = new_spi_value;
			break;
		case 6:
			ack = __HAL_TIM_GET_COMPARE(&htim5, TIM_CHANNEL_1) >> 16;
			break;
		case 7:
			ack = __HAL_TIM_GET_COMPARE(&htim5, TIM_CHANNEL_1) & 0x00FF;
			break;
		case 8:
			ack =   __HAL_TIM_GET_COMPARE(&htim5, TIM_CHANNEL_4) >> 16;
			break;
		case 9:
			ack = __HAL_TIM_GET_COMPARE(&htim5, TIM_CHANNEL_4) & 0x00FF;
			break;
		case 10:
			ack = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1);
			break;
		case 11:
			enable_channels = new_spi_value;
		  enable_channel_A = enable_channels & 1;
		  enable_channel_B = enable_channels>>1 & 1;

			ack = new_spi_value;
			break;
		case 12:
			ack = enable_channels;
			break;

		}


	}

		char buffer[32];
			snprintf(buffer, 32, "%d", SPI_rx[0]);
			debugPrintln(&huart2, buffer);
/*
			snprintf(buffer, 32, "%d", SPI_rx[1]);
			debugPrintln(&huart2, buffer);

			snprintf(buffer, 32, "%d", SPI_rx[2]);
			debugPrintln(&huart2, buffer);

			snprintf(buffer, 32, "%d", SPI_rx[3]);
			debugPrintln(&huart2, buffer);

			snprintf(buffer, 32, "%d", SPI_rx[4]);
			debugPrintln(&huart2, buffer);*/

	 /* __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_cycle_LED);

	  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, duty_cycle_channel_A);
	  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, duty_cycle_channel_B);

	  if (enable_channel_A)
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
	  else
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);

	  if (enable_channel_B)
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
	  else
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
*/

			SPI_tx[0]=ack;

  //HAL_SPI_Transmit_DMA(&hspi1, (uint16_t*)&ack, 1);


			 HAL_SPI_Transmit_DMA(&hspi1, (uint16_t*)&SPI_tx, 1);

  HAL_SPI_Receive_DMA(&hspi1, (uint16_t*)&SPI_rx, 1);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){

		  HAL_SPI_Receive_DMA(&hspi1, (uint16_t*)&SPI_rx, 1);
	 HAL_SPI_Transmit_DMA(&hspi1, (uint16_t*)&SPI_tx, 1);
}

/* USER CODE BEGIN 1 */
void debugPrintln(UART_HandleTypeDef *huart, char _out[]){
 HAL_UART_Transmit(huart, (uint8_t *) _out, strlen(_out), 10);
 char newline[2] = "\r\n";
 HAL_UART_Transmit(huart, (uint8_t *) newline, 2, 10);
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
