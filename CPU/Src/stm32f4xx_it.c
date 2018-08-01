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
extern UART_HandleTypeDef huart2;

#define maxevent 100 //we don't expect more events than this
#define FREQ 42000000                   // Clock frequency
#define MFRQ 40000000                   // Sanity check frequency value
unsigned long evttime [maxevent];
unsigned long time_between_events [1200];
unsigned long time_between_event=0;

static int pll_flag = 0;
int pll_pulse = 0;
long eventCount = 0;
int eventstack = 0;
int gps_ok = 0;         // Chip OK flag
int pps_recieved = 0;
long ppcnt = 0;     // PPS count

static unsigned char RxData[64];
static unsigned char RxIndex = 0;

extern uint32_t received;

extern uint16_t ADCReadings[2]; //ADC Readings
extern uint8_t SPIReadings[1]; //SPI Readings

extern DMA_HandleTypeDef hadc1;
extern uint8_t completed1;

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

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
* @brief This function handles TIM2 global interrupt.
*/
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
* @brief This function handles EXTI line[15:10] interrupts.
*/
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADCReadings, 2);

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
* @brief This function handles TIM5 global interrupt.
*/
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */

  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */

  /* USER CODE END TIM5_IRQn 1 */
}

/**
* @brief This function handles DMA2 stream0 global interrupt.
*/
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */
  // ADC
	char buffer[32];
	snprintf(buffer, 32, "%d", ADCReadings[0]);
	debugPrintln(&huart2, buffer);

	snprintf(buffer, 32, "%d", ADCReadings[1]);
	debugPrintln(&huart2, buffer);
	completed1 =1;
  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/**
* @brief This function handles DMA2 stream2 global interrupt.
*/
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_rx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
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

/* USER CODE BEGIN 1 */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *tim_handle)
{

	//At 82 mhz one tick is generated each 1/82000000 seconds so the counter reach 2^32-1=4 milliards in 52s
	// that is large enough

	/* RAY */
    // An array for storing the two most recent captured counter values.
    static unsigned long channel_2_ray_event_counters[2] = {0, 0};
    // An array index to identify the most recently captured counter value.
    static int channel_2_ray_event_counter_index = 0;
    static unsigned long channel_2_ray_event_current_counter=0;
    static unsigned long channel_2_ray_event_previous_counter=0;

    /* PPS */
    // An array for storing the two most recent captured counter values.
    static unsigned long channel_3_pps_event_counters[2] = {0, 0};
    // An array index to identify the most recently captured counter value.
    static int channel_3_pps_event_counter_index = 0;
    static unsigned long channel_3_pps_event_current_counter=0;
    static unsigned long channel_3_pps_event_previous_counter=0;

    static long delta_between_pps_and_ray = 0;

    if (tim_handle == &htim2)
    {

			if (tim_handle->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
			{
				//long count_pps = __HAL_TIM_GetCounter(tim_handle);
				unsigned long temp = __HAL_TIM_GetCounter(tim_handle);
				channel_2_ray_event_previous_counter = channel_2_ray_event_current_counter;
				channel_2_ray_event_current_counter = temp;

				//__HAL_TIM_SetCounter(tim_handle, 0);    //reset counter after input capture interrupt occurs


				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_2);

				//uint16_t this_tach_timer_value = HAL_TIM_ReadCapturedValue(tim_handle, TIM_CHANNEL_1);

				/* GET THE RAY EVENT PERIOD, time between each RAY EVENT */

				unsigned long channel_2_ray_event_period;

				/* Period Computation */
				if (channel_2_ray_event_current_counter > channel_2_ray_event_previous_counter){

					channel_2_ray_event_period = channel_2_ray_event_current_counter - channel_2_ray_event_previous_counter;
				}else{
					// it means counter has reset automatically, then compute difference with the end
					channel_2_ray_event_period =  (0xFFFFFFFF - channel_2_ray_event_previous_counter) + channel_2_ray_event_current_counter;
				}

				/* Delta Computation */
				if (channel_2_ray_event_current_counter > channel_3_pps_event_current_counter){

					delta_between_pps_and_ray = channel_2_ray_event_current_counter - channel_3_pps_event_current_counter;
				}else{
					// it means counter has reset automatically, then like in arduino reset counter or take only ray counter difference
					delta_between_pps_and_ray =  (0xFFFFFFFF - channel_3_pps_event_current_counter) + channel_2_ray_event_current_counter;
					//delta_between_pps_and_ray = channel_2_ray_event_current_counter;

				}


				//Time of each event
				//Current counter added to the last one
				  if (eventstack > 0){
					  evttime[eventstack] = channel_2_ray_event_period+evttime[eventstack-1];
				  }
				  else
				  {
					  evttime[eventstack] = channel_2_ray_event_period;
				  }


				  /*
				  if (eventCount > 1 && eventCount < 1100){

					   time_between_events[eventCount] = delta_between_pps_and_ray;

				    if (time_between_events[eventCount]>time_between_events[eventCount-1]){
				    	time_between_event= time_between_events[eventCount]-time_between_events[eventCount-1];

						/* DEBUGGING BEGIN */
					/*	char buffer[32];
						snprintf(buffer, 32, "%d", time_between_event);

						debugPrintln(&huart2, buffer);

				    }

				  /*
				     if (time_between_events[1]>time_between_events[2])
				     time_between_event2_and_event3= time_between_events[1]-time_between_events[2];
				    else
				     time_between_event2_and_event3= time_between_events[2]-time_between_events[1];

				     }
*/

				  eventstack++; //increment the event stack for this second
					  eventCount++;


						/* DEBUGGING BEGIN */
						char buffer[32];
						snprintf(buffer, 32, "%d", delta_between_pps_and_ray);

						//char data[33+32]="RAY interrupt: period is about: ";

						//strcat(data, buffer);

						debugPrintln(&huart2, buffer);

				//char buffer2[32];
				//snprintf(buffer2, 32, "%d", eventCount);

				//char data[33+32]="RAY interrupt: counter is about: ";

				//strcat(data, buffer);

				//HAL_UART_Transmit(&huart2, buffer, 32, 0xFFFFFFFF);
				//char newline[2] = "\r\n";
				 //HAL_UART_Transmit(&huart2, (uint8_t *) newline, 2, 10);
				//debugPrintln(&huart2, buffer2);


				//if (eventCount%3==0)
				//	debugPrintln(&huart2, "3 events detected");
				//HAL_UART_Transmit(&huart2, data, 65, 0xFFFFFFFF);
				/* DEBUGGING END */
			}
			// PPS
			// PPS is used as time reference, it is coming from the GPS at 1 Hz
			// At each PPS event, we reset other timers and we measure time
			// Between PPS and Event

			if (tim_handle->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
			{
				ppcnt++;                                // PPS count
				gps_ok = 1;                          // Its OK because we got a PPS
				pll_flag = 1;                        // Inhibit PLL, dont take over PPS arrived
				pps_recieved = 1;

				// GET PPS value, must be ~82 000 000 if no prescaler and signal frequency f 1Hz
				//long count_pps = __HAL_TIM_GetCounter(tim_handle);
				unsigned long temp = __HAL_TIM_GetCounter(tim_handle);
				channel_3_pps_event_previous_counter = channel_3_pps_event_current_counter;
				channel_3_pps_event_current_counter = temp;
				//if (channel_3_pps_event_current_counter < MFRQ)	// Sanity check against noise
				//	channel_3_pps_event_current_counter = FREQ;	// Use nominal value

				//__HAL_TIM_SetCounter(&htim5, channel_3_pps_event_current_counter);	// Set the PLL count to what we just counted


				//__HAL_TIM_SetCounter(tim_handle, 0);    //reset counter after input capture interrupt occurs


				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_1);

				//uint16_t this_tach_timer_value = HAL_TIM_ReadCapturedValue(tim_handle, TIM_CHANNEL_1);

				/* GET THE PPS PERIOD, time between each PPS */

				unsigned long channel_3_pps_event_period;

				if (channel_3_pps_event_current_counter > channel_3_pps_event_previous_counter){

					channel_3_pps_event_period = channel_3_pps_event_current_counter - channel_3_pps_event_previous_counter;
				}else{
					// it means counter has reset automatically, then compute difference with the end
					channel_3_pps_event_period =  (0xFFFFFFFF - channel_3_pps_event_previous_counter) + channel_3_pps_event_current_counter;
				}

				/* RESET RAY/PPS TIMERS */
				//__HAL_TIM_SetCounter(&htim2, 0);
				//channel_2_ray_event_current_counter = 0;
				// Set PLL period to PPS period
				__HAL_TIM_SET_AUTORELOAD(&htim5, channel_3_pps_event_period);
				channel_2_ray_event_current_counter=channel_3_pps_event_current_counter;
				//channel_2_ray_event_counters[channel_2_ray_event_counter_index] = channel_3_pps_event_current_counter;
				//pll_event_counters[pll_event_counter_index] = channel_3_pps_event_current_counter;


				/* DEBUGGING BEGIN */
				/*char buffer[32];
				snprintf(buffer, 32, "%d", channel_3_pps_event_period);

				char data[33+32]="PPS interrupt: period is about: ";

				strcat(data, buffer);

				debugPrintln(&huart2, data);

				/* DEBUGGING BEGIN */
				//char buffer[32];
				//snprintf(buffer, 32, "%d", channel_3_pps_event_period);

				//char data[33+32]="PPS interrupt: counter is about: ";

				//strcat(data, buffer);

				//HAL_UART_Transmit(&huart2, data, 65, 0xFFFFFFFF);
				/* DEBUGGING END */

			}


    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *tim_handle)
{

    /* PLL */
    if (tim_handle == &htim5)
    {



		//debugPrintln(&huart2, data_out1);

		if (pll_flag == 0) {
			/* RESET OTHER TIMERS */
			//__HAL_TIM_SetCounter(&htim2, 0);
			//__HAL_TIM_SetCounter(&htim5, 0);

			pll_pulse = 1;
			gps_ok = 0;
			ppcnt++;


			// No need to compute period because we already fixed it in main, it interrupt
			// once counter reach the fixed period

			/* RESET OTHER TIMERS */
			//__HAL_TIM_SetCounter(&htim2, 0);
			//__HAL_TIM_SetCounter(&htim5, 0);
			// Or maybe resynchronize counter ray and pll like that
			//channel_2_ray_event_counters[channel_2_ray_event_counter_index] = pll_event_current_counter;


			/* DEBUGGING BEGIN */

			//char data[33]="PLL interrupt ";


			//HAL_UART_Transmit(&huart2, data, 33, 0xFFFFFFFF);
			/* DEBUGGING END */


		}
		pll_flag = 0;
    }
}

void debugPrintln(UART_HandleTypeDef *huart, char _out[]){
 HAL_UART_Transmit(huart, (uint8_t *) _out, strlen(_out), 10);
 char newline[2] = "\r\n";
 HAL_UART_Transmit(huart, (uint8_t *) newline, 2, 10);
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
