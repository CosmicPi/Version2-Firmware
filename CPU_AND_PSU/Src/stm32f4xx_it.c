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

extern uint16_t ADC_SIPM_VALUES[2]; //ADC Readings
extern uint16_t ADC_FEEDBACK_VOLTAGE_VALUES[2]; //ADC Readings

#define maxevent 100 //we don't expect more events than this
#define FREQ 42000000                   // Clock frequency
#define MFRQ 40000000                   // Sanity check frequency value
unsigned long evttime [maxevent];
unsigned long time_between_events [1200];
unsigned long time_between_event=0;

static unsigned long channel_3_pps_event_period;
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

extern uint8_t adc_regular_completed; //ADC Readings
extern uint8_t spi_transmit_receive;

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_spi2_rx;
extern DMA_HandleTypeDef hdma_spi2_tx;
extern TIM_HandleTypeDef htim2;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

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
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
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
* @brief This function handles DMA1 stream3 global interrupt.
*/
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */

  /* USER CODE END DMA1_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi2_rx);
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

  /* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
* @brief This function handles DMA1 stream4 global interrupt.
*/
void DMA1_Stream4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream4_IRQn 0 */

  /* USER CODE END DMA1_Stream4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi2_tx);
  /* USER CODE BEGIN DMA1_Stream4_IRQn 1 */

  /* USER CODE END DMA1_Stream4_IRQn 1 */
}

/**
* @brief This function handles ADC1 global interrupt.
*/
void ADC_IRQHandler(void)
{
  /* USER CODE BEGIN ADC_IRQn 0 */

  /* USER CODE END ADC_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC_IRQn 1 */

  /* USER CODE END ADC_IRQn 1 */
}

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
* @brief This function handles DMA2 stream0 global interrupt.
*/
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */
	adc_regular_completed=1;

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/* USER CODE BEGIN 1 */

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){

	spi_transmit_receive = 1;

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{

/*
	ADC_SIPM_VALUES[0] = HAL_ADC_GetValue(hadc);
	ADC_SIPM_VALUES[1] = HAL_ADC_GetValue(hadc);


  HAL_ADC_Start_IT(&hadc1);
  */

/*
	char buffer[32];
	snprintf(buffer, 32, "Regular Channel 1 : %d", ADC_SIPM_VALUES[0]);
	debugPrintln(&huart2, buffer);

	snprintf(buffer, 32, "Regular Channel 2 : %d", ADC_SIPM_VALUES[1]);
	debugPrintln(&huart2, buffer);

	debugPrintln(&huart2, "\n");

*/
	//ADCReadings[0] = ADC_DMA_buffer[0];
	//ADCReadings[1] = ADC_DMA_buffer[1];
	//adc_regular_completed=1;


}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{

	ADC_FEEDBACK_VOLTAGE_VALUES[0] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
	ADC_FEEDBACK_VOLTAGE_VALUES[1] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);

/*
		char buffer[32];
		snprintf(buffer, 32, "Injected Channel 1 : %d", ADC_FEEDBACK_VOLTAGE_VALUES[0]);
		debugPrintln(&huart2, buffer);

		snprintf(buffer, 32, "Injected Channel 2 : %d", ADC_FEEDBACK_VOLTAGE_VALUES[1]);
		debugPrintln(&huart2, buffer);

		debugPrintln(&huart2, "\n");
*/

	HAL_ADCEx_InjectedStart_IT(&hadc1);


	//ADCReadings[0] = ADC_DMA_buffer[0];
	//ADCReadings[1] = ADC_DMA_buffer[1];


}
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

			if (tim_handle->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
			{

				//long count_pps = __HAL_TIM_GetCounter(tim_handle);
				unsigned long temp = __HAL_TIM_GetCounter(tim_handle);
				channel_2_ray_event_previous_counter = channel_2_ray_event_current_counter;
				channel_2_ray_event_current_counter = temp;

				//__HAL_TIM_SetCounter(tim_handle, 0);    //reset counter after input capture interrupt occurs

				// LED event notification
				//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_2);

				// FLAG0 event notification
				//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);

				HAL_GPIO_TogglePin(GPIOC, DEBUG_RAY_Pin);

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

						//debugPrintln(&huart2, buffer);

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


				//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_1);

				HAL_GPIO_TogglePin(GPIOA, DEBUG_PPS_Pin);

				//uint16_t this_tach_timer_value = HAL_TIM_ReadCapturedValue(tim_handle, TIM_CHANNEL_1);

				/* GET THE PPS PERIOD, time between each PPS */

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
				//__HAL_TIM_SET_AUTORELOAD(&htim5, channel_3_pps_event_period);
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

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *tim_handle)
{

    uint64_t current_compare_value = 0;
    uint64_t pps_period = channel_3_pps_event_period;

    uint32_t future_compare_value = 0;
    uint64_t temp_compare_value = 0;

        if (tim_handle == &htim2){


            if (tim_handle->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
            {

                if (pll_flag == 0) {

                    pll_pulse = 1;
                    gps_ok = 0;
                    ppcnt++;

                    HAL_GPIO_TogglePin(GPIOA, DEBUG_PLL_Pin);
                 }

            current_compare_value = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2);

            temp_compare_value = current_compare_value + pps_period + T_100_MS;

            if ( temp_compare_value > 0xFFFFFFFF){
            	temp_compare_value = temp_compare_value - 0xFFFFFFFF;
            	future_compare_value = temp_compare_value & 0x00000000FFFFFFFF;
            }else
                future_compare_value = temp_compare_value;


            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, future_compare_value);

            pll_flag = 0;

        }

        if (tim_handle->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
            {

            HAL_GPIO_TogglePin(GPIOA, DEBUG_PSU_Pin);
    		HAL_GPIO_TogglePin(GPIOB, FAKE_RAY_Pin);
    		HAL_GPIO_TogglePin(GPIOB, FAKE_PPS_Pin);
            //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
            //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

        // Start ADC for FEEDBACK VOLTAGE - The injected group of channels (high priority)

    		//HAL_ADCEx_InjectedStart_DMA(&hadc1);

    	HAL_ADCEx_InjectedStart_IT(&hadc1);


        // -- Enables ADC DMA request
    	//HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)ADC_FEEDBACK_VOLTAGE_VALUES, 2);
    	//HAL_ADC_Stop_DMA(&hadc1);
        //HAL_ADCEx_InjectedStart_IT(&hadc1);
        //HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADC_FEEDBACK_VOLTAGE_VALUES, 2);

        current_compare_value = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_4);

		temp_compare_value = current_compare_value + T_5_MINUTES;

		if ( temp_compare_value > 0xFFFFFFFF){
        	temp_compare_value = temp_compare_value - 0xFFFFFFFF;
        	future_compare_value = temp_compare_value & 0x00000000FFFFFFFF;
		}else
			future_compare_value = temp_compare_value;


		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, future_compare_value);

		/*char buffer[32];
		snprintf(buffer, 32, "pps period %d", pps_period);
		debugPrintln(&huart2, buffer);*/


            }

    }
}



/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
