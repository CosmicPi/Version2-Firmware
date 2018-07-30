
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
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint16_t SPI_tx[1]; //ADC Readings
uint16_t SPI_rx[1]; //ADC Readings

//uint16_t duty_cycle_LED = 50;

uint16_t duty_cycle_LED = 0;

//uint32_t duty_cycle_channel_A = 56;
uint32_t duty_cycle_channel_A = 0;

//uint32_t duty_cycle_channel_B = 56;
uint32_t duty_cycle_channel_B = 0;

uint8_t enable_channel_A = 0;
uint8_t enable_channel_B = 0;

uint8_t enable_channels = 0;

uint8_t spi_transmit_receive;

uint16_t new_spi_value = 0;
uint16_t old_spi_value = 0;
uint16_t ack = 0;
uint8_t command = 0;
uint16_t rx = 0;
uint16_t number_of_values_received = 0;
uint8_t state = 0;
uint16_t rx_values[2];
uint16_t value6=0;
uint16_t value7=0;
uint16_t value8=0;
uint16_t value9=0;
uint16_t value10=0;

uint16_t ADCReadings[2]; //ADC Readings
uint16_t ADC_DMA_buffer[2]; //ADC Readings

uint8_t adc_completed=0; //ADC Readings

char buffer[32];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM5_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */


  /********** TIM 1 PWM 1 CONF****************/
  // Set the period
  __HAL_TIM_SET_AUTORELOAD(&htim1, 50000);
  // Set the prescaler
  __HAL_TIM_SET_PRESCALER(&htim1, 168-1);
  // Set the duty cycle
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_cycle_LED);

  if (HAL_TIM_Base_Start(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1) != HAL_OK)
  {
	_Error_Handler(__FILE__, __LINE__);
  }

  /********** TIM 5 PWM 1 CONF****************/

  //duty_cycle_channel_A, duty_cycle_channel_B = 56;//30 % of 280

  // Set the period
  __HAL_TIM_SET_AUTORELOAD(&htim5, 280);
  // Set the prescaler
  __HAL_TIM_SET_PRESCALER(&htim5, 0);
  // Set the duty cycle
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, duty_cycle_channel_A);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, duty_cycle_channel_B);

  if (HAL_TIM_Base_Start(&htim5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1) != HAL_OK)
  {
	_Error_Handler(__FILE__, __LINE__);
  }


  if (HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_4) != HAL_OK)
  {
	_Error_Handler(__FILE__, __LINE__);
  }

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);

  SPI_tx[0] = 255;
  //  HAL_SPI_Receive_DMA(&hspi1, (uint16_t*)&SPI_rx, 1);

	//while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY){}
  //HAL_SPI_Receive_DMA(&hspi1,(uint8_t*)&SPI_rx, 1);

   if(HAL_SPI_TransmitReceive_DMA(&hspi1, (uint16_t*)&SPI_tx, (uint16_t*)&SPI_rx, 1) != HAL_OK)
   {
       /* Transfer error in transmission process */
       Error_Handler();
   }

   while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
    {
    }

  //HAL_SPI_Transmit_DMA(&hspi1, (uint16_t*)&SPI_tx, 1);


   HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADCReadings, 2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	  if (spi_transmit_receive == 1 ){


		  spi_transmit_receive = 0;

			old_spi_value = new_spi_value;
			new_spi_value = SPI_rx[0];
			rx_values[state]=new_spi_value;

					switch(state){

					case 0:

						//valid command
						if (new_spi_value<15 && new_spi_value>0){
							command = new_spi_value;
							ack = command;
						}else{
							ack = new_spi_value;

						}
						state = 1;

						break;

					case 1:



						if(command == 1){

									if (number_of_values_received==1){
									//duty_cycle_channel_A = new_spi_value << 16;
									duty_cycle_channel_A = rx_values[state^1];
									duty_cycle_channel_A = duty_cycle_channel_A << 16;
									}


									ack = new_spi_value;
									//value6 = rx_values[state^1];
								}else if(command==2){

									//duty_cycle_channel_A = duty_cycle_channel_A | new_spi_value ;

									if (number_of_values_received==1){
									duty_cycle_channel_A = duty_cycle_channel_A | rx_values[state^1] ;



									  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, duty_cycle_channel_A);
									}

									ack = new_spi_value;
									value7 = rx_values[state^1];
								}else if(command==3){

									//duty_cycle_channel_B = new_spi_value << 16;
									if (number_of_values_received==1){
									duty_cycle_channel_B = rx_values[state^1];
									duty_cycle_channel_B = duty_cycle_channel_B << 16;
									}
									ack = new_spi_value;

									//value8 = rx_values[state^1];
								}else if(command==4){
									//duty_cycle_channel_B = duty_cycle_channel_B | new_spi_value ;

									if (number_of_values_received==1){
									duty_cycle_channel_B = duty_cycle_channel_B | rx_values[state^1] ;


								  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, duty_cycle_channel_B);
									}
									ack = new_spi_value;
									value9 = rx_values[state^1];

								}else if(command==5){
									//duty_cycle_LED = new_spi_value;
									if (number_of_values_received==1){
									duty_cycle_LED = rx_values[state^1];

									 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_cycle_LED);
									}
									ack = new_spi_value;
									//value10 = rx_values[state^1];
								}else if(command==6){

									ack = __HAL_TIM_GET_COMPARE(&htim5, TIM_CHANNEL_1) >> 16;
									//ack= value6;
								}else if(command==7){
									ack = __HAL_TIM_GET_COMPARE(&htim5, TIM_CHANNEL_1) & 0x0000FFFF;
									//ack = value7;
								}else if(command==8){
									ack = __HAL_TIM_GET_COMPARE(&htim5, TIM_CHANNEL_4) >> 16;
									//ack = value8;
								}else if(command==9){
									ack = __HAL_TIM_GET_COMPARE(&htim5, TIM_CHANNEL_4) & 0x0000FFFF;
									//ack= value9;
								}else if(command==10){
									ack = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1);
									//ack = value10;
								}else if(command==11){
									//enable_channels = new_spi_value;
									if (number_of_values_received==1){
										enable_channels = rx_values[state^1];

										enable_channel_A = enable_channels & 1;
								  	  enable_channel_B = enable_channels>>1 & 1;

								  	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, enable_channel_A);

								  	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, enable_channel_B);

									}

									ack = new_spi_value;
								}else if(command==12){
									ack = enable_channels;
								}else if(command==13){
									ack = ADCReadings[0];
								}else if(command==14){
									ack = ADCReadings[1];
								}


								state = 0;

								if(number_of_values_received==1)
									number_of_values_received = 0;
								else
									number_of_values_received ++;

					default:
						break;


					}



					/*snprintf(buffer, 32, "%d", SPI_rx[0]);
					debugPrintln(&huart2, buffer);


					snprintf(buffer, 32, "%d", state);
					debugPrintln(&huart2, buffer);

					snprintf(buffer, 32, "ack %d", ack);
					debugPrintln(&huart2, buffer);

					snprintf(buffer, 32, "new spi %d", new_spi_value);
					debugPrintln(&huart2, buffer);

					snprintf(buffer, 32, "old spi %d", new_spi_value);
					debugPrintln(&huart2, buffer);

					snprintf(buffer, 32, "ack %d", ack);
					debugPrintln(&huart2, buffer);

					snprintf(buffer, 32, "cmd %d", command);
					debugPrintln(&huart2, buffer);

					snprintf(buffer, 32, "state %d", state);
					debugPrintln(&huart2, buffer);

					snprintf(buffer, 32, "rx value %d", rx_values[state]);
					debugPrintln(&huart2, buffer);
*/


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

					//if(new_spi_value !=255)
						rx^=1;


						debugPrintln(&huart2, "");
					  //HAL_SPI_Transmit_DMA(&hspi1, (uint16_t*)&SPI_tx, 1);

						  while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
						{
						}

					  if(HAL_SPI_TransmitReceive_DMA(&hspi1, (uint16_t*)&SPI_tx, (uint16_t*)&SPI_rx, 1) != HAL_OK)
					   {
						   /* Transfer error in transmission process */
						   Error_Handler();
					   }


		  //HAL_SPI_Receive_DMA(&hspi1, (uint16_t*)&SPI_rx, 1);

	  }

	  //HAL_Delay(500);
	  //user_pwm_setvalue((53*perc)/100);
/*	  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, pulse);
	  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, pulse);

	  pulse+=3;
	  HAL_Delay(15000);

	  if (pulse == 168)
		  pulse=56;
	  /*  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
	    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);*/
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

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM5 init function */
static void MX_TIM5_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim5);

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC1 PC2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
  while(1)
  {
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
