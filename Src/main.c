
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
  * COPYRIGHT(c) 2020 STMicroelectronics
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
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

int sam[3] = {0},sample_ready = 0,rx_count =0;
uint8_t start_bit = 1;
uint8_t tx_bit=0,rx_bit = 0,was_down = 0;
uint8_t parity_bit=0;
uint8_t phy_tx_busy = 0;
uint8_t dll_to_phy_tx_bus_vaild =0;
uint8_t phy_to_dll_rx_bus_valid=0;
uint8_t dll_has_new_data = 0;
uint8_t phy_has_new_data = 0;
uint32_t idr_data = 0;
uint32_t odr_data = 0;
uint32_t* idr_adress = (uint32_t*)(GPIO_A + GPIO_IDR);
uint32_t* odr_adress = (uint32_t*)(GPIO_B + GPIO_ODR);
uint8_t interface_clock=0,tx_clock=0,rx_hand = 0;
uint8_t interface_old_clock =0,tx_old_clock =0;
uint8_t tx_rising_edge=0,rx_falling_edge=0,tx_falling_edge = 0;
uint8_t interface_rising_edge=0,interface_falling_edge = 0;
static int8_t tx_counter = 0, rx_counter = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void phy_layer(void);
void interface(void);
void dll_to_phy_interface(void);
void phy_tx(void);
void phy_rx(void);
void phy_to_dll_interface(void);
void read_phy_pins(void);
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
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim2);
	//HAL_TIM_Base_Start_IT(&htim3);
	//HAL_TIM_Base_Start_IT(&htim4);
	HAL_GPIO_WritePin(Phy_live_GPIO_Port,Phy_live_Pin,GPIO_PIN_SET);//Set Alive Pin
	HAL_GPIO_WritePin(Tx_data_GPIO_Port,Tx_data_Pin,GPIO_PIN_SET); //Send Idle
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
		//read_phy_pins();
	dll_to_phy_interface();
	phy_tx();
	phy_rx();
	phy_to_dll_interface();

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

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = presc_val-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = tx_period-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = presc_val-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = tx_period/divider-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Tx_data_GPIO_Port, Tx_data_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Bit_2_output_Pin|Bit_3_output_Pin|Bit_4_output_Pin|Bit_5_output_Pin 
                          |Bit_6_output_Pin|Bit_7_output_Pin|Interface_clock_Pin|Phy_live_Pin 
                          |Phy_tx_busy_Pin|Phy_to_dll_valid_Pin|Bit_0_output_Pin|Bit_1_output_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Tx_data_Pin */
  GPIO_InitStruct.Pin = Tx_data_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Tx_data_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Bit_0_input_Pin Bit_1_input_Pin Bit_2_input_Pin Bit_3_input_Pin 
                           Bit_4_input_Pin Bit_5_input_Pin Bit_6_input_Pin Bit_7_input_Pin */
  GPIO_InitStruct.Pin = Bit_0_input_Pin|Bit_1_input_Pin|Bit_2_input_Pin|Bit_3_input_Pin 
                          |Bit_4_input_Pin|Bit_5_input_Pin|Bit_6_input_Pin|Bit_7_input_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Rx_data_Pin Dll_to_phy_valid_Pin */
  GPIO_InitStruct.Pin = Rx_data_Pin|Dll_to_phy_valid_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Bit_2_output_Pin Bit_3_output_Pin Bit_4_output_Pin Bit_5_output_Pin 
                           Bit_6_output_Pin Bit_7_output_Pin Interface_clock_Pin Phy_live_Pin 
                           Phy_tx_busy_Pin Phy_to_dll_valid_Pin Bit_0_output_Pin Bit_1_output_Pin */
  GPIO_InitStruct.Pin = Bit_2_output_Pin|Bit_3_output_Pin|Bit_4_output_Pin|Bit_5_output_Pin 
                          |Bit_6_output_Pin|Bit_7_output_Pin|Interface_clock_Pin|Phy_live_Pin 
                          |Phy_tx_busy_Pin|Phy_to_dll_valid_Pin|Bit_0_output_Pin|Bit_1_output_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void phy_layer(void){

}

void dll_to_phy_interface(){

	if(interface_falling_edge){
		dll_to_phy_tx_bus_vaild=HAL_GPIO_ReadPin(Dll_to_phy_valid_GPIO_Port,Dll_to_phy_valid_Pin);
		phy_tx_busy=HAL_GPIO_ReadPin(Phy_tx_busy_GPIO_Port,Phy_tx_busy_Pin);	
		if(dll_to_phy_tx_bus_vaild==1&&phy_tx_busy==0)
		{
			idr_data = *idr_adress & 0xff;
			dll_has_new_data = valid;
		}
	}
}
void phy_tx(){
	uint8_t mask_tx=0x1;
	
	if(dll_has_new_data&&tx_rising_edge&&was_down){
		if(tx_counter==0){	//adding start and stop bits to the frame
			HAL_GPIO_WritePin(Phy_tx_busy_GPIO_Port,Phy_tx_busy_Pin,GPIO_PIN_SET);
			idr_data|=two_stop_bits;
			idr_data<<=one_start_bit;
		}
		if(tx_counter<frame_size)
		{
			if(tx_counter==parity_place)
				tx_bit=parity_bit;
			else{
				tx_bit=idr_data&mask_tx;
				parity_bit^=tx_bit;
			}       
			HAL_GPIO_WritePin(Tx_data_GPIO_Port,Tx_data_Pin,(GPIO_PinState)tx_bit);
			tx_counter++;
			idr_data>>=1;		//delete lsb
		}
		else{
			//reset local variables
			HAL_GPIO_WritePin(Phy_tx_busy_GPIO_Port,Phy_tx_busy_Pin,GPIO_PIN_RESET);
			dll_has_new_data = 0;
			tx_counter = 0;
			parity_bit = 0;
   }
		was_down = 0;
 }
	
 if(tx_falling_edge)
	 was_down = 1;
}

void phy_rx(){
	static uint32_t status=0,parity_check=0,start_bit_sampled = 0;
	
	if(status == start_bit_status&&!start_bit_sampled){							//if we have'nt sampled a start bit yet
		start_bit = HAL_GPIO_ReadPin(Rx_data_GPIO_Port,Rx_data_Pin);
		if(start_bit == 0)																						//if we got a start bit
		{
			HAL_TIM_Base_Start_IT(&htim4);
			odr_data = 0;
			rx_hand = 0;
			start_bit_sampled = 1;
		}
	}
  if(sample_ready){//&&start_bit_sampled){														//if we got 3 samples and start bit
		if(sam[0]+sam[1]+sam[2]>1)
			rx_bit=1;
		else
			rx_bit = 0;
			
		switch(status){
			case start_bit_status:{
				if(rx_bit==0)
					status = byte_status;
				break;
			}
			case byte_status:{	//byte sampling
				if(rx_counter < data_size){
					parity_check^=rx_bit;
					rx_bit<<=rx_counter;
					odr_data|=rx_bit;
					rx_counter++;
					}
				else  {	//parity sampling
					if(parity_check==rx_bit){
						status=2;
						rx_counter++;
					}	
					else//parity fail
						status = reset;
				}
				break;
			}
			case stop_bits_status:{	//stop bits sampling
				if(rx_bit==1){
					if(rx_counter == last_stop_bit)
					{
						phy_has_new_data=1;
						status = reset;
					}
					else
						rx_counter++;
				}
				else//stop bit fail
					status = reset;
			break;
			}
		}
		if(status == reset)
		{
			status=0;
			start_bit=1;
			rx_counter=0;
			parity_check = 0;
			start_bit_sampled = 0;
			HAL_TIM_Base_Stop_IT(&htim4);
		}
		sample_ready = 0;
	}
		
}
void phy_to_dll_interface(){
static uint32_t valid_raised =0;
	
	
	if(phy_has_new_data==1&&interface_rising_edge){
		if(!valid_raised)
			{
				odr_data = odr_data<<8;
				*odr_adress=*odr_adress & 0xFFFF00FF;
				*odr_adress|=odr_data;
				HAL_GPIO_WritePin(Phy_to_dll_valid_GPIO_Port,Phy_to_dll_valid_Pin,GPIO_PIN_SET);
				valid_raised = 1;
			}
			if(valid_raised==2)
				{
					HAL_GPIO_WritePin(Phy_to_dll_valid_GPIO_Port,Phy_to_dll_valid_Pin,GPIO_PIN_RESET);
					phy_to_dll_rx_bus_valid = 0;
					phy_has_new_data = 0;
					odr_data = 0;
					valid_raised = 0;
				}
		}
		if(interface_falling_edge&&valid_raised)
			valid_raised = 2;
				
}

void read_phy_pins(void){
		
	//update edges

	

}

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
