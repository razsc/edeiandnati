/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
#define tx_period 200
#define presc_val 10
#define divider 3

#define Tx_data_Pin GPIO_PIN_15
#define Tx_data_GPIO_Port GPIOC
#define Bit_0_input_Pin GPIO_PIN_0
#define Bit_0_input_GPIO_Port GPIOA
#define Bit_1_input_Pin GPIO_PIN_1
#define Bit_1_input_GPIO_Port GPIOA
#define Bit_2_input_Pin GPIO_PIN_2
#define Bit_2_input_GPIO_Port GPIOA
#define Bit_3_input_Pin GPIO_PIN_3
#define Bit_3_input_GPIO_Port GPIOA
#define Bit_4_input_Pin GPIO_PIN_4
#define Bit_4_input_GPIO_Port GPIOA
#define Bit_5_input_Pin GPIO_PIN_5
#define Bit_5_input_GPIO_Port GPIOA
#define Bit_6_input_Pin GPIO_PIN_6
#define Bit_6_input_GPIO_Port GPIOA
#define Bit_7_input_Pin GPIO_PIN_7
#define Bit_7_input_GPIO_Port GPIOA
#define Rx_data_Pin GPIO_PIN_0
#define Rx_data_GPIO_Port GPIOB
#define Bit_2_output_Pin GPIO_PIN_10
#define Bit_2_output_GPIO_Port GPIOB
#define Bit_3_output_Pin GPIO_PIN_11
#define Bit_3_output_GPIO_Port GPIOB
#define Bit_4_output_Pin GPIO_PIN_12
#define Bit_4_output_GPIO_Port GPIOB
#define Bit_5_output_Pin GPIO_PIN_13
#define Bit_5_output_GPIO_Port GPIOB
#define Bit_6_output_Pin GPIO_PIN_14
#define Bit_6_output_GPIO_Port GPIOB
#define Bit_7_output_Pin GPIO_PIN_15
#define Bit_7_output_GPIO_Port GPIOB
#define Dll_to_phy_valid_Pin GPIO_PIN_3
#define Dll_to_phy_valid_GPIO_Port GPIOB
#define Interface_clock_Pin GPIO_PIN_4
#define Interface_clock_GPIO_Port GPIOB
#define Phy_live_Pin GPIO_PIN_5
#define Phy_live_GPIO_Port GPIOB
#define Phy_tx_busy_Pin GPIO_PIN_6
#define Phy_tx_busy_GPIO_Port GPIOB
#define Phy_to_dll_valid_Pin GPIO_PIN_7
#define Phy_to_dll_valid_GPIO_Port GPIOB
#define Bit_0_output_Pin GPIO_PIN_8
#define Bit_0_output_GPIO_Port GPIOB
#define Bit_1_output_Pin GPIO_PIN_9
#define Bit_1_output_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
//Gpio ports:
#define GPIO_A							0x40010800
#define GPIO_B							0x40010C00
#define GPIO_C							0x40011000

//Gpio Offset:
#define	GPIO_CRL						0x00
#define	GPIO_CRH						0x04
#define	GPIO_IDR						0x08
#define	GPIO_ODR						0x0C
#define	GPIO_BSRR						0x10

//User defines:
#define two_stop_bits				0x600
#define	one_start_bit				1
#define	frame_size					12
#define parity_place				9
#define valid								1
#define samples_num					7
#define	data_size						8
#define last_stop_bit				10
#define start_bit_status		0
#define byte_status					1
#define stop_bits_status		2
#define	reset								4

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
