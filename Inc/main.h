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
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define EXT_SENSOR_INT_Pin GPIO_PIN_13
#define EXT_SENSOR_INT_GPIO_Port GPIOC
#define EXT_SENSOR_INT_EXTI_IRQn EXTI15_10_IRQn
#define EXT_SENSOR_RST_Pin GPIO_PIN_14
#define EXT_SENSOR_RST_GPIO_Port GPIOC
#define IMU_IRQ_Pin GPIO_PIN_15
#define IMU_IRQ_GPIO_Port GPIOC
#define IMU_IRQ_EXTI_IRQn EXTI15_10_IRQn
#define PRIM_BATT_VOLTAGE_Pin GPIO_PIN_0
#define PRIM_BATT_VOLTAGE_GPIO_Port GPIOA
#define SEC_BATT_VOLTAGE_Pin GPIO_PIN_0
#define SEC_BATT_VOLTAGE_GPIO_Port GPIOB
#define VSYS_CURRENT_Pin GPIO_PIN_1
#define VSYS_CURRENT_GPIO_Port GPIOB
#define MOT1_PWM_Pin GPIO_PIN_9
#define MOT1_PWM_GPIO_Port GPIOE
#define MOT2_PWM_Pin GPIO_PIN_11
#define MOT2_PWM_GPIO_Port GPIOE
#define MOT3_PWM_Pin GPIO_PIN_13
#define MOT3_PWM_GPIO_Port GPIOE
#define MOT4_PWM_Pin GPIO_PIN_14
#define MOT4_PWM_GPIO_Port GPIOE
#define PMU_STAT_Pin GPIO_PIN_10
#define PMU_STAT_GPIO_Port GPIOD
#define PMU_IRQ_Pin GPIO_PIN_11
#define PMU_IRQ_GPIO_Port GPIOD
#define PMU_IRQ_EXTI_IRQn EXTI15_10_IRQn
#define SBUS_UART_TX_Pin GPIO_PIN_6
#define SBUS_UART_TX_GPIO_Port GPIOC
#define SBUS_UART_RX_Pin GPIO_PIN_7
#define SBUS_UART_RX_GPIO_Port GPIOC
#define BUTTON_1_Pin GPIO_PIN_6
#define BUTTON_1_GPIO_Port GPIOD
#define BUTTON_2_Pin GPIO_PIN_7
#define BUTTON_2_GPIO_Port GPIOD
#define LED1_Pin GPIO_PIN_5
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_6
#define LED2_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_7
#define LED3_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

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
