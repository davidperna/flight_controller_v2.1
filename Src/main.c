
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
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

#include "string.h"
#include "stdbool.h"
#include <stdlib.h>
#include "MPU9250.h"

#define M_PI 3.1415926
#define EPSILON 0.0000001

#define NUM_ADC_CHANNELS 3
#define DEBUG_TX_BUFFER_SIZE 50
#define DEBUG_RX_BUFFER_SIZE 50
#define SBUS_RX_BUFFER_SIZE 50

// Motor timer channels
// FL = Front-left
// RR = Rear-right
#define MOTOR_RL_TIM_CHANNEL TIM_CHANNEL_1
#define MOTOR_FL_TIM_CHANNEL TIM_CHANNEL_2
#define MOTOR_FR_TIM_CHANNEL TIM_CHANNEL_3
#define MOTOR_RR_TIM_CHANNEL TIM_CHANNEL_4
#define MOTOR_TIMER_INSTANCE &htim1

#define MOTOR_MIN_PULSE 1000 // In microseconds
#define MOTOR_MAX_PULSE 2000 // In microseconds
#define MOTOR_IDLE_SPEED 7 // In %

#define __ADC_RAW_TO_VOLTS(adc_val) ((double)adc_val)*0.0007326 //1LSB = 3.0V/4095
#define __PRIM_BATT_VOLTAGE(adc_volts) (1.500 + ((double)adc_volts)*0.3667)*5.5946
#define __SEC_BATT_VOLTAGE(adc_volts) (1.500 + ((double)adc_volts)*0.3667)*1.8667
#define __SYS_CURRENT(adc_volts) (((double)adc_volts)/200.0)/0.01 //200V/V gain and 0R01 sense resistor

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

typedef enum
{
	GET_PRIM_BATTERY_V = 0,
	GET_SEC_BATTERY_V = 1,
	GET_SYS_CURRENT_I = 2
} debug_command_t;

char buffer[1024]; // to store data
uint32_t ADC_BUFF[NUM_ADC_CHANNELS];
bool led3_state = 0;
uint32_t sbus_packets = 0;

__IO uint8_t sbus_rx_buffer[SBUS_RX_BUFFER_SIZE];

__IO uint8_t debug_rx_buffer[DEBUG_RX_BUFFER_SIZE];
__IO uint8_t debug_tx_buffer[DEBUG_TX_BUFFER_SIZE];

__IO uint16_t receiver_inputs[17];
__IO uint8_t imu_data_buffer[14];

__IO bool new_imu_data_flag = false;

__IO bool i2c1_mem_rx_irq_flag = false;
__IO bool i2c1_mem_tx_irq_flag = false;

__IO bool flg_new_debug_message = false;
__IO bool flg_new_receiver_input = false;

#define DEBUG_OVER_WIRELESS

#ifdef DEBUG_OVER_WIRELESS
#define SERIAL_DEBUG_UART_INSTANCE &huart5
#endif

#ifndef DEBUG_OVER_WIRELESS
#define SERIAL_DEBUG_UART_INSTANCE &huart1
#endif

#define SBUS_UART_INSTANCE &huart6

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

// Debug print functions
void send_debug_string_blocking(UART_HandleTypeDef *huart, char *string);
void send_debug_string_dma(UART_HandleTypeDef *huart, char *string);

// Init functions
void ADC_Init(void);
void sbus_init(void);

void config_LEDs_for_GPIO(void);
void config_LEDs_for_PWM(void);

// IMU helper functions
void start_imu_dma_transfer(void);

// Receiver helper functions
void update_receiver_inputs(void);

// DMA helper functions
void flush_sbus_dma_buffer(void);
void flush_debug_dma_buffer(void);

// ADC callbacks
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);

// UART transfer callbacks
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart);

// UART interrupt callbacks
void USART1_IRQCallback(void);
void UART5_IRQCallback(void);
void UART6_IRQCallback(void);

// I2C callbacks
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);

// GPIO callbacks
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

// Microsecond timer functions
void Microseconds_Timer_Init(void);
void reset_micros(void);
uint32_t get_micros(void);

// Motor helper functions
void set_motor_speed(uint32_t motor_Channel, float speed_percent);
void idle_all_motors();
void stop_all_motors();
void stop_motor(uint32_t motor_Channel);

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
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_UART5_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_SPI4_Init();
  MX_USART6_UART_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  ADC_Init();
  sbus_init();
  Microseconds_Timer_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  config_LEDs_for_GPIO();

  // Enable idle line interrupts for serial debug
  __HAL_UART_ENABLE_IT(SERIAL_DEBUG_UART_INSTANCE, UART_IT_IDLE);

  // Enable idle line interrupt for SBUS
  __HAL_UART_ENABLE_IT(SBUS_UART_INSTANCE, UART_IT_IDLE);

  // Start DMA for serial debug
  HAL_UART_Receive_DMA(SERIAL_DEBUG_UART_INSTANCE, debug_rx_buffer, sizeof(debug_rx_buffer));

  float prim_batt_volts = __PRIM_BATT_VOLTAGE(__ADC_RAW_TO_VOLTS(ADC_BUFF[0]));
  float sec_batt_volts = __SEC_BATT_VOLTAGE(__ADC_RAW_TO_VOLTS(ADC_BUFF[1]));
  float sys_current_amps = __SYS_CURRENT(__ADC_RAW_TO_VOLTS(ADC_BUFF[2]));

  sprintf (buffer, "Primary battery: %0.3f V\n", prim_batt_volts);
  send_debug_string_blocking(SERIAL_DEBUG_UART_INSTANCE, buffer);

  sprintf (buffer, "Secondary battery: %0.3f V\n", sec_batt_volts);
  send_debug_string_blocking(SERIAL_DEBUG_UART_INSTANCE, buffer);

  sprintf (buffer, "System current: %0.3f A\n", sys_current_amps);
  send_debug_string_blocking(SERIAL_DEBUG_UART_INSTANCE, buffer);

  uint8_t i2c1_rx_buffer[1];

	// Read BMP280 ID register
	HAL_I2C_Mem_Read_DMA(&hi2c1, 0x76<<1, 0xD0, 1 , i2c1_rx_buffer, 1);
	i2c1_mem_rx_irq_flag = false;
	while(!i2c1_mem_rx_irq_flag);
	sprintf (buffer, "BMP280 ID: %X\n", i2c1_rx_buffer[0]);
	send_debug_string_blocking(SERIAL_DEBUG_UART_INSTANCE, buffer);


	uint8_t reg_val = MPU9250_PRIV_whoAmI();
	sprintf (buffer, "MPU9250 ID: %X\n", reg_val);
	send_debug_string_blocking(SERIAL_DEBUG_UART_INSTANCE, buffer);

	MPU9250_begin();
	MPU9250_readSensor();

	while(1)
	{
		MPU9250_readSensor();
		sprintf (buffer, "Accel: %0.3f %0.3f %0.3f Gyro: %0.3f %0.3f %0.3f Mag: %0.3f %0.3f %0.3f Temp: %0.3f\n",
		MPU9250_getAccelX_mss(),
		MPU9250_getAccelY_mss(),
		MPU9250_getAccelZ_mss(),
		MPU9250_getGyroX_rads(),
		MPU9250_getGyroY_rads(),
		MPU9250_getGyroZ_rads(),
		MPU9250_getMagX_uT(),
		MPU9250_getMagY_uT(),
		MPU9250_getMagZ_uT(),
		MPU9250_getTemperature_C());

		send_debug_string_blocking(SERIAL_DEBUG_UART_INSTANCE, buffer);

		HAL_Delay(100);
	}
	uint32_t loop_counter = 0;

  while (1)
  {
	reset_micros();

	// Update battery voltage
	prim_batt_volts = __PRIM_BATT_VOLTAGE(__ADC_RAW_TO_VOLTS(ADC_BUFF[0]));
	sec_batt_volts = __SEC_BATT_VOLTAGE(__ADC_RAW_TO_VOLTS(ADC_BUFF[1]));
	sys_current_amps = __SYS_CURRENT(__ADC_RAW_TO_VOLTS(ADC_BUFF[2]));

	if(flg_new_receiver_input)
	{
		sbus_packets ++;
		if(sbus_packets%10 == 0)
		{
			led3_state = !led3_state;
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, led3_state);
		}

		update_receiver_inputs();
		flush_sbus_dma_buffer();
		flg_new_receiver_input = false;
	}

	if(flg_new_debug_message)
	{
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
		//Make a copy of the message so the DMA can get flushed
		uint8_t message_copy[DEBUG_RX_BUFFER_SIZE];
		memcpy(message_copy, debug_rx_buffer, strlen(debug_rx_buffer));

		//flush_UART1_dma_buffer();
		flush_debug_dma_buffer();

		// Check what type of message was received
		if(message_copy[0] == 'C')
		{

			char command_string[] = {message_copy[2]};
			uint8_t command = atoi(command_string);

			if(command == GET_PRIM_BATTERY_V)
			{
				sprintf(debug_tx_buffer, "R %d %0.3f \r\n", GET_PRIM_BATTERY_V, prim_batt_volts);
				send_debug_string_dma(SERIAL_DEBUG_UART_INSTANCE, debug_tx_buffer);
			}
			else if(command == GET_SEC_BATTERY_V)
			{
				sprintf(debug_tx_buffer, "R %d %0.3f \r\n", GET_SEC_BATTERY_V, sec_batt_volts);
				send_debug_string_dma(SERIAL_DEBUG_UART_INSTANCE, debug_tx_buffer);
			}
			else if(command == GET_SYS_CURRENT_I)
			{
				sprintf(debug_tx_buffer, "R %d %0.3f \r\n", GET_SYS_CURRENT_I, sys_current_amps);
				send_debug_string_dma(SERIAL_DEBUG_UART_INSTANCE, debug_tx_buffer);
			}
		}
		// Clear message received flag
		flg_new_debug_message = false;
	}

	//idle_all_motors();
	set_motor_speed(MOTOR_RL_TIM_CHANNEL, (((float)receiver_inputs[0] - 170)/(1850-170))*100 );
	set_motor_speed(MOTOR_FL_TIM_CHANNEL, (((float)receiver_inputs[1] - 170)/(1850-170))*100 );
	set_motor_speed(MOTOR_FR_TIM_CHANNEL, (((float)receiver_inputs[2] - 170)/(1850-170))*100 );
	set_motor_speed(MOTOR_RR_TIM_CHANNEL, (((float)receiver_inputs[3] - 170)/(1850-170))*100 );

	if(loop_counter%40 == 0)
	{
		//sprintf (debug_tx_buffer, "Channels: %d %d %d %d\r\n", receiver_inputs[0], receiver_inputs[1], receiver_inputs[2], receiver_inputs[3]);
		//send_debug_string_dma(SERIAL_DEBUG_UART_INSTANCE, debug_tx_buffer);
	}
	while((get_micros()) < 2500);
	loop_counter ++;

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

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 26;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/8000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK_DIV8);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

// Initialize SBUS receive
void sbus_init(void)
{
	HAL_UART_Receive_DMA(SBUS_UART_INSTANCE, (uint8_t *) sbus_rx_buffer, sizeof(sbus_rx_buffer));
}

// Clear SBUS DMA buffer
void flush_sbus_dma_buffer(void)
{
	HAL_UART_DMAStop(SBUS_UART_INSTANCE);
	memset(sbus_rx_buffer, 0, sizeof(sbus_rx_buffer));
	HAL_UART_Receive_DMA(SBUS_UART_INSTANCE, (uint8_t *) sbus_rx_buffer, sizeof(sbus_rx_buffer));
	return;
}

// Clear debug DMA buffer
void flush_debug_dma_buffer(void)
{
	HAL_UART_DMAStop(SERIAL_DEBUG_UART_INSTANCE);
	memset(debug_rx_buffer, 0, sizeof(debug_rx_buffer));
	HAL_UART_Receive_DMA(SERIAL_DEBUG_UART_INSTANCE, (uint8_t *) debug_rx_buffer, sizeof(debug_rx_buffer));
	return;
}

// UART DMA RX transfer complete callback (receive buffer full)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if (huart->Instance == USART1)
	{
		// This is an unexpected, since it indicates the buffer filled up
		// The idle line interrupt should trigger a DMA restart before this RxCpltCallback
		return;
	}
	if (huart->Instance == UART5)
	{
		// This is an unexpected, since it indicates the buffer filled up
		// The idle line interrupt should trigger a DMA restart before this RxCpltCallback
		return;
	}
	if (huart->Instance == USART6)
	{
		// This is an unexpected, since it indicates the buffer filled up
		// The idle line interrupt should trigger a DMA restart before this RxCpltCallback
		return;
	}
}

// UART DMA TX half transfer complete callback
void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
		return;
	}
	return;

}

// UART DMA TX transfer complete callback
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
		return;
	}
	return;
}

// UART 1 interrupt callback
void USART1_IRQCallback()
{
	//If UART RX line is idle, kick-off a new DMA transfer
	if (USART1->SR & USART_SR_IDLE)
	{
		__HAL_UART_CLEAR_IDLEFLAG (&huart1);
		//sprintf(uart1_tx_buffer, "%s", uart1_rx_buffer);
		//flush_UART1_dma_buffer();
		flg_new_debug_message = true;

	}
	return;
}

// UART 5 interrupt callback
void UART5_IRQCallback()
{
	//If UART RX line is idle, kick-off a new DMA transfer
	if (UART5->SR & USART_SR_IDLE)
	{
		__HAL_UART_CLEAR_IDLEFLAG (&huart5);
		flg_new_debug_message = true;
	}
	return;
}

// UART 6 interrupt callback
void UART6_IRQCallback()
{
	if (USART6->SR & USART_SR_IDLE)
	{
		__HAL_UART_CLEAR_IDLEFLAG (SBUS_UART_INSTANCE);
		flg_new_receiver_input = true;
	}
	return;
}

// I2C memory write complete callback
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c->Instance == I2C1)
		i2c1_mem_tx_irq_flag = true;
	return;
}

// I2C memory read complete callback
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c->Instance == I2C1)
		i2c1_mem_rx_irq_flag = true;
	return;
}

// GPIO interrupt callback
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//PC15 is connected to IMU interrupt pin
  if(GPIO_Pin == GPIO_PIN_15)
  {
	  new_imu_data_flag = true;
  }
}

// Convert the SBUS packet into its individual channels
void update_receiver_inputs()
{
	receiver_inputs[0]  = ((sbus_rx_buffer[1]|sbus_rx_buffer[2]<<8) & 0x07FF);
	receiver_inputs[1]  = ((sbus_rx_buffer[2]>>3|sbus_rx_buffer[3]<<5)  & 0x07FF);
	receiver_inputs[2]  = ((sbus_rx_buffer[3]>>6 |sbus_rx_buffer[4]<<2 |sbus_rx_buffer[5]<<10)  & 0x07FF);
	receiver_inputs[3]  = ((sbus_rx_buffer[5]>>1 |sbus_rx_buffer[6]<<7) & 0x07FF);
	receiver_inputs[4]  = ((sbus_rx_buffer[6]>>4 |sbus_rx_buffer[7]<<4) & 0x07FF);
	receiver_inputs[5]  = ((sbus_rx_buffer[7]>>7 |sbus_rx_buffer[8]<<1 |sbus_rx_buffer[9]<<9)   & 0x07FF);
	receiver_inputs[6]  = ((sbus_rx_buffer[9]>>2 |sbus_rx_buffer[10]<<6) & 0x07FF);
	receiver_inputs[7]  = ((sbus_rx_buffer[10]>>5|sbus_rx_buffer[11]<<3) & 0x07FF);
	receiver_inputs[8]  = ((sbus_rx_buffer[12]   |sbus_rx_buffer[13]<<8) & 0x07FF);
	receiver_inputs[9]  = ((sbus_rx_buffer[13]>>3|sbus_rx_buffer[14]<<5) & 0x07FF);
	receiver_inputs[10] = ((sbus_rx_buffer[14]>>6|sbus_rx_buffer[15]<<2|sbus_rx_buffer[16]<<10) & 0x07FF);
	receiver_inputs[11] = ((sbus_rx_buffer[16]>>1|sbus_rx_buffer[17]<<7) & 0x07FF);
	receiver_inputs[12] = ((sbus_rx_buffer[17]>>4|sbus_rx_buffer[18]<<4) & 0x07FF);
	receiver_inputs[13] = ((sbus_rx_buffer[18]>>7|sbus_rx_buffer[19]<<1|sbus_rx_buffer[20]<<9)  & 0x07FF);
	receiver_inputs[14] = ((sbus_rx_buffer[20]>>2|sbus_rx_buffer[21]<<6) & 0x07FF);
	receiver_inputs[15] = ((sbus_rx_buffer[21]>>5|sbus_rx_buffer[22]<<3) & 0x07FF);
	receiver_inputs[16] = ((sbus_rx_buffer[23]));
	return;
}

// Transmit a debug message over UART in blocking mode
void send_debug_string_blocking(UART_HandleTypeDef *huart, char *string)
{
	HAL_UART_Transmit(huart, (uint8_t *) string, strlen (string), 2000);
	return;
}

// Transmit a debug message over UART in non-blocking mode
void send_debug_string_dma(UART_HandleTypeDef *huart, char *string)
{
	HAL_UART_Transmit_DMA(huart, (uint8_t*)string, strlen(string));
	return;
}

// ADC conversion complete callback
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == ADC1)
	{
		//HAL_ADC_Stop_DMA(&hadc1);
		//memset(ADC_BUFF, 0, sizeof(ADC_BUFF));
		HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADC_BUFF,NUM_ADC_CHANNELS);
	}
	return;
}

// Initialize ADC
void ADC_Init(void)
{
	memset(ADC_BUFF, 0, sizeof(ADC_BUFF));
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADC_BUFF,NUM_ADC_CHANNELS);
	return;
}

// Initiate a non-blocking I2C register read on the IMU data registers
void start_imu_dma_transfer()
{
	HAL_I2C_Mem_Read_DMA(&hi2c1, 0x68<<1, 0x3B, 1 , imu_data_buffer, sizeof(imu_data_buffer));
	i2c1_mem_rx_irq_flag = false;
	return;
}

// Start the microseconds timer
void Microseconds_Timer_Init()
{
	HAL_TIM_Base_Start(&htim2);
	return;
}

// Reset the microseconds counter back to 0
void reset_micros()
{
	TIM2->CNT = 0;
	return;
}

// Return the number of microseconds since the last call to reset_micros()
// 32-bit up-counter so it will overflow after 71 minutes
uint32_t get_micros()
{
	return TIM2->CNT;
}

// Set spped of a single motor
// motor_channel: TIMER channel of the motor
// speed_percent: positive float <= 100
void set_motor_speed(uint32_t motor_Channel, float speed_percent)
{
	uint16_t pulse = MOTOR_MIN_PULSE + (speed_percent*0.01)*(MOTOR_MAX_PULSE - MOTOR_MIN_PULSE);

	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = pulse;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

	HAL_TIM_PWM_ConfigChannel(MOTOR_TIMER_INSTANCE, &sConfigOC, motor_Channel);
	HAL_TIM_PWM_Start(MOTOR_TIMER_INSTANCE, motor_Channel);
	return;
}

// Set all motors to a very low speed
void idle_all_motors()
{
	set_motor_speed(MOTOR_RL_TIM_CHANNEL, MOTOR_IDLE_SPEED);
	set_motor_speed(MOTOR_FL_TIM_CHANNEL, MOTOR_IDLE_SPEED);
	set_motor_speed(MOTOR_FR_TIM_CHANNEL, MOTOR_IDLE_SPEED);
	set_motor_speed(MOTOR_RR_TIM_CHANNEL, MOTOR_IDLE_SPEED);
	return;
}

// Stop all motors
void stop_all_motors()
{
	stop_motor(MOTOR_RL_TIM_CHANNEL);
	stop_motor(MOTOR_FL_TIM_CHANNEL);
	stop_motor(MOTOR_FR_TIM_CHANNEL);
	stop_motor(MOTOR_RR_TIM_CHANNEL);
	return;
}

// Stop a single motor
// motor_channel: TIMER channel of the motor
void stop_motor(uint32_t motor_Channel)
{
	HAL_TIM_PWM_Stop(MOTOR_TIMER_INSTANCE, motor_Channel);
	return;
}

// Configure the MCU pins as regular GPIOs
void config_LEDs_for_GPIO(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED3_GPIO_Port, &GPIO_InitStruct);

  return;
}

// Configure the MCU pins as PWM outputs
void config_LEDs_for_PWM(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(LED3_GPIO_Port, &GPIO_InitStruct);

  return;
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
