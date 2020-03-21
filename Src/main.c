
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
#include "math.h"
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
#include "MahonyAHRS.h"

#define M_PI 3.1415926
#define EPSILON 0.0000001

#define NUM_ADC_CHANNELS 3
#define DEBUG_TX_BUFFER_SIZE 50
#define DEBUG_RX_BUFFER_SIZE 50
#define SBUS_RX_BUFFER_SIZE 50

// Motor timer channels
// FL = Front-left
// RR = Rear-right
#define MOTOR_RL_TIM_CHANNEL TIM_CHANNEL_3
#define MOTOR_FL_TIM_CHANNEL TIM_CHANNEL_4
#define MOTOR_FR_TIM_CHANNEL TIM_CHANNEL_1
#define MOTOR_RR_TIM_CHANNEL TIM_CHANNEL_2
#define MOTOR_TIMER_INSTANCE &htim1

#define MOTOR_MIN_PULSE 1000 // In microseconds
#define MOTOR_MAX_PULSE 2000 // In microseconds
#define MOTOR_IDLE_SPEED 7 // In %

#define __ADC_RAW_TO_VOLTS(adc_val) ((double)adc_val)*0.0007326 //1LSB = 3.0V/4095
#define __PRIM_BATT_VOLTAGE(adc_volts) (1.500 + ((double)adc_volts)*0.3667)*5.5946
#define __SEC_BATT_VOLTAGE(adc_volts) (1.500 + ((double)adc_volts)*0.3667)*1.8667
#define __SYS_CURRENT(adc_volts) (((double)adc_volts)/200.0)/0.01 //200V/V gain and 0R01 sense resistor

#define DEBUG_OVER_WIRELESS

#ifdef DEBUG_OVER_WIRELESS
#define SERIAL_DEBUG_UART_INSTANCE &huart5
#endif

#ifndef DEBUG_OVER_WIRELESS
#define SERIAL_DEBUG_UART_INSTANCE &huart1
#endif

#define SBUS_UART_INSTANCE &huart6

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
__IO uint8_t imu_data_buffer[21];

__IO bool imu_data_ready_flag = false;

__IO bool i2c1_mem_rx_irq_flag = false;
__IO bool i2c1_mem_tx_irq_flag = false;

__IO bool flg_new_debug_message = false;
__IO bool flg_new_receiver_input = false;

__IO bool continuous_imu_collection = false;

bool kill_switch_active = true;
bool started_main_loop = false;

//YMFC variables//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 2;               //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.05;//0.0002;              //Gain setting for the roll I-controller
float pid_d_gain_roll = 5.0;              //Gain setting for the roll D-controller
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = 2;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = 0.05;//0.0002;//pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = 5.0;//pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = 400;//pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 10.0;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)

bool auto_level = false;                 //Auto level on (true) or off (false)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//uint8_t last_channel_1, last_channel_2, last_channel_3, last_channel_4;
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, loop_counter;
int esc_1, esc_2, esc_3, esc_4;
int throttle, battery_voltage;
int cal_int, start, gyro_address, num_samples;
int receiver_input[5];
int temperature;
int acc_axis[4], gyro_axis[4];
float roll_level_adjust, pitch_level_adjust;

long acc_x, acc_y, acc_z, acc_total_vector;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
unsigned long loop_timer;
double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_axis_cal[4];
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
bool gyro_angles_set;

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
void flush_imu_dma_buffer(void);

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
void set_motor_speed_percent(uint32_t motor_Channel, float percent);
void set_motor_speed_pulse(uint32_t motor_Channel, float pulse);
void idle_all_motors();
void stop_all_motors();
void stop_motor(uint32_t motor_Channel);

float convert_AHRS_pitch(float AHRS_pitch);
float convert_AHRS_roll(float AHRS_roll);
float convert_AHRS_yaw(float AHRS_yaw);

bool check_kill_switch(void);

// YMFC functions
void convertInputToYMFC();
void calculate_pid();


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
  stop_all_motors();
  // Enable idle line interrupts for serial debug
  __HAL_UART_ENABLE_IT(SERIAL_DEBUG_UART_INSTANCE, UART_IT_IDLE);

  // Flush SBUS dma buffer
  flush_sbus_dma_buffer();

  // Enable idle line interrupt for SBUS
  __HAL_UART_ENABLE_IT(SBUS_UART_INSTANCE, UART_IT_IDLE);

  // Start DMA for serial debug
  HAL_UART_Receive_DMA(SERIAL_DEBUG_UART_INSTANCE, debug_rx_buffer, sizeof(debug_rx_buffer));

  // Give time for ADC to collect sample
  HAL_Delay(10);

  float prim_batt_volts = __PRIM_BATT_VOLTAGE(__ADC_RAW_TO_VOLTS(ADC_BUFF[0]));
  float sec_batt_volts = __SEC_BATT_VOLTAGE(__ADC_RAW_TO_VOLTS(ADC_BUFF[1]));
  float sys_current_amps = __SYS_CURRENT(__ADC_RAW_TO_VOLTS(ADC_BUFF[2]));

  sprintf (buffer, "Primary battery: %0.3f V\n", prim_batt_volts);
  send_debug_string_blocking(SERIAL_DEBUG_UART_INSTANCE, buffer);

  sprintf (buffer, "Secondary battery: %0.3f V\n", sec_batt_volts);
  send_debug_string_blocking(SERIAL_DEBUG_UART_INSTANCE, buffer);

  sprintf (buffer, "System current: %0.3f A\n", sys_current_amps);
  send_debug_string_blocking(SERIAL_DEBUG_UART_INSTANCE, buffer);

  // If under voltage, blink red LED to indicate
	if(prim_batt_volts < 10.5)
	{
		sprintf (buffer, "Low battery!\n");
		send_debug_string_blocking(SERIAL_DEBUG_UART_INSTANCE, buffer);
		for(uint8_t i = 0; i < 10; i++)
		{
			HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
			HAL_Delay(100);
			HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
			HAL_Delay(100);
		}
	}

	if(prim_batt_volts < 10.0)
	{
		sprintf (buffer, "Battery level too low to start\n");
		send_debug_string_blocking(SERIAL_DEBUG_UART_INSTANCE, buffer);
		while(1);
	}

  /*
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
*/
  	int8_t ret = 0;
	ret = MPU9250_begin();
	sprintf (buffer, "%d IMU initialized\n", ret);
	send_debug_string_blocking(SERIAL_DEBUG_UART_INSTANCE, buffer);

	// setting the accelerometer full scale range to +/-8G
	ret = MPU9250_setAccelRange(ACCEL_RANGE_8G);
	sprintf (buffer, "%d IMU accel range updated\n", ret);
	send_debug_string_blocking(SERIAL_DEBUG_UART_INSTANCE, buffer);

	// setting the gyroscope full scale range to +/-500 deg/s
	ret = MPU9250_setGyroRange(GYRO_RANGE_500DPS);
	sprintf (buffer, "%d IMU gyro range updated\n", ret);
	send_debug_string_blocking(SERIAL_DEBUG_UART_INSTANCE, buffer);

	// setting DLPF bandwidth to 184 Hz
	ret = MPU9250_setDlpfBandwidth(DLPF_BANDWIDTH_184HZ);
	sprintf (buffer, "%d IMU bandwidth updated\n", ret);
	send_debug_string_blocking(SERIAL_DEBUG_UART_INSTANCE, buffer);

	// setting SRD to 2 for a 500 Hz update rate
	//MPU9250_setSrd(0);

	// IMU ready and calibrated, set start to 0
	start = 0;

	MPU9250_enableDataReadyInterrupt();
	continuous_imu_collection = true;
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	// Wait a little bit to ensure an IMU sample has come in
	HAL_Delay(3);

	// Begin Mahony filter and set to 1kHz sample frequency
	Mahony_begin(1000);

	float acc_x = 0;//MPU9250_getAccelX_mss();
	float acc_y = 0;//MPU9250_getAccelY_mss();
	float acc_z = 0;//MPU9250_getAccelZ_mss();

	float gyro_x = 0;//(-1)*MPU9250_getGyroX_rads();
	float gyro_y = 0;//(-1)*MPU9250_getGyroY_rads();
	float gyro_z = 0;//MPU9250_getGyroZ_rads();

	float mag_x = 0;//MPU9250_getMagX_uT();
	float mag_y = 0;//MPU9250_getMagY_uT();
	float mag_z = 0;//MPU9250_getMagZ_uT();

	uint32_t loop_counter = 0;

	// Enable all PWM channels so the ESC start-up
	set_motor_speed_percent(MOTOR_RL_TIM_CHANNEL, 0);
	set_motor_speed_percent(MOTOR_FL_TIM_CHANNEL, 0);
	set_motor_speed_percent(MOTOR_FR_TIM_CHANNEL, 0);
	set_motor_speed_percent(MOTOR_RR_TIM_CHANNEL, 0);

	// Do not start until kill switch goes from on to off
	while(!check_kill_switch()) //Wait until kill switch is on
	{
		sprintf(buffer, "%d %d %d \r\n", receiver_inputs[0], receiver_inputs[4],check_kill_switch());
		send_debug_string_dma(SERIAL_DEBUG_UART_INSTANCE, buffer);
		HAL_Delay(100);

	}
	while(check_kill_switch()) //Wait until kill switch is off
	{
			sprintf(buffer, "%d %d %d \r\n", receiver_inputs[0], receiver_inputs[4],check_kill_switch());
			send_debug_string_dma(SERIAL_DEBUG_UART_INSTANCE, buffer);
			HAL_Delay(100);

	}
	// Startup motors
	started_main_loop = true; // Set this flag to let some interrupts know

  while (1)
  {
	reset_micros();

	// Blink LED
	HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
	HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);

	// Update battery voltage
	prim_batt_volts = __PRIM_BATT_VOLTAGE(__ADC_RAW_TO_VOLTS(ADC_BUFF[0]));
	sec_batt_volts = __SEC_BATT_VOLTAGE(__ADC_RAW_TO_VOLTS(ADC_BUFF[1]));
	sys_current_amps = __SYS_CURRENT(__ADC_RAW_TO_VOLTS(ADC_BUFF[2]));

	// Get new IMU data
	acc_x = MPU9250_getAccelX_mss()/9.81f;
	acc_y = MPU9250_getAccelY_mss()/9.81f;
	acc_z = MPU9250_getAccelZ_mss()/9.81f;

	gyro_x = MPU9250_getGyroX_rads()*57.29578f;
	gyro_y = MPU9250_getGyroY_rads()*57.29578f;
	gyro_z = MPU9250_getGyroZ_rads()*57.29578f;

	//mag_x = MPU9250_getMagX_uT();
	//mag_y = MPU9250_getMagY_uT();
	//mag_z = MPU9250_getMagZ_uT();

	// Update AHRS filter
	Mahony_update(gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, mag_x, mag_y, mag_z);

	// Update YMFC angle variables

	// Use complementary filter for gyro inputs
	gyro_roll_input = (gyro_roll_input * 0.7) + (gyro_x * 0.3);   //Gyro pid input is deg/sec.
	gyro_pitch_input = (gyro_pitch_input * 0.7) + (gyro_y * 0.3);//Gyro pid input is deg/sec.
	gyro_yaw_input = (gyro_yaw_input * 0.7) + (gyro_z * 0.3);      //Gyro pid input is deg/sec.

	angle_pitch = convert_AHRS_pitch(Mahony_getPitch());
	angle_roll = convert_AHRS_roll(Mahony_getRoll());

	pitch_level_adjust = angle_pitch * 15;                                    //Calculate the pitch angle correction
	roll_level_adjust = angle_roll * 15;                                      //Calculate the roll angle correction

	if(!auto_level){                                                          //If the quadcopter is not in auto-level mode
		pitch_level_adjust = 0;                                                 //Set the pitch angle correction to zero.
		roll_level_adjust = 0;                                                  //Set the roll angle correcion to zero.
	}

	//For starting the motors: throttle low and yaw left (step 1).
	if(receiver_input_channel_3 < 1050 && receiver_input_channel_4 < 1050)start = 1;
	//When yaw stick is back in the center position start the motors (step 2).
	if(start == 1 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1450){
		start = 2;


		angle_pitch = angle_pitch_acc;                                          //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
		angle_roll = angle_roll_acc;                                            //Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.
		gyro_angles_set = true;                                                 //Set the IMU started flag.

		//Reset the PID controllers for a bumpless start.
		pid_i_mem_roll = 0;
		pid_last_roll_d_error = 0;
		pid_i_mem_pitch = 0;
		pid_last_pitch_d_error = 0;
		pid_i_mem_yaw = 0;
		pid_last_yaw_d_error = 0;

	}

	//Stopping the motors: throttle low and yaw right.
	if(start == 2 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1950)start = 0;


	//The PID set point in degrees per second is determined by the roll receiver input.
	//In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
	pid_roll_setpoint = 0;
	//We need a little dead band of 16us for better results.
	if(receiver_input_channel_1 > 1508)pid_roll_setpoint = receiver_input_channel_1 - 1508;
	else if(receiver_input_channel_1 < 1492)pid_roll_setpoint = receiver_input_channel_1 - 1492;

	pid_roll_setpoint -= roll_level_adjust;                                   //Subtract the angle correction from the standardized receiver roll input value.
	pid_roll_setpoint /= 3.0;                                                 //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.


	//The PID set point in degrees per second is determined by the pitch receiver input.
	//In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
	pid_pitch_setpoint = 0;
	//We need a little dead band of 16us for better results.
	if(receiver_input_channel_2 > 1508)pid_pitch_setpoint = (receiver_input_channel_2 - 1508)*-1;
	else if(receiver_input_channel_2 < 1492)pid_pitch_setpoint = (receiver_input_channel_2 - 1492)*-1;

	pid_pitch_setpoint -= pitch_level_adjust;                                  //Subtract the angle correction from the standardized receiver pitch input value.
	pid_pitch_setpoint /= 3.0;                                                 //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

	//The PID set point in degrees per second is determined by the yaw receiver input.
	//In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
	pid_yaw_setpoint = 0;

	//We need a little dead band of 16us for better results.
	if(receiver_input_channel_3 > 1050){ //Do not yaw when turning off the motors.
		if(receiver_input_channel_4 > 1508)pid_yaw_setpoint = ((receiver_input_channel_4 - 1508)/3.0)*1;//*-1 to invert yaw direction
		else if(receiver_input_channel_4 < 1492)pid_yaw_setpoint = ((receiver_input_channel_4 - 1492)/3.0)*1;//*-1 to invert yaw direction
	}

	calculate_pid();                                                            //PID inputs are known. So we can calculate the pid output.

	throttle = receiver_input_channel_3;                                      //We need the throttle signal as a base signal.

	if (start == 2){                                                          //The motors are started.
		if (throttle > 1800) throttle = 1800;                                   //We need some room to keep full control at full throttle.
		esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
		esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
		esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
		esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)

		if (esc_1 < 1100) esc_1 = 1100;                                         //Keep the motors running.
		if (esc_2 < 1100) esc_2 = 1100;                                         //Keep the motors running.
		if (esc_3 < 1100) esc_3 = 1100;                                         //Keep the motors running.
		if (esc_4 < 1100) esc_4 = 1100;                                         //Keep the motors running.

		if(esc_1 > 2000)esc_1 = 2000;                                           //Limit the esc-1 pulse to 2000us.
		if(esc_2 > 2000)esc_2 = 2000;                                           //Limit the esc-2 pulse to 2000us.
		if(esc_3 > 2000)esc_3 = 2000;                                           //Limit the esc-3 pulse to 2000us.
		if(esc_4 > 2000)esc_4 = 2000;                                           //Limit the esc-4 pulse to 2000us.
	}

	else{
	esc_1 = 1050;                                                           //If start is not 2 keep a 1000us pulse for ess-1.
	esc_2 = 1050;                                                           //If start is not 2 keep a 1000us pulse for ess-2.
	esc_3 = 1050;                                                           //If start is not 2 keep a 1000us pulse for ess-3.
	esc_4 = 1050;                                                           //If start is not 2 keep a 1000us pulse for ess-4.
	}

	if(!check_kill_switch())
	{
		set_motor_speed_pulse(MOTOR_FR_TIM_CHANNEL, esc_1);
		set_motor_speed_pulse(MOTOR_RR_TIM_CHANNEL, esc_2);
		set_motor_speed_pulse(MOTOR_RL_TIM_CHANNEL, esc_3);
		set_motor_speed_pulse(MOTOR_FL_TIM_CHANNEL, esc_4);
	}

	if(flg_new_receiver_input)
	{
		sbus_packets ++;
		update_receiver_inputs();
		convertInputToYMFC();
		flush_sbus_dma_buffer();
		flg_new_receiver_input = false;
		check_kill_switch();
	}

	if(flg_new_debug_message)
	{
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

	if(loop_counter%40 == 0)
	{
		//sprintf (debug_tx_buffer, "Channels: %d %d %d %d\r\n", receiver_inputs[0], receiver_inputs[1], receiver_inputs[2], receiver_inputs[3]);
		//send_debug_string_dma(SERIAL_DEBUG_UART_INSTANCE, debug_tx_buffer);
	}
	if(loop_counter%100 == 0)
	{
		//sprintf (buffer, "Gyro: %0.2f %0.2f %0.2f -- Pitch: %0.1f Roll: %0.1f Yaw: %0.1f\n", gyro_x, gyro_y, gyro_z, convert_AHRS_pitch(Mahony_getPitch()), convert_AHRS_roll(Mahony_getRoll()), convert_AHRS_yaw(Mahony_getYaw()));
		//send_debug_string_dma(SERIAL_DEBUG_UART_INSTANCE, buffer);
	}
	loop_counter ++;
	while((get_micros()) < 1000);

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

// Clear IMU DMA buffer
void flush_imu_dma_buffer(void)
{
	HAL_UART_DMAStop(&hi2c1);
	memset(imu_data_buffer, 0, sizeof(imu_data_buffer));
	HAL_UART_Receive_DMA(&hi2c1, (uint8_t *) imu_data_buffer, sizeof(imu_data_buffer));
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
		flush_debug_dma_buffer();
		return;
	}
	if (huart->Instance == USART6)
	{
		flush_debug_dma_buffer();
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
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		__HAL_UART_CLEAR_IDLEFLAG (SBUS_UART_INSTANCE);
		flg_new_receiver_input = true;
		if(!started_main_loop)
		{
			update_receiver_inputs();
		}
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
	{
		// Raise I2C RX interrupt flag
		i2c1_mem_rx_irq_flag = true;

		if(continuous_imu_collection)
			MPU9250_convertRawData(imu_data_buffer);
	}
	return;
}

// GPIO interrupt callback
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//PC15 is connected to IMU interrupt pin
  if(GPIO_Pin == GPIO_PIN_15)
  {
	  // New data is ready. Initiate new transfer
	  imu_data_ready_flag = true;
	  if(continuous_imu_collection)
		  start_imu_dma_transfer();
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
	// Start a new DMA request for IMU data
	HAL_I2C_Mem_Read_DMA(&hi2c1, MPU9250_ADDRESS<<1, 0x3B, 1 , imu_data_buffer, sizeof(imu_data_buffer));

	// Reset the I2C receive flag
	i2c1_mem_rx_irq_flag = false;

	// Clear the data ready flag
	imu_data_ready_flag = false;

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
void set_motor_speed_percent(uint32_t motor_Channel, float percent)
{
	uint16_t pulse = MOTOR_MIN_PULSE + (percent*0.01)*(MOTOR_MAX_PULSE - MOTOR_MIN_PULSE);
	set_motor_speed_pulse(motor_Channel, pulse);


	return;
}

void set_motor_speed_pulse(uint32_t motor_Channel, float pulse)
{
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
	set_motor_speed_percent(MOTOR_RL_TIM_CHANNEL, MOTOR_IDLE_SPEED);
	set_motor_speed_percent(MOTOR_FL_TIM_CHANNEL, MOTOR_IDLE_SPEED);
	set_motor_speed_percent(MOTOR_FR_TIM_CHANNEL, MOTOR_IDLE_SPEED);
	set_motor_speed_percent(MOTOR_RR_TIM_CHANNEL, MOTOR_IDLE_SPEED);
	return;
}

// Stop all motors
void stop_all_motors()
{
	//set_motor_speed_percent(MOTOR_RL_TIM_CHANNEL, 0);
	//set_motor_speed_percent(MOTOR_FL_TIM_CHANNEL, 0);
	//set_motor_speed_percent(MOTOR_FR_TIM_CHANNEL, 0);
	//set_motor_speed_percent(MOTOR_RR_TIM_CHANNEL, 0);
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

float convert_AHRS_pitch(float AHRS_pitch)
{
	float pitch = AHRS_pitch*(-1);
	return pitch;
}

float convert_AHRS_roll(float AHRS_roll)
{
	float roll = 0;
	if(AHRS_roll > 0)
		roll = AHRS_roll - 180;
	else if(AHRS_roll < 0)
		roll = AHRS_roll + 180;
	return roll;
}

float convert_AHRS_yaw(float AHRS_yaw)
{
	float yaw = 0;
	if(AHRS_yaw > 180)
		yaw = 360 - AHRS_yaw;
	else if(AHRS_yaw < 180)
		yaw = AHRS_yaw*(-1);
	return yaw;
}

// Check if kill switch is activated. If so, put all ESCs in reset
bool check_kill_switch(void)
{
	// Kill switch activated as soon as channel 4 stops reading 172
	if(receiver_inputs[4] != 172)
	{
		kill_switch_active = true;
		stop_all_motors();
		start = 0;
	}
	else
	{
		kill_switch_active = false;
	}
	return kill_switch_active;
}

// Convert the reveiver inputs to the expected YMFC variable names
void convertInputToYMFC()
{
	//Values in receiver_inputs array vary from 170 - 1820
	//YMFC variables (receiver_input_channel_x) vary from 1000 - 2000
	//Scale accordingly
	receiver_input_channel_1 = 1000 + ((receiver_inputs[1] - 170) * 0.606); //roll input
	receiver_input_channel_2 = 1000 + ((receiver_inputs[2] - 170) * 0.606); //pitch input
	receiver_input_channel_3 = 1000 + ((receiver_inputs[0] - 170) * 0.606); //throttle input
	receiver_input_channel_4 = 1000 + ((receiver_inputs[3] - 170) * 0.606); //yaw input
	return;
}

void calculate_pid()
{
  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;
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
