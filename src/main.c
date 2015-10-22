/*
 * main.c
 *
 *  Created on: Sep 24, 2015
 *      Author: Edward Elson
 *
 *	This is the codes to control electronics for thrust measurement device
 *	sensors and actuators:
 *	1. accelerometer
 *	2. distance sensor
 *	3. valve control circuit
 *	4. TFT LCD
 *	5. SD Card
 *
 *	For Queue Implementation reference:
 *	https://www.keil.com/pack/doc/CMSIS/RTOS/html/group___c_m_s_i_s___r_t_o_s___message.html
 */

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_adc.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_cortex.h"
#include "stm32f4xx_hal_pwr_ex.h"
#include "stm32f4xx_hal_flash_ex.h"
#include "cmsis_os.h"
#include "string.h"
#include "math.h"

#include "ee_functions.h"
#include "ee_hmc5883l.h"
#include "ee_mpu6050.h"
#include "ee_kalman.h"
#include "ee_3Dcoordinates.h"

/* Data Structure Declarations ------------------------------------------------*/
typedef struct {
	float data1;
	float data2;
	float data3;
	float data4;
	float data5;
	float data6;
} data_queue;

/* Constant Declarations ------------------------------------------------------*/
#define task1_period 10
#define task2_period 1000
#define task3_period 10

/* Private Variable Declarations ---------------------------------------------*/
float mag[3] = {0}; //mag[0] = magX, mag[1] = magY, mag[2] = magZ
float acc[3] = {0}; //acc[0] = accX, acc[1] = accY, acc[2] = accZ
float gyro[3] = {0}; //gyro[0] = gyroX, gyro[1] = gyroY, gyro[2] = gyroZ
float magGain[3] = {0};
float magOffset[3] = { (MAG0MAX + MAG0MIN) / 2, (MAG1MAX + MAG1MIN) / 2, (MAG2MAX + MAG2MIN) / 2 };

float roll, pitch, yaw; // raw data
float kalX = 0, kalY = 0, kalZ = 0; // roll, pitch, yaw
float gyroX = 0, gyroY = 0, gyroZ = 0; // raw roll, pitch, yaw;

//float P[2][2] = {0};

/* OS and ARM Variable Declarations ------------------------------------------*/
UART_HandleTypeDef huart2;
I2C_HandleTypeDef hi2c1;
ADC_HandleTypeDef hadc1;

osThreadId task1Thread; //IMU Reading
osThreadId task2Thread; //LED blinking
osThreadId task3Thread; //read Queue and UART Transmission
osMessageQId osQueue; // Queue Declaration
extern osMessageQId valve_queue; // Queue Declaration
osPoolId q_pool; // Memory Management for Structure
extern osPoolId valve_pool; // Memory Management for Structure

/* Private function prototypes -----------------------------------------------*/
void Clock_Config();
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
void IMU_init(void);

void task1Function(void const * argument);
void task2Function(void const * argument);
void task3Function(void const * argument);

/* Main ----------------------------------------------------------------------*/
int main(void)
{
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	Clock_Config();

	/* Configure GPIO settings */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_ADC1_Init();
	MX_I2C1_Init();

	/* IMU Module Initialization */
	IMU_init();

	/* Threads Creation */
	osThreadDef(task1, task1Function, osPriorityNormal, 0, 128);
	task1Thread = osThreadCreate(osThread(task1), NULL);

	osThreadDef(task2, task2Function, osPriorityHigh, 0, 128);
	task2Thread = osThreadCreate(osThread(task2), NULL);

	osThreadDef(task3, task3Function, osPriorityNormal, 0, 256);
	task3Thread = osThreadCreate(osThread(task3), NULL);

	/* Create Pool */
	osPoolDef(q_pool, 16, data_queue); // data_queue can keep 16 data maximum
	q_pool = osPoolCreate(osPool(q_pool));
	osPoolDef(valve_pool, 2, uint32_t); // data_queue can keep 16 data maximum
	valve_pool = osPoolCreate(osPool(valve_pool));

	/* Create Queue */
	osMessageQDef(osQueue, 16, data_queue);
	osQueue = osMessageCreate (osMessageQ(osQueue), NULL);
	osMessageQDef(valve_queue, 2, uint32_t);
	valve_queue = osMessageCreate (osMessageQ(valve_queue), NULL);


	/* Start freeRTOS kernel */
	osKernelStart();

	while (1);
}

void IMU_init(void)
{
	mpu6050_init(&hi2c1);
	hmc5883l_init(&hi2c1);
	hmc5883l_calibration(&hi2c1, magGain);
	HAL_Delay(100);

	mpu6050_read(&hi2c1, acc, gyro);
	hmc5883l_read(&hi2c1, mag);

	obtain_pitch(&pitch, acc);
	obtain_roll(&roll, acc);
	obtain_yaw(&yaw, mag, magGain, magOffset, kalX, kalY);

	kalX = roll;
	kalY = pitch;
	kalZ = yaw;

	gyroX = roll;
	gyroY = pitch;
	gyroZ = yaw;

//	timer = HAL_GetTick();
}

/* Task 1 : IMU Reading ----------------------------------------------------*/
void task1Function(void const * argument)
{
	static data_queue *q_ptr; // pointer to data structure
	static uint32_t timer = 0;
	static float dt = 0;
	static float gyroXrate = 0, gyroYrate = 0 , gyroZrate = 0;
	static float P[2][2] = {{0,0},{0,0}};

    while(1)
	{
		// Update Values of Angle obtained from IMU + Kalman Filter
    	mpu6050_read(&hi2c1, acc, gyro);
    	hmc5883l_read(&hi2c1, mag);

    	float dt = (float)(HAL_GetTick() - timer) / (float) 1000;
    	timer = HAL_GetTick();

    	/* Update Pitch and Roll */
    	obtain_pitch(&pitch, acc);
    	obtain_roll(&roll, acc);

    	float gyroXrate = (float) gyro[0] / (float) 131.0; // Convert to deg/s
    	float gyroYrate = (float) gyro[1] / (float) 131.0; // Convert to deg/s
    	float gyroZrate = (float) gyro[2] / (float) 131.0; // Convert to deg/s

    	if ((roll < -90 && kalX > 90) || (roll > 90 && kalX < -90))
    	{
    		kalX = roll;
    		gyroX = roll;
    	}
    	else
    	{
    	    kalX = update_kalman(roll, gyroXrate, dt, kalX); // Calculate the angle using a Kalman filter
  	}

    	if (abs(kalX) > 90)
    	{
    		gyroYrate = -gyroYrate; // Invert rate, so it fits the restricted accelerometer reading
    		kalY = update_kalman(pitch, gyroYrate, dt, kalY);
    	}

//    	/* Update Yaw */
    	obtain_yaw(&yaw, mag, magGain, magOffset, kalX, kalY);

    	// This fixes the transition problem when the yaw angle jumps between -180 and 180 degrees
    	if ((yaw < -90 && kalZ > 90) || (yaw > 90 && kalZ < -90))
    	{
			kalZ = yaw;
			gyroZ = yaw;
	   	}
    	else
    	{
    		kalZ = update_kalman(yaw, gyroZrate, dt, kalZ); // Calculate the angle using a Kalman filter
    	}

    	/* Estimate angles using gyro only */
    	gyroX += gyroXrate * dt; // Calculate gyro angle without any filter
		gyroY += gyroYrate * dt;
		gyroZ += gyroZrate * dt;

    	// Reset the gyro angles when they has drifted too much
    	if (gyroX < -180 || gyroX > 180)
    	  gyroX = kalX;
    	if (gyroY < -180 || gyroY > 180)
    	  gyroY = kalY;
    	if (gyroZ < -180 || gyroZ > 180)
    	  gyroZ = kalZ;

		// Write angle readings to queue buffer
		q_ptr = osPoolAlloc(q_pool); //allocate memory out of the 16 available to keep this data
		q_ptr->data1 = kalX;
		q_ptr->data2 = kalY;
		q_ptr->data3 = kalZ;
		q_ptr->data4 = gyroX;
		q_ptr->data5 = gyroY;
		q_ptr->data6 = gyroZ;

		osMessagePut(osQueue, (uint32_t)q_ptr, osWaitForever);

		osDelay(task1_period);
	}
}

/* Task 2: LED blinking -------------------------------------------------------------------- */
void task2Function (void const * argument)
{

	while(1)
	{
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);

		osDelay(task2_period);
	}
}

/* Task 3 : Read Queue and UART Transmission ---------------------------------------------------*/
void task3Function(void const * argument)
{
	static data_queue *qr_ptr;
	static osEvent evt;

	static float kalX_send;
	static float kalY_send;
	static float kalZ_send;
	static float gyroX_send;
	static float gyroY_send;
	static float gyroZ_send;

	static char data_transmit[150] = {0};
	uint32_t timer = 0;


    while(1)
	{
    	//get the queue value from Queue Buffer
    	evt = osMessageGet(osQueue, NULL);

    	//if there is message available in osqueue
    	if (evt.status == osEventMessage)
    	{
    		qr_ptr = evt.value.p;
    		kalX_send = qr_ptr->data1;
    		kalY_send = qr_ptr->data2;
    		kalZ_send = qr_ptr->data3;
    		gyroX_send = qr_ptr->data4;
    		gyroY_send = qr_ptr->data5;
    		gyroZ_send = qr_ptr->data6;
    		osPoolFree(q_pool, qr_ptr); //free the memory allocated to message

			//UART Transmission
			memset(data_transmit, '0', 149);

			char kalX_string[32] = {0};
			char kalY_string[32] = {0};
			char kalZ_string[32] = {0};
			char gyroX_string[32] = {0};
			char gyroY_string[32] = {0};
			char gyroZ_string[32] = {0};

			ee_floatTostring(kalX, kalX_string, 32);
			ee_floatTostring(kalY, kalY_string, 32);
			ee_floatTostring(kalZ, kalZ_string, 32);
			ee_floatTostring(roll, gyroX_string, 32);
			ee_floatTostring(pitch, gyroY_string, 32);
			ee_floatTostring(yaw, gyroZ_string, 32);

			snprintf(data_transmit, sizeof(data_transmit), "%s,%s,%s,%s,%s,%s\n\r",
						kalX_string, kalY_string, kalZ_string, gyroX_string, gyroY_string, gyroZ_string);

			HAL_UART_Transmit(&huart2, (uint8_t*)data_transmit, strlen(data_transmit), 0xFFFF);
    	}

    	osDelay(task3_period);

	}
}

/* System Clock Configuration --------------------------------------------------------------------
 * Taken from STM32F4CubeMX auto-generated files */
void Clock_Config()
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

}

/* GPIO configuration --------------------------------------------------------------------*/
void MX_GPIO_Init(void)
{
	  GPIO_InitTypeDef GPIO_InitStruct;

	  /* GPIO Ports Clock Enable */
	  __GPIOC_CLK_ENABLE();
	  __GPIOH_CLK_ENABLE();
	  __GPIOA_CLK_ENABLE();
	  __GPIOB_CLK_ENABLE();

	  /* The following pin configurations depend on application.
	   * As we'd like to turn on LED and use the switch,
	   * we need to set them correspondingly
	   * check datasheet for more information
	   */

	  /*Configure GPIO pin : PC13 */
	  GPIO_InitStruct.Pin = GPIO_PIN_13;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	  /*Configure GPIO pins : PA2 PA3 */
	  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /* EXTI interrupt init*/
	  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
	  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);


	  /*Configure GPIO pins : PA5 PA6 */
	  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USART2 init function -------------------------------------------------------------------- */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);

}

/* ADC1 init function --------------------------------------------------------------------*/
void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION12b;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = EOC_SINGLE_CONV;
  HAL_ADC_Init(&hadc1);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

}

/* I2C1 init function --------------------------------------------------------------------*/
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000; //I2C speed is 100 kHz
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
  HAL_I2C_Init(&hi2c1);

}

/* additional code from STM32 */
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
#endif
