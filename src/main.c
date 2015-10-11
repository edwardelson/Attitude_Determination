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

/* Data Structure Declarations ------------------------------------------------*/
typedef struct {
	uint32_t data1;
	uint32_t data2;
	uint32_t data3;
	uint32_t data4;
} data_queue;

/* Constant Declarations ------------------------------------------------------*/
#define task1_period 5000
#define task2_period 100
#define task3_period 500
#define task4_period 1000
#define valve_delay_time 30

/* Constant Declarations ------------------------------------------------------*/


/* Private Variable Declarations ---------------------------------------------*/

/* OS and ARM Variable Declarations ------------------------------------------*/
UART_HandleTypeDef huart2;
I2C_HandleTypeDef hi2c1;
ADC_HandleTypeDef hadc1;

osThreadId task1Thread; //IMU Reading
osThreadId task2Thread; //LED blinking
osThreadId task3Thread; //read Queue and UART Transmission
osThreadId task4Thread; //empty
osMessageQId osQueue; // Queue Declaration
osMessageQId distance_queue; // Queue Declaration
extern osMessageQId valve_queue; // Queue Declaration
osPoolId q_pool; // Memory Management for Structure
osPoolId distance_pool; // Memory Management for Structure
extern osPoolId valve_pool; // Memory Management for Structure

/* Private function prototypes -----------------------------------------------*/
void Clock_Config();
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);

void task1Function(void const * argument);
void task2Function(void const * argument);
void task3Function(void const * argument);
void task4Function(void const * argument);

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

	/* Threads Creation */
	osThreadDef(task1, task1Function, osPriorityNormal, 0, 128);
	task1Thread = osThreadCreate(osThread(task1), NULL);

	osThreadDef(task2, task2Function, osPriorityNormal, 0, 128);
	task2Thread = osThreadCreate(osThread(task2), NULL);

	osThreadDef(task3, task3Function, osPriorityNormal, 0, 128);
	task3Thread = osThreadCreate(osThread(task3), NULL);

	osThreadDef(task4, task4Function, osPriorityNormal, 0, 128);
	task4Thread = osThreadCreate(osThread(task4), NULL);

	/* Create Pool */
	osPoolDef(q_pool, 16, data_queue); // data_queue can keep 16 data maximum
	q_pool = osPoolCreate(osPool(q_pool));
	osPoolDef(distance_pool, 16, uint32_t); // data_queue can keep 16 data maximum
	distance_pool = osPoolCreate(osPool(distance_pool));
	osPoolDef(valve_pool, 2, uint32_t); // data_queue can keep 16 data maximum
	valve_pool = osPoolCreate(osPool(valve_pool));

	/* Create Queue */
	osMessageQDef(osQueue, 16, data_queue);
	osQueue = osMessageCreate (osMessageQ(osQueue), NULL);
	osMessageQDef(distance_queue, 16, uint32_t);
	distance_queue = osMessageCreate (osMessageQ(distance_queue), NULL);
	osMessageQDef(valve_queue, 2, uint32_t);
	valve_queue = osMessageCreate (osMessageQ(valve_queue), NULL);

	/* IMU Module Initialization */
	mpu6050_init(&hi2c1);
	hmc5883l_init(&hi2c1);

	/* Start freeRTOS kernel */
	osKernelStart();
	while (1);
}

/* Task 1 : IMU Reading ----------------------------------------------------*/
/* for now is just led blinking and queue testing */
void task1Function(void const * argument)
{
	uint32_t voltage = 10;
	uint32_t current = 100;
	uint32_t counter = 1000;

	data_queue *q_ptr; // pointer to data structure

    while(1)
	{
    	// Toggle LED
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

		// Update Values of Voltage, Current and Counter
		voltage++;
		current++;
		counter++;
		//		//read adc value from PA0
		//		HAL_ADC_Start(&hadc1);
		//		if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY)== 0x00)
		//		{
		//			acc = HAL_ADC_GetValue(&hadc1);
		//		}
		//		HAL_ADC_Stop(&hadc1);
		//


		// Write Voltage, Current and Counter to
		q_ptr = osPoolAlloc(q_pool); //allocate memory out of the 16 available to keep this data
		q_ptr->data1 = voltage;
		q_ptr->data2 = current;
		q_ptr->data3 = counter;
		osMessagePut(osQueue, (uint32_t)q_ptr, osWaitForever);

		osDelay(task1_period);
	}
}

/* Task 2: LED blinking -------------------------------------------------------------------- */
void task2Function (void const * argument)
{
//	uint32_t *valve_ptr;
//	osEvent valve_event;
//	uint32_t valve_enable;

	while(1)
	{
//		valve_event = osMessageGet(valve_queue, NULL);
//
//		if (valve_event.status == osEventMessage)
//		{
//    		valve_ptr = valve_event.value.p;
//    		valve_enable = *valve_ptr;
//    		osPoolFree(valve_pool, valve_ptr);
//
//    		if (valve_enable == 1)
//    		{
    			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
//    			HAL_Delay(valve_delay_time);
//    			/* HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);*/ HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
//    		}
//		}

		osDelay(task2_period);

	}
}

/* Task 3 : Read Queue and UART Transmission ---------------------------------------------------*/
void task3Function(void const * argument)
{
	data_queue *qr_ptr;
	uint32_t *distance_ptr;
	osEvent evt;
	osEvent distance_event;

	uint32_t voltage;
	uint32_t current;
	uint32_t counter;
	uint32_t distance;

	char data_transmit[30] = {0};

    while(1)
	{
    	//get the queue value from Queue Buffer
    	evt = osMessageGet(osQueue, NULL);
    	distance_event = osMessageGet(distance_queue, NULL);

    	//if there is message available in osqueue
    	if (evt.status == osEventMessage)
    	{
    		qr_ptr = evt.value.p;
    		voltage = qr_ptr->data1;
    		current = qr_ptr->data2;
    		counter = qr_ptr->data3;
    		osPoolFree(q_pool, qr_ptr); //free the memory allocated to message
    	}

    	//if there is message available in distance_queue
    	if (distance_event.status == osEventMessage)
    	{
    		distance_ptr = distance_event.value.p;
    		distance = *distance_ptr;
    		osPoolFree(distance_pool, distance_ptr);
    	}

		//UART Transmission
		memset(data_transmit, '0', 30);
//		snprintf(data_transmit, sizeof(data_transmit), "%d,%d,%d,%d\n\r", voltage, current, counter, distance); //will send old values if the sensors have not updated it yet

		voltage = hmc5883l_init(&hi2c1);
		snprintf(data_transmit, sizeof(data_transmit), "%d\n\r", voltage);

		HAL_UART_Transmit(&huart2, (uint8_t*)data_transmit, strlen(data_transmit), 0xFFFF);

    	osDelay(task3_period);
	}
}

/* Task 4 : Distance Sensor Reading -------------------------------------------------------------*/
//now just update distance value regularly
void task4Function(void const * argument)
{
	uint32_t distance = 10000;
	uint32_t *q_ptr; // pointer to data structure

	while(1)
	{

		// Update Values of Voltage, Current and Counter
		distance++;

		// Write Voltage, Current and Counter to
		q_ptr = osPoolAlloc(distance_pool); //allocate memory out of the 16 available to keep this data
		*q_ptr = distance;
		osMessagePut(distance_queue, (uint32_t)q_ptr, osWaitForever);

		osDelay(task4_period);

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
  hi2c1.Init.ClockSpeed = 400000; //I2C speed is 100 kHz
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
