/*
 * ee_mpu6050.h
 *
 *  Created on: Oct 11, 2015
 *      Author: Edward
 */

#ifndef EE_MPU6050_H_
#define EE_MPU6050_H_

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

/* Constant Declarations ------------------------------------------------------*/

/* Private Variable Declarations ---------------------------------------------*/
#define MPU6050 0x68 // If AD0 is logic low on the PCB the address is 0x68, otherwise set this to 0x69

/* Private function prototypes -----------------------------------------------*/
uint8_t mpu6050_init(I2C_HandleTypeDef *);
uint8_t mpu6050_status(I2C_HandleTypeDef *);
void mpu6050_read(I2C_HandleTypeDef*, float*, float*);

#endif /* EE_MPU6050_H_ */
