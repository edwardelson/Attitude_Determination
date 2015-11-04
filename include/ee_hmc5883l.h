/*
 * ee_hmc5883l.h
 *
 *  Created on: Oct 11, 2015
 *      Author: Edward
 */

#ifndef EE_HMC5883L_H_
#define EE_HMC5883L_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"

/* Private Variable Declarations ---------------------------------------------*/
#define HMC5883L 0x1E // Address of magnetometer

// Calibration Result
#define MAG0MAX 603
#define MAG0MIN -578
#define MAG1MAX 542
#define MAG1MIN -701
#define MAG2MAX 547
#define MAG2MIN -556

uint8_t hmc5883l_init(I2C_HandleTypeDef*);
void hmc5883l_calibration(I2C_HandleTypeDef*, float*);
void hmc5883l_read(I2C_HandleTypeDef*, float*);

#endif /* EE_HMC5883L_H_ */
