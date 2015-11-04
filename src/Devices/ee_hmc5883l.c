/*
 * ee_hmc5883l.c
 *
 *  Created on: Oct 11, 2015
 *      Author: Edward
 */

#include "ee_hmc5883l.h"
#include "stm32f4xx_hal.h"

// return 1 if properly initialized
uint8_t hmc5883l_init(I2C_HandleTypeDef *hi2c)
{
	uint8_t hmc5883l_set[1] = {0};

	hmc5883l_set[0] = 0x00; // Configure device for continuous mode

	if (HAL_I2C_Mem_Write(hi2c, (uint16_t)(HMC5883L << 1), (uint16_t) (0x02), 1, hmc5883l_set, 1, (uint32_t)0xFFFF) != HAL_OK)
	{
		return 0;
	}

	return 1;

}


void hmc5883l_read(I2C_HandleTypeDef* hi2c, float* mag)
{
	uint8_t hmc5883l_mag[6] = {0};
	int16_t hmc5883l_int[3] = {0};

	/* read from 0x03, memory address in HMC5883l */
	if(HAL_I2C_Mem_Read(hi2c, (uint16_t)(HMC5883L << 1), (uint16_t)0x03, 1, hmc5883l_mag, 6, (uint32_t)0xFFFF) == HAL_OK)
	{
		hmc5883l_int[0] = (hmc5883l_mag[0] << 8) | hmc5883l_mag[1];
		hmc5883l_int[2] = (hmc5883l_mag[2] << 8) | hmc5883l_mag[3];
		hmc5883l_int[1] = (hmc5883l_mag[4] << 8) | hmc5883l_mag[5];

		mag[0] = (float) hmc5883l_int[0];
		mag[1] = (float) hmc5883l_int[1];
		mag[2] = (float) hmc5883l_int[2];

	}

	return;
}

void hmc5883l_calibration(I2C_HandleTypeDef *hi2c, float* magGain)
{
	uint8_t hmc5883l_set[1] = {0};
	float magPosOff[3] = {0};
	float magNegOff[3] = {0};

	hmc5883l_set[0] = 0x11;
	if (HAL_I2C_Mem_Write(hi2c, (uint16_t)(HMC5883L << 1), (uint16_t) (0x00), 1, hmc5883l_set, 1, (uint32_t)0xFFFF) != HAL_OK)
	{
		return;
	}

	HAL_Delay(100);
	hmc5883l_read(hi2c, magPosOff);

	hmc5883l_set[0] = 0x12;
	if (HAL_I2C_Mem_Write(hi2c, (uint16_t)(HMC5883L << 1), (uint16_t) (0x00), 1, hmc5883l_set, 1, (uint32_t)0xFFFF) != HAL_OK)
	{
		return;
	}

	HAL_Delay(100);
	hmc5883l_read(hi2c, magNegOff);

	hmc5883l_set[0] = 0x10;
	if (HAL_I2C_Mem_Write(hi2c, (uint16_t)(HMC5883L << 1), (uint16_t) (0x00), 1, hmc5883l_set, 1, (uint32_t)0xFFFF) != HAL_OK)
	{
		return;
	}

	magGain[0] = (float) 2500.0 / (float) (magPosOff[0] - magNegOff[0]);
	magGain[1] = (float) 2500.0 / (float) (-magNegOff[1] + magPosOff[1]);
	magGain[2] = (float) 2500.0 / (float) (-magNegOff[2] + magPosOff[2]);

	return;
}
