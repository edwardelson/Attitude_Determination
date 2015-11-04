/*
 * ee_mpu6050.c
 *
 *  Created on: Oct 11, 2015
 *      Author: Edward
 */

#include "ee_mpu6050.h"

//return 1 if initialization is successful, otherwise 0
uint8_t mpu6050_init(I2C_HandleTypeDef *hi2c)
{
	uint8_t mpu6050_set[6] = {0};

	mpu6050_set[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
	mpu6050_set[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
	mpu6050_set[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
	mpu6050_set[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g

	if (HAL_I2C_Mem_Write(hi2c, (uint16_t)(MPU6050 << 1), (uint16_t) (0x19), 1, mpu6050_set, 4, (uint32_t)0xFFFF) != HAL_OK)
	{
		return 0;
	}

	mpu6050_set[0] = 0x01;

	if (HAL_I2C_Mem_Write(hi2c, (uint16_t)(MPU6050 << 1) , (uint16_t) (0x6B), 1, mpu6050_set, 1, (uint32_t)0xFFFF) != HAL_OK)
	{
		return 0;
	}

	return 1;

}

// check whether we are talking to mpu6050
uint8_t mpu6050_status(I2C_HandleTypeDef *hi2c)
{
	uint8_t mpu6050_set[1] = {0};

	// Read "WHO_AM_I" register
	if(HAL_I2C_Mem_Read(hi2c, (uint16_t)(MPU6050 << 1), (uint16_t)0x75, 1, mpu6050_set, 1, (uint32_t)0xFFFF) == HAL_OK)
	{
		if (mpu6050_set[0] == 0x68) //MPU6050 address
		{
			return 1;
		}
	}

	return 0;
}

void mpu6050_read(I2C_HandleTypeDef* hi2c, float* acc, float* gyro)
{
	uint8_t mpu6050_data[14] = {0};
	int16_t mpu6050_int[6] = {0};

	/* read from 0x3B, memory address in MPU6050 */
	if(HAL_I2C_Mem_Read(hi2c, (uint16_t)(MPU6050 << 1), (uint16_t)0x3B, 1, mpu6050_data, 14, (uint32_t)0xFFFF) == HAL_OK)
	{

		mpu6050_int[0] = (mpu6050_data[0] << 8) | mpu6050_data[1];
		mpu6050_int[1] = -((mpu6050_data[2] << 8) | mpu6050_data[3]);
		mpu6050_int[2] = (mpu6050_data[4] << 8) | mpu6050_data[5];
		mpu6050_int[3] = (-(mpu6050_data[8] << 8)) | mpu6050_data[9];
		mpu6050_int[4] = (mpu6050_data[10] << 8) | mpu6050_data[11];
		mpu6050_int[5] = (-(mpu6050_data[12] << 8)) | mpu6050_data[13];

		acc[0] = (float) mpu6050_int[0];
		acc[1] = (float) mpu6050_int[1];
		acc[2] = (float) mpu6050_int[2];
		gyro[0] = (float) mpu6050_int[3];
		gyro[1] = (float) mpu6050_int[4];
		gyro[2] = (float) mpu6050_int[5];

	}

	return;

//	void updateMPU6050() {
//	  while (i2cRead(MPU6050, 0x3B, i2cData, 14)); // Get accelerometer and gyroscope values
//	  accX = ((i2cData[0] << 8) | i2cData[1]);
//	  accY = -((i2cData[2] << 8) | i2cData[3]);
//	  accZ = ((i2cData[4] << 8) | i2cData[5]);
//	  tempRaw = (i2cData[6] << 8) | i2cData[7];
//	  gyroX = -(i2cData[8] << 8) | i2cData[9];
//	  gyroY = (i2cData[10] << 8) | i2cData[11];
//	  gyroZ = -(i2cData[12] << 8) | i2cData[13];
//	}
}
