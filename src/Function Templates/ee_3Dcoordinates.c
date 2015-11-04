/*
 * ee_3Dcoordinates.c
 *
 *  Created on: Oct 17, 2015
 *      Author: Edward
 */

#include "ee_3Dcoordinates.h"

void obtain_yaw (float* yaw, float* mag, float* magGain, float* magOffset, float kalX, float kalY)
{
	  mag[0] *= -1; // Invert axis - this it done here, as it should be done after the calibration
	  mag[2] *= -1;

	  mag[0] *= magGain[0];
	  mag[1] *= magGain[1];
	  mag[2] *= magGain[2];

	  mag[0] -= magOffset[0];
	  mag[1] -= magOffset[1];
	  mag[2] -= magOffset[2];

	  float rollAngle = kalX * DEG_TO_RAD;
	  float pitchAngle = kalY * DEG_TO_RAD;

	  float Bfy = mag[2] * sin(rollAngle) - mag[1] * cos(rollAngle);
	  float Bfx = mag[0] * cos(pitchAngle) + mag[1] * sin(pitchAngle) * sin(rollAngle) + mag[2] * sin(pitchAngle) * cos(rollAngle);
	  *yaw = atan2(-Bfy, Bfx) * RAD_TO_DEG;

	  *yaw *= -1;
}

void obtain_roll (float* roll, float* acc)
{
	*roll = atan2(acc[1], acc[2]) * RAD_TO_DEG;
	return;
}

void obtain_pitch (float* pitch, float* acc)
{
	*pitch = atan(-acc[0] / sqrt(acc[1] * acc[1] + acc[2] * acc[2])) * RAD_TO_DEG;
	return;
}
