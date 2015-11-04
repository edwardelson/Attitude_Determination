/*
 * ee_3Dcoordinates.h
 *
 *  Created on: Oct 17, 2015
 *      Author: Edward
 */

#ifndef FUNCTION_TEMPLATES_EE_3DCOORDINATES_H_
#define FUNCTION_TEMPLATES_EE_3DCOORDINATES_H_

#include "stm32f4xx_hal.h"
#include "math.h"

#define RAD_TO_DEG 57.296
#define DEG_TO_RAD 0.017

void obtain_yaw (float*, float*, float*, float*, float, float);
void obtain_roll (float*, float*);
void obtain_pitch (float*, float*);

#endif /* FUNCTION_TEMPLATES_EE_3DCOORDINATES_H_ */
