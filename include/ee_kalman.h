/*
 * ee_kalman.h
 *
 *  Created on: Oct 17, 2015
 *      Author: Edward
 *
 *  Based on: Kristian Lauszus from TKJElectronics (https://github.com/TKJElectronics/KalmanFilter)
 */

#ifndef FUNCTION_TEMPLATES_EE_KALMAN_H_
#define FUNCTION_TEMPLATES_EE_KALMAN_H_

//Private Variable Declaration
//float Q_angle = 0; // Process noise variance for the accelerometer
//float Q_bias = 0; // Process noise variance for the gyro bias
//float R_measure = 0.03; // Measurement noise variance - this is actually the variance of the measurement noise
//
////float angle = 0; // The angle calculated by the Kalman filter - part of the 2x1 state vector
//float bias = 0; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
//float rate = 0; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate
//
//float P[2][2] = {{0}}; // Error covariance matrix - This is a 2x2 matrix
//float K[2]; // Kalman gain - This is a 2x1 vector
//float y; // Angle difference
//float S; // Estimate error

float update_kalman (float, float, float, float);

#endif /* FUNCTION_TEMPLATES_EE_KALMAN_H_ */
