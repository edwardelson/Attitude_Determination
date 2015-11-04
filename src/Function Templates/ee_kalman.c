/*
 * ee_kalman.c
 *
 *  Created on: Oct 17, 2015
 *      Author: Edward
 *      Ported from TKJElectronics
 */

#include "ee_kalman.h"

//P must also be updated
float update_kalman (float newAngle, float newRate, float dt, float angle)
{
	//Variable Declaration
	static float Q_angle = 0.001; // Process noise variance for the accelerometer
	static float Q_bias = 0.003; // Process noise variance for the gyro bias
	static float R_measure = 0.03; // Measurement noise variance - this is actually the variance of the measurement noise

	static float bias = 0; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
	static float rate = 0; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

	static float P[2][2] = {{0,0},{0,0}}; // Error covariance matrix - This is a 2x2 matrix
	static float K[2]; // Kalman gain - This is a 2x1 vector
	static float y; // Angle difference
	static float S; // Estimate error

    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
     /* Step 1 */

	 rate = newRate - bias;
     angle += dt * rate;

     // Update estimation error covariance - Project the error covariance ahead
     /* Step 2 */
     P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
     P[0][1] -= dt * P[1][1];
     P[1][0] -= dt * P[1][1];
     P[1][1] += Q_bias * dt;

     // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
     // Calculate Kalman gain - Compute the Kalman gain
     /* Step 4 */
     S = P[0][0] + R_measure;
     /* Step 5 */
     K[0] = P[0][0] / S;
     K[1] = P[1][0] / S;

     // Calculate angle and bias - Update estimate with measurement zk (newAngle)
     /* Step 3 */
     y = newAngle - angle;
     /* Step 6 */
     angle += K[0] * y;
     bias += K[1] * y;

     // Calculate estimation error covariance - Update the error covariance
     /* Step 7 */
     P[0][0] -= K[0] * P[0][0];
     P[0][1] -= K[0] * P[0][1];
     P[1][0] -= K[1] * P[0][0];
     P[1][1] -= K[1] * P[0][1];

     return angle;
}
