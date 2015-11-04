/*
 * ADXL335.h
 *
 *  Created on: Sep 24, 2015
 *      Author: Edward
 *
 *      ADXL335 Accelerometer -> Find Angle in Z-Axis
 *      equation derivation can be found in EG3301 Logbook page "ADXL335 Configuration"
 *
 *  Adapted from:
 *      http://wiring.org.co/learning/basics/accelerometer.html
 *      https://www.electronicsblog.net/simple-angle-meter-using-adxl335-accelerometer-arduino/*
 */

#ifndef ADXL335_H_
#define ADXL335_H_

/*
 Author: Edward
 ADXL335 Accelerometer -> Angle in Z-Axis
 derivation can be found in EG3301 logbook entry 24/9/2015
 http://wiring.org.co/learning/basics/accelerometer.html
 https://www.electronicsblog.net/simple-angle-meter-using-adxl335-accelerometer-arduino/
*/

//based on settings
#define MAX_ADC 1024
#define MAX_ADC_Voltage 3.3

//to be determined based on calibration
//refer to datasheet for position of calibration
#define vx_1g 615
#define vx_min_1g 384
#define vy_1g 624
#define vy_min_1g 392
#define vz_1g 607
#define vz_min_1g 408

double x_resolution = 1.0 * ((vx_1g - vx_min_1g) * MAX_ADC_Voltage / MAX_ADC) / 2; //typically 0.3 V/g, determined based on maximum and minimum calibration value
double y_resolution = 1.0 *((vy_1g - vy_min_1g) * MAX_ADC_Voltage / MAX_ADC) / 2; //typically 0.3 V/g
double z_resolution = 1.0 *((vz_1g - vz_min_1g) * MAX_ADC_Voltage / MAX_ADC) / 2; //typically 0.3 V/g
double vx_0g = 1.0 *((vx_1g + vx_min_1g) / 2 ) * MAX_ADC_Voltage / MAX_ADC;
double vy_0g = 1.0 *((vy_1g + vy_min_1g) / 2) * MAX_ADC_Voltage / MAX_ADC;
double vz_0g = 1.0 *((vz_1g + vz_min_1g) / 2) * MAX_ADC_Voltage / MAX_ADC;

double vx, vy, vz;
double x, y, z;
double angle_z;

double get_angle_acc(void);

#endif /* ADXL335_H_ */
