/*
 * ee_functions.c
 *
 *  Created on: Oct 17, 2015
 *      Author: Edward
 */

#include "ee_functions.h"

//Convert Float to String with 3 significant digits
void ee_floatTostring (float float_data, char* string_data, uint8_t string_length)
{
	uint32_t number, decimal, dec_count;

	if (float_data < 0)
	{
		string_data[0] = '-';
		string_data++;
		string_length--;

		float_data = -float_data;
	}

	number = (int) float_data;
	decimal = ((int)(float_data*1000)) - number*1000;


	if (decimal < 10)
	{
		snprintf(string_data, string_length, "%d.00%d", number, decimal);
	}
	else if (decimal < 100)
	{
		snprintf(string_data, string_length, "%d.0%d", number, decimal);
	}
	else
	{
		snprintf(string_data, string_length, "%d.%d", number, decimal);
	}


	return;
}
