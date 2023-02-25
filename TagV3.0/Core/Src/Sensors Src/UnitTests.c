/*
 * UnitTests.c
 *
 *  Created on: Feb. 9, 2023
 *      Author: amjad
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "UnitTests.h"

void Keller_UT(Keller_HandleTypedef *keller_sensor){
	Keller_Get_Data(keller_sensor);
	printf("Keller Sensor Unit Test\r\n");

	if(fabs(keller_sensor->pressure - ATM_PRES) > PRES_TOL){
		printf("\tPressure Failed: %.2fatm (1 atm ref)\r\n", keller_sensor->pressure);
	}
	else{
		printf("\tPressure Passed: %.2fatm (1 atm ref)\r\n", keller_sensor->pressure);
	}

	if(fabs(keller_sensor->temperature - AMB_TEMP) > TEMP_TOL){
		printf("\tTemperature Failed: %.2fC\r\n", keller_sensor->temperature);
	}
	else{
		printf("\tTemperature Passed: %.2fC\r\n", keller_sensor->temperature);
	}
}

void Light_UT(Light_Sensor_HandleTypedef *light_sensor){
	// ALS Integration Time is the measurement time for each ALS cycle
	// Set a delay with the maximum integration time to ensure new data is read
	HAL_Delay(350);
	Light_Sensor_Get_Data(light_sensor);
	printf("Light (ALS) Sensor Unit Test:\r\n");

	if(abs(light_sensor->visible - AVG_LUX) > LUX_TOL){
		printf("\tVisible Failed: %d lux\r\n", light_sensor->visible);
	}
	else{
		printf("\tVisible Passed: %d lux\r\n", light_sensor->visible);
	}

	if(abs(light_sensor->infrared - AVG_IR_LUX) > LUX_TOL){
		printf("\tInfrared Failed: %d lux\r\n", light_sensor->infrared);
	}
	else{
		printf("\tInfrared Passed: %d lux\r\n", light_sensor->infrared);
	}
}

// Function for printing to SWV
int _write(int file, char *ptr, int len) {
	int DataIdx;
	for (DataIdx = 0; DataIdx < len; DataIdx++) {
		ITM_SendChar(*ptr++);
	}
	return len;
}
