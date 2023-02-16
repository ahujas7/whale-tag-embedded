/*
 * UnitTests.c
 *
 *  Created on: Feb. 9, 2023
 *      Author: amjad
 */

#include "UnitTests.h"

uint8_t Keller_Pressure_UT(Keller_HandleTypedef *keller_sensor){
	Keller_Get_Data(keller_sensor);

	if((keller_sensor->pressure - ATM_PRES) < PRES_TOL){
		return 1;
	}

	return 0;
}

uint8_t Keller_Temp_UT(Keller_HandleTypedef *keller_sensor){
	Keller_Get_Data(keller_sensor);

	if((keller_sensor->pressure - AMB_TEMP) < TEMP_TOL){
		return 1;
	}

	return 0;
}
