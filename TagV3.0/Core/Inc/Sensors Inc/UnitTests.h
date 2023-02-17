/*
 * UnitTests.h
 *
 *  Created on: Feb. 9, 2023
 *      Author: amjad
 */

#ifndef INC_UNITTESTS_H_
#define INC_UNITTESTS_H_

#include "KellerDepth.h"
#include "LightSensor.h"

// Atmospheric Pressure and ambient temperature
#define ATM_PRES	1.0f
#define AMB_TEMP	21.0f

// TODO: Find out what the output of the light sensor is and the average lux
// Average room lux
#define AVG_LUX		50

// Pressure and temperature tolerance
#define PRES_TOL	0.2f
#define TEMP_TOL	2.0f
#define LUX_TOL		5

uint8_t Keller_Pressure_UT(Keller_HandleTypedef *keller_sensor);

uint8_t Keller_Temp_UT(Keller_HandleTypedef *keller_sensor);

uint8_t Light_UT(Light_Sensor_HandleTypedef *light_sensor);

//TODO: Include all the necessary header files to write the unit tests

//TODO: Make a prototype of all the unit tests

//TODO: Make a prototype of the a function that would run all the unit tests based on the macros

#endif /* INC_UNITTESTS_H_ */
