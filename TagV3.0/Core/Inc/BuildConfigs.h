/*
 * BuildConfigs.h
 * This file can be used to specify what platform you're working on
 *  Created on: Feb. 9, 2023
 *      Author: amjad
 */

#ifndef INC_BUILDCONFIGS_H_
#define INC_BUILDCONFIGS_H_

//Use this macro to define what board you're working on
#define engboard
#ifndef engboard
#define TagV3
#endif

//Use this macro to dis/enable unit tests
#define UNIT_TESTS
#ifndef UNIT_TESTS
#define INTEGRATION
#endif

//Define the sensors being used
#define ECG
#define IMU
#define LIGHT_SNSR
#define DEPTH_SNSR
#define FUEL_GAUGE

//Use this macro to define that you're working on the recovery part
//#define RECOVERY

#endif /* INC_BUILDCONFIGS_H_ */
