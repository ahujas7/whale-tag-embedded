/*
 * LightSensor.h
 *
 *  Created on: Feb. 9, 2023
 *      Author: Amjad Halis
 *      Sensor: LTR-329ALS-01_DS_V1
 *      Datasheet: optoelectronics.liteon.com/upload/download/DS86-2014-0006/LTR-329ALS-01_DS_V1.pdf
 */

#ifndef INC_LIGHTSENSOR_H_
#define INC_LIGHTSENSOR_H_

#include "stm32l4xx_hal.h"

// ALS == Ambient Light Sensor
// ALS register map. Only the relevant registers are included
#define ALS_ADDR	 	0x29

// Read/Write
#define ALS_CONTR		0x80
#define ALS_MEAS_RATE	0x85

// Read only registers
#define ALS_DATA		0x88
// Channel 1 is the IR diode
// 0x88 Channel 1 Lower byte
// 0x89 Channel 1 Upper byte
// Channel 0 is the visible + IR diode
// 0x8A Channel 0 Lower byte
// 0x8B Channel 0 Upper byte

#define ALS_DATA_LEN	4

#define ALS_STATUS		0x8C
// Bit 7   Data Valid: 0 == valid, 1 == invalid
// Bit 6:4 Data gain range (below)
// Bit 2   Data Status: 0 == old/read data, 1 == new data

// ALS_CONTR register: ALS Gain set in bits 4:2
typedef enum {
    GAIN_DEF	= 0b000,	// Gain x1 -> 1 lux to 64k lux
    GAIN_2X		= 0b001,	// Gain x2 -> 0.5 lux to 32k lux
    GAIN_4X		= 0b010,	// Gain x4 -> 0.25 lux to 16k lux
    GAIN_8X		= 0b011,	// Gain x8 -> 0.125 lux to 8k lux
    RESERVED0	= 0b100,	// Invalid value
    RESERVED1	= 0b101,	// Invalid value
    GAIN_48X	= 0b110,	// Gain x48 -> 0.02 lux to 1.3k lux
    GAIN_96X	= 0b111,	// Gain x96 -> 0.01 lux to 600 lux
} ALS_Gain;
// ALS_CONTR register: SW reset set in bit 1. Set to one to start a reset
#define LIGHT_RESET		0b10
// ALS_CONTR register: ALS mode set in bit 0. Set to one to put in active mode
#define LIGHT_WAKEUP	0b01

// ALS_MEAS_RATE register: ALS integration time set in bits 5:3
typedef enum {
    INTEG_TIME_DEF	= 0b000,	// 100ms (default)
	INTEG_TIME_2	= 0b001,	// 50ms
	INTEG_TIME_3	= 0b010,	// 200ms
	INTEG_TIME_4	= 0b011,	// 400ms
	INTEG_TIME_5	= 0b100,	// 150ms
	INTEG_TIME_6	= 0b101,	// 250ms
	INTEG_TIME_7	= 0b110,	// 300ms
	INTEG_TIME_8	= 0b111,	// 350ms
} ALS_Integ_Time;

// ALS_MEAS_RATE register: ALS measurement rate set in bits 2:0
typedef enum {
    MEAS_TIME_1		= 0b000,	// 50ms
	MEAS_TIME_2		= 0b001,	// 100ms
	MEAS_TIME_3		= 0b010,	// 200ms
	MEAS_TIME_DEF	= 0b011,	// 500ms (default)
	MEAS_TIME_5		= 0b100,	// 1000ms
	MEAS_TIME_6		= 0b101,	// 2000ms
	MEAS_TIME_7		= 0b110,	// 2000ms
	MEAS_TIME_8		= 0b111,	// 2000ms
} ALS_Meas_Rate;


typedef struct __Light_Sensor_TypeDef
{
	I2C_HandleTypeDef *i2c_handler;

	ALS_Gain gain;

	// Data buffers for I2C data
	uint8_t raw_data[4];
	uint8_t status;

	// Channel 0 is the visible + IR diode
	uint16_t visible;

	// Channel 1 is the IR diode
	uint16_t infrared;

} Light_Sensor_HandleTypedef;

void Light_Sensor_Init(Light_Sensor_HandleTypedef *light_sensor, I2C_HandleTypeDef *hi2c_device);
HAL_StatusTypeDef Light_Sensor_WakeUp(Light_Sensor_HandleTypedef *light_sensor, ALS_Gain gain);
HAL_StatusTypeDef Light_Sensor_Set_DataRate(Light_Sensor_HandleTypedef *light_sensor, ALS_Integ_Time int_time, ALS_Meas_Rate meas_rate);
HAL_StatusTypeDef Light_Sensor_Get_Data(Light_Sensor_HandleTypedef *light_sensor);


#endif /* INC_LIGHTSENSOR_H_ */
