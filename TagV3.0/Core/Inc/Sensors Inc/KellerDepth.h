/*
 * KellerDepth.h
 *
 *  Created on: Feb. 9, 2023
 *      Author: amjad
 */

#ifndef INC_KELLERDEPTH_H_
#define INC_KELLERDEPTH_H_

#include "stm32l4xx_hal.h"

#define KELLER_ADDR 	0x40
#define KELLER_R		(KELLER_ADDR << 1) + 1
#define KELLER_W		(KELLER_ADDR << 1) + 0
#define KELLER_REQ		0xAC
#define KELLER_DLEN		5

//TODO: NEED TO FIND VALUES
#define P_MIN			-1.0f
#define P_MAX			30.0f

#define TO_DEGC(RAW_IN)	(float)(((RAW_IN >> 4) - 24.0f) * 0.05 - 50.0f)
#define TO_BAR(RAW_IN)	(float)((RAW_IN - 16384) * (P_MAX - P_MIN) / 32768 + P_MIN)

typedef struct __Keller_Depth_TypeDef
{
	I2C_HandleTypeDef *keller_i2c_handler;

	//Data buffer for I2C data
	uint8_t raw_data[5];

	//raw values read from sensor
	uint8_t status;
	uint16_t raw_pressure;
	uint16_t raw_temp;

	//Pressure in Bar
	float pressure;

	//Temperature in celsius
	float temperature;

} Keller_HandleTypedef;

void Keller_Init(Keller_HandleTypedef *keller_sensor, I2C_HandleTypeDef *hi2c_device);
void Keller_Get_Data(Keller_HandleTypedef *keller_sensor);

#endif /* INC_KELLERDEPTH_H_ */
