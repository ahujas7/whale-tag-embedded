/*
 * KellerDepth.c
 *
 *  Created on: Feb. 9, 2023
 *      Author: amjad
 */

#include "KellerDepth.h"

I2C_HandleTypeDef *_keller_i2c_port = NULL;
double pressureData[2] = {0};

void Keller_Init(Keller_HandleTypedef *keller_sensor, I2C_HandleTypeDef *hi2c_device) {
	keller_sensor->keller_i2c_handler = hi2c_device;
}

//TODO: Add return value to add error handling capabilities
//TODO: Return an appropriate failed message in accordance with firmware state machine
void Keller_Get_Data(Keller_HandleTypedef *keller_sensor) {
	uint8_t data_buf[2] = {0};
	data_buf[0] = KELLER_W;
	data_buf[1] = KELLER_REQ;

	if(HAL_I2C_Master_Transmit(keller_sensor->keller_i2c_handler, KELLER_ADDR << 1, data_buf, 2, 100) != HAL_OK){
		return;
	}

	// Wait 8ms> or wait for EOC to go high (VDD) or check status byte for the busy flag
	HAL_Delay(8);

	data_buf[0] = KELLER_R;
	if(HAL_I2C_Master_Transmit(keller_sensor->keller_i2c_handler, KELLER_ADDR << 1, data_buf, 1, 100) != HAL_OK){
		return;
	}

	if(HAL_I2C_Master_Receive(keller_sensor->keller_i2c_handler, KELLER_ADDR << 1, keller_sensor->raw_data, KELLER_DLEN, 100)){
		return;
	}

	keller_sensor->status = keller_sensor->raw_data[0];
	keller_sensor->raw_pressure = ((uint16_t)keller_sensor->raw_data[1] << 8) | keller_sensor->raw_data[2];
	keller_sensor->raw_temp = ((uint16_t)keller_sensor->raw_data[3] << 8) | keller_sensor->raw_data[4];

	keller_sensor->temperature = TO_DEGC(keller_sensor->raw_temp);
	keller_sensor->pressure = TO_BAR(keller_sensor->raw_pressure);
}
