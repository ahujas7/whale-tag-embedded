/*
 * LightSensor.c
 *
 *  Created on: Feb. 9, 2023
 *      Author: Amjad Halis
 */

#include "LightSensor.h"

// Wait 100ms minimum after VDD is supplied to light sensor
HAL_StatusTypeDef Light_Sensor_Init(Light_Sensor_HandleTypedef *light_sensor, I2C_HandleTypeDef *hi2c_device) {
	HAL_StatusTypeDef ret_val = HAL_ERROR;
	light_sensor->i2c_handler = hi2c_device;
	// Maximum initial startup time is 1000 ms
	HAL_Delay(1000);

	ret_val = Light_Sensor_WakeUp(light_sensor, GAIN_DEF);
	// Wait 10ms maximum for wakeup time of light sensor
	// A non-blocking solution can probably be found
	HAL_Delay(10);

	// Uncomment lines below and change
	//Light_Sensor_Set_DataRate(light_sensor, INTEG_TIME_DEF, MEAS_TIME_DEF);

	return ret_val;

}

HAL_StatusTypeDef Light_Sensor_WakeUp(Light_Sensor_HandleTypedef *light_sensor, ALS_Gain gain){
	HAL_StatusTypeDef ret_val = HAL_ERROR;
	uint8_t data_buf[1] = {0};
	data_buf[0] = (gain << 2) | LIGHT_WAKEUP;

	ret_val = HAL_I2C_Mem_Write(light_sensor->i2c_handler, ALS_ADDR << 1, ALS_CONTR, I2C_MEMADD_SIZE_8BIT, data_buf, 1, 100);

	return ret_val;
}


HAL_StatusTypeDef Light_Sensor_Set_DataRate(Light_Sensor_HandleTypedef *light_sensor, ALS_Integ_Time int_time, ALS_Meas_Rate meas_rate){
	HAL_StatusTypeDef ret_val = HAL_ERROR;
	uint8_t data_buf = (int_time << 3) & meas_rate;

	ret_val = HAL_I2C_Mem_Write(light_sensor->i2c_handler, ALS_ADDR << 1, ALS_MEAS_RATE, I2C_MEMADD_SIZE_8BIT, &data_buf, 1, 100);

	return ret_val;
}

HAL_StatusTypeDef Light_Sensor_Get_Data(Light_Sensor_HandleTypedef *light_sensor) {
	HAL_StatusTypeDef ret_val = HAL_ERROR;

	ret_val = HAL_I2C_Mem_Read(light_sensor->i2c_handler, ALS_ADDR << 1, ALS_STATUS, I2C_MEMADD_SIZE_8BIT, &(light_sensor->status), 1, 100);

	if(ret_val != HAL_OK){
		return ret_val;
	}

	// Check ALS data valid bit. If bit is 1, data is invalid
	if(light_sensor->status >> 7 == 1){
		return HAL_ERROR;
	}
	// Check data status bit. If 0 data is old
	if((light_sensor->status & 0b100) == 1){
		return HAL_ERROR;
	}

	ret_val = HAL_I2C_Mem_Read(light_sensor->i2c_handler, ALS_ADDR << 1, ALS_DATA, I2C_MEMADD_SIZE_8BIT, light_sensor->raw_data, ALS_DLEN, 100);

	if(ret_val != HAL_OK){
		return ret_val;
	}

	// TODO: Need to request lux formula from
	light_sensor->infrared = light_sensor->raw_data[1] << 8 | light_sensor->raw_data[0];
	light_sensor->visible = light_sensor->raw_data[3] << 8 | light_sensor->raw_data[2];

	return ret_val;
}

HAL_StatusTypeDef Light_Sensor_Sleep(Light_Sensor_HandleTypedef *light_sensor){
	HAL_StatusTypeDef ret_val = HAL_ERROR;
	uint8_t data_buf[1] = {0};
	data_buf[0] = (light_sensor->gain << 2) | LIGHT_SLEEP;

	ret_val = HAL_I2C_Mem_Write(light_sensor->i2c_handler, ALS_ADDR << 1, ALS_CONTR, I2C_MEMADD_SIZE_8BIT, data_buf, 1, 100);

	return ret_val;
}
