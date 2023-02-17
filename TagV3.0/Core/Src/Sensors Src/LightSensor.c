/*
 * LightSensor.c
 *
 *  Created on: Feb. 9, 2023
 *      Author: Amjad Halis
 */

#include "LightSensor.h"

// Wait 100ms minimum after VDD is supplied to light sensor
void Light_Sensor_Init(Light_Sensor_HandleTypedef *light_sensor, I2C_HandleTypeDef *hi2c_device) {
	light_sensor->i2c_handler = hi2c_device;

	// Wait 10ms maximum for wakeup time of light sensor
	// A non-blocking solution can probably be found
	Light_Sensor_WakeUp(light_sensor, GAIN_DEF);

	// Uncomment lines below and change
	//HAL_Delay(10);
	//Light_Sensor_Set_DataRate(light_sensor, INTEG_TIME_DEF, MEAS_TIME_DEF);
}

HAL_StatusTypeDef Light_Sensor_WakeUp(Light_Sensor_HandleTypedef *light_sensor, ALS_Gain gain){
	HAL_StatusTypeDef ret_message = HAL_OK;
	uint8_t* data_buf= NULL;
	uint8_t data = (gain << 2) & LIGHT_WAKEUP;

	*data_buf = data;

	ret_message = HAL_I2C_Mem_Write(light_sensor->i2c_handler, ALS_ADDR << 1, ALS_CONTR, I2C_MEMADD_SIZE_8BIT, data_buf, 1, 100);

	return ret_message;
}

HAL_StatusTypeDef Light_Sensor_Set_DataRate(Light_Sensor_HandleTypedef *light_sensor, ALS_Integ_Time int_time, ALS_Meas_Rate meas_rate){
	HAL_StatusTypeDef ret_message = HAL_OK;
	uint8_t* data_buf = NULL;
	uint8_t data = (int_time << 3) & meas_rate;

	*data_buf = data;

	ret_message = HAL_I2C_Mem_Write(light_sensor->i2c_handler, ALS_ADDR << 1, ALS_MEAS_RATE, I2C_MEMADD_SIZE_8BIT, data_buf, 1, 100);

	return ret_message;
}

HAL_StatusTypeDef Light_Sensor_Get_Data(Light_Sensor_HandleTypedef *light_sensor) {
	HAL_StatusTypeDef ret_message = HAL_OK;

	ret_message = HAL_I2C_Mem_Read(light_sensor->i2c_handler, ALS_ADDR << 1, ALS_STATUS, I2C_MEMADD_SIZE_8BIT, &(light_sensor->status), 1, 100);

	if(ret_message != HAL_OK){
		return ret_message;
	}

	// Check data valid bit, if 1 data is invalid
	if(light_sensor->status >> 7){
		return HAL_ERROR;
	}
	// Check data status bit, if 0 data is old
	if(~(light_sensor->status & 0b10)){
		return HAL_ERROR;
	}

	ret_message = HAL_I2C_Mem_Read(light_sensor->i2c_handler, ALS_ADDR << 1, ALS_DATA, I2C_MEMADD_SIZE_8BIT, light_sensor->raw_data, ALS_DATA_LEN, 100);

	light_sensor->infrared = light_sensor->raw_data[1] << 8 & light_sensor->raw_data[0];
	light_sensor->visible = light_sensor->raw_data[3] << 8 & light_sensor->raw_data[2];

	return ret_message;
}
