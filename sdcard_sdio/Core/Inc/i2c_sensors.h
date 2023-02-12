/*
 * i2c_sensors.h
 *
 *      Author: Kelly Ostrom
 */

#ifndef INC_I2C_SENSORS_H_
#define INC_I2C_SENSORS_H_

#include <stdint.h>
#include "stm32l4xx_hal.h"
#include "bno055.h"

// I2C Device Address definitions
#define KELLER_ADDR 0x40
#define LIGHT_ADDR 0x29
#define BMS_ADDR 0x59

// BMS DS2778 Register definitions
#define PROTECT 0x00
#define BATT_CTL 0x60
#define OVER_VOLTAGE 0x7F
#define CELL_1_V_MS 0x0C
#define CELL_1_V_LS 0x0D
#define CELL_2_V_MS 0x1C
#define CELL_2_V_LS 0x1D
#define BATT_I_MS 0x0E
#define BATT_I_LS 0x0F
#define R_SENSE 0.025

#define BATT_CTL_VAL 0X8E  //SETS UV CUTOFF TO 2.6V
#define OV_VAL 0x5A //SETS OV CUTOFF TO 4.2V

#define MAX_STATE_STRING_LEN (32) // For the name of the states
#define GPS_LOCATION_LENGTH (255)

// Type defines for managing I2C busses
extern I2C_HandleTypeDef *_ds2778_i2c_port;
extern I2C_HandleTypeDef *_keller_i2c_port;
extern I2C_HandleTypeDef *_light_i2c_port; // Ambient light sensor LiteON LTR-329ALS-01
extern double pressureData[2];
extern double batteryData[3];
extern int32_t ambientLight;
extern char gpsLocation[GPS_LOCATION_LENGTH];
extern char lineToLog[512];
extern bno055_vector_t vectorData;
extern uint8_t _shutdown_flag;

typedef enum {       // Tag operational states for deployment sequencing
    ST_CONFIG = 0,   // get the deployment parameters from config file
    ST_START,        // turn on the audio recorder, illuminate ready LED
    ST_DEPLOY,       // wait for whale to dive
    ST_REC_SUB,      // recording while underwater
    ST_REC_SURF,     // recording while surfaced - trying for a GPS fix
    ST_BRN_ON,       // burnwire is on, may or may not be at the surface when in
                     // this state, keep recording until battery is lower
    ST_RETRIEVE,     // burnwire timed out, likely at surface, monitor GPS and
                     // transmit coord if enough battery. No more audio
					 // recording but still other sensors
    ST_SHUTDOWN,     // battery critical, put system in minimum power mode
    ST_UNKNOWN
} wt_state_t;

static const char state_str[][MAX_STATE_STRING_LEN] = {
    "CONFIG",
    "START",
    "DEPLOY",
    "REC_SUB",
    "REC_SURF",
    "BRN_ON",
    "RETRIEVE",
    "SHUTDOWN",
    "ST_UNKNOWN"
};

// Assignment initialization functions
void fuelGauge_assignI2C(I2C_HandleTypeDef *hi2c_device);
void keller_assignI2C(I2C_HandleTypeDef *hi2c_device);
void light_assignI2C(I2C_HandleTypeDef *hi2c_device);

// Sensor initialization functions
int sensors_initI2C(void);

// Sensor data getters
void getBatteryStatus();
void getPressureData();
void getAmbientLight();

// state machine
int sensors_updatePeriodic();
int sensors_updateState();
const char* sensors_getStateStr(wt_state_t state);
int sensors_getAll(uint8_t* rtc_time, uint8_t* rtc_date);

// helper functions
int burnwireOn();
int burnwireOff();
int recoveryOff();

#endif /* INC_I2C_SENSORS_H_ */
