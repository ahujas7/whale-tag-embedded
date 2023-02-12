/*
 * i2c_sensors.c
 *
 *      Author: Kelly Ostrom
 */

#include "i2c_sensors.h"
#include "cetiConfig.h"
#include "logging.h"

I2C_HandleTypeDef *_ds2778_i2c_port = NULL;
I2C_HandleTypeDef *_keller_i2c_port = NULL;
I2C_HandleTypeDef *_light_i2c_port = NULL;

static int presentState = ST_CONFIG;
static int timeout_minutes, timeout_seconds;

double pressureData[2] = {0};
double batteryData[3] = {0};
int32_t ambientLight = 0;
char gpsLocation[GPS_LOCATION_LENGTH] = "No GPS update available";
char textToLog[512] = "";
bno055_vector_t vectorData;
uint8_t _shutdown_flag = 0;


// Assignment initialization functions
void fuelGauge_assignI2C(I2C_HandleTypeDef *hi2c_device) {
  _ds2778_i2c_port = hi2c_device;
}

void keller_assignI2C(I2C_HandleTypeDef *hi2c_device) {
  _keller_i2c_port = hi2c_device;
}

void light_assignI2C(I2C_HandleTypeDef *hi2c_device) {
  _light_i2c_port = hi2c_device;
}


// Read functions
HAL_StatusTypeDef fuelGauge_readData(uint8_t reg, uint8_t *data, uint8_t len) {
	HAL_StatusTypeDef hal_status;
	hal_status = HAL_I2C_Master_Transmit(_ds2778_i2c_port, BMS_ADDR << 1, &reg, 1,
		  	 								100);
	hal_status = HAL_I2C_Master_Receive(_ds2778_i2c_port, BMS_ADDR << 1, data, len,
							   	   	   	   	100);
	return hal_status;
}

HAL_StatusTypeDef keller_readData(uint8_t reg, uint8_t *data, uint8_t len) {
	HAL_StatusTypeDef hal_status;
	hal_status = HAL_I2C_Master_Transmit(_keller_i2c_port, KELLER_ADDR << 1, &reg, 1,
		  	 								100);
	HAL_Delay(20);
	hal_status = HAL_I2C_Master_Receive(_keller_i2c_port, KELLER_ADDR << 1, data, len,
							   	   	   	   	100);
	return hal_status;
}

HAL_StatusTypeDef light_readData(uint8_t reg, uint8_t *data, uint8_t len) {
	HAL_StatusTypeDef hal_status;
	hal_status = HAL_I2C_Master_Transmit(_light_i2c_port, LIGHT_ADDR << 1, &reg, 1,
		  	 								100);
	hal_status = HAL_I2C_Master_Receive(_light_i2c_port, LIGHT_ADDR << 1, data, len,
							   	   	   	   	100);
	return hal_status;
}

// Sensor initialization
/**
 * Based on the sensor flags, initializes the sensors
 */
int sensors_initI2C(void) {
#if USE_LIGHT_SNSR
		// Light sensor LTR-329 initialization
	  uint8_t lightCmd[2] = {0x80, 0x01};
	  HAL_I2C_Master_Transmit(_light_i2c_port, LIGHT_ADDR << 1, lightCmd, 2, 100);
#endif
#if USE_FUEL_GAUGE
	  //establish undervoltage cutoff
	  uint8_t underVolt[2] = {BATT_CTL, BATT_CTL_VAL};
	  HAL_I2C_Master_Transmit(_ds2778_i2c_port, BMS_ADDR << 1, underVolt, 2, 100);
	  //establish overvoltage cutoff
	  uint8_t overVolt[2] = {OVER_VOLTAGE, OV_VAL};
	  HAL_I2C_Master_Transmit(_ds2778_i2c_port, BMS_ADDR << 1, overVolt, 2, 100);
#endif
	  return 0;
}


void getBatteryStatus() {
	// BMS Battery Gas Gauge
	  uint8_t bms_result = 0;

	  // CELL 1
	  fuelGauge_readData(CELL_1_V_MS, &bms_result, 1);
	  short voltage = bms_result << 3;

	  bms_result = 0;
	  fuelGauge_readData(CELL_1_V_LS, &bms_result, 1);

	  voltage = (voltage | (bms_result >> 5));
	  batteryData[0] = 4.883e-3 * voltage;

	  // CELL 2
	  bms_result = 0;
	  fuelGauge_readData(CELL_2_V_MS, &bms_result, 1);
	  voltage = bms_result << 3;

	  bms_result = 0;
	  fuelGauge_readData(CELL_2_V_LS, &bms_result, 1);

	  voltage = (voltage | (bms_result >> 5));
	  batteryData[1] = 4.883e-3 * voltage;

	  // CURRENT
	  bms_result = 0;
	  fuelGauge_readData(BATT_I_MS, &bms_result, 1);

	  int16_t current = bms_result << 8;

	  bms_result = 0;
	  fuelGauge_readData(BATT_I_LS, &bms_result, 1);

	  current = current | bms_result;
	  batteryData[2] = 1000.0 * (double)current * (1.5625e-6 / R_SENSE);
}


void getPressureData() {
	// Keller depth/pressure sensor: move to new file later
	uint8_t reg = 0xAC;
	uint8_t kellerData[5] = {0};
	int32_t temperature, pressure;

	keller_readData(reg, kellerData, 5);

	temperature = kellerData[3] << 8;
	temperature = temperature + kellerData[4];

	// convert to deg C
	// convert to bar - see Keller data sheet for the particular sensor in use
	pressureData[0] =  (float)((temperature >> 4) - 24) * .05 - 50;
	pressure = kellerData[1] << 8;
	pressure = pressure + kellerData[2];
	// convert to bar
	pressureData[1] = (200.0f / 32768.0f) * ((float)pressure - 16384.0);
}


void getAmbientLight() {
	uint8_t reg = 0x88;
	uint8_t data[4] = {0};
	uint8_t stat_reg = 0x8C;
	uint8_t status = 0;

	light_readData(stat_reg, &status, 1);

	if ((status & 0x04) > 0) {
		light_readData(reg, data, 4);

		// Combining lower and upper bytes to give 16-bit Ch1 data
		// Goes: (Ch1 << 16) | Ch0
		ambientLight = (((data[1] << 8) | data[0]) << 16) | ((data[3] << 8) | data[2]);
	}
}

void getIMUData() {
	vectorData = bno055_getVectorQuaternion();
}

// Gets all the data and then formats it all
int sensors_getAll(uint8_t* rtc_time, uint8_t* rtc_date) {
	// Get the data from the battery, the pressure/depth sensor, light sensor, and IMU
	// Also get GPS data
	if (!_shutdown_flag) {
		getBatteryStatus();
		getPressureData();
		getAmbientLight();
		getIMUData();

		sprintf(textToLog, "%s:%s [%s]: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %ld, %s\n",
			  rtc_date, rtc_time,
			  sensors_getStateStr(presentState),
			  pressureData[0], pressureData[1],
			  batteryData[0], batteryData[1], batteryData[2],
			  vectorData.w, vectorData.x, vectorData.y, vectorData.z,
			  ambientLight,
			  gpsLocation);
	}


	sensors_updateState();

	return 0;
}

int sensors_updatePeriodic() {

	logging_writeToFile(textToLog);

	return 0;
}

int sensors_updateState() {

    // timing
//    static unsigned int startTime = 0;

    // Deployment sequencer FSM

    switch (presentState) {
		case (ST_CONFIG):
			// Load the deployment configuration
			// the configuration file units are minutes
			timeout_seconds = T0 * 60;
			timeout_minutes = T0;

			presentState = ST_START;
			break;

		case (ST_START):
			// Originally the ADC capture started here but it won't anymore
			presentState = ST_DEPLOY;    // underway!
			break;

		case (ST_DEPLOY):

			// Waiting for 1st dive
			if (batteryData[0] + batteryData[1] < V_LOW) {//||
//				(rtcCount - startTime > timeout_seconds)) {
				// TODO fix this to pull the start time and current RTC so this can trigger
				burnwireOn();
				presentState = ST_BRN_ON;
				break;
			}

			if (pressureData[1] > V_CRIT)
				presentState = ST_REC_SUB; // 1st dive after deploy

			break;

		case (ST_REC_SUB):
			// Recording while sumberged
			if (batteryData[0] + batteryData[1] < V_LOW) { //||
//				(rtcCount - startTime > timeout_seconds)) {
				// TODO fix this to pull the start time and current RTC so this can trigger
				burnwireOn();
				presentState = ST_BRN_ON;
				break;
			}

			if (pressureData[1] < P1) {
				presentState = ST_REC_SURF; // came to surface
				break;
			}

			break;

		case (ST_REC_SURF):
			// Recording while at surface, trying to get a GPS fix

			if (batteryData[0] + batteryData[1] < V_LOW) { //||
//				(rtcCount - startTime > timeout_seconds)) {
				// TODO fix this to pull the start time and current RTC so this can trigger
				burnwireOn();
				presentState = ST_BRN_ON;
				break;
			}

			if (pressureData[1] > P2) {
				presentState = ST_REC_SUB; // back under....
				break;
			}
			break;

		case (ST_BRN_ON):
			// Releasing
			if (batteryData[0] + batteryData[1] < V_CRIT) {
				presentState = ST_SHUTDOWN; // critical battery
				break;
			}

			if (batteryData[0] + batteryData[1] < V_LOW) {
				presentState = ST_RETRIEVE; // low battery
				break;
			}

			// at surface, Recovery Board should be on, stop data capture
			if (pressureData[1] < P1) {
				// [TODO]: Stop the ADC audio data capture to preserve battery now
				// that the tag is on the surface
				presentState = ST_RETRIEVE; // low battery
				break;
			}

			// still under or resubmerged
			if (pressureData[1] > P2) {
				// [TODO]: keep recording the ADC audio data since still submerged
			}
			break;

		case (ST_RETRIEVE):
			//  Waiting to be retrieved.

			// critical battery
			if (batteryData[0] + batteryData[1] < V_CRIT) {
				presentState = ST_SHUTDOWN;
				break;
			}

			// low battery
			if (batteryData[0] + batteryData[1] < V_LOW) {
				burnwireOn(); // redundant, is already on
			}
			break;

		case (ST_SHUTDOWN):
			//  Shut everything off in an orderly way if battery is critical to
			//  reduce file system corruption risk
			burnwireOff();
			recoveryOff();
			_shutdown_flag = 1; // Stop logging sensors
			break;
    }
    return(0);
}

/**
 * Returns the name of a state as a string (char*)
 * @param state An enum state from the state machine
 */
const char* sensors_getStateStr(wt_state_t state) {
    if ( (state < ST_CONFIG) || (state > ST_UNKNOWN) ) {
//        CETI_LOG("get_state_str(): presentState is out of bounds. Setting to ST_UNKNOWN. Current value: %d", presentState);
        state = ST_UNKNOWN;
    }
    return state_str[state];
}


/**
 * Right now this just simply turns on an LED
 * since there's no burn wire connected
 */
int burnwireOn() {
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	return 0;
}

/**
 * Right now this just turns off the LED
 * since there's no burn wire connected
 */
int burnwireOff() {
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	return 0;
}

int recoveryOff() {
	// [TODO]: put the recovery board into the indefinite deep sleep.
	// Does not recover from this without a power cycle

	return 0;
}
