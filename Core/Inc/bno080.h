
#ifndef	_BNO080_H
#define	_BNO080_H

#include "main.h"
//////////////////////////////////////////////////////////////////////////

/**
 * @brief Definition for connected to SPI2 (APB1 PCLK = 42MHz)
 */
#define BNO080_SPI_CHANNEL		SPI2

#define BNO080_SPI_SCLK_PIN		LL_GPIO_PIN_10
#define BNO080_SPI_SCLK_PORT	GPIOB
#define BNO080_SPI_SCLK_CLK		LL_AHB1_GRP1_PERIPH_GPIOB

#define BNO080_SPI_MISO_PIN		LL_GPIO_PIN_2
#define BNO080_SPI_MISO_PORT	GPIOC
#define BNO080_SPI_MISO_CLK		LL_AHB1_GRP1_PERIPH_GPIOBC

#define BNO080_SPI_MOSI_PIN		LL_GPIO_PIN_3
#define BNO080_SPI_MOSI_PORT	GPIOC
#define BNO080_SPI_MOSI_CLK		LL_AHB1_GRP1_PERIPH_GPIOC

#define BNO080_SPI_CS_PIN		GPIO_PIN_4
#define BNO080_SPI_CS_PORT		GPIOB
#define BNO080_SPI_CS_CLK		LL_AHB1_GRP1_PERIPH_GPIOB

#define BNO080_PS0_WAKE_PIN		GPIO_PIN_13
#define BNO080_PS0_WAKE_PORT	GPIOC
#define BNO080_PS0_WAKE_CLK		LL_AHB1_GRP1_PERIPH_GPIOC

#define BNO080_RST_PIN			GPIO_PIN_3
#define BNO080_RST_PORT			GPIOB
#define BNO080_RST_CLK			LL_AHB1_GRP1_PERIPH_GPIOB

#define BNO080_INT_PIN			GPIO_PIN_5
#define BNO080_INT_PORT			GPIOB
#define BNO080_INT_CLK			LL_AHB1_GRP1_PERIPH_GPIOB

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
#define CHIP_SELECT(BNO080)		HAL_GPIO_WritePin(BNO080_SPI_CS_PORT, BNO080_SPI_CS_PIN, GPIO_PIN_RESET)
#define CHIP_DESELECT(BNO080)	HAL_GPIO_WritePin(BNO080_SPI_CS_PORT, BNO080_SPI_CS_PIN, GPIO_PIN_SET)

#define WAKE_HIGH()				HAL_GPIO_WritePin(BNO080_PS0_WAKE_PORT, BNO080_PS0_WAKE_PIN, GPIO_PIN_SET)
#define WAKE_LOW()				HAL_GPIO_WritePin(BNO080_PS0_WAKE_PORT, BNO080_PS0_WAKE_PIN, GPIO_PIN_RESET)

#define RESET_HIGH()			HAL_GPIO_WritePin(BNO080_RST_PORT, BNO080_RST_PIN, GPIO_PIN_SET)
#define RESET_LOW()				HAL_GPIO_WritePin(BNO080_RST_PORT, BNO080_RST_PIN, GPIO_PIN_RESET)
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

//Registers
enum Registers
{
	CHANNEL_COMMAND = 0,
	CHANNEL_EXECUTABLE = 1,
	CHANNEL_CONTROL = 2,
	CHANNEL_REPORTS = 3,
	CHANNEL_WAKE_REPORTS = 4,
	CHANNEL_GYRO = 5
};

//All the ways we can configure or talk to the BNO080, figure 34, page 36 reference manual
//These are used for low level communication with the sensor, on channel 2
#define SHTP_REPORT_COMMAND_RESPONSE 0xF1
#define SHTP_REPORT_COMMAND_REQUEST 0xF2
#define SHTP_REPORT_FRS_READ_RESPONSE 0xF3
#define SHTP_REPORT_FRS_READ_REQUEST 0xF4
#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define SHTP_REPORT_PRODUCT_ID_REQUEST 0xF9
#define SHTP_REPORT_BASE_TIMESTAMP 0xFB
#define SHTP_REPORT_SET_FEATURE_COMMAND 0xFD

//All the different sensors and features we can get reports from
//These are used when enabling a given sensor
#define SENSOR_REPORTID_ACCELEROMETER 0x01
#define SENSOR_REPORTID_GYROSCOPE 0x02
#define SENSOR_REPORTID_MAGNETIC_FIELD 0x03
#define SENSOR_REPORTID_LINEAR_ACCELERATION 0x04
#define SENSOR_REPORTID_ROTATION_VECTOR 0x05
#define SENSOR_REPORTID_GRAVITY 0x06
#define SENSOR_REPORTID_GAME_ROTATION_VECTOR 0x08
#define SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR 0x09
#define SENSOR_REPORTID_TAP_DETECTOR 0x10
#define SENSOR_REPORTID_STEP_COUNTER 0x11
#define SENSOR_REPORTID_STABILITY_CLASSIFIER 0x13
#define SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER 0x1E

//Record IDs from figure 29, page 29 reference manual
//These are used to read the metadata for each sensor type
#define FRS_RECORDID_ACCELEROMETER 0xE302
#define FRS_RECORDID_GYROSCOPE_CALIBRATED 0xE306
#define FRS_RECORDID_MAGNETIC_FIELD_CALIBRATED 0xE309
#define FRS_RECORDID_ROTATION_VECTOR 0xE30B

//Command IDs from section 6.4, page 42
//These are used to calibrate, initialize, set orientation, tare etc the sensor
#define COMMAND_ERRORS 1
#define COMMAND_COUNTER 2
#define COMMAND_TARE 3
#define COMMAND_INITIALIZE 4
#define COMMAND_DCD 6
#define COMMAND_ME_CALIBRATE 7
#define COMMAND_DCD_PERIOD_SAVE 9
#define COMMAND_OSCILLATOR 10
#define COMMAND_CLEAR_DCD 11

#define CALIBRATE_ACCEL 0
#define CALIBRATE_GYRO 1
#define CALIBRATE_MAG 2
#define CALIBRATE_PLANAR_ACCEL 3
#define CALIBRATE_ACCEL_GYRO_MAG 4
#define CALIBRATE_STOP 5

#define MAX_PACKET_SIZE 128 //Packets can be up to 32k but we don't have that much RAM.
#define MAX_METADATA_SIZE 9 //This is in words. There can be many but we mostly only care about the first 9 (Qs, range, etc)

typedef struct {
  double w;
  double x;
  double y;
  double z;
} bno080_vector_t;


extern SPI_HandleTypeDef *_bno08x_spi_port;

int bno080_initialization(SPI_HandleTypeDef *hspi);
int bno080_receiveSPIPacket(uint8_t *read, uint8_t maxLen);
int bno080_sendFeatureCommand(uint8_t featureID, uint16_t timeDelta, uint32_t config);
int bno080_sendSPIPacket(uint8_t *write, uint8_t len);

bno080_vector_t bno080_getVectorQuaternion();


int bno080_waitForLowINT(void);

#endif
