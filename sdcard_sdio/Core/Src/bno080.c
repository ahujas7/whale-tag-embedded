
#include "BNO080.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

//Global Variables
uint8_t sequenceNumber[6] = {0, 0, 0, 0, 0, 0}; //There are 6 com channels. Each channel has its own seqnum
uint8_t commandSequenceNumber = 0;				//Commands have a seqNum as well. These are inside command packet, the header uses its own seqNum per channel

//These Q values are defined in the datasheet but can also be obtained by querying the meta data records
//See the read metadata example for more info
int16_t rotationVector_Q1 = 14;
int16_t accelerometer_Q1 = 8;
int16_t linear_accelerometer_Q1 = 8;
int16_t gyro_Q1 = 9;
int16_t magnetometer_Q1 = 4;

SPI_HandleTypeDef *_bno08x_spi_port = NULL;

int bno080_initialization(SPI_HandleTypeDef *hspi)
{
	_bno08x_spi_port = hspi;

	uint8_t shtpData[MAX_PACKET_SIZE] = {0};

	// Start the restart and initialization
	CHIP_DESELECT(BNO080);
	RESET_HIGH();

	//Configure the BNO080 for SPI communication
	WAKE_HIGH();	//Before boot up the PS0/WAK pin must be high to enter SPI mode
	RESET_LOW();	//Reset BNO080
	HAL_Delay(11);	//Min length not specified in datasheet?
	RESET_HIGH();	//Bring out of reset
	// Internal initialization, wait for high on INT then low on INT for the first assertion
	HAL_Delay(91);

	// Now waiting for the advertisement data
	bno080_waitForLowINT(); //Wait until INT pin goes low.
	bno080_receiveSPIPacket(shtpData, MAX_PACKET_SIZE);

	// Waiting for the unsolicited response

	bno080_waitForLowINT(); //Wait until INT pin goes low.
	bno080_receiveSPIPacket(shtpData, MAX_PACKET_SIZE);

	// Check for the reset message
	bno080_waitForLowINT(); //Wait until INT pin goes low.
	bno080_receiveSPIPacket(shtpData, MAX_PACKET_SIZE);

//	//Check communication with device
//	uint8_t comData[6];
//	comData[0] = 0x06;
//	comData[1] = 0;
//	comData[2] = CHANNEL_CONTROL;
//	comData[3] = 0;
//	comData[4] = SHTP_REPORT_PRODUCT_ID_REQUEST; //Request the product ID and reset info
//	comData[5] = 0;							  //Reserved
////	//Transmit packet on channel 2, 2 bytes
//	bno080_sendSPIPacket(comData, 6);

//	bno080_waitForLowINT(); //Wait until INT pin goes low.
//	bno080_receiveSPIPacket(shtpData, MAX_PACKET_SIZE);


	return 0;
}

int bno080_sendFeatureCommand(uint8_t featureID, uint16_t timeDelta, uint32_t config) {
	// Format all the things
	uint8_t shtpData[21] = {0};
	uint8_t shtpResponse[MAX_PACKET_SIZE] = {0};

//	long microsDelta = (long)timeDelta * 1000L;

	// SHTP header portion of the code
	shtpData[0] = 0x15;
	shtpData[1] = 0;
	shtpData[2] = CHANNEL_CONTROL;
	// Everything else to set the command
	shtpData[4] = SHTP_REPORT_SET_FEATURE_COMMAND; // command for feature setting
	shtpData[5] = featureID;
	shtpData[6] = 0; //feature flags
	shtpData[7] = 0; // sensitivity (LSB)
	shtpData[8] = 0; // sensitivity (MSB)

	shtpData[9] = 0x10;//(microsDelta >> 0) & 0xFF;  //Report interval (LSB) in microseconds. 0x7A120 = 500ms
	shtpData[10] = 0x27; //(microsDelta >> 8) & 0xFF;  //Report interval
	shtpData[11] = 0x00; //(microsDelta >> 16) & 0xFF; //Report interval
	shtpData[12] = 0x00; //(microsDelta >> 24) & 0xFF; //Report interval (MSB)
	shtpData[13] = 0;								   //Batch Interval (LSB)
	shtpData[14] = 0;								   //Batch Interval
	shtpData[15] = 0;								   //Batch Interval
	shtpData[16] = 0;								   //Batch Interval (MSB)
	shtpData[17] = (config >> 0) & 0xFF;	   //Sensor-specific config (LSB)
	shtpData[18] = (config >> 8) & 0xFF;	   //Sensor-specific config
	shtpData[19] = (config >> 16) & 0xFF;	  //Sensor-specific config
	shtpData[20] = (config >> 24) & 0xFF;	  //Sensor-specific config (MSB)


//	bno080_waitForLowINT();
//	bno080_receiveSPIPacket(shtpResponse, MAX_PACKET_SIZE);

	if (HAL_GPIO_ReadPin(BNO080_INT_PORT, BNO080_INT_PIN) == GPIO_PIN_SET) {
		bno080_sendSPIPacket(shtpData, 21);
	}


	bno080_waitForLowINT();
	bno080_receiveSPIPacket(shtpResponse, MAX_PACKET_SIZE);

	bno080_waitForLowINT();
	bno080_receiveSPIPacket(shtpResponse, MAX_PACKET_SIZE);

	WAKE_LOW();
	bno080_waitForLowINT();
		bno080_receiveSPIPacket(shtpResponse, MAX_PACKET_SIZE);
		WAKE_HIGH();

	return 0;
}

int bno080_receiveSPIPacket(uint8_t *read, uint8_t maxLen) {
	uint8_t shtpHeader[4] = {0};

	// read in more data
	CHIP_SELECT(BNO080);
//	WAKE_HIGH();

	HAL_SPI_Receive(_bno08x_spi_port, shtpHeader, 4, 60);
	uint16_t dataLength = (((uint16_t)shtpHeader[1]) << 8) | ((uint16_t)shtpHeader[0]);
	dataLength &= ~(1 << 15); // clears the msbit

	if (dataLength != 0){
		//Packet is not empty
		dataLength -= 4; //Remove the header bytes from the data count

		if (dataLength > MAX_PACKET_SIZE) {
			dataLength = MAX_PACKET_SIZE;
		}
		if (dataLength > maxLen) {
			dataLength = maxLen;
		}
		//Read incoming data into the shtpData array
		HAL_SPI_Receive(_bno08x_spi_port, read, dataLength, 60);
	} else {
		CHIP_DESELECT();
		return 1;
	}


	CHIP_DESELECT();

	return 0;
}

int bno080_sendSPIPacket(uint8_t *write, uint8_t len)
{

	CHIP_SELECT(BNO080);

	HAL_SPI_Transmit(_bno08x_spi_port, write, len, 100);

	CHIP_DESELECT(BNO080);


	return 0;

}

bno080_vector_t bno080_getVectorQuaternion(void) {
	uint8_t pktBuff[256] = {0};

//	WAKE_LOW();
	bno080_waitForLowINT();
	bno080_receiveSPIPacket(pktBuff, 255);
	// get the shtp header to see how many bytes are available


	bno080_vector_t quat = {.w = 0, .x = 0, .y = 0, .z = 0};

	if (pktBuff[2] == 0x03) { // make sure we have the right channel
		quat.w = (pktBuff[14] << 8) + pktBuff[13];
		quat.x = (pktBuff[16] << 8) + pktBuff[15];
		quat.y = (pktBuff[18] << 8) + pktBuff[17];
		quat.z = (pktBuff[20] << 8) + pktBuff[19];
	}
//	WAKE_HIGH();

	return quat;
}


//Blocking wait for BNO080 to assert (pull low) the INT pin
//indicating it's ready for comm. Can take more than 104ms
//after a hardware reset
int bno080_waitForLowINT(void)
{

	for (uint32_t counter = 0; counter < 0xffffffff; counter++) //Don't got more than 255
	{
		if (HAL_GPIO_ReadPin(BNO080_INT_PORT, BNO080_INT_PIN) == GPIO_PIN_RESET)
		{
			return (1);
		}
	}
	return (0);
}



///////////////////////////////////////////////////////////////////////////


