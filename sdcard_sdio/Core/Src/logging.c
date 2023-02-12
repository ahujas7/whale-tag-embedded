/*
 * logging.c
 *
 *      Author: Kelly Ostrom
 */

#include "logging.h"
#include <stdio.h>

uint32_t byteswritten = 0;
char cetiSensorFilename[13] = "";

void logging_setFilename(uint8_t* time) {
	snprintf(cetiSensorFilename, 13, "%s.txt", time);

}

void logging_writeToFile(const char* const line) {
	FRESULT res; /* FatFs function common result code */

		//Open file for writing (Create)
		if(f_open(&SDFile, cetiSensorFilename, FA_OPEN_APPEND | FA_WRITE) == FR_OK) {
			//Write to the text file
			res = f_write(&SDFile, line, strlen((char *)line), (void *)&byteswritten);
			if((byteswritten == 0) || (res != FR_OK)) {
//				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
				Error_Handler();

			} else {
//				  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

				f_close(&SDFile);
			}
		} else {
//			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
			Error_Handler();
		}


}

int logging_mountSD() {
	return f_mount(&SDFatFS, (TCHAR const*)SDPath, 1);
}

int logging_unmountSD() {
	return f_mount(&SDFatFS, (TCHAR const*)NULL, 0);
}
