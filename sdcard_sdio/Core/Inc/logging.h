/*
 * logging.h
 *
 *      Author: Kelly Ostrom
 */

#ifndef INC_LOGGING_H_
#define INC_LOGGING_H_

#include "fatfs.h"
#include "string.h"

#define ERR_LOG_FN "errorLog.txt"

extern char cetiSensorFilename[13];

extern uint32_t byteswritten, bytesread; /* File write/read counts */


void logging_setFilename(uint8_t* time);
void logging_writeToFile(const char* const line);
int logging_mountSD();
int logging_unmountSD();


#endif /* INC_LOGGING_H_ */
