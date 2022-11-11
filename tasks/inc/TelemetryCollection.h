/*
 * telemetryCollection.h
 *
 *  Created on: Mar 9, 2014
 *      Author: Mike McCann KB2GHZ
 */

#ifndef TELEMETRYCOLLECTION_H_
#define TELEMETRYCOLLECTION_H_

#include <pacsat.h>
#include "stdint.h"
#include "CANSupport.h"
#include "MET.h"

/* this structure is used to return the number of minimum and maximum
 * and maximum values that have been updated in MRAM
 */
typedef struct {
	uint8_t maxUpdate;
	uint8_t minUpdate;
} updateCounts_t;

#define DIAG_PAYLOAD_NUMBER_OF_BUFFERS 2
#define INVALID_BYTE 255

void TelemetryCollectTask(void *pvParameters);


uint32_t computeCRC(uint32_t *address, int words);
bool GetMramCRCState(void);
void SetMramCRCGood(void);
void SetMramCRCError(void);
void ChangeWODSaved(uint16_t size);
bool getBusVoltage( uint8_t *);
void getLastLIHUDiagTime(logicalTime_t *lastTime);
void getLastRTDiagTime(logicalTime_t *lastTime);
void SetInEclipse(bool eclipse);
#endif /* TELEMETRYCOLLECTION_H_ */
