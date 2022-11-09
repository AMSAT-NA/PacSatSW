/* Project Fox */
/* Mission Elapsed Time (MET) Interface */
/* 12/20/2012  Mike McCann KB2GHZ */
#ifndef MET_H_
#define MET_H_


#include <pacsat.h>
#include <stdint.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "os_queue.h"
#include "os_semphr.h"
#include "os_task.h"

typedef struct {
			uint32_t METcount;      /* 1 second time periods since the last IHU restart */
			uint16_t IHUresetCnt;	/* IHU reset event counter */
} logicalTime_t;

typedef struct {
	logicalTime_t LogicalTime;
	portTickType TicksAtLastSecond;
} highResolutionTime_t;

typedef enum {
    NoCommandTimeout = 0,
    NoTimeCommandTimeout,
    MinMaxResetTimeout,
    MaxNumberOfTimeouts
}TimeoutType;

/* Routines for initialization and debug*/
void initMET(void);
void initSecondsInOrbit(void);
void InitResetCnt(void);
void EnableTimePrint(bool,int);

/* These are for the local time */
void getTime(logicalTime_t * ld);
uint32_t getSeconds(void);
uint16_t getResets(void);

/* These are for the telemetry timestamp/epoc */
void getTimestamp(logicalTime_t *tm);
uint16_t getEpoch(void);

/* This is for the continually incrementing time-in-orbit*/
uint32_t getSecondsInOrbit(void);
void SaveSecondsInOrbit(); // Periodically called to ensure current value is in NVRam. Also look for things to do at this time

/* These allow us to do stuff based on the seconds in orbit.  Not too much happening yet.  May move to another
 * module
 */
void SetInternalSchedules(TimeoutType type, uint32_t seconds);
void GetTimeoutTimes(uint32_t *);
void CheckScheduledEvents();


/* These are for updating among the CPUs */

void UpdateTelemEpoch(uint16_t resets);
void UpdateSecondsInOrbit(uint32_t seconds);
void ReportPSOCEpoch(uint16_t epoch); // Called by CAN to say the PSOC has send an epoch
uint16_t GetPSOCEpoch(void); // Get the value the PSOC gave us

/* These all relate to timing at startup*/

void METTelemetryReady(void); //Notification from telemetry that it's running so MET can send time messages
void StartStableCount(void); // Notification that all tasks are running and we can start countdown to stable
bool IsStabilizedAfterBoot(void); // Returns true or false that stable time has passed


#endif
