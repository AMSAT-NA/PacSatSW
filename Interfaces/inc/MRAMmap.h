/* AMSAT Golf-1 MRAM Memory Map */
/* 12/27/2012  Mike McCann KB2GHZ for Fox
 * Lots of intermediate changes
 * 6/5/2019 Port to Golf0-1, Burns Fisher WB1FJ
 */

#ifndef _MRAM_MAP
#define _MRAM_MAP

#include <stdint.h>
#include <stdbool.h>
#include "downlink.h"
//#include "telemetryCollectionInterface.h"
#include "MET.h"

typedef uint32_t CRCtype_t;
/* IHU reset counter */
typedef struct {
	uint16_t IHUresetCnt;			/* reset counter (part of time) */
	uint16_t IHUresetCntComp;		/* complement of the reset counter */
	CRCtype_t Crc;
} IHUReset_t;

/* MRAM Min/Max Delta Times */
typedef struct {
	logicalTime_t deltaMin;
	logicalTime_t deltaMax;
	CRCtype_t Crc;
} MinMaxDelta_t;

enum _StateTypes {
     StateCommandedSafeMode=0
    ,StateAutoSafe=1
    ,StateAutoSafeAllow=2
    ,StateCommandReceived=3
    ,StatePbEnabled=4         // True if the PACSAT Broadcast is enabled
    ,StateCommandTimeCheck=5
    ,StateMinMaxCRCError=6
    ,StateUplinkEnabled=7     // True if the FTL0 Uplink is enabled
    ,StateTransmitInhibit=8
    ,StateInOrbit=9
    ,StateNormalRfPowerLevel=10
    ,StateSafeRfPowerLevel=11
    ,StateExp1Disabled
    ,StateAx25Rate9600       // True if we are using 9600bps packet
    ,StateSpare1, StateSpare2
    ,MaxStates
};

typedef struct {
	/* Each entry has two because we want extra bits for checking */
// These we don't want to change with each MRAM change.  They are generally set only once
// for each processor.
    uint32_t DCTRxFrequency[2];
    uint32_t DCTTxFrequency[2];
    uint32_t DCTDriveLowPower[2];
    uint32_t DCTDriveHighPower[2];
// These are initted by init mram or preflight init
	uint32_t WODHkDownlinkIndex[2];
	uint32_t WODSciDownlinkIndex[2];
	uint32_t WODRagDownlinkIndex[2];
	uint32_t WODHkStoreIndex[2];
	uint32_t WODSciStoreIndex[2];
	uint32_t WODRagStoreIndex[2];
	uint32_t TimeSinceFirstBoot[2];
    uint32_t MinMaxResetTimeSecs[2];
    uint16_t MinMaxResetTimeEpoch[2];
	uint16_t WODFrequency[2];
	uint16_t WODSize[2];
	uint16_t NumberOfResets[2];
	uint16_t TimestampResets[2];
	uint16_t AutoSafeEnter[2];
	uint16_t AutoSafeExit[2];
    uint32_t TimeoutTimes[MaxNumberOfTimeouts][2];
    uint32_t PostLVTimeout[2];
    uint16_t  EclipseActionCommanded[2];
    uint32_t HighestFileNumber[2];
    uint32_t SpareData[13];
    uint8_t  NonVolatileStates[MaxStates][2];

} StateSavingMRAM_t;



typedef struct _authKey {
    uint8_t key[16];
    uint32_t keyChecksum;
    uint32_t magic; //Make sure it was initialized
} AuthKey_t;

/* This stores the details of an in process file upload */
typedef struct _inProcessFileUpload {
    char callsign[MAX_CALLSIGN_LEN]; /* The callsign of the stations that initiated the upload */
    uint32_t file_id; /* The file id that was allocated by the dir.  A standard function calculates the tmp file name on disk */
    uint32_t length;  /* The promised length of the file given by the station when it requested the upload */
    uint32_t offset;  /* The offset at the end of the latest block uploaded */
    uint32_t request_time; /* The date/time that this upload was requested */
} InProcessFileUpload_t;


#define MRAM_VERSION 5

/* Top level MRAM storage map */
typedef struct {
        uint32_t testSize[2]; // This will be used to check the address size of the MRAM (it is stored in the status register)
        uint32_t MRAMVersion1; // This should always have the real version number
        StateSavingMRAM_t StatesInMRAM;
		AuthKey_t AuthenticateKey;
		InProcessFileUpload_t FileUploadsTable[MAX_IN_PROCESS_FILE_UPLOADS];
        uint32_t MRAMVersion2; // This is likely to be wrong if something above changed size
} MRAMmap_t;



/*
 * Gag...this will give a non-very-descriptive compile-time warning if MRAM_t gets too big.
 * No, you can't do this with #if.  Yes I could write it to look nicer, but not much.  It's in
 * a struct, btw, so that it will not actually allocate anything unless someone uses it in a declaration.
 */
#define MR25H40_SIZE ((1024*1024*4)/8) // 4 Megabits in bytes
//struct _ASSERT {
//	char x[(sizeof(MRAMmap_t) < MR25H40_SIZE)?1:-1];
//};
#define MRAM_MIN8 0
#define MRAM_MIN12 0
#define MRAM_MIN16 0
#define MRAM_MAX8 255
#define MRAM_MAX12 4095
#define MRAM_MAX16 0xFFFF


#define CRClen(x)  sizeof(x)/4-1  /* compute the number of 32-bit words required to compute a MRAM CRC field */

#endif
