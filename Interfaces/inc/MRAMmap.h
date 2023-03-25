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
    ,StateTransponderEnabled=4
    ,StateCommandTimeCheck=5
    ,StateMinMaxCRCError=6
    ,StateIHUInCommand=7
    ,StateTransmitInhibit=8
    ,StateInOrbit=9
    ,StateNormalRfPowerLevel=10
    ,StateSafeRfPowerLevel=11
    ,StateExp1Disabled
    ,StateSpare1, StateSpare2, StateSpare3
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
    uint32_t SpareData[15];
    uint8_t  NonVolatileStates[MaxStates][2];

} StateSavingMRAM_t;



typedef struct _authKey {
    uint8_t key[16];
    uint32_t keyChecksum;
    uint32_t magic; //Make sure it was initialized
} AuthKey_t;

#define MRAM_VERSION 1

/* This is a very simple file allocation table for testing the Pacsat Directory */
typedef struct mram_node {
    uint32_t upload_time;
    uint32_t file_id;
    uint32_t address; /* The address in MRAM where the data starts for this file */
    uint32_t file_size; /* the number of bytes in the file */
    uint16_t body_offset; /* This is the length of the Pacsat Header */
} MRAM_FILE;

#define SIZE_OF_MRAM_FAT 1024

/* Top level MRAM storage map */
typedef struct {
        uint32_t MRAMVersion1; // This should always have the real version number
        StateSavingMRAM_t StatesInMRAM;
		AuthKey_t AuthenticateKey;
        /* Here is the file system. */
		uint32_t NumberOfFiles;
		MRAM_FILE MRAMFiles[SIZE_OF_MRAM_FAT];
        uint32_t MRAMVersion2; // This is likely to be wrong if something above changed size
} MRAMmap_t;



/*
 * Gag...this will give a non-very-descriptive compile-time warning if MRAM_t gets too big.
 * No, you can't do this with #if.  Yes I could write it to look nicer, but not much.  It's in
 * a struct, btw, so that it will not actually allocate anything unless someone uses it in a declaration.
 */
#define MR25H40_SIZE ((1024*1024*4)/8) // 4 Megabits in bytes
struct _ASSERT {
	char x[(sizeof(MRAMmap_t) < MR25H40_SIZE)?1:-1];
};
#define MRAM_MIN8 0
#define MRAM_MIN12 0
#define MRAM_MIN16 0
#define MRAM_MAX8 255
#define MRAM_MAX12 4095
#define MRAM_MAX16 0xFFFF


#define CRClen(x)  sizeof(x)/4-1  /* compute the number of 32-bit words required to compute a MRAM CRC field */

#endif
