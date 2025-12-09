/*
 * errors.h
 *
 *  Created on: Mar 18, 2013
 *      Author: fox
 */

#ifndef ERRORS_H_
#define ERRORS_H_

#include <pacsat.h>
#include "config.h"


/* Here are the errors that we can report */
typedef enum _error {
     Unspecified = 0
	,PowerCycle = 1
	,IntWatchdog=2
	,SoftwareReset=3
	,ExternalReset
	,OscFailure //5
	,StackOverflow
	,NMIExc
	,SPIInUse
	,SPIOperationTimeout
	,SPIMramTimeout //10
	,UnexpectedBehavior
	,SemaphoreFail
	,USARTError
	,DMAInUseTimeout
	,IllegalGPIOOutput //15
	,IllegalGPIOInput
	,IllegalGPIOWait
	,MRAMcrc
	,MRAMread
	,MRAMwrite //20
	,RTOSfailure
    ,I2CInUse
	,I2C1failure
	,I2C2failure
	,ControlQueueOverflow //25
	,ControlTimerNotStarted
	,CoordTimerNotStarted
	,CANWriteTimeout
	,ExperimentFailure
	,DebugStartup
	,TxPacketDropped
	,RxPacketDropped
	,CANInUse
	,EndOfErrors //34 -TODO -- this used to have a max of 32,  Make sure that is not still the case in any telemetry
} ErrorType_t;
//#define EndOfErrors I2C2InUse+1

/* These are the structures that we downlink to the ground...32 bits each*/
#if 0
typedef struct __attribute__((__packed__)) _DownlinkError{
	uint32_t wdReports:9;
	ErrorType_t errorCode:5;
	unsigned int mramErrorCount:3;
	unsigned int nonFatalErrorCount:3;
	unsigned int taskNumber:4;
	uint8_t errorData;  /* Extra data passed to ReportError */
} DownlinkError_t;

typedef struct __attribute__((__packed__)) _DownlinkSoftError{
	uint8_t DACoverflows;
	uint8_t I2C1Retries:4;
	uint8_t I2C2Retries:4;
	uint8_t SPIRetries;
	uint8_t MramCRCs;
} DownlinkSoftError;
#endif

typedef enum _IHUInfoType {
	 Last3DLStates=4
	,CommandRing
	,HwSwCommandCount
	,I2C1Errors
	,I2C2Errors
	,MRAMReadErr
	,MRAMWriteErr
	,Version=14
	,AutosafeVolts = 18
	,CommandFailureTypes=19
} IHUInfoType;

typedef enum _InterruptRoutine {
	Int_NONE=0,
	Int_ADC,
	Int_DMA1_2,
	Int_DMA1_3,
	Int_DMA1_4,
	Int_DMA1_5,
	Int_DMA1_6,
	Int_DMA1_7,
	Int_I2C1_1,
	Int_I2C1_1_Tx,
	Int_I2C1_1_Rx,
	Int_I2C1_2,
	Int_I2C2_1,
	Int_I2C2_2,
	Int_SPI1,
	Int_SPI2,
	Int_EXTI1,
	Int_EXTI2,
	Int_EXTI3,
	Int_EXTI4,
	Int_RxCD,
	Int_CAN,
	Int_CMD,
	Int_Serial1,
	Int_Serial2,
	Int_Serial3,
	Int_USB
} InterruptRoutine;

#define CMD_RING_BUFFER_SIZE 6
#define END_RING_MARKER 0xf;
typedef struct __attribute__((__packed__)) _Nybble2 {
	uint8_t nybble0:4;
	uint8_t nybble1:4;
} Nybble2;
typedef struct __attribute__((__packed__)) _Bit16and8 {
	uint16_t bit16;
	uint8_t bit8;
} Bit16and8;
typedef struct __attribute__((__packed__)) _Bit12and12 {
	uint16_t bit12_0:12;
	uint16_t bit12_1:12;
} Bit12and12;
#define SW_CMD_RING_SIZE 4

#define INSERT_IN_NYBBLE_RING_BUFFER(info,index, input)\
	{\
	uint8_t structIndex = (index>>1); \
	bool oddNybble = ((index & 1)==1);\
	if(oddNybble){\
		info.data.nybblePairs[structIndex].nybble1 = (input & 0xf);\
	} else { \
		info.data.nybblePairs[structIndex].nybble0 = (input & 0xf);\
	    }\
};
#define FETCH_FROM_RING_BUFFER(info,index,out) \
{\
	uint8_t structIndex = (index>>1); \
	bool oddNybble = ((index & 1)==1);\
	if(oddNybble){\
		out = info.data.nybblePairs[structIndex].nybble1;\
	} else { \
		out = info.data.nybblePairs[structIndex].nybble0;\
	}\
}

typedef struct __attribute__((__packed__)) _DownlinkIHUInfo{
	IHUInfoType type:8;
	union __attribute__((__packed__)) _data {
		uint8_t byte[3];
		Nybble2 nybblePairs[3];
		int bit24:24;
		Bit12and12 bit12Pair;
		Bit16and8 bit16and8;
	}data;
} DownlinkIHUInfo;


/* This is a subset of the FreeRTOS task structure so that we can find the stack */
typedef struct _stack  {
	int currentSP;
	int junk[11];
	int stackBase;
} DebugTaskHandle_t;

/* Here is what gets pushed on the stack when there is an exception*/
typedef struct _excStack {
	int r0;
	int r1;
	int r2;
	int r3;
	int r12;
	int lr;
	int pc;
	int psr;
	int HFSReg; /* Not part of arm architecture.  Code fills in Hard Fault Status */
	int CFSReg; /* Code fills in Config Status Register (with Bus, MMG, and usage fault) */
} ExceptionFrame_t;


/* This is the type of info passed with ReportError...mostly for ground testing */
typedef enum _erInfoType {
	RoutineAddress
	,PortNumber
	,CharString
	,ReturnAddr
	,ErrorBits
	,TaskNumber
	,EndOfErrorTypes
} ErrorInfoType_t;

/* Public interfaces to the error support */

void InitErrors(void);
void ReportError(ErrorType_t code, bool fatal, ErrorInfoType_t info, int data);
void RotateDiagnosticDownlink();
void RecordNewTask(uint32_t taskNo);
void ClearShortBootFlag(void);
void ReportInterruptRoutine(InterruptRoutine intRout);
void EndInterruptRoutine(void);
char *ErrorMessageString(ErrorType_t code);

/* This is the structure that we keep in memory across resets */
#define ERR_VALID_CODE  0xA5A55A5A /* We assume at least one of these gets changed... */
#define ERR_VALID_CHECK 0x5A5AA5A5 /* ...if the error info is invalidated by power cycle */

/*
 * These have the RT1 name so #define can change them to the local form
 */

typedef struct __attribute__((__packed__)) _save { //We really want this packet into two words for saving
    unsigned int wdReports:9;       //Offset=0
    unsigned int errorCode:5;       //Offset=9
    unsigned int taskNumber:4;      //Offset=17
    unsigned int previousTask:4;    //Offset=21
    bool         wasStillEarlyInBoot:1; //Offset = 25
    uint8_t      earlyResetCount:3;   //Offset=26
    unsigned int filler:3;// Offset 29
    uint32_t     errorData;          // Offset 32
}resetMemoryFields_t;
typedef union _saveUnion {
    uint32_t words[2];
    resetMemoryFields_t fields;
}resetMemory_t;

#ifndef ERRORS_C
// Don't mark this as an extern reference if we are in the errors routine itself
// where it is actually allocated
extern  resetMemory_t SaveAcrossReset;

#endif
#endif /* ERRORS_H_ */
