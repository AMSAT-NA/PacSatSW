/*
 * FoxUplinkCommands.h
 *
 *  Created on: Jan 29, 2014
 *      Author: fox
 */

#ifndef UPLINKCOMMANDS_H_
#define UPLINKCOMMANDS_H_

typedef enum {
	/*
	 * These are the hardware commands that are actually uplinked.
	 * Note that there are 4 bits of h/w commands, but that 3 of them
	 * are hardwired, 2 on the LIHU side, and 1 on the RTIHU side (for
	 * Golf-T at least)
	 *
	 * Bit 0 - always interpreted by software
	 * Bit 1 - hardwired to power off the RT-IHU and DCT
	 * Bit 2 - hardwired to power off the LIHU
	 * Bit 3 - hardwired to power off the RxTx and SDR
	 *         On the RT-IHU software turns off the DCT
	 *
	 * So bit 3 is the "FCC Commands us to turn off Tx"
	 * Bit 1 can be used as a backup if bit 3 fails on the DCT
	 */
/*
 * Note that the processor only sees bits 0 and 3, and they appear adjacent to each other
 * when I read them, so that ends up with the values 0, 1, 2 (and 3).
 */
	 CMD_ALL_ON=0
	,CMD_RTIHU_SECOND_OFF = 1
    ,CMD_RTIHU_PRIM_OFF = 2 // We won't be seeing this one
    ,CMD_LIHU_OFF = 4
    ,CMD_TX_OFF = 8

} UplinkCommands;
	/*
	 * Above this (including the 4 uplink commands) are command decoder states.
	 * They select a row of the state table.
	 */
#define NUM_UPLINK_COMMANDS 5
#define UPLINK_CMD_IHU_MASK (3)  /* Ignore hardcoded bits on top */
#define UPLINK_CMD_HARDWIRE_MASK (0xf)

/*
 * The following are for Fox-1e and later software commands
 *
 */
/*
 * Following is the data structure representing software uplink commands
 */
typedef struct  __attribute__((__packed__)) {
	uint16_t command;
	uint16_t arguments[4];
} CommandAndArgs;

#if 1 /*This is authenticated but without any of the CAN stuff */
typedef struct  __attribute__((__packed__)){
	uint16_t resetNumber;
	unsigned int secondsSinceReset:24; // Actually this will be pairs of seconds, i.e. seconds/2
	uint8_t address;
	uint8_t special;
	uint8_t namespaceNumber;
	CommandAndArgs comArg;
    uint8_t AuthenticationVector[32];
}SWCmdUplink;
#endif
#if 0
typedef struct  __attribute__((__packed__)){
	uint8_t filler;
	uint8_t namespaceNumber;
	CommandAndArgs comArg;
}NonCAN;

typedef union  __attribute__((__packed__)){
	NonCAN noncan;	 /* NonCAN.filler must be 0 to indicate this is noncan*/
	//CANPacket_t can; /* Data length is always 8, so repurpose length field.  8 means can */
}CanOrNot;

typedef struct  __attribute__((__packed__)){
	uint16_t resetNumber;
	unsigned int secondsSinceReset:24; // Actually this will be pairs of seconds, i.e. seconds/2
	uint8_t address;
	CanOrNot remainder;
	//Add the following if we implement authentication
	uint8_t AuthenticationVector[32];
}SWCmdUplink;
#endif
#define AUTH_VECTOR_SIZE 32
#define SW_COMMAND_SIZE 18


/*
 * Here are the definitions of the name spaces and commands
 */

typedef enum {
	 SWCmdNSReserved=0
	,SWCmdNSSpaceCraftOps
	,SWCmdNSTelemetry
	,SWCmdNSExperiment1
	,SWCmdNSExperiment2
	,SWCmdNSExperiment3
	,SWCmdNSExperiment4
	,SWCmdNSCAN
	,SWCmdNSInternal
}SWCommandNameSpace;
typedef enum {
     SWCmdIntReserved=0
    ,SWCmdIntSetEclipseState // Argument 1 for enter, 0 for exit
    ,SWCmdIntEclipsesafeMode
    ,SWCmdIntEclipseInhibTx
    ,SWCmdIntAutosafeMode
}SWInternalCommands;
typedef enum {
	 SWCmdOpsReserved=0
	,SWCmdOpsSafeMode
	,SWCmdOpsHealthMode
	,SWCmdOpsScienceMode
	,SWCmdOpsDisableAutosafe
	,SWCmdOpsEnableAutosafe
	,SWCmdOpsClearMinMax
	,SWCmdOpsNoop
	,SWCmdOpsEnablePb
    ,SWCmdOpsFormatFs = 9
    ,SWCmdOpsEnableDigi
	,SWCmdOpsEnableUplink=12
	,SWCmdOpsDeployAntennas   // Args = (bus, antennaNumber,time, override)
	,SWCmdOpsSetTime // Args = (unix time)
	,SWCmdOpsEnableCommandTimeCheck //16
	,SWCmdOpsSetAutosafeVoltages //17
	,SWCmdOpsResetSpecified = 21 //Args = (LIHU, RTPrimary, RTSecondary), each one 1 or 0
	,SWCmdOpsDCTTxInhibit // 22
	,SWCmdOpsSelectDCTRFPower
    ,SWCmdOpsSpare
    ,SWCmdOpsNumberOfCommands
}SWOpsCommands;
const static uint8_t SWCmdOpsArgSize[SWCmdOpsNumberOfCommands]={0,0,0,1,0,0,0};

typedef enum {
    SWCmdTlmReserved=0
   ,SWCmdTlmFrequency
   ,SWCmdTlmNumberOfCommands
}SWTlmCommands;

typedef enum {
	 SwCmdExp1Reserved=0
	,SWCmdExp1CycleTiming
	,SWCmdExp1SetBoard
	,SwCmdExp1NumberOfCommands
}SWExp1Commands;

const static uint8_t SWCmdTlmArgSize[SWCmdTlmNumberOfCommands]={0,1};

#define SW_CMD_STRUCT_SIZE (sizeof(SWCmdUplink))
#define SW_UPLINK_FEC_RATE 2  /*(Actually 1/2)*/
#define SW_UPLINK_CRC 2
#define SW_UPLINK_END_LEN 2
#define SW_UPLINK_BYTES (((SW_CMD_STRUCT_SIZE+SW_UPLINK_CRC+SW_UPLINK_END_LEN)*SW_UPLINK_FEC_RATE))
#define SW_UPLINK_BITS SW_UPLINK_BYTES*8

//For command link layer

#define SYNC_LFSR_UP (0x4d215d8f)
#define SYNC_LFSR_UP_BITS 31
#define SYNC_LFSR_UP_BITS_MASK 0x7FFFFFFF

#define UPLINK_COMMAND_MARK 1
#define UPLINK_COMMAND_SPACE 0
#define UPLINK_MARK_VALUE 3000 /* For deciding an A/D value is a mark or a space */

// Initial test address
#define OUR_ADDRESS 0x1A



#endif /* UPLINKCOMMANDS_H_ */
