/*
 * MRAMManagement.c
 *
 *  Created on: Jun 5, 2019
 *      Author: bfisher
 *
 *
 *      Manage the overall contents of MRAM.
 *
 */
#include <pacsat.h>
#include "nonvolManagement.h"
#include "nonvol.h"
#include "spiDriver.h"
#include "CANSupport.h"

static const MRAMmap_t *ptr = (MRAMmap_t *) 0;


void EncodeUint32(uint32_t number,uint32_t *data){
    data[0] = number;
    data[1] = ~number;
}
uint32_t DecodeUint32(uint32_t *data,uint32_t defaultVal){
    if(data[0] == ~data[1])return data[0];
    // Ok, we have an error.  Start from 0.
    return defaultVal;
}

void EncodeUint16(uint16_t number,uint16_t *data){
    data[0] = number;
    data[1] = ~number;
}
uint16_t DecodeUint16(uint16_t *data,int failret){
    if(data[0] == (uint16_t)(~data[1]))return data[0];
    return failret;
}

//===========================
bool GetBooleanVal(uint8_t *data){
    int safeCount=0;
    if ((data[0] & 0xf)==0xf) {
        safeCount++;
    }
    if ((data[0] & 0xf0)==0xf0) {
        safeCount++;
    }
    if ((data[1] & 0xf)==0xf) {
        safeCount++;
    }
    if ((data[1] & 0xf0)==0xf0) {
        safeCount++;
    }
    return (safeCount > 2);  /* If at least three entries agree that we are in safe mode, we are */
}
void SetBooleanVal(bool bit,uint8_t *data){
    if(bit){
        data[0] = 0xff;
        data[1] = 0xff;
    } else {
        data[0] = 0;
        data[0] = 0;
    }
}
/*
 * Here is where we read and write the non-boolean values.  Start out with some macros since the code
 * for reading and writing each state value is nearly identical.
 */

#define WRITE_UINT32(field,value) \
    bool NVstat;\
    StateSavingMRAM_t states;\
    NVstat = readNV(&states, sizeof(StateSavingMRAM_t), NVConfigData,\
                    (int) &(ptr->StatesInMRAM));\
    if (NVstat == FALSE)\
        ReportError(MRAMread, FALSE, ReturnAddr, (int) WriteMRAMWODFreq);\
    EncodeUint32(value,states.field);\
    NVstat = writeNV(&states, sizeof(StateSavingMRAM_t), NVConfigData,\
                     (int) &(ptr->StatesInMRAM));\
    if (NVstat == FALSE)\
        ReportError(MRAMwrite, FALSE, CharString, (int)#field);
#define WRITE_UINT16(field,value) \
    bool NVstat;\
    StateSavingMRAM_t states;\
    NVstat = readNV(&states, sizeof(StateSavingMRAM_t), NVConfigData,\
                    (int) &(ptr->StatesInMRAM));\
    if (NVstat == FALSE)\
        ReportError(MRAMread, FALSE, ReturnAddr, (int) WriteMRAMWODFreq);\
    EncodeUint16(value,states.field);\
    NVstat = writeNV(&states, sizeof(StateSavingMRAM_t), NVConfigData,\
                     (int) &(ptr->StatesInMRAM));\
    if (NVstat == FALSE)\
        ReportError(MRAMwrite, FALSE, CharString, (int)#field);

#define READ_UINT32(field,defaultOnFail)\
    bool NVstat;\
    StateSavingMRAM_t states;\
    NVstat = readNV(&states, sizeof(StateSavingMRAM_t), NVConfigData,\
                    (int) &(ptr->StatesInMRAM));\
    if (NVstat == FALSE)\
        ReportError(MRAMread, FALSE, ReturnAddr, (int)#field);\
    return DecodeUint32(states.field,defaultOnFail);

#define READ_UINT16(field,defaultOnFail)\
    { bool NVstat;\
    StateSavingMRAM_t states;\
    NVstat = readNV(&states, sizeof(StateSavingMRAM_t), NVConfigData,\
                    (int) &(ptr->StatesInMRAM));\
    if (NVstat == FALSE)\
        ReportError(MRAMread, FALSE, ReturnAddr, (int)#field);\
    return DecodeUint16(states.field,defaultOnFail);}

#define READ_UINT16_TYPE(field,defaultOnFail,retType)\
    bool NVstat;\
    StateSavingMRAM_t states;\
    NVstat = readNV(&states, sizeof(StateSavingMRAM_t), NVConfigData,\
                    (int) &(ptr->StatesInMRAM));\
    if (NVstat == FALSE)\
        ReportError(MRAMread, FALSE, ReturnAddr, (int)#field);\
    return (retType)DecodeUint16(states.field,defaultOnFail)

void WriteMRAMWODHkDownlinkIndex(uint32_t index){
    WRITE_UINT32(WODHkDownlinkIndex,index);
}

uint32_t ReadMRAMWODHkDownlinkIndex(void){
    READ_UINT32(WODHkDownlinkIndex,0);
}
void WriteMRAMWODHkStoreIndex(uint32_t index){
    WRITE_UINT32(WODHkStoreIndex,index);
}

uint32_t ReadMRAMWODHkStoreIndex(void){
    READ_UINT32(WODHkStoreIndex,0);
}


void WriteMRAMWODSciDownlinkIndex(uint32_t index){
    WRITE_UINT32(WODSciDownlinkIndex,index);
}

uint32_t ReadMRAMWODSciDownlinkIndex(void){
    READ_UINT32(WODSciDownlinkIndex,0);
}

void WriteMRAMWODSciStoreIndex(uint32_t index){
    WRITE_UINT32(WODSciStoreIndex,index);
}
uint32_t ReadMRAMWODSciStoreIndex(void){
    READ_UINT32(WODSciStoreIndex,0);
}

void WriteMRAMWODRagDownlinkIndex(uint32_t index){
    WRITE_UINT32(WODRagDownlinkIndex,index);
}

uint32_t ReadMRAMWODRagDownlinkIndex(void){
    READ_UINT32(WODRagDownlinkIndex,0);
}

void WriteMRAMWODRagStoreIndex(uint32_t index){
    WRITE_UINT32(WODRagStoreIndex,index);
}

uint32_t ReadMRAMWODRagStoreIndex(void){
    READ_UINT32(WODRagStoreIndex,0);
}


void WriteMRAMWODFreq(uint16_t freq){
    WRITE_UINT16(WODFrequency,freq);
}

uint16_t ReadMRAMWODFreq(void){
    READ_UINT16(WODFrequency,DEFAULT_WOD_FREQUENCY);
}
void WriteMRAMWODSaved(uint16_t size){
    WRITE_UINT16(WODSize,size);
}

uint16_t ReadMRAMWODSaved(void){
    READ_UINT16(WODSize,DEFAULT_NUM_WOD_SAVED);
}
void WriteMRAMResets(uint16_t resets){
    WRITE_UINT16(NumberOfResets,resets);
}

uint16_t ReadMRAMResets(void){
    READ_UINT16(NumberOfResets,1010);
}

void WriteMRAMTimestampResets(uint16_t epoch){
    WRITE_UINT16(TimestampResets,epoch);
}

uint16_t ReadMRAMTimestampResets(void){
    READ_UINT16(TimestampResets,1010);
}

void WriteMinMaxResetSeconds(uint32_t seconds){
    WRITE_UINT32(MinMaxResetTimeSecs,seconds)
}
uint32_t ReadMinMaxResetSeconds(void){
    READ_UINT32(MinMaxResetTimeSecs,0xffffffff)
}
void WriteMinMaxResetEpoch(uint16_t epoch){
    WRITE_UINT16(MinMaxResetTimeEpoch,epoch)
}
uint32_t ReadMinMaxResetEpoch(void){
    READ_UINT16(MinMaxResetTimeEpoch,0xffff)
}

void WriteMRAMEnterAutosafe(uint16_t value){
    WRITE_UINT16(AutoSafeEnter,value);
}

uint16_t ReadMRAMEnterAutosafe(void){
    READ_UINT16(AutoSafeEnter,DEFAULT_AUTOSAFE_INTO);
}
void WriteMRAMExitAutosafe(uint16_t value){
    WRITE_UINT16(AutoSafeExit,value);
}

uint16_t ReadMRAMExitAutosafe(void){
    READ_UINT16(AutoSafeExit,DEFAULT_AUTOSAFE_OUTOF);
}

void WriteMRAMBoolState(int index,bool newState){

    /*
     * Write a boolean into the MRAM state section
     */
     bool NVstat;
    StateSavingMRAM_t states;
    NVstat = readNV(&states, sizeof(StateSavingMRAM_t), NVConfigData,
                    (int) &(ptr->StatesInMRAM));
    if (!NVstat)
        ReportError(MRAMread, FALSE, ReturnAddr, (int) WriteMRAMBoolState);
    SetBooleanVal(newState,states.NonVolatileStates[index]);
    NVstat = writeNV(&states, sizeof(StateSavingMRAM_t), NVConfigData,
                     (int) &(ptr->StatesInMRAM));
    if (NVstat == FALSE)
        ReportError(MRAMwrite, FALSE, ReturnAddr, (int) WriteMRAMBoolState);
}
void WriteMRAM2BoolState(int index1,bool state1,int index2,bool state2){

    /*
     * Write a boolean into the MRAM state section
     */
     bool NVstat;
    StateSavingMRAM_t states;
    // Read in all the states block
    NVstat = readNV(&states, sizeof(StateSavingMRAM_t), NVConfigData,
                    (int) &(ptr->StatesInMRAM));
    if (!NVstat) ReportError(MRAMread, FALSE, ReturnAddr, (int) WriteMRAMBoolState);
    // Now set the new values in that block
    SetBooleanVal(state1,states.NonVolatileStates[index1]);
    SetBooleanVal(state2,states.NonVolatileStates[index2]);
    // And write them back
    NVstat = writeNV(&states, sizeof(StateSavingMRAM_t), NVConfigData,
                     (int) &(ptr->StatesInMRAM));
    if (NVstat == FALSE)
        ReportError(MRAMwrite, FALSE, ReturnAddr, (int) WriteMRAMBoolState);
}
bool ReadMRAMBoolState(int index){
    bool NVstat;
    StateSavingMRAM_t safemode;
    NVstat = readNV(&safemode, sizeof(StateSavingMRAM_t), NVConfigData,
                    (int) &(ptr->StatesInMRAM));
    if (NVstat == FALSE)
        ReportError(MRAMread, FALSE, ReturnAddr, (int) ReadMRAMBoolState);

    return GetBooleanVal(safemode.NonVolatileStates[index]);
}

void WriteMRAMSecondsOnOrbit(uint32_t seconds){
    WRITE_UINT32(TimeSinceFirstBoot,seconds);
}
uint32_t ReadMRAMSecondsOnOrbit(void){
    READ_UINT32(TimeSinceFirstBoot,0x7FFFFFF);
}
void WriteMRAMCountdownAfterRelease(uint32_t minutes){
    WRITE_UINT32(PostLVTimeout,minutes);
}
uint32_t ReadMRAMCountdownAfterRelease(void){
    READ_UINT32(PostLVTimeout,POST_LAUNCH_WAIT_TIME);
}

void WriteMRAMVersionNumber(void){
    uint32_t version = MRAM_VERSION;
    writeNV(&version, sizeof(uint32_t), NVConfigData,
            (int) &(ptr->MRAMVersion1));
    writeNV(&version, sizeof(uint32_t), NVConfigData,
            (int) &(ptr->MRAMVersion2));
}
bool CheckMRAMVersionNumber(void){
    uint32_t version1,version2;
    readNV(&version1, sizeof(uint32_t), NVConfigData,
            (int) &(ptr->MRAMVersion1));
    readNV(&version2, sizeof(uint32_t), NVConfigData,
            (int) &(ptr->MRAMVersion2));
    return (version1 == MRAM_VERSION) && (version2 == MRAM_VERSION);
}

void WriteMRAMTimeout(TimeoutType type,uint32_t seconds){
    WRITE_UINT32(TimeoutTimes[type],seconds);
}
uint32_t ReadMRAMTimeout(TimeoutType type){
    READ_UINT32(TimeoutTimes[type],0);
}

void WriteMRAMCommandFreq(uint32_t freq){
    WRITE_UINT32(DCTRxFrequency,freq);
 }

uint32_t ReadMRAMCommandFreq(void){
    READ_UINT32(DCTRxFrequency,DCT_DEFAULT_RX_FREQ);
}
void WriteMRAMTelemFreq(uint32_t freq){
    WRITE_UINT32(DCTTxFrequency,freq);
 }

uint32_t ReadMRAMTelemFreq(void){
    READ_UINT32(DCTTxFrequency,DCT_DEFAULT_TX_FREQ);
}
void WriteMRAMDCTDriveLowPower(uint32_t regVal){
    WRITE_UINT32(DCTDriveLowPower,regVal);
}
uint32_t ReadMRAMDCTDriveLowPower(void){
    READ_UINT32(DCTDriveLowPower,DCT_DEFAULT_LOW_POWER);
}

void WriteMRAMDCTDriveHighPower(uint32_t regVal){
    WRITE_UINT32(DCTDriveHighPower,regVal);
}

uint32_t ReadMRAMDCTDriveHighPower(void){
    READ_UINT32(DCTDriveHighPower,DCT_DEFAULT_HIGH_POWER);
}
/*
 * Those are all the read/write routines for the MRAM.  Below are the initialization routines
 */

void SetupMRAMStates() {
    /*
     * These are initialized by preflight init.  That means that these values are the ones
     * that are in MRAM when we first power up in orbit.
     */

    WriteMRAMBoolState(StateCommandedSafeMode,true);
    WriteMRAMBoolState(StateAutoSafe,false);
    WriteMRAMBoolState(StateAutoSafeAllow,true);
    WriteMRAMBoolState(StateCommandReceived,false);
    WriteMRAMBoolState(StateTransponderEnabled,false);
    WriteMRAMBoolState(StateCommandTimeCheck,false);
    WriteMRAMBoolState(StateTransmitInhibit,false); // This is if the FCC orders a shutdown
    WriteMRAMBoolState(StateNormalRfPowerLevel,false); //False is low power
    WriteMRAMBoolState(StateSafeRfPowerLevel,false);
#ifdef UNDEFINE_BEFORE_FLIGHT
    /*
     * For now the experiment is not in place and causes I2c errors
     */
    WriteMRAMBoolState(StateExp1Disabled,true);
#else
    WriteMRAMBoolState(StateExp1Disabled,false);
#endif

#ifdef THIS_IHU_IS_DEFAULT
    WriteMRAMBoolState(StateIHUInCommand,true); // Initially, this IHU is in control
#else
    WriteMRAMBoolState(StateIHUInCommand,true); // Initially, this IHU is in control
#endif
    WriteMRAMWODFreq(DEFAULT_WOD_FREQUENCY);
    WriteMRAMWODSaved(DEFAULT_NUM_WOD_SAVED);
    WriteMRAMWODSciDownlinkIndex(0);
    WriteMRAMWODHkDownlinkIndex(0);
    WriteMRAMWODRagDownlinkIndex(0);
    WriteMRAMWODSciStoreIndex(0);
    WriteMRAMWODHkStoreIndex(0); //todo:  Should this be -1?  Is this called from preflight init?
    WriteMRAMWODRagStoreIndex(0);
    initSecondsInOrbit(); //Must use this to prevent an update from resetting the in orbit time
    WriteMRAMEnterAutosafe(DEFAULT_AUTOSAFE_INTO);
    WriteMRAMExitAutosafe(DEFAULT_AUTOSAFE_OUTOF);

    /* These are like 'set internal schedule' but sets relative to startup, not to current time */
    WriteMRAMTimeout(NoCommandTimeout,NO_COMMAND_TIMEOUT);
    WriteMRAMTimeout(NoTimeCommandTimeout,TIMEOUT_NONE); /* We start out with time turned off, so there is no timeout */
    WriteMRAMTimeout(MinMaxResetTimeout,MIN_MAX_CLEAR_SECONDS);
    WriteMRAMCountdownAfterRelease(POST_LAUNCH_WAIT_TIME);

    if(ReadMRAMCountdownAfterRelease() != POST_LAUNCH_WAIT_TIME) {
        printf("POST LAUNCH TIME NOT SET!");
    } else {
        printf("Post Launch Time Set to %d minutes",POST_LAUNCH_WAIT_TIME);
    }
}

// compute an updated MRAM CRC and update in place
bool updateCRC(uint32_t address, uint32_t size) {
    uint32_t w = (size / 4) - 1; // number of 32-bit data words
    uint32_t datum; // data word read from MRAM
    uint32_t result; // CRC result
    bool NVstat;
    bool returnCode = TRUE;

    //CRC_ResetDR(); // reset the CRC generator
    // read the data area word-by-word while calculating the CRC
    while (w > 0) {
        // read a data word from MRAM
        NVstat = readNV(&datum, 4, NVConfigData, address);
        if (NVstat == FALSE) {
            ReportError(MRAMread, FALSE, ReturnAddr, (int) updateCRC); // NVread service failure
            returnCode = FALSE;
        }
        //todo: Calculate
        //result = CRC_CalcCRC(datum); // update the CRC
        --w;
        address += 4; // next 32-bit word
    }

    // now write the result into MRAM
    NVstat = writeNV(&result, 4, NVConfigData, address);
    if (NVstat == FALSE) {
        ReportError(MRAMwrite, FALSE, ReturnAddr, (int) updateCRC); // NVwrite service failure
        returnCode = FALSE;
    }
    return (returnCode);
}


void IHUInitSaved(void){
    /*
     * This initializes the MRAM that has to happen with a new IHU, but which
     * we don't want to change each time we do a preflight init.  This sets them to a default
     * value that has to be tweaked for each processor (or each DCT in this case)
     */
    WriteMRAMCommandFreq(DCT_DEFAULT_RX_FREQ);
    WriteMRAMTelemFreq(DCT_DEFAULT_TX_FREQ);
    WriteMRAMDCTDriveHighPower(DCT_DEFAULT_HIGH_POWER);
    WriteMRAMDCTDriveLowPower(DCT_DEFAULT_LOW_POWER);
}
int SetupMRAM(void){
    uint32_t size = getSizeNV(NVConfigData); /* Will initialize. */

    SetupMRAMStates(); //This should be first.  It might overwrite part of MRAM.
    printf("Set to start in safe mode\n");
    printf("WOD Frequency,size set to Default, and WOD Indices initialized\n");

    WriteMinMaxResetSeconds(0); // Clear sets reset time to THIS epoch.  We need preflight init epoch
    WriteMinMaxResetEpoch(0);

    printf("Telemetry min/max in MRAM initialized\n");

    InitResetCnt();
    printf("Reset Count set to 0\n");

    //SaveAcrossReset.errorInfo.nonFatalErrorCount = 0; //todo: This should be a routine in errors.h
    printf("Nonfatal error count zeroed (TBD)\n");

    //POST_CalculateAndSaveCRC(); /* Initialize correct Flash CRC */
    printf("CRC calculated for init code and full code (TBD)\n");

    WriteMRAMVersionNumber();
    printf("MRAM Version Number Initialized\n");
    printf("MRAM config partition size is %d bytes\n",getSizeNV(NVConfigData));
    printf("MRAM file partition 1 size is %d\n",getSizeNV(NVFileSystem));
    if(size < sizeof(MRAMmap_t)){
        printf("!!!Not enough MRAM in this IHU!!!");
    }

    return size;
}

