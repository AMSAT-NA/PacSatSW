/*
 * consoleRoutines.c
 *
 *  Created on: Sep 21, 2020
 *      Author: bfisher
 */
#include <pacsat.h>
#include "stdarg.h"
#include "stdlib.h"
#include "rti.h"
#include "consoleRoutines.h" // My own
#include "downlink.h"
#include "serialDriver.h"
#include "nonvolManagement.h"
#include "nonvol.h"
#include "ADS7828.h"
#include "errors.h"
#include "LinearInterp.h"
#include "inet.h"
#include "DownlinkControl.h"
#include "adc.h"


extern bool InSafeMode,InScienceMode,InHealthMode;
extern rt1Errors_t localErrorCollection;

extern uint16_t WODFrequency,NumWODSaved;
extern int32_t WODHkStoreIndex,WODSciStoreIndex,WODSciFetchIndex,WODHkFetchIndex;
extern int32_t WODRagStoreIndex,WODRagFetchIndex;
extern char *ErrMsg[], *LIHUErrMsg[],*TaskNames[];

static char *nextNum;
static const char *getTaskName(int task){
    return TaskNames[task];
}
void print8BitTemp(uint8_t temp8){
    int16_t temp = temp8;
    printf("%d (%3d.%1d degrees C)",temp8,
           (temp>>1)-20,(temp&1)*5);
}
void print8BitVolts(uint8_t volt8,uint16_t maxV){
    // Specify the voltage that 255 is equivalent too as max*100.  That is, for a max
    // of 13.3V, specify 1330.
    int16_t volt=volt8,interpVal;
    interpVal=LinearInterpolate(volt,255,maxV);
    printf("%3d (%2d.%1d volts) ",volt8,
           (interpVal/100),((interpVal%100)+5)/10);
}

bool AntennaAsk(void){
    char receivedChar;
    bool releaseOK=false;
    /*
     * For antenna deployment, make double sure this is not an accident
     */
    //      if (GPIORead(UmbilicalAttached) == 0) {
    if(GPIORead(UmbilicalAttached) == 1){
        printf("Umbilical is attached! Are you sure? ");
    }
    printf(
            "Type any key within 5 seconds to abort deploying\n\r");
    if (SerialGetChar(PRINTF_COM, &receivedChar, SECONDS(5))) {
        releaseOK = false;
    } else {
        releaseOK = true; /* Umbilical not connected and first Y was entered */
    }

    if(!releaseOK){
        printf("Deploy command aborted\n\r");
    }
    return releaseOK;
}

uint16_t parseNumber(char *afterCommand){
    return (uint16_t)strtol(afterCommand,&nextNum,0);
}
uint16_t parseNextNumber(void){
    char *string;
    string = nextNum;
    return (uint16_t)strtol(string,&nextNum,0);
}


void receiveLine(COM_NUM ioCom, char *commandString, char prompt, bool echo) {
    /* Wait until something arrives in the queue - this routine will block
     indefinitely provided INCLUDE_vTaskSuspend is set to 1 in
     FreeRTOSConfig.h. */
    static char prevCommandString[COM_STRING_SIZE],prevSize; /* For up-arrow */
    char receivedChar;
    int charNum = 0;
    const char deleteString[4] = { '\b', ' ', '\b', 0 }; // Back overwrite space, back
    int escSeq=0;
    if(!CheckMRAMVersionNumber()){
        printf("\n ***MRAM format has changed\n ***Command 'preflight init' or 'init mram' required!\n");
    }
    if (echo)
        printf("");
    SerialPutChar(ioCom, prompt, 0);
    while (TRUE) {
        bool gotChar;
        /*
         * Read a character and process it
         */
        gotChar = SerialGetChar(ioCom, &receivedChar, SHORT_WAIT_TIME);
        dwdReset();
        if(gotChar){
            receivedChar &= 0x7f; // 7-bit ASCII
            if ((receivedChar == '\x7f') | (receivedChar == '\b')) { // Is it a delete character?
                escSeq=0;
                if (charNum > 0) { // If there is a character, delete it...
                    commandString[--charNum] = 0; //...in the buffer and...
                    if (echo)
                        SerialPutString(ioCom, deleteString, 0); //...on the screen
                }
            } else if (escSeq==0 && receivedChar == 27) {/* Escape */
                escSeq=1;
            } else if ((escSeq==1) && receivedChar == '[') {
                escSeq=2;
            }
            else if ((escSeq == 2) && receivedChar == 'A') { /* Here we got an up arrow */
                escSeq = 0;
                strncpy(commandString,prevCommandString,COM_STRING_SIZE);
                charNum = prevSize+1;
                commandString[charNum]=0;
                printf("\r%c%s",prompt,commandString);

            } else if (receivedChar != '\n') { /*Ignore line feed.  Only watch for CR*/
                /* Echo the received character and record it in the buffer */
                escSeq=0;
                if (echo)
                    SerialPutChar(ioCom, receivedChar, 0); // COM0, value, block time is unused
                commandString[charNum++] = receivedChar | 0x20; /* Force it to be lower case, plus it won't change digits */
            }
            /*
             * All the code above echos type characters and allows one to use delete
             * to edit characters.  We don't really do anything else until we see
             * a carriage return (\r), and then we execute the if clause below, which
             * is pretty much the rest of the routine.
             */

            if (receivedChar == '\r') { // The user hit 'enter' or carriage return
                int dest = 0, src = 0;
                if (echo)
                    SerialPutChar(ioCom, '\n', 0); // Also echo a line feed
                /* Ok, we have the end of the command line.  Decode it and do the command */
                commandString[--charNum] = 0; // Replace the CR with a null terminator
                /* Now squeeze out extra spaces */
                while (commandString[src] != 0) {
                    commandString[dest] = commandString[src++];
                    /*
                     * If we have multiple spaces, only take the last one
                     */
                    if ((commandString[dest] != ' ') || (commandString[src] != ' '))
                        dest++;
                }
                if(commandString[0]!= 0){
                    strncpy(prevCommandString,commandString,COM_STRING_SIZE);
                    prevSize = --dest;
                }
                return;
            }

        }
    }
}
void PreflightInitNow(CanIDNode cameFrom){
    int size;
    size=MRAMInit(); //This should be first.  It might overwrite part of MRAM.
    WriteMRAMBoolState(StateInOrbit,false);
    WODHkStoreIndex = -1;
    WODSciStoreIndex = -1;
    WODRagStoreIndex = -1;
    WODHkFetchIndex = 0;
    WODSciFetchIndex = 0;
    WODRagFetchIndex = 0;
    //todo: The rest of the stuff has not been implemented yet
    printf("WOD Frequency,size set to Default, and WOD Indices initialized\n");

    printf("WOD Storage and All variables initialized and ready for flight\n");

    printf("MRAM size found to be %d bytes\n",size);
    printf("Size of MRAM data structure is %d\n",sizeof(MRAMmap_t));
    if(size < sizeof(MRAMmap_t)){
        printf("!!!Not enough MRAM in this IHU!!!");
    }
}
void DisplayTelemetry(uint32_t typeRequested){
    anyPayload_t telemBuf,*telemBuffer;
    header_t header;
    char *typeString;
#if 0
    union {
        unsigned int  integer;
        DownlinkSoftError softError;
        DownlinkIHUInfo ihuDiag;
        DownlinkError_t resetError;
    }printUnion;
#endif

    telemBuffer = &telemBuf;
    typeString = "";
    switch(typeRequested){
    case 0:{

        /*
         * Telemetry type 0 is actually status in the debug task
         */
        int i;
        printf("I2c device state:\n"
                "    ICR ADC: %d    Solar ADC:   %d,   CSS ADC: %d\n",
                ICRTelemIsOk(),SolarTelemIsOk(),CSSTelemIsOk());
#if 0 /* Is LIHU*/
        if ((POST_Test_Results() & POST_START_FAILURE) == POST_START_FAILURE)  {
            printf("Startup CRC faulty\n\r");
        } else {
            printf("Startup CRC OK\n\r");
        }
        if ((POST_Test_Results() & POST_ROM_FAILURE) == POST_ROM_FAILURE)  {
            printf("Main code CRC faulty\n\r");
        } else {
            printf("Main code CRC OK\n\r");
        }
#endif
        if(InSafeMode){
            printf("In SAFE mode\n\r");
        }
        if(InScienceMode){
            printf("In SCIENCE mode");
        }

        if (InHealthMode){
            printf("in HEALTH mode");
        }
        if(!InSafeMode){
            printf(" with transponder ");
            if(IsTransponderEnabled()){
                printf("ENABLED\n\r");
            } else {
                printf("DISABLED\n\r");
            }
        }

        printf("MRAM State Values:\n\r"
                " CommandedSafeMode=%d,Autosafe=%d\n\r"
                " CommandRcvd=%d,AllowAutoSafe=%d\n\r",

                ReadMRAMBoolState(StateCommandedSafeMode),ReadMRAMBoolState(StateAutoSafe),
                ReadMRAMBoolState(StateCommandReceived),ReadMRAMBoolState(StateAutoSafeAllow)
        );
        printf(" Uncommanded Seconds in Orbit=%d\n\r",
                (unsigned int)ReadMRAMSecondsOnOrbit());
        // todo:  Have to put the on-orbit flag somewhere
        //        readNV(&onOrbit,sizeof(int),LocalEEPROMData,(int)&eepromMemMap->HaveWaitedInOrbit);
        //        printf(" On-orbit flag is ");
        //        if(onOrbit){
        //            printf("TRUE\n\r");
        //        } else {
        //            printf("FALSE\n\r");
        //        }
        printf("\n*************************************\n\n");
        printf("\nIHU reset cause=0x%x,reset data=0x%x,IHUWdReport=0x%x,\n"
                "LastTask=%s\n"
                "Reason=%s\n",
                localErrorCollection.errorCode,
                localErrorCollection.errorData,
                localErrorCollection.wdReports,
                getTaskName(localErrorCollection.taskNumber),
                ErrorMessageString((ErrorType_t)localErrorCollection.errorCode));



        i = xPortGetFreeHeapSize();
        printf("Free heap size is %d\n\r", i);
        //        printf("I2C2 timeouts: [0]=%d,[1]=%d.[2]=%d\n\r",I2C2Error.data.byte[0],I2C2Error.data.byte[1],
        //                I2C2Error.data.byte[2]);
        //        printf("I2C1 timeouts: [0]=%d,[1]=%d.[2]=%d\n\r",I2C1Error.data.byte[0],I2C1Error.data.byte[1],
        //                I2C1Error.data.byte[2]);

        //            printf("Current non-reset memory MRAM errors: %d nonFatal Errors: %d, task=%d, data=%d, code=%d\n\r",
        //                    localErrorCollection.errorInfo.mramErrorCount,localErrorCollection.errorInfo.nonFatalErrorCount,
        //                    localErrorCollection.errorInfo.taskNumber,localErrorCollection.errorInfo.errorData,
        //                    localErrorCollection.errorInfo.errorCode);
        //todo: What should we print above
        printf("\nWOD:StoreHkIndex=%d,StoreSciIndex=%d,StoreRagIndex=%d,FetchHkIndex=%d,"
                "FetchSciIndex=%d,FetchRagIndex=%d,Freq=%d,NumSaved=%d\n\r",
                (int)WODHkStoreIndex,(int)WODSciStoreIndex,(int)WODRagStoreIndex,
                (int)WODHkFetchIndex,(int)WODSciFetchIndex,(int)WODRagFetchIndex,
                (int)WODFrequency,(int)NumWODSaved);

        return;
    }
    case RT_HK_PAYLOAD: {
        typeString = "Real Time";
        break;
    }
    case MAX_VALS_PAYLOAD:{
        typeString = "Highs";
        break;
    }
    case MIN_VALS_PAYLOAD:{
        typeString="Lows";
        break;
    }
    case WOD_HK_PAYLOAD:{
        typeString = "WOD Houskeeping";
        break;
    }
    case DIAGNOSTIC_PAYLOAD:{
        typeString = "Diagnostic";
        break;
    }
    default: {
        typeString = "????";
        break;
    }
    }
    CreateHeader(typeRequested, &header);

    printf("%s header:\n\r Type=%d, GolfID=%d, Reset Count=%d, Uptime=%d,Telem Version %d.%d,\n"
            "Firmware Version %d.%c%c\n",
            typeString,
            header.type,
            header.satelliteID,
            (int)ttohs(header.resetCnt),
            (int)ttohl(header.uptime),
            header.protocolVersion>>4,header.protocolVersion &0x0F,
            header.satelliteID,header.versionMajor,header.versionMinor);

    CreatePayload(typeRequested, &telemBuf);
    if(header.inHealthMode) typeString = "HEALTH";
    else if (header.inSafeMode) typeString="SAFE";
    else if (header.inScienceMode) typeString="SCIENCE";
    else typeString = "???";
    printf("In %s Mode ",typeString);
    switch(typeRequested){
    case WOD_HK_PAYLOAD:{
        printf("\nUnique to WOD:\n\r  Collection Time: Resets=%d,seconds=%d\n\r",
                (int)ttohs(telemBuffer->HKWod.wodInfo.WODTimestampReset),
                (int)ttohl(telemBuffer->HKWod.wodInfo.WODTimestampUptime)
        );
        break;
    }
    case RT_HK_PAYLOAD: {
        printf("\n"
        "Reset Counts -- LIHU: %d, RTIHU1: %d, RTIHU2: %d MinMax: %d\n",
        telemBuffer->rtHealth.common2.LIHUResets,
        telemBuffer->rtHealth.common2.RTIHU1Resets,
        telemBuffer->rtHealth.common2.RTIHU2Resets,
        telemBuffer->rtHealth.common2.TLMresets);

        break;
    }
    case MAX_VALS_PAYLOAD: {
        printf("\nUnique to Max:\n\r"
                "  Max Timestamp=%d resets,%d seconds\n",
                ttohs(telemBuffer->maxVals.MaxValuesData.maxTimestampEpoch),
                (int)ttohl(telemBuffer->maxVals.MaxValuesData.maxTimestampUptime)
        );
        break;
    }
    case MIN_VALS_PAYLOAD: {
        printf("\nUnique to Min:\n"
                "  Min Timestamp Change Epoch=%d, Seconds=%d\n"
                "  Min/Max Reset Time   Epoch=%d ,Seconds=%d\n",
                ttohs(telemBuffer->minVals.MinValuesData.minTimestampEpoch),
                ttohl((int)telemBuffer->minVals.MinValuesData.minTimestampUptime),
                ttohs(telemBuffer->minVals.MinValuesData.minmaxResetEpoch),
                ttohl(telemBuffer->minVals.MinValuesData.minmaxResetSecs)
        );

        break;
    }
    case DIAGNOSTIC_PAYLOAD:{
#define rt1(y) telemBuffer->diags.primaryErrors.y
#define rt2(y) telemBuffer->diags.secondryErrors.RT2 ## y
#define leg(y) telemBuffer->diags.legacyErrors.Leg ## y
#define tsk1(y) TaskNames[telemBuffer->diags.primaryErrors.y]
#define tsk2(y) TaskNames[telemBuffer->diags.secondryErrors.RT2 ## y]
#define tskL(y) TaskNames[telemBuffer->diags.legacyErrors.Leg ## y]
        uint32_t *firstLword;
        firstLword = (uint32_t *)&telemBuffer->diags.legacyErrors;
        *firstLword = ttohl(*firstLword); //Fix endian so I can print it
        firstLword = (uint32_t *)&telemBuffer->diags.primaryErrors;
        *firstLword = ttohl(*firstLword); //Fix endian so I can print it
        firstLword = (uint32_t *)&telemBuffer->diags.secondryErrors;
        *firstLword = ttohl(*firstLword); //Fix endian so I can print it
        firstLword = (uint32_t *)&telemBuffer->diags.info;
        firstLword[0] = ttohl(firstLword[0]);
        firstLword[1] = ttohl(firstLword[1]);
        firstLword[2] = ttohl(firstLword[2]);
        printf("\nTime:  EpochTime Uptime  UTC Hours Minutes Seconds Valid\n");
        printf("         %d      %d          %d     %d      %d      %d\n",
               telemBuffer->diags.info.secsInEpoch,
               telemBuffer->diags.info.Uptime,telemBuffer->diags.info.UTCHours,telemBuffer->diags.info.UTCMinutes,
               telemBuffer->diags.info.UTCSeconds,telemBuffer->diags.info.UTCValid);


        printf("\nCause of last reset:\n");
        printf("       Wd CurrentTask  PrevTask  EarlyResetCnt wasEarly  ErrorCode    Valid \n");
        printf("LIHU   %02x %s %s      %2d        %2d    %s   %d\n"
               "RTIHU1 %02x %s %s      %2d        %2d    %s   %d\n"
               "RTIHU2 %02x %s %s      %2d        %2d    %s   %d\n",
               leg(WdReports),tskL(TaskNumber),tskL(PreviousTask),leg(EarlyResetCount),leg(WasStillEarlyInBoot),LIHUErrMsg[leg(ErrorCode)],leg(Valid),
               rt1(wdReports),tsk1(taskNumber),tsk1(previousTask),rt1(earlyResetCount),rt1(wasStillEarlyInBoot),ErrMsg[rt1(errorCode)],rt1(valid),
               rt2(wdReports),tsk2(taskNumber),tsk2(previousTask),rt2(earlyResetCount),rt2(wasStillEarlyInBoot),ErrMsg[rt2(errorCode)],rt2(Valid)
               );
        printf("\nBus problems:\n");
        printf("      I2C1Err I2C1Reset I2C1Retry I2C2Retry SPIRetry MRAM CRC  MRAMRd MRAMWt\n");
        printf("LIHU    %3d     %3d       ---        ---       %3d     %3d       %3d    %3d\n"
               "RTIHU1  %3d     %3d       %3d        %3d       %3d     %3d       %3d    %3d\n"
               "RTIHU2  %3d     %3d       %3d        %3d       %3d     %3d       %3d    %3d\n",
               leg(I2C1Error),leg(I2C1Reset),leg(SPIRetries),leg(MramCRC),leg(MramRead),leg(MramWrite),
               rt1(I2C1ErrorCnt),rt1(I2C1ResetCnt),rt1(I2C1RetryCnt),rt1(I2C2RetryCnt),rt1(SPIRetryCnt),rt1(MramCRCCnt),rt1(MramRdErrorCnt),rt1(MramWtErrorCnt),
               rt2(I2C1ErrorCnt),rt2(I2C1ResetCnt),rt2(I2C1RetryCnt),rt2(I2C2RetryCnt),rt2(SPIRetriesCnt),rt2(MramCRCCnt),rt2(MramRdErrorCnt),rt2(MramWtErrorCnt)
               );
        return;
    }
    } /* End of switch */
    if(typeRequested != RT_RAD_PAYLOAD){  // This is the 'common' stuff for health, min, max, WOD
        char * moduState;
        printf(
                "\n"
                "  Gyro Chip:     Spin     Accel    Mag\n\r"
                "         X       %04d     %04d     %04d\n\r"
                "         Y       %04d     %04d     %04d\n\r"
                "         Z       %04d     %04d     %04d\n\r"
                "    Chip Temp: ",
                telemBuffer->rtHealth.common.Xspin,
                telemBuffer->rtHealth.common.Xaccel,
                telemBuffer->rtHealth.common.Xmag,
                telemBuffer->rtHealth.common.Yspin,
                telemBuffer->rtHealth.common.Yaccel,
                telemBuffer->rtHealth.common.Ymag,
                telemBuffer->rtHealth.common.Zspin,
                telemBuffer->rtHealth.common.Zaccel,
                telemBuffer->rtHealth.common.Zmag
        );
        print8BitTemp(telemBuffer->rtHealth.common.gTemp);
        printf("\n");

        uint16_t Icr3V = LinearInterpolate(telemBuffer->rtHealth.common.ICR3VProt,255,362);
        uint16_t ICRTemp =telemBuffer->rtHealth.common.ICRTemp;
        uint16_t VGainAmp = LinearInterpolate(telemBuffer->rtHealth.common.LtVGACtl,255,362);
        uint16_t Ipa = LinearInterpolate(telemBuffer->rtHealth.common.TxPAi,255,309); //7.72V = 3.09A.

        printf(
                "\nLegacy Rx and Tx\n"
                "      ICR 3VProt: %04d       ICR Temp %03d (%d.%01dC)\n"
                "       Fwd Power: %04d Rflt Power: %04d   VGainAmp: %04d\n"
                "      PA Current: %04d   ICR RSSI: %04d   LT Temp: ",
                telemBuffer->rtHealth.common.ICR3VProt,Icr3V/100,Icr3V%100,
                telemBuffer->rtHealth.common.ICRTemp, (ICRTemp/2)-20,(ICRTemp&1)*5,
                telemBuffer->rtHealth.common.FwdPower,
                telemBuffer->rtHealth.common.ReflectedPwr,
                telemBuffer->rtHealth.common.LtVGACtl,
                telemBuffer->rtHealth.common.TxPAi, /* transmitter power amplifier current */
                telemBuffer->rtHealth.common.rssi
         );
         print8BitTemp(telemBuffer->rtHealth.common.TxTemp);

        if(telemBuffer->rtHealth.common.moduTemp == 0xff){
            printf("\n\nModulator: No data available\n");
        } else {
            if ((telemBuffer->rtHealth.common.moduTemp & 1) != 0){
            moduState = " Powered off.  Data is stale";
            } else {
            moduState = "";
            }
            printf(
                    "\nLegacy Telemetry Modulator %s\n"
                    "  Modu LO: 0x%02x  ModuGain: -%02ddB  Modu Mode: 0x%02x\n",
                    moduState,
                    telemBuffer->rtHealth.common2.ModuLO,
                    telemBuffer->rtHealth.common2.ModuGain & 0x1F,
                    telemBuffer->rtHealth.common2.ModuMode
                    );
            printf("  Modulator chip temp is ");print8BitTemp(telemBuffer->rtHealth.common.moduTemp); printf("\n");
        }
        {
            int16_t primRssi = telemBuffer->rtHealth.common.DCT1RSSI;
            int16_t secndRssi = telemBuffer->rtHealth.common.DCT2RSSI;
            primRssi -=255;
            secndRssi-=255;
            printf("\n Digital Com Transceiver:\n"
                    "  DCT1 Fwd Power: %04d; Rflt Power: NOT YET; RSSI %ddBm\n"
                    "  DCT2 Fwd Power: %04d; Rflt Power: NOT YET; RSSI %ddBm\n"
                    " Receive/Transmit in Half Duplex: %s\n",
                    telemBuffer->rtHealth.common.DCT1Power,primRssi,
                    telemBuffer->rtHealth.common.DCT2Power,secndRssi,
                    telemBuffer->rtHealth.common2.halfDupMode?"True":"False"
             );
        }
        printf(
                "\nPower (CIU)\n");
        printf(  "   3.3V #1: "); print8BitVolts(telemBuffer->rtHealth.common.CIU3V3_1,1330);
        printf("\n   3.3V #2: "); print8BitVolts(telemBuffer->rtHealth.common.CIU3V3_2,1330);
        printf("\n      Vsys: "); print8BitVolts(telemBuffer->rtHealth.common.CIUVsys,1330);
        printf("\n       12V: "); print8BitVolts(telemBuffer->rtHealth.common.CIU12v,1330);
        printf(
                "\n\nPanels: Fixed Sun  Fixed Dark  Deployed +Y  Deployed -Y\n"
                "  Volts:  %04d       %04d        %04d         %04d\n"
                "  Temp:   %04d       %04d        %04d         %04d\n",
                telemBuffer->rtHealth.common.CIU3V3_1,
                telemBuffer->rtHealth.common.CIU3V3_2,
                telemBuffer->rtHealth.common.CIUVsys,
                telemBuffer->rtHealth.common.CIU12v,
                telemBuffer->rtHealth.common.XVolts,
                telemBuffer->rtHealth.common.MinusXVolts,
                telemBuffer->rtHealth.common.YVolts,
                telemBuffer->rtHealth.common.MinusYVolts,
                telemBuffer->rtHealth.common.XTemp,
                telemBuffer->rtHealth.common.MinusXTemp,
                telemBuffer->rtHealth.common.YTemp,
                telemBuffer->rtHealth.common.MinusYTemp
        );
        printf("\n Coarse Sun Sensors:\n");
        printf("   1: %03d  2: %03D  3: %03d  4: %04d\n",
               telemBuffer->rtHealth.common.CSS1,telemBuffer->rtHealth.common.CSS2,
               telemBuffer->rtHealth.common.CSS3,telemBuffer->rtHealth.common.CSS4);
        printf("   5: %03d  6: %03D  7: %03d  8: %04d\n",
               telemBuffer->rtHealth.common.CSS5,telemBuffer->rtHealth.common.CSS6,
               telemBuffer->rtHealth.common.CSS7,telemBuffer->rtHealth.common.CSS8);
        {

            uint8_t temp1 = telemBuffer->rtHealth.common.RT1Temp;
            uint8_t temp2 = telemBuffer->rtHealth.common.RT2Temp;
            //660 Does both the 3.3V ref, the voltage divider, and turning it into hundredths of a volt
            uint16_t oldBusV = LinearInterpolate(telemBuffer->rtHealth.common.busV,255,660);  // This is for the old LIHU
            uint16_t newBusV = LinearInterpolate(telemBuffer->rtHealth.common.busV,255,1018); // This is for the newest LIHU

            printf(
                    "\nIHU:\n\r"
                    "   Bus Voltage via LIHU     %04d (%d.%dV or %d.%dV)\n"
                    "   Legacy MCU Temp          ",
                    telemBuffer->rtHealth.common.busV,oldBusV/100,oldBusV%100,
                    newBusV/100,newBusV%100
             );
            print8BitTemp(telemBuffer->rtHealth.common.IHUcpuTemp);printf("\n");

            printf(
                    "   Primary RT-IHU Temp      "); print8BitTemp(temp1);
            printf( "\n"
                    " Secondary RT-IHU Temp      "); print8BitTemp(temp2);//%02d.%1dC)\n",
            printf( "\n");

        }
    }
    if((typeRequested == RT_HK_PAYLOAD) || (typeRequested == WOD_HK_PAYLOAD) ){ // This is stuff in common 2 which is for WOD and RT
        printf( "Radio Power Cycles:\n"
                "                Primary    Secondary\n"
                "DCT Power Amp      %d           %d\n"
                "DCT                %d           %d\n",
                telemBuffer->rtHealth.common2.PA1PwrCnt,
                telemBuffer->rtHealth.common2.PA2PwrCnt,
                telemBuffer->rtHealth.common2.DCT1PwrCnt,
                telemBuffer->rtHealth.common2.DCT2PwrCnt);
        printf("\nDevice States\n"
               "MRAM Status Primary=%x,Secondary=%x,Legacy=%x\n"
               "CIU Reset Count1: %d, Count2: %d\n",
               telemBuffer->rtHealth.common2.PrimMRAMstatus,telemBuffer->rtHealth.common2.SecndMRAMstatus,
                   telemBuffer->rtHealth.common2.LMRAMstatus,
               telemBuffer->rtHealth.common2.CIUResetCnt1,
               telemBuffer->rtHealth.common2.CIUResetCnt2
        );
    }

}
void DisplayWODRange(bool contents){
    /*
     * Look at the WOD memory and find the oldest and newest and print them out
     *
     */
    MRAMmap_t *MRAMBase=0;
    WODHkMRAM_t buf;
    WODHkMRAM_t *buffer=&buf;
    WODSciMRAM_t sciBuf;
    WODRagMRAM_t ragBuf;

    bool indexChange=true,index0=false;
    int saveIndex=-1,count,i;
    /*
     * The one pointed to by WODHkFetchIndex is the next one to be transmitted
     */
    if(contents){
        printf("Housekeeping                                 Science\n\r");
        for(i=0;i<=NumWODSaved;i++){
            readNV(buffer, sizeof(WODHkMRAM_t), ExternalMRAMData,
                    (int) &(MRAMBase->WODHousekeeping[i]));
            printf("Index %3d, reset %3d, seconds %7d\n",i,buffer->wodHKPayload.wodInfo.WODTimestampReset,
                    buffer->wodHKPayload.wodInfo.WODTimestampUptime);
            //Science
            readNV(&sciBuf, sizeof(WODHkMRAM_t), ExternalMRAMData,
                    (int) &(MRAMBase->WODScience[i]));
        }
    }
    readNV(&sciBuf, sizeof(WODSciMRAM_t), ExternalMRAMData,
            (int) &(MRAMBase->WODScience[WODSciFetchIndex]));
    printf("WOD Science Range\n    Next time to Downlink: %d/%d (Index %d), Store index: %d\n",
            sciBuf.wodRadPayload.wodInfo.WODTimestampReset,
            sciBuf.wodRadPayload.wodInfo.WODTimestampUptime,(int)WODSciFetchIndex,
            WODSciStoreIndex);

    readNV(&ragBuf, sizeof(WODHkMRAM_t), ExternalMRAMData,
            (int) &(MRAMBase->WODRagnarok[WODRagFetchIndex]));
    printf("WOD Ragnarok Range\n    Next time to Downlink: %d/%d (Index %d), Store index: %d\n",
            ragBuf.wodRagPayload.wodInfo.WODTimestampReset,//WODTimestampReset,
            ragBuf.wodRagPayload.wodInfo.WODTimestampUptime,(int)WODRagFetchIndex,
                    WODRagStoreIndex);

    readNV(buffer, sizeof(WODHkMRAM_t), ExternalMRAMData,
            (int) &(MRAMBase->WODHousekeeping[WODHkFetchIndex]));
    printf("WOD Housekeeping Range\n    Next time to Downlink: %d/%d (Index %d), Store index %d\n",
            buffer->wodHKPayload.wodInfo.WODTimestampReset,//WODTimestampReset,
            buffer->wodHKPayload.wodInfo.WODTimestampUptime,(int)WODHkFetchIndex,
                    WODHkStoreIndex);

    /*
     * Here we are going to get the oldest one, which is normally pointed at by WODHkStoreIndex,
     * but there are complications...
     */

    saveIndex = WODHkStoreIndex;
    count = 0;
    while((index0 | indexChange)&& (count++<2)){
        /* indexChange is always true the first time.  2nd time if the store index changes or time uninitted*/
        readNV(buffer, sizeof(WODHkMRAM_t), ExternalMRAMData,
                (int) &(MRAMBase->WODHousekeeping[saveIndex]));

        if ((!index0) && (saveIndex != WODHkStoreIndex)){
            /*
             * If we are here, it means that the telemetry task wrote some WOD data in the middle of us
             * reading it, so we might have gotten the newest rather than the oldest.  Do it again.
             */
            indexChange=true;
            saveIndex = WODHkStoreIndex;
        }
        else {
            indexChange=false;
        }

        if(buffer->wodHKPayload.wodInfo.WODTimestampUptime == 0){
            /*
             * If we are here, the data has not wrapped in memory yet, so we are not looking
             * at the oldest.  We are looking at an unwritten memory.  Go back to 0 for oldest.
             */
            saveIndex = 0;
            index0=true;
        } else {
            index0=false;
        }
    }
    printf("    Oldest time stored: %d/%d (index %d)\n\r",
            buffer->wodHKPayload.wodInfo.WODTimestampReset,
            buffer->wodHKPayload.wodInfo.WODTimestampUptime,saveIndex);
    /*
     * And now finally, the newest one saved is right behind the WodHkStoreIndex unless it
     * just wrapped.
     */
    saveIndex = WODHkStoreIndex;
    if(saveIndex == 0)saveIndex = NumWODSaved+1; // Wrap backwards if needed.

    readNV(buffer, sizeof(WODHkMRAM_t), ExternalMRAMData,
            (int) &(MRAMBase->WODHousekeeping[saveIndex-1]));

    printf("    Newest time stored: %d/%d (index %d)\n\r",
            buffer->wodHKPayload.wodInfo.WODTimestampReset,
            buffer->wodHKPayload.wodInfo.WODTimestampUptime,saveIndex-1);

}
void printID(void){
    int boardVersion;
    {
        // Get the board version from the ADC (hey, that's how it is wired!)
        adcData_t data[4];
        int i=0;
        adcResetFiFo(adcREG1,adcGROUP2);
        adcStartConversion(adcREG1,adcGROUP2);
        while(adcIsConversionComplete(adcREG1,adcGROUP2)==0){
            // Really should be only one tick
            vTaskDelay(1);
        }
        adcStopConversion(adcREG1,adcGROUP2);
        adcGetData(adcREG1,adcGROUP2,data);
        boardVersion = 0;
        for(i=0;i<4;i++){
            if(data[i].value > 50)boardVersion |= (1<<i);

        }
    }


    printf("\nAMSAT-NA Golf-TEE RT-IHU Console\n");
    printf("RT-IHU Flight Software %s (built on %s at %s)\n",
           RTIHU_FW_VERSION_STRING, __DATE__, __TIME__);
    printf("Software Version " "IHU_FW_VERSION_STRING \n");
    printf("   Built %s %s\n\n",__DATE__, __TIME__);//__GNUC__,__GNUC_MINOR__);

#if defined (COMPILE_DEBUG)
    printf("Compiled **DEBUG**\n");
#else
    printf("Compiled *RELEASE*\n");
#endif
#if !defined(UNDEFINE_BEFORE_FLIGHT) && !defined (ENGINEERING_MODEL) && !defined(COMPILE_DEBUG)
    printf("Configured for Flight\n");
#else
    printf("**X, E, U, or DEBUG Version: Not configured for flight**\n");
#endif
    printf("\nFree heap size is %d\n", xPortGetFreeHeapSize());
#ifdef WATCHDOG_ENABLE
    printf("Watchdog Enabled\n");
#else
    printf("Watchdog NOT Enabled\n");
#endif
#ifdef RTIHU_BOARD_LAUNCHPAD
    printf("Compiled for the Launchpad\n");
#endif
#ifdef RTIHU_BOARD_V11
    printf("Compiled for V1.1 board\n");
#endif

    printf("Free heap size is %d\n",xPortGetFreeHeapSize());
    {
        uint32_t *addr= (uint32_t *)0xf008015c,value,megs,kilos,size;
        value = ((*addr) & 0xFFFF);
        megs = value/1024;
        kilos = value % 1024;
        printf("Flash memory size %dMb+%dKb; ",megs,kilos);
        size = getSizeNV(ExternalMRAMData);
        printf("MRAM size is 0x%x, structure size is 0x%x\n\n\n", size,sizeof(MRAMmap_t));

    }

    //    printf("Previous reboot reason=%d (%s), WD reports=%x,task=%d\n",localErrorCollection.LegErrorCode,
    //            ErrMsg[localErrorCollection.LegErrorCode],
    //            localErrorCollection.LegWdReports,localErrorCollection.LegTaskNumber);
}
void printHelp(char *search,commandPairs *commands, int numberOfCommands){
    int commandNum,sizeCmd,numSpace,space;
    for (commandNum = 0; commandNum < numberOfCommands; commandNum++) {
        char *command = commands[commandNum].typedCommand;
        char *helpText = commands[commandNum].help;
        if(strstr(command,search)==NULL &&
                strstr(helpText,search) ==  NULL){
            continue;
        }
        sizeCmd = strlen(command);
        numSpace = 20-sizeCmd;
        printf("%s",command);
        for(space=0; space<numSpace;space++){
            printf(" ");
        }
        printf("%s\n",commands[commandNum].help);
    }
}

