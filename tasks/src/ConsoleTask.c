/*
 * ConsoleTask.c
 *
 *  Created on: Mar 4, 2019
 *      Author: burns
 */
/*
 * Standard headers for Golf
 */
#include <ax5043-ax25.h>
#include <pacsat.h>
#include "stdarg.h"
#include "stdlib.h"
#include "i2cEmulator.h"
#include "nonvolManagement.h"
#include "sys_common.h"
#include "serialDriver.h"
#include "uartEmulator.h"
#include "UplinkCommands.h"
#include "CommandTask.h"
#include "TelemetryRadio.h"
#include "consoleRoutines.h"
#include "ax5043.h"
#include "system.h"
#include "sys_core.h"
#include "gio.h"
#include "pinmux.h"
#include "spi.h"
#include "het.h"
#include "i2c.h"
#include "rti.h"
#include "esm.h"
#include "adc.h"
#include "ax5043-2M-AFSK-externs.h"
#include "config-2M-AFSK.h"

//FreeRTOS headers
#include "FreeRTOS.h"
#include "os_task.h"

//Flight software headers
#include "downlink.h"
#include "SystemServiceEntry.h"
#include "spiDriver.h"
#include "i2cDriver.h"
#include "serialDriver.h"
#include "TMS570Hardware.h"
#include "ADS7828.h"
#include "MET.h"
#include "nonvol.h"
#include "canDriver.h"
#include "CANSupport.h"
#include "ax5043.h"
#include "Max31725Temp.h"
#include "GPIO9539.h"
#include "inet.h"
#include "errors.h"
#include "gpioDriver.h"
#include "LinearInterp.h"
#include "Buzzer.h"
#include "keyfile.h"

#include "TxTask.h" // for test routines
#include "PbTask.h" // for test routines
#include "pacsat_header.h" // for test routines
#include "pacsat_dir.h" // for dir commands and test routines
//Extern definition
extern uint8_t SWCmdRing[SW_CMD_RING_SIZE],SWCmdIndex;


static char commandString[COM_STRING_SIZE]; /* Must be long enough for worst case: Uploadtest20xxxxxx  which is 86 */
static uint32_t DCTTxFreq,DCTRxFreq;


extern bool InSafeMode,InScienceMode,InHealthMode;
extern bool TransponderEnabled,onOrbit,SimDoppler;
extern resetMemory_t SaveAcrossReset;
extern char *ErrMsg[];
extern bool monitorPackets;
extern bool pb_shut;

bool TestPlayDead=false;


enum {
    nada=0

    ,getTemp
    ,reset
    ,resetBoth
    ,startWD
    ,preflight
    ,time
    ,version
    ,clrMinMax
    ,getI2cState
    ,internalWDTimeout
    ,externalWDTimeout
    ,telem0
    ,pollI2c
    ,readMRAMsr
    ,writeMRAMsr
    ,dropBus
    ,healthMode
    ,inhibitTx
    ,getState
    ,toneTx
    ,noToneTx
    ,testLED
    ,restartCAN
    ,doClearMRAM
    ,enbCanPrint
    ,dsbCanPrint
    ,EnableComPr
    ,DisableComPr
    ,GetRfPower
    ,SelectRFPowerLevels
    ,SetDCTDrivePower
    ,RaiseTxFreq
    ,LowerTxFreq
    ,RaiseRxFreq
    ,LowerRxFreq
    ,SaveFreq
    ,LoadKey
    ,showDownlinkSize
    ,ignoreUmb
    ,noticeUmb
    ,initSaved
    ,DisablePA
    ,EnablePA
    ,StartADC1
    ,StartADC2
    ,ReadADC1
    ,ReadADC2
    ,TestMemScrub
    ,GetCommands
    ,GetGpios
    ,getax5043
    ,getRSSI
    ,testRxFreq
    ,testPLLrange
    ,HelpAll
    ,HelpDevo
    ,HelpSetup
    ,Help
    ,Prime
    ,MRAMWrEn
    ,testAllMRAM
    ,sizeMRAM
    ,mramSleep
    ,mramAwake
    ,startRx
#ifdef DEBUG
    ,testPacsat
    ,testCallsigns
    ,testTx
    ,testPbOk
    ,testPbStatus
    ,testPbList
    ,testPbClearList
    ,testPfh
    ,testPfhFiles
    ,makePfhFiles
    ,testDir
#endif
    ,monitorOn
    ,monitorOff
    ,pbShut
    ,pbOpen
    ,mramLs
    ,dirLoad
    ,dirClear
    ,listDir
};


commandPairs setupCommands[] = {
                                {"init new proc","Init DCT stuff that will be set once for each unit",initSaved}
                                ,{"start tx tone", "Send a tone with 5043", toneTx}
                                ,{"stop tx tone", "Stop sending the tone",noToneTx}
                                ,{"raise tx freq","Raise the telem frequency by n Hz",RaiseTxFreq}
                                ,{"lower tx freq","Lower the telem frequency by n Hz",LowerTxFreq}
                                ,{"raise rx freq","Raise the command frequency by n Hz",RaiseRxFreq}
                                ,{"lower rx freq","Lower the command frequency by n Hz",LowerRxFreq}
                                ,{"save freq","Save the current frequency in MRAM",SaveFreq}
                                ,{"test freq", "xmit on receive frequency",testRxFreq}
                                ,{"test internal wd","Force internal watchdog to reset CPU",internalWDTimeout}
                                ,{"test external wd","Force external watchdog to reset CPU",externalWDTimeout}
                                ,{"test leds","Flash the LEDs in order",testLED}
                                ,{"test adc1","Read group 1 of ADC",StartADC1}
                                ,{"test adc2","Read group 2 of ADC",StartADC2}
                                ,{"test mram","Write and read mram with n blocksize (4,8,16,32,64,128)",testAllMRAM}
                                ,{"get mram size","Get the total size of all MRAMs",sizeMRAM}
                                ,{"load key","Load an authorization key for uplink commands",LoadKey}

};
commandPairs debugCommands[] = {

                                {"test scrub","Run the memory scrub routine once",TestMemScrub}
                                ,{"test pll", "test ax5043 PLL frequency range",testPLLrange}
                                ,{"start rx","Start up the 5043 receiver",startRx}
                                ,{"clear mram","Initializes MRAM state,WOD,Min/max but does not clear InOrbit--testing only",
                                  doClearMRAM}
                                ,{"reset ihu","Reset this processor",reset}
                                ,{"reset both","Reset both primary and secondary processor",resetBoth}
                                ,{"reset can","Reset the CAN devices",restartCAN}
                                ,{"poll i2c","Poll to see which I2c devices are there",pollI2c}
                                ,{"inhibit tx","Simulate FCC command to turn off tx",inhibitTx}
                                ,{"get mram sr","Get the MRAM status register",readMRAMsr}
                                ,{"get downlink size","Debug-get sizes of downlink payloads and frames",showDownlinkSize}
                                ,{"get temp","Get RT-IHU board temperature",getTemp}
                                ,{"prime","Do prime number benchmark",Prime}
                                ,{"mram wren","Write enable MRAM",MRAMWrEn}
                                ,{"mram wake","Send wake command to MRAM",mramAwake}
                                ,{"mram sleep","Send sleep command to MRAM",mramSleep}
#ifdef DEBUG
                                ,{"test pacsat","Run all of the PACSAT self tst routines",testPacsat}
                                ,{"test callsigns","Test the AX25 callsign routines",testCallsigns}
                                ,{"test tx","Test the Pacsat TX Packet routines",testTx}
                                ,{"test pb ok","Test the Pacsat Broadcast by sending OK packet",testPbOk}
                                ,{"send pb status","Send PB status",testPbStatus}
                                ,{"test pb list","Test the PB List add and remove functions",testPbList}
                                ,{"clear pb list","Clear the PB List add remove all stations",testPbClearList}
                                ,{"test pfh","Test the Pacsat File Header Routines",testPfh}
                                ,{"test psf","Test the Pacsat Files in MRAM",testPfhFiles}
                                ,{"make psf","Make a set of test Pacsat Files in MRAM",makePfhFiles}
                                ,{"test dir","Test the Pacsat Directory.  The command 'make psf' must already have been run",testDir}
#endif
                                ,{"monitor on","Monitor sent and received packets",monitorOn}
                                ,{"monitor off","Stop monitoring packets",monitorOff}
                                ,{"pb shut","Shut the PB",pbShut}
                                ,{"pb open","Open the PB for use",pbOpen}
                                ,{"mram ls","List the files in MRAM",mramLs}
                                ,{"load dir","Load the directory from MRAM",dirLoad}
                                ,{"clear dir","Clear the directory but leave the files in MRAM",dirClear}
                                ,{"list dir","List the Pacsat Directory.",listDir}

};
commandPairs commonCommands[] = {
                                 {"get i2c","What I2c devices are working?",getI2cState}
                                 ,{"get time","Display reset number and seconds",time}
                                 ,{"get status","Get some general status info",telem0}
                                 ,{"get rssi","Get the current RSSI reading from the AX5043 Rx",getRSSI}
                                 ,{"get state","Mainly for debug: Get the current state of the downlink state machine",getState}
                                 ,{"get version","Get the software version number and build time",version}
                                 ,{"get rf power","Print the RF power settings",GetRfPower}
                                 ,{"get commands","Get a list the last 4 s/w commands",GetCommands}
                                 ,{"get gpios","Display the values of all GPIOS",GetGpios}
                                 ,{"get ax","Get ax5043 status",getax5043}
                                 ,{"health mode","Set health mode",healthMode}
                                 ,{"set dct drive power","Set drive power for high and low power",SetDCTDrivePower}
                                 ,{"select dct power","Set rf power used in safe or normal modes",SelectRFPowerLevels}
                                 ,{"set mram sr","Set the MRAM status register",writeMRAMsr}
                                 ,{"set pa on","Turn on the RF power amp",EnablePA}
                                 ,{"set pa off","Turn off the RF power amp",DisablePA}
                                 ,{"start watchdog","Start the watchdog",startWD}
                                 ,{"preflight init","Initialize MRAM etc for flight",preflight}
                                 ,{"clear minmax","Clear the min, max, and CIU counts",clrMinMax}
                                 ,{"drop bus","Assert a fail briefly so the bus switches drop out",dropBus}
                                 ,{"ignore umbilical","Allow radios to go on with umbilical connected",ignoreUmb}
                                 ,{"heed umbilical","Reverse effects of ignore umbilical",noticeUmb}
                                 ,{"enable canprint","Debug can--number follows",enbCanPrint}
                                 ,{"disable canprint","Debug can--number follows",dsbCanPrint}
                                 ,{"enable comprint","Show uplink commands",EnableComPr}
                                 ,{"disable comprint","Turn off uplink command print",DisableComPr}
                                 ,{"helpall","List all commands",HelpAll}
                                 ,{"helpdevo","List commands for software devlopers",HelpDevo}
                                 ,{"helpsetup","List commands for setting up new board",HelpSetup}
                                 ,{"help","List common commands",Help}
                                 ,{"","",0}
};

char * ResetReasons[] = {
                         0,0,0,
                         "External",
                         "Software",
                         "CPU Reset",
                         0,0,0,0,0,0,0,
                         "Watchdog",
                         "Oscillator",
                         "Power On"
};

void RealConsoleTask(void)
{
    /* Block for 500ms. */
    int numberOfCommonCommands = sizeof(commonCommands)/sizeof(commandPairs);
    int numberOfSetupCommands = sizeof(setupCommands)/sizeof(commandPairs);
    int numberOfDebugCommands = sizeof(debugCommands)/sizeof(commandPairs);
    char * afterCommand;
    bool DoEcho = true;
    DCTTxFreq = ReadMRAMTelemFreq();
    DCTRxFreq = ReadMRAMCommandFreq();
    vTaskSetApplicationTaskTag((xTaskHandle) 0, (pdTASK_HOOK_CODE)ConsoleTsk); // For watchdog when it is called with "current task"

    while (true) {
        int commandNumber,index=0;
        if(DoEcho){
            SerialPutString(PRINTF_COM,"Pacsat",0);
        }
        receiveLine(PRINTF_COM, commandString, '>', DoEcho);
        if (commandString[0] == 0)
            continue; /* Ignore blank lines */
        /*
         * Start of command parsing.  First search the array of command string for one that
         * matches the beginning of the typed line.  When it is found, dispatch on the string
         * number.  There are enums which match the strings to use for the case statements.
         */
        for (commandNumber = 0; commandNumber < numberOfCommonCommands; commandNumber++) {
            /* If a command is found at the start of the type commands, leave the loop */
            if (strstr(commandString, commonCommands[commandNumber].typedCommand) == commandString){
                index = commonCommands[commandNumber].indexVal;
                afterCommand = commandString + strlen(commonCommands[commandNumber].typedCommand);

                break;
            }
        };
        if(index==0){
            for (commandNumber = 0; commandNumber < numberOfDebugCommands; commandNumber++) {
                /* If a command is found at the start of the type commands, leave the loop */
                if (strstr(commandString, debugCommands[commandNumber].typedCommand) == commandString){
                    index = debugCommands[commandNumber].indexVal;
                    afterCommand = commandString + strlen(debugCommands[commandNumber].typedCommand);
                    break;
                }
            }
        };
        if(index==0){
            for (commandNumber = 0; commandNumber < numberOfSetupCommands; commandNumber++) {
                /* If a command is found at the start of the type commands, leave the loop */
                if (strstr(commandString, setupCommands[commandNumber].typedCommand) == commandString){
                    index = setupCommands[commandNumber].indexVal;
                    afterCommand = commandString + strlen(setupCommands[commandNumber].typedCommand);
                    break;
                }
            }
        };
        /*
         * Here is the main command dispatch based on which command number was found above.
         */
        switch (index) {

        case nada: {
            printf("Unknown command\n");
            break;
        }
        case sizeMRAM:{
            int i;
            for (i=0;i<PACSAT_MAX_MRAMS;){
                printf("MRAM%d size is %dkB",i,getMRAMSize(MRAM_Devices[i])/1024);
                i++;
                if(i%2 == 0){
                    printf("\n");
                } else {
                    printf(", ");
                }
            }

            //           printf("Size of MRAM0 is %dKB\n",getMRAMSize(MRAM0Dev)/1024);
            //           printf("Size of MRAM1 is %dKB\n",getMRAMSize(MRAM1Dev)/1024);
            //           printf("Size of MRAM2 is %dKB\n",getMRAMSize(MRAM2Dev)/1024);
            //           printf("Size of MRAM3 is %dKB\n",getMRAMSize(MRAM3Dev)/1024);
            break;
        }
        case testAllMRAM:{
            int add = parseNumber(afterCommand);
            testMRAM(add);
            break;
        }
        case MRAMWrEn:{
            bool stat;
            stat=MRAMWriteEnable(MRAM0Dev);
            printf("stat for MRAM 0 is %d; sr is %x\n",stat,ReadMRAMStatus(MRAM0Dev));
            stat=MRAMWriteEnable(MRAM1Dev);
            printf("stat for MRAM 1 is %d; sr is %x\n",stat,ReadMRAMStatus(MRAM1Dev));
            //readNV(data,8,ExternalMRAMData,0);
            //printf("MRAM at address 0 and 1 are now now %d and %d\n",data[0],data[1]);
            break;
        }
        case Prime:{
            int n, i,flag, count=0, ms1,ms2;
            int maxNumber = parseNumber(afterCommand);
            if(maxNumber<2)maxNumber=2;
            ms1=xTaskGetTickCount();
            for(n=2;n<maxNumber;n++){
                flag = 0; //Init to assume it is prime

                for (i = 2; i <= n / 2; ++i) {

                    // if n is divisible by i, then n is not prime
                    // change flag to 1 for non-prime number
                    if (n % i == 0) {
                        flag = 1;
                        break;
                    }
                }

                // flag is 0 for prime numbers
                if (flag == 0)
                    count++;
            }
            ms2 = xTaskGetTickCount();
            printf("There are %d primes less than %d; This took %d centiseconds\n",count,maxNumber,(ms2-ms1));
            break;
        }
#if 1
        case GetGpios:{
            int i;
            char *gpioNames[NumberOfGPIOs]={
                                            "LED1","LED2","DCTInterrupt","CommandStrobe","CommandBits"
            };
            for (i=0;i<NumberOfGPIOs;i++){
                if(i%4 == 0){
                    printf("\n");
                }
                printf("%s:%d ",gpioNames[i],GPIORead((Gpio_Use)i));
            }
            printf("\n");
            break;
        }
#endif
        case GetCommands: {
            extern uint8_t SWCmdRing[SW_CMD_RING_SIZE];
            int i=0;
            printf("Commands received since reset: Hw=%d,Sw=%d\n\r",GetHWCmdCount(),GetSWCmdCount());

            printf("\n\rSoftware Commands received:\n\r");
            for (i=0;i<SW_CMD_RING_SIZE;i++){
                printf("%d/%d ",SWCmdRing[i]>>5,SWCmdRing[i] & 0x1f);
            }

            printf("\n\r");
            break;
        }
        case TestMemScrub:{
            ScrubECCMemory((uint64_t *)0x08000000,0x20000);
            break;
        }
        case StartADC1:{
            adcData_t data;
            adcStartConversion(adcREG1,adcGROUP1);
            vTaskDelay(SECONDS(1));
            adcGetData(adcREG1,adcGROUP1,&data);
            printf("Group 1 id=%d,value=%d\n",data.id,data.value);
            break;
        }
        case StartADC2:{
            adcData_t data[4];
            int i=0;
            adcResetFiFo(adcREG1,adcGROUP2);
            adcStartConversion(adcREG1,adcGROUP2);
            while(adcIsConversionComplete(adcREG1,adcGROUP2)==0){
                i++;
                vTaskDelay(1);
            }
            adcStopConversion(adcREG1,adcGROUP2);
            adcGetData(adcREG1,adcGROUP2,data);
            printf("Delay=%d\n",i);
            for(i=0;i<4;i++){
                printf("Group 2.%d id=%d,value=%d\n",i,data[i].id,data[i].value);
            }
            break;

        }
        case RaiseTxFreq:{
            int number = parseNumber(afterCommand);
            DCTTxFreq += number;
            printf("TxFreq=%d\n",DCTTxFreq);
            quick_setfreq(DCTDev0, DCTTxFreq);
            break;
        }
        case LowerTxFreq:{
            int number = parseNumber(afterCommand);
            DCTTxFreq -= number;
            printf("TxFreq=%d\n",DCTTxFreq);
            quick_setfreq(DCTDev0, DCTTxFreq);
            break;
        }
        case RaiseRxFreq:{
            int number = parseNumber(afterCommand);
            DCTRxFreq += number;
            printf("RxFreq=%d\n",DCTRxFreq);
            quick_setfreq(DCTDev1, DCTRxFreq);
            break;
        }
        case LowerRxFreq:{
            int number = parseNumber(afterCommand);
            DCTRxFreq -= number;
            printf("RxFreq=%d\n",DCTRxFreq);
            quick_setfreq(DCTDev1, DCTRxFreq);
            break;
        }
        case SaveFreq:{
            printf("Saving Rx frequency %d and Tx frequency %d to MRAM\n",DCTRxFreq,DCTTxFreq);
            WriteMRAMTelemFreq(DCTTxFreq);
            WriteMRAMCommandFreq(DCTRxFreq);
            break;
        }
        case LoadKey:{
            uint8_t key[16],i;
            uint32_t magic = ENCRYPTION_KEY_MAGIC_VALUE,checksum=0;
            static const MRAMmap_t *LocalFlash = 0;
            char *str = afterCommand;
            bool stat;
            for(i=0;i<16;i++){
                char *next = strtok(str," ,\n");
                str = NULL;
                if (next != NULL){
                    key[i] = (uint8_t)strtol(next,0,16);
                    printf("%d=0x%x ",i,key[i]);
                    checksum+=key[i];
                } else {
                    printf("Not enough numbers...");
                    break;
                }
            }
            printf("\n");
            if(i==16){
                printf("Writing key...");
                stat = writeNV(key,sizeof(LocalFlash->AuthenticateKey.key),ExternalMRAMData,(int)&LocalFlash->AuthenticateKey.key);
            } else {
                stat = false;
            }
            if(stat){
                printf("Writing checksum=%x...",checksum);
                stat = writeNV(&checksum,sizeof(LocalFlash->AuthenticateKey.keyChecksum),ExternalMRAMData,
                               (int)&LocalFlash->AuthenticateKey.keyChecksum);
            }
            if(stat){
                printf("Writing valid\n");
            } else {
                magic = 0;
                printf("Invalidating stored key\n");
            }
            stat = writeNV(&magic,sizeof(LocalFlash->AuthenticateKey.magic),ExternalMRAMData,(int)&LocalFlash->AuthenticateKey.magic);
            break;
        }

        case GetRfPower:
            //printf("Safe Rf Power Level is %s\n",GetSafeRfPowerLevel()?"HIGH":"LOW");
            //printf("Normal Rf Power Level is %s\n",GetNormalRfPowerLevel()?"HIGH":"LOW");
            break;

        case SelectRFPowerLevels:
        {
            uint16_t args[4];
            args[0] = parseNumber(afterCommand);
            args[1] = parseNextNumber();
            args[2] = parseNextNumber();
            args[3] = parseNextNumber();

            printf("Select %s power for safe, %s power for normal on primary\n",args[0]?"HIGH":"LOW",args[1]?"HIGH":"LOW");
            printf("Select %s power for safe, %s power for normal on secondary\n",args[2]?"HIGH":"LOW",args[3]?"HIGH":"LOW");
            SimulateSwCommand(SWCmdNSSpaceCraftOps,SWCmdOpsSelectDCTRFPower,args,4); //Send to others
            break;
        }
        case SetDCTDrivePower:
        {
            uint16_t args[4];
            args[0] = parseNumber(afterCommand);
            args[1] = parseNextNumber();
            args[2] = parseNextNumber();
            args[3] = parseNextNumber();
            printf("Set primary low power to %d, secondary low power to %d\n",args[0],args[1]);
            printf("Set primary high power to %d, secondary high power to %d\n",args[2],args[3]);
            SimulateSwCommand(SWCmdNSTelemetry,SWCmdTlmDCTDrivePwr,args,4);
            break;
        }

        case showDownlinkSize:{
#define memberSize(type, member) sizeof(((type *)0)->member)
            printf("\nPayload Sizes: Header=%d,RTHealth=%d (common=%d,common2=%d,specific=%d),\nmin=%d,max=%d,WODHealth=%d,Diag=%d\n",
                   sizeof(header_t),sizeof(realTimePayload_t),
                   sizeof(commonRtMinmaxWodPayload_t),sizeof(commonRtWodPayload_t),sizeof(realtimeSpecific_t),
                   sizeof(minValuesPayload_t),sizeof(maxValuesPayload_t),
                   sizeof(WODHousekeepingPayload_t),sizeof(DiagnosticPayload_t));
            printf("               RagnarokRT=%d,RagnarokWod=%d,RadiationRT=%d,RadiationWod=%d\n\n",
                   sizeof(ragnarok_t),sizeof(WODRagnarokPayload_t),sizeof(radiation_t),sizeof(WODRadiationPayload_t));

            printf("Frame sizes: \n"
                    "      Payload Only,   Current Filler   Current Size\n");

            printf("RT1        %03d           %03d             %03d\n"
                    "RT2        %03d           %03d             %03d\n",
                    sizeof(realTimeMinFrame_t)-memberSize(realTimeMinFrame_t,filler),memberSize(realTimeMinFrame_t,filler),sizeof(realTimeMinFrame_t),
                    sizeof(realTimeMaxFrame_t)-memberSize(realTimeMaxFrame_t,filler),memberSize(realTimeMaxFrame_t,filler),sizeof(realTimeMaxFrame_t));

            printf("AllWOD1    %03d           %03d             %03d\n"
                    "AllWOD2    %03d           %03d             %03d\n"
                    "AllWOD3    %03d           %03d             %03d\n",
                    sizeof(allWOD1Frame_t)-memberSize(allWOD1Frame_t,filler),memberSize(allWOD1Frame_t,filler),sizeof(allWOD1Frame_t),
                    sizeof(allWOD2Frame_t)-memberSize(allWOD2Frame_t,filler),memberSize(allWOD2Frame_t,filler),sizeof(allWOD2Frame_t),
                    sizeof(allWOD3Frame_t)-memberSize(allWOD3Frame_t,filler),memberSize(allWOD3Frame_t,filler),sizeof(allWOD3Frame_t)
            );
            printf("SafeData1  %03d           %03d             %03d\n"
                    "SafeData2  %03d           %03d             %03d\n"
                    "SafeWOD    %03d           %03d             %03d\n"
                    "Diagnostic %03d           %03d             %03d\n"
                    ,sizeof(safeData1Frame_t)-memberSize(safeData1Frame_t,filler),memberSize(safeData1Frame_t,filler),sizeof(safeData1Frame_t)
                    ,sizeof(safeData2Frame_t)-memberSize(safeData2Frame_t,filler),memberSize(safeData2Frame_t,filler),sizeof(safeData2Frame_t)
                    ,sizeof(safeWODFrame_t)-memberSize(safeWODFrame_t,filler),memberSize(safeWODFrame_t,filler),sizeof(safeWODFrame_t)
                    ,sizeof(diagFrame_t)-memberSize(diagFrame_t,filler),memberSize(diagFrame_t,filler),sizeof(diagFrame_t)
            );
            printf("\nAllFrames = %d\n",sizeof(allFrames_t));

            break;
        }
        case mramSleep:{
            int num = parseNumber(afterCommand);
            MRAMSleep(num);
            break;
        }
        case mramAwake:{
            int num = parseNumber(afterCommand);
            MRAMWake(num);
            break;
        }
        case doClearMRAM:{
            SetupMRAM();
            WriteMRAMBoolState(StateInOrbit,true); // Don't get confused by in orbit state!
            break;
        }
        case ignoreUmb:
            //OverrideUmbilical(true);
            break;
        case noticeUmb:
            //OverrideUmbilical(false);
            break;
        case restartCAN:{
            CANPacket_t readData;
            int i;
            CANRestart();
            printf("All CAN buses restarted\n");
            for(i=10;i<=12;i++){
                printf("CAN1 read return:  Msg Box=%d, status=%d\n",i,
                       CANReadMessage(CAN1,i,&readData));
                printf("CAN2 read return:  Msg Box=%d, status=%d\n",i,
                       CANReadMessage(CAN2,i,&readData));
            }
            break;
        }
        case testLED:{
            GPIOSetOff(LED1);
            GPIOSetOff(LED2);
            vTaskDelay(SECONDS(2));
            GPIOSetOn(LED1);
            vTaskDelay(SECONDS(2));
            GPIOSetOff(LED1);
            GPIOSetOn(LED2);
            vTaskDelay(SECONDS(2));
            GPIOSetOff(LED2);
            break;
        }

        case inhibitTx:{
            SimulateHWCommand(CMD_TX_OFF);
            break;
        }
        case EnableComPr:
            EnableCommandPrint(true);
            break;
        case DisableComPr:
            EnableCommandPrint(false);
            break;
        case healthMode:{
            SimulateSwCommand(SWCmdNSSpaceCraftOps,SWCmdOpsHealthMode,NULL,0);
            break;
        }
        case readMRAMsr:{
            int i;
            for (i=0;i<PACSAT_MAX_MRAMS;){
                printf("MRAM%d: status %x",i,ReadMRAMStatus(MRAM_Devices[i]));
                i++;
                if(i%2 == 0){
                    printf("\n");
                } else {
                    printf(", ");
                }
            }
            break;
        }
        case writeMRAMsr:{
            uint8_t stat = parseNumber(afterCommand);
            int i;
            for (i=0;i<PACSAT_MAX_MRAMS;i++){
                WriteMRAMStatus(MRAM_Devices[i],stat);
            }
            break;
        }
        case internalWDTimeout:{
            ForceInternalWatchdogTrigger();
            break;
        }
        case externalWDTimeout:{
            ForceExternalWatchdogTrigger();
            break;
        }
        case pollI2c:{
            I2CDevicePoll();
            break;
        }
        case getI2cState:{
            printf("I2c device state:\n"
                    "    ICR ADC: %d    Solar ADC:   %d,   CSS ADC: %d\n"
                    "RT-IHU Temp %d\n",
                    ICRTelemIsOk(),SolarTelemIsOk(),CSSTelemIsOk(),
                    RTTempIsOk());
            break;
        }
        case telem0:{
            DisplayTelemetry(0);
            break;
        }
        case enbCanPrint:{
            int canType = parseNumber(afterCommand);
            switch(canType){
            case 1:
                CANPrintCoord = true; break;
            case 2:
                CANPrintTelemetry = true; break;
            case 3:
                CANPrintCommands = true; break;
            case 4:
                CANPrintCount = true; break;
            case 5:
                CANPrintAny = true; break;
            case 6:
                CANPrintErrors = true; break;
            case 7:
                CANPrintAny = true; break;
            case 8:
                CANPrintEttus = true; break;
            default:{
                printf("Specify 1=Coordinate, 2=telemetry, 3=Commands,\n");
                printf("4=Poll and CAN Received packet count,5=All packets\n");
                break;
            }
            }
            break;
        }

        case dsbCanPrint:{
            int canType = parseNumber(afterCommand);
            switch(canType){
            case 1:
                CANPrintCoord = false; break;
            case 2:
                CANPrintTelemetry = false; break;
            case 3:
                CANPrintCommands = false; break;
            case 4:
                CANPrintCount = false; break;
            case 5:
                CANPrintAny = false; break;
            case 6:
                CANPrintErrors = false; break;
            case 7:
                CANPrintAny = false; break;
            case 8:
                CANPrintEttus = false; break;
            }
            break;
        }

        case noToneTx:{
            printf("NOT IMPLEMENMTED\n");
//            printf("Turning off tone; telemetry enabled\n");
//            AudioSetMixerSource(MixerSilence);
            break;
        case toneTx: {
            printf("NOT IMPLEMENMTED\n");
//            printf("Sending a tone\n");
//            AudioSetMixerSource(MixerSilence);  //Stop everything
//            AudioSetMixerSource(MixerTone);
//            ax5043StartTx(DCTDev0);
            break;
        }
        }
        case startRx:{
            ax5043StartRx(DCTDev1);
            break;
        }

        case version:{
            printID();
            break;
        }
        case time:{
            logicalTime_t time;
            getTime(&time);
            printf("IHU time:  Resets=%i,seconds=%i\n",time.IHUresetCnt,time.METcount);
            getTimestamp(&time);
            printf("Timestamp time:  Epoch=%i,seconds=%i\n",time.IHUresetCnt,time.METcount);
            printf("Poweron Time since preflight: %d seconds\n",getSecondsInOrbit());

            printf("Short boot count: %d, short boot flag %d\n\r",
                   SaveAcrossReset.fields.earlyResetCount,
                   SaveAcrossReset.fields.wasStillEarlyInBoot
            );

            break;
        }
        case preflight:{
            PreflightInitNow(Any_Node); //We will take Any_Node to mean send to everyone
            break;

        }
        case startWD:{
            SWISetWDTimer();
            dwdReset();
            SWIStartWatchdog();
            break;
        }
        case resetBoth:{
            uint16_t args[]={0,1,1};
            SimulateSwCommand(SWCmdNSSpaceCraftOps,SWCmdOpsResetSpecified,args,3);
            break;
        }
        case reset:{
            vTaskDelay(CENTISECONDS(50));
            ProcessorReset();
            break;
        }
        case getTemp:{
            uint8_t temp8;
            if(Get8BitTemp31725(&temp8)){
                printf("Board temp is ");
                print8BitTemp(temp8);
                printf(" (%d)\n",temp8);
            } else {
                printf("I2c temp request failed\n");
            }
            break;
        }

        case getax5043:{
            SPIDevice dev = DCTDev0;
            printf("AX5043 DEVICE: %d\n",dev);
            printf(" FIFOSTAT: %02x\n", ax5043ReadReg(dev, AX5043_FIFOSTAT));
            printf(" PWRMODE:: %02x\n", ax5043ReadReg(dev, AX5043_PWRMODE));
            printf(" XTALCAP: %d\n", ax5043ReadReg(dev, AX5043_XTALCAP));
            printf(" PLLLOOP: %02.2x\n", ax5043ReadReg(dev, AX5043_PLLLOOP));
            printf(" PLLCPI: %02.2x\n", ax5043ReadReg(dev, AX5043_PLLCPI));
            printf(" PLLVCOI: %02.2x\n", ax5043ReadReg(dev, AX5043_PLLVCOI));
            printf(" PLLRANGINGA: %02.2x\n", ax5043ReadReg(dev, AX5043_PLLRANGINGA));
            printf(" PLLVCODIV: %02.2x\n", ax5043ReadReg(dev, AX5043_PLLVCODIV));
            printf(" FREQ: %x", ax5043ReadReg(dev, AX5043_FREQA0));
            printf(" %x", ax5043ReadReg(dev, AX5043_FREQA1));
            printf(" %x", ax5043ReadReg(dev, AX5043_FREQA2));
            printf(" %x\n", ax5043ReadReg(dev, AX5043_FREQA3));
            printf(" MODULATION: %x\n", ax5043ReadReg(dev, AX5043_MODULATION));
            printf(" TXPWRCOEFFB0: %x\n", ax5043ReadReg(dev, AX5043_TXPWRCOEFFB0));
            printf(" TXPWRCOEFFB1: %x\n", ax5043ReadReg(dev, AX5043_TXPWRCOEFFB1));

            break;
        }

        case getRSSI:{
            int rssi = get_rssi(DCTDev1);
            printf("RSSI is %d\n",((int16_t)rssi) - 255);
            break;
        }
        case testRxFreq: {
            // This is so we can find what the receive frequency is on first build
            // {
            //                int freq = 145835000;
            //
            //                ///ax5043PowerOn();
            //
            //                printf("Transmitting on receive freq: %d\n", freq);
            //
            //                uint8_t retVal = axradio_init_2m(freq);
            //                printf("axradio_init_2m: %d\n",retVal);
            //
            //
            //                retVal = mode_tx_2m();
            //                printf("mode_tx_2m: %d\n",retVal);
            //
            //                ax5043WriteReg(AX5043_PWRMODE, AX5043_PWRSTATE_FULL_TX);
            //                printf("Powerstate is FULL_TX\n");
            //
            //                printf("AX5043_XTALCAP: %d\n", ax5043ReadReg(AX5043_XTALCAP));
            //
            //                fifo_repeat_byte(0xAA, 100, 0);
            //                fifo_repeat_byte(0xAA, 100, 0);
            //                fifo_repeat_byte(0xAA, 100, 0);
            //                fifo_repeat_byte(0xAA, 100, 0);
            //                fifo_repeat_byte(0xAA, 100, 0);
            //                fifo_repeat_byte(0xAA, 100, 0);
            //                fifo_repeat_byte(0xAA, 100, 0);
            //                fifo_repeat_byte(0xAA, 100, 0);
            //                fifo_repeat_byte(0xAA, 100, 0);
            //                fifo_repeat_byte(0xAA, 100, 0);
            //                fifo_repeat_byte(0xAA, 100, 0);
            //                fifo_repeat_byte(0xAA, 100, 0);
            //                fifo_repeat_byte(0xAA, 100, 0);
            //                fifo_repeat_byte(0xAA, 100, 0);
            //
            //                fifo_commit();
            //            }
            break;
        }

        case testPLLrange:{
            printf("Sorry, this was temporarily removed while two radio devices are being implemented\n");

            //            test_pll_range(DCTDev0);
            break;
        }

#ifdef DEBUG
        /* G0KLA TEST ROUTINES */
        case testPacsat:{

            if(! pb_test_callsigns()) {  debug_print("### Callsign TEST FAILED\n"); break; }
            if(! pb_test_list()) {  debug_print("### pb list TEST FAILED\n"); break; }
            if(! pb_clear_list()) {  debug_print("### pb list clear TEST FAILED\n"); break; }
            if(! tx_test_make_packet()) {  debug_print("### tx make packet TEST FAILED\n"); break; }
            if(! test_pfh()) {  debug_print("### pfh TEST FAILED\n"); break; }
            if(! test_pfh_files()) {  debug_print("### pfh TEST FILES FAILED\n"); break; }

            //if(! test_pfh_make_files()) {  debug_print("### pfh TEST MAKE FILES FAILED\n"); break; }

            debug_print("### ALL TESTS PASSED\n");
            break;
        }
        case testCallsigns:{
            bool rc = pb_test_callsigns();
            break;
        }
        case testPbOk:{
            bool rc = pb_test_ok();
            break;
        }
        case testPbStatus:{
            bool rc = pb_test_status();
            break;
        }
        case testPbList:{
            bool rc = pb_test_list();
            break;
        }
        case testPbClearList:{
            bool rc = pb_clear_list();
            break;
        }
        case testTx:{
            bool rc = tx_test_make_packet();
            break;
        }
        case testPfh:{
            bool rc = test_pfh();
            break;
        }
        case testPfhFiles:{
            bool rc = test_pfh_files();
            break;
        }
        case makePfhFiles:{
            bool rc = test_pfh_make_files();
            break;
        }
        case testDir:{
            bool rc = test_pacsat_dir();
            break;
        }

#endif /* DEBUG */

        case monitorOn:{
            monitorPackets=true;
            printf("monitorPackets = true\n");
            break;
        }
        case monitorOff:{
            monitorPackets=false;
            printf("monitorPackets = false\n");
            break;
        }
        case pbShut:{
            pb_shut=true;
            printf("pb_shut = true\n");
            break;
        }
        case pbOpen:{
            pb_shut=false;
            printf("pb_shut = false\n");
            break;
        }
        case mramLs:{
            printf("MRAM Directory Listing\n");
            static const MRAMmap_t *LocalFlash = 0;
            MRAM_FILE mramFile;
            int j = 0;
            uint32_t numOfFiles = 0;
            bool rc;

            rc = readNV(&numOfFiles, sizeof(uint32_t),ExternalMRAMData, (int)&(LocalFlash->NumberOfFiles));
            if (!rc) {
                debug_print("Read MRAM number of files - FAILED\n");
                break;
            }
            debug_print("-- files: %d\n",numOfFiles);

            while (j < numOfFiles) {
//                rc = readNV(&mramFile, sizeof(mramFile),ExternalMRAMData, (int)&(LocalFlash->MRAMFiles[j++]));
                rc = dir_mram_get_node(j,&mramFile);
                if (!rc) {
                    debug_print("Read MRAM FAT - FAILED\n");
                    break;
                }
                printf("%d: Id: %04x ",j,mramFile.file_id);
                printf("Size: %d ",mramFile.file_size);
                printf("Header Len: %d ",mramFile.body_offset);
                printf("Address: %d ",mramFile.address);
                printf("Uploaded: %d\n",mramFile.upload_time);
                j++;
                uint8_t buffer[256];
                uint32_t len = mramFile.file_size;
                if (len > sizeof(buffer))
                    len = sizeof(buffer);
                rc = readNV(&buffer, len ,ExternalMRAMData, (int)mramFile.address);
                int q;
                for (q=0; q< sizeof(buffer); q++) {
                    printf("%02x ", buffer[q]);
                    if (q != 0 && q % 20 == 0 ) printf("\n");
                }
                printf("\n");
            }
            break;
        }
        case dirLoad:{
            bool rc = dir_load();
            break;
        }
        case dirClear:{
            dir_free();
            break;
        }


        case listDir:{
            bool rc = dir_debug_print(NULL); // pass NULL to print from the head of the list
            break;
        }

        case HelpAll:{
            int numSpace=0;
            char *srchStrng;
            while(afterCommand[numSpace] == ' ') numSpace++;
            srchStrng = &afterCommand[numSpace];
            if(strlen(srchStrng)== 0){
                printf("\n***LIST OF ALL COMMANDS:\n\n");
            } else {
                printf("\n***LIST OF ALL COMMANDS CONTAINING %s:\n\n",srchStrng);
            }
            printHelp(srchStrng,commonCommands,numberOfCommonCommands);
            printHelp(srchStrng,setupCommands,numberOfSetupCommands);
            printHelp(srchStrng,debugCommands,numberOfDebugCommands);
            break;
        }
        case HelpDevo:{
            int numSpace=0;
            char *srchStrng;
            while(afterCommand[numSpace] == ' ') numSpace++;
            srchStrng = &afterCommand[numSpace];
            if(strlen(srchStrng)== 0){
                printf("\n***LIST OF DEBUG/DEVELOPER COMMANDS:\n\n");
            } else {
                printf("\n***LIST OF DEBUG/DEVELOPER COMMANDS CONTAINING %s:\n\n",srchStrng);
            }
            printHelp(srchStrng,debugCommands,numberOfDebugCommands);
            break;
        }
        case HelpSetup:{
            int numSpace=0;
            char *srchStrng;
            while(afterCommand[numSpace] == ' ') numSpace++;
            srchStrng = &afterCommand[numSpace];
            if(strlen(srchStrng)== 0){
                printf("\n***LIST OF NEW BOARD SETUP/TEST COMMANDS:\n\n");
            } else {
                printf("\n***LIST OF NEW BOARD SETUP/TEST COMMANDS CONTAINING %s:\n\n",srchStrng);
            }
            printHelp(srchStrng,setupCommands,numberOfSetupCommands);
            break;
        }
        case Help: {
            int numSpace=0;
            char *srchStrng;
            while(afterCommand[numSpace] == ' ') numSpace++;
            srchStrng = &afterCommand[numSpace];
            if(strlen(srchStrng)== 0){
                printf("***List of common commands:\n\n");
            } else {
                printf("***List of common commands containing %s:\n\n",srchStrng);
            }

            printHelp(srchStrng,commonCommands,numberOfCommonCommands);
            break;
        }

        default:
            printf("Unknown command\n");
        }
    }
}
