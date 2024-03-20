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
#include "nonvolManagement.h"
#include "sys_common.h"
#include "serialDriver.h"
#include "UplinkCommands.h"
#include "CommandTask.h"
#include "TelemetryRadio.h"
#include "consoleRoutines.h"
#include "ax5043.h"
#include "ax5043_access.h"
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
#include "mram.h"

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
#include "I2cPoll.h"
#include "MET.h"
#include "nonvol.h"
#include "ax5043.h"
#include "Max31725Temp.h"
#include "Max31331Rtc.h"
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
#include "redposix.h"
#include "Ax25Task.h"
#include "UplinkTask.h"
//Extern definition
extern uint8_t SWCmdRing[SW_CMD_RING_SIZE],SWCmdIndex;


static char commandString[COM_STRING_SIZE]; /* Must be long enough for worst case: Uploadtest20xxxxxx  which is 86 */
static uint32_t DCTTxFreq,DCTRxFreq[4];


extern bool InSafeMode,InScienceMode,InHealthMode;
extern bool TransponderEnabled,onOrbit,SimDoppler;
extern resetMemory_t SaveAcrossReset;
extern char *ErrMsg[];
extern bool monitorPackets;

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
    ,MountFS
    ,UnMountFS
    ,FormatFS
    ,LsFS
    ,RmFS
    ,mkdirFS
    ,rmdirFS
    ,GetRfPower
    ,SelectRFPowerLevels
    ,SetDCTDrivePower
    ,RaiseTxFreq
    ,LowerTxFreq
    ,RaiseRxFreq
    ,LowerRxFreq
    ,ReadFreqs
    ,SaveFreq
    ,LoadKey
    ,showDownlinkSize
    ,ignoreUmb
    ,noticeUmb
    ,initSaved
    ,DisablePA
    ,EnablePA
    ,DisableAx
    ,EnableAx
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
    ,testAX5043
    ,HelpAll
    ,HelpDevo
    ,HelpSetup
    ,Help
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
    ,testPfhFile
    ,testDecode
    ,makePfhFiles
    ,testDir
    ,sendUplinkStatus
    ,testRetransmission
    ,testUploadTable
    ,listUploadTable
#endif
    ,monitorOn
    ,monitorOff
    ,pbShut
    ,pbOpen
    ,uplinkShut
    ,uplinkOpen
    ,digiShut
    ,digiOpen
    ,mramHxd
    ,dirLoad
    ,dirClear
    ,listDir
    ,getNextFileNumber
    ,resetNextFileNumber
    ,heapFree
    ,setUnxTime
    ,getUnxTime
    ,setRate9600
    ,setRate1200
    ,getRtc
    ,setRtc
    ,regRtc
    ,setDigi
};


commandPairs setupCommands[] = {
                                {"init new proc","Init DCT and MRAM stuff that will be set once for each unit",initSaved}
                                ,{"start tx tone", "Send a tone with 5043", toneTx}
                                ,{"stop tx tone", "Stop sending the tone",noToneTx}
                                ,{"raise tx freq","Raise the telem frequency by n Hz",RaiseTxFreq}
                                ,{"lower tx freq","Lower the telem frequency by n Hz",LowerTxFreq}
                                ,{"raise rx freq","Raise the command frequency by n Hz",RaiseRxFreq}
                                ,{"lower rx freq","Lower the command frequency by n Hz",LowerRxFreq}
                                ,{"save freq","Save the current frequency in MRAM",SaveFreq}
                                ,{"read freq","Read all the frequencies from MRAM",ReadFreqs}
                                ,{"test freq", "xmit on receive frequency of specified device",testRxFreq}
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
                                ,{"test ax","Read the revision and scratch registers",testAX5043}
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
                                ,{"test psf","Test the Pacsat Files in MRAM",testPfhFile}
                                ,{"test decode","Test decode of AX25 packets",testDecode}
                                ,{"make psf","Make a set of test Pacsat Files in MRAM",makePfhFiles}
                                ,{"test dir","Test the Pacsat Directory.  The command 'make psf' must already have been run",testDir}
                                ,{"send uplink status","Send Uplink status",sendUplinkStatus}
                                ,{"test retransmission","Test the AX25 I frame retransmission",testRetransmission}
                                ,{"test upload table","Test the storage of Upload records in the MRAM table",testUploadTable}
                                ,{"list upload table","List the Upload records in the MRAM table",listUploadTable}
                                ,{"set digi","Set the digipeater mode",setDigi}
#endif
                                ,{"monitor on","Monitor sent and received packets",monitorOn}
                                ,{"monitor off","Stop monitoring packets",monitorOff}
                                ,{"shut pb","Shut the PB",pbShut}
                                ,{"open pb","Open the PB for use",pbOpen}
                                ,{"shut uplink","Shut the FTL0 Uplink",uplinkShut}
                                ,{"open uplink","Open the FTL0 Uplink for use",uplinkOpen}
                                 ,{"shut digi","Disable the Digipeater",digiShut}
                                 ,{"open digi","Enable the Digipeater",digiOpen}
                                ,{"hxd","Display Hex for file number",mramHxd}
                                ,{"load dir","Load the directory from MRAM",dirLoad}
                                ,{"clear dir","Clear the directory but leave the files in MRAM",dirClear}
                                ,{"list dir","List the Pacsat Directory.",listDir}
                                ,{"get next filenumber","Show the next file number that the Dir will assign to an uploaded file",getNextFileNumber}
                                ,{"reset next filenumber","Reset the next Dir file number to zero.",resetNextFileNumber}
                                ,{"heap free","Show free bytes in the heap.",heapFree}
                                ,{"get unix time","Get the number of seconds since the Unix epoch",getUnxTime}
                                ,{"set unix time","Set the Real Time Clock and update the IHU Unix time",setUnxTime}
                                ,{"set rate 1200","Set the radio to 1200 bps AFSK packets",setRate1200}
                                ,{"set rate 9600","Set the radio to 9600 bps GMSK packets",setRate9600}
                                ,{"get rtc","Get the status and time from the Real Time Clock",getRtc}
                                ,{"set rtc","Set the Real Time Clock and update the IHU Unix time",setRtc}
                                ,{"get regrtc","Read the specified register in the rtc",regRtc}

};
commandPairs commonCommands[] = {
                                 {"get i2c","What I2c devices are working?",getI2cState}
                                 ,{"get time","Display reset number and seconds",time}
                                 ,{"get status","Get some general status info",telem0}
                                 ,{"get rssi","Get the current RSSI reading from the AX5043 Rx",getRSSI}
                                 ,{"get state","Mainly for debug: Get the current state of the downlink state machine",getState}
                                 ,{"get version","Get the software version number and build time",version}
                                 ,{"mount fs","Mount the filesystem",MountFS}
                                 ,{"unmount fs","unmount the filesystem",UnMountFS}
                                 ,{"format fs","Format the filesystem",FormatFS}
                                 ,{"ls","List files and directories in the filesystem",LsFS}
                                 ,{"rm","Remove a file from the filesystem",RmFS}
                                 ,{"mkdir","Make a directory in the filesystem",mkdirFS}
                                 ,{"rmdir","Remove a directory from the filesystem",rmdirFS}
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
                                 ,{"set ax on","Turn on the RF power amp",EnableAx}
                                 ,{"set ax off","Turn off the RF power amp",DisableAx}
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
    DCTRxFreq[0] = ReadMRAMReceiveFreq(0);
    DCTRxFreq[1] = ReadMRAMReceiveFreq(1);
    DCTRxFreq[2] = ReadMRAMReceiveFreq(2);
    DCTRxFreq[3] = ReadMRAMReceiveFreq(3);
    if((DCTTxFreq<999000) || (DCTTxFreq>600000000)){
        DCTTxFreq = DCT_DEFAULT_TX_FREQ;
    }
    if((DCTRxFreq[0]<999000) || (DCTRxFreq[0]>600000000)){
        DCTRxFreq[0] = DCT_DEFAULT_RX_FREQ[0];
    }
    quick_setfreq(RX1_DEVICE, DCTRxFreq[0]);
    quick_setfreq(RX2_DEVICE, DCTRxFreq[1]);
    quick_setfreq(RX3_DEVICE, DCTRxFreq[2]);
    quick_setfreq(RX4_DEVICE, DCTRxFreq[3]);
    quick_setfreq(TX_DEVICE, DCTTxFreq);

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
        case DisablePA:{
            GPIOSetOn(SSPAPower); //The switch is inverted
            break;
        }
        case EnablePA:{
            GPIOSetOff(SSPAPower);
            break;
        }
        case DisableAx:{
            GPIOSetOn(AX5043Power);
            break;
        }
        case EnableAx:{
            GPIOSetOff(AX5043Power);
            break;
        }

        case sizeMRAM:{
            int i;
            printf("MRAM Address Size=%d\n",getMRAMAddressSize());
            printf("Partition 0 size=%d, partition 1=%d\n",getMRAMPartitionSize(0),getMRAMPartitionSize(1));
            for (i=0;i<PACSAT_MAX_MRAMS;){
                printf("MRAM%d size is %dKBytes",i,getMRAMSize(i)/1024);
                i++;
                if(i%2 == 0){
                    printf("\n");
                } else {
                    printf(", ");
                }
            }

 //           printf("Size of MRAM0 is %dKB\n",getMRAMSize(0)/1024);
 //           printf("Size of MRAM1 is %dKB\n",getMRAMSize(1)/1024);
 //           printf("Size of MRAM2 is %dKB\n",getMRAMSize(MRAM2Dev)/1024);
 //           printf("Size of MRAM3 is %dKB\n",getMRAMSize(MRAM3Dev)/1024);
            break;
        }
        case regRtc:{
            uint8_t readReg=parseNumber(afterCommand);
            bool status;
            //uint8_t data[8];
            uint8_t data;
            status = I2cSendCommand(MAX31331_PORT,MAX31331_ADDR,&readReg,1,&data,1);
 //           printf("Status=%d,reg values from %d are: %x %x %x %x %x %x %x %x\n",
 //                   status,readReg,data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7]);
            printf("Status=%d,reg %d value: 0x%x\n",
                    status,readReg,data);
            break;
        }
        case testAllMRAM:{
            int add = parseNumber(afterCommand);
            int secret = parseNextNumber();
            if (add == 0)add=4;
            printf("NOTE:  This test will wipe out the file system and configuration values in MRAM.\n\n");
            if(secret!=42){
                printf("  To confirm you must give the command with a second argument of 42, for example 'test mram %d 42'\n",add);
                break;
            }
            testMRAM(add);
            break;
        }
        case MRAMWrEn:{
            bool stat;
            int num = parseNumber(afterCommand);

            stat = writeEnableMRAM(num);
            printf("stat for MRAM %d is %d; sr is %x\n", num, stat,
		   readMRAMStatus(num));
            //readNV(data,8,NVConfigData,0);
            //printf("MRAM at address 0 and 1 are now now %d and %d\n",data[0],data[1]);
            break;
        }
#ifdef DEBUG
        case GetGpios:{
            int i;
            char *gpioNames[NumberOfGPIOs]={
               "LED1","LED2","LED3","Rx0Interrupt","Rx1Interrupt","Rx2Interrupt2","Rx3Interrupt"
               ,"TxInterrupt4","CommandStrobe","CommandBits",
               "SSPAPower","AX5043Power"
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
            quick_setfreq(AX5043Dev1, DCTTxFreq);
            break;
        }
        case LowerTxFreq:{
            int number = parseNumber(afterCommand);
            DCTTxFreq -= number;
            printf("TxFreq=%d\n",DCTTxFreq);
            quick_setfreq(AX5043Dev4, DCTTxFreq);
            break;
        }
        case RaiseRxFreq:{
            const AX5043Device rxList[]={RX1_DEVICE,RX2_DEVICE,RX3_DEVICE,RX4_DEVICE};
            uint16_t rxNumber = parseNumber(afterCommand);
            int number = parseNextNumber();
            if(rxNumber==0 || rxNumber > 4){
                printf("Must specify receiver number between 1 and 4--ex: 'raise rx freq 2 500'\n");
            }
            DCTRxFreq[rxNumber-1] += number;
            printf("RxFreq=%d\n",DCTRxFreq[rxNumber-1]);
            quick_setfreq(rxList[rxNumber-1], DCTRxFreq[rxNumber-1]);
            break;
        }
        case LowerRxFreq:{
            const AX5043Device rxList[]={RX1_DEVICE,RX2_DEVICE,RX3_DEVICE,RX4_DEVICE};
            uint16_t rxNumber = parseNumber(afterCommand);
            int number = parseNextNumber();
            if(rxNumber==0 || rxNumber > 4){
                printf("Must specify receiver number between 1 and 4--ex: 'raise rx freq 2 500'\n");
            }
            DCTRxFreq[rxNumber-1] -= number;
            printf("RxFreq=%d\n",DCTRxFreq[rxNumber-1]);
            quick_setfreq(rxList[rxNumber-1], DCTRxFreq[rxNumber-1]);
            break;
        }
        case ReadFreqs:{
            int i;
            for(i=0;i<4;i++){
                printf("Rx%d--MRAM: %d Memory: %d\n",i+1,ReadMRAMReceiveFreq(i),DCTRxFreq[i]);
            }
            printf("Tx Frequency--MRAM: %d, Memory: %d\n",ReadMRAMTelemFreq(),DCTTxFreq);
            break;
        }
        case SaveFreq:{
            uint8_t i;
            printf("Saving Rx frequency %d and Tx frequency %d to MRAM\n",DCTRxFreq,DCTTxFreq);
            WriteMRAMTelemFreq(DCTTxFreq);
            for(i=0;i<4;i++){
                WriteMRAMReceiveFreq(i,DCTRxFreq[i]);
            }
            break;
        }
        case initSaved:{
            initMRAM(true); //Init this thing from scratch (address size, data size, partitions etc)
            IHUInitSaved(); //Init stuff that we won't want to change on reboot
            SetupMRAM();    //Init stuff that do change (epoch number etc)
            break;
        }
       case LoadKey:{
            uint8_t key[AUTH_KEY_SIZE],i;
            uint32_t magic = ENCRYPTION_KEY_MAGIC_VALUE,checksum;
            const MRAMmap_t *LocalFlash = 0;
            char *str = afterCommand;
            bool stat;
            for(i=0;i<sizeof(key);i++){
                char *next = strtok(str," ,\n");
                str = NULL;
                if (next != NULL){
                    key[i] = (uint8_t)strtol(next,0,16);
                    printf("%d=0x%x ",i,key[i]);
                } else {
                    printf("Not enough numbers...");
                    break;
                }
            }
            checksum = key_checksum(key);
            printf("\n");
            if(i==sizeof(key)){
                printf("Writing key...");
                stat = writeNV(key,sizeof(LocalFlash->AuthenticateKey.key),NVConfigData,(int)&LocalFlash->AuthenticateKey.key);
            } else {
                stat = false;
            }
            if(stat){
                printf("Writing checksum=%x...",checksum);
                stat = writeNV(&checksum,sizeof(LocalFlash->AuthenticateKey.keyChecksum),NVConfigData,
                              (int)&LocalFlash->AuthenticateKey.keyChecksum);
            }
            if(stat){
                printf("Writing valid\n");
            } else {
                magic = 0;
                printf("Invalidating stored key\n");
            }
            stat = writeNV(&magic,sizeof(LocalFlash->AuthenticateKey.magic),NVConfigData,(int)&LocalFlash->AuthenticateKey.magic);
            break;
        }

        case MountFS:
        {
            if (red_mount("/") == -1) {
                printf("Unable to mount filesystem: %s\n",
                       red_strerror(red_errno));
            } else {
                printf("Filesystem mounted\n");
            }
            break;
        }

        case UnMountFS:
        {
            if (red_umount("/") == -1) {
                printf("Unable to unmount filesystem: %s\n",
                       red_strerror(red_errno));
            } else {
                printf("Filesystem unmounted\n");
            }
            break;
        }

        case FormatFS:
        {
            if (red_format("/") == -1) {
                printf("Unable to format filesystem: %s\n",
                       red_strerror(red_errno));
            } else {
                printf("Filesystem formatted\n");
            }
            break;
        }

        case LsFS:
        {
            int numSpace=0;
            char *srchStrng;
            while(afterCommand[numSpace] == ' ') numSpace++;
            srchStrng = &afterCommand[numSpace];
            REDDIR *pDir;
            printf("%s:\n",srchStrng);
            printf("Name       Blks  Size \n");
            printf("---------- ----- --------\n");
            if(strlen(srchStrng)== 0){
                pDir = red_opendir("//");
            } else {
                pDir = red_opendir(srchStrng);
            }
            if (pDir == NULL) {
                printf("Unable to open dir: %s\n", red_strerror(red_errno));
                printf("Make sure the path is fully qualified and starts with //\n");
                break;
            }
            uint32_t total_bytes = 0;
            uint32_t total_blocks = 0;
            REDDIRENT *pDirEnt;
            red_errno = 0; /* Set error to zero so we can distinguish between a real error and the end of the DIR */
            pDirEnt = red_readdir(pDir);
            while (pDirEnt != NULL) {
                printf("%10s %5d %8d\n", pDirEnt->d_name, pDirEnt->d_stat.st_blocks, (int)pDirEnt->d_stat.st_size );
                total_bytes += pDirEnt->d_stat.st_size;
                total_blocks += pDirEnt->d_stat.st_blocks;
                pDirEnt = red_readdir(pDir);
            }
            if (red_errno != 0) {
                printf("Error reading directory: %s\n", red_strerror(red_errno));

            }
            int32_t rc = red_closedir(pDir);
            if (rc != 0) {
                printf("Unable to close dir: %s\n", red_strerror(red_errno));
            }
            printf("Total File Blocks: %d Bytes: %d\n",total_blocks, total_bytes);

            // Now check and print the available disk space
            REDSTATFS redstatfs;
            rc = red_statvfs("/", &redstatfs);
            if (rc != 0) {
                printf("Unable to check disk space with statvfs: %s\n", red_strerror(red_errno));
            } else {
                printf("Free blocks: %d of %d.  Free Bytes: %d\n",redstatfs.f_bfree, redstatfs.f_blocks, redstatfs.f_frsize * redstatfs.f_bfree);
                printf("Available File Ids: %d of %d.  \n",redstatfs.f_ffree, redstatfs.f_files);
            }
            break;
        }
        case RmFS:
        {
            int numSpace=0;
            char *srchStrng;
            while(afterCommand[numSpace] == ' ') numSpace++;
            srchStrng = &afterCommand[numSpace];
            if(strlen(srchStrng)== 0){
                printf("Usage: rm <file name with path>\n");
                break;
            }
            int32_t fp = red_unlink(srchStrng);
            if (fp == -1) {
                printf("Unable to remove file %s : %s\n", srchStrng, red_strerror(red_errno));
            }

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
            printf("\nPayload Sizes: Header=%d,RTHealth=%d (common=%d,common2=%d,specific=%d),\nmin=%d,max=%d,WODHealth=%d\n",
                   sizeof(header_t),sizeof(realTimePayload_t),
                   sizeof(commonRtMinmaxWodPayload_t),sizeof(commonRtWodPayload_t),sizeof(realtimeSpecific_t),
                   sizeof(minValuesPayload_t),sizeof(maxValuesPayload_t),
                   sizeof(WODHousekeepingPayload_t));

            printf("Frame sizes: \n"
                    "      Payload Only,   Current Filler   Current Size\n");

            printf("RT1        %03d           \n",
                    //"RT2        %03d          \n",
                    //sizeof(realTimeFrame_t),
                    sizeof(realTimeFrame_t));
//                    sizeof(realTimeMinFrame_t)-memberSize(realTimeMinFrame_t,filler),memberSize(realTimeMinFrame_t,filler),sizeof(realTimeMinFrame_t),
//                    sizeof(realTimeMaxFrame_t)-memberSize(realTimeMaxFrame_t,filler),memberSize(realTimeMaxFrame_t,filler),sizeof(realTimeMaxFrame_t));

            printf("AllWOD1    %03d       \n",
                    sizeof(allWOD1Frame_t)
//                    sizeof(allWOD1Frame_t)-memberSize(allWOD1Frame_t,filler),memberSize(allWOD1Frame_t,filler),sizeof(allWOD1Frame_t),
//                    sizeof(allWOD2Frame_t)-memberSize(allWOD2Frame_t,filler),memberSize(allWOD2Frame_t,filler),sizeof(allWOD2Frame_t),
//                    sizeof(allWOD3Frame_t)-memberSize(allWOD3Frame_t,filler),memberSize(allWOD3Frame_t,filler),sizeof(allWOD3Frame_t)
            );
            printf("SafeData1  %03d       \n"
                    "SafeWOD    %03d      \n"
                    ,sizeof(safeData1Frame_t)
                    ,sizeof(safeWODFrame_t)
//                    ,sizeof(safeData1Frame_t)-memberSize(safeData1Frame_t,filler),memberSize(safeData1Frame_t,filler),sizeof(safeData1Frame_t)
//                    ,sizeof(safeWODFrame_t)-memberSize(safeWODFrame_t,filler),memberSize(safeWODFrame_t,filler),sizeof(safeWODFrame_t)
            );

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
                printf("MRAM%d: status %x",i,readMRAMStatus(i));
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
                writeMRAMStatus(i,stat);
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

            printf("I2c device state: (1 is ok)\n"
                    "   PacSat CPU Temp: %d, Tx Temp: %d    RTC: %d\n",
                    CpuTempIsOk(),TxTempIsOk(),RTCIsOk());
            break;
        }
        case telem0:{
            DisplayTelemetry(0);
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
            ax5043StartRx(AX5043Dev0, ANT_DIFFERENTIAL);
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
            printf("Unix time in secs: %d\n",getUnixTime());

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
            if(Get8BitTemp31725(CpuTemp,&temp8)){
                printf("Cpu temp: ");
                print8BitTemp(temp8);
            } else {
                printf("CPU Temp request failed\n");
            }
            if(Get8BitTemp31725(TxTemp,&temp8)){
                printf("  Transmitter temp: ");
                print8BitTemp(temp8);
                printf("\n");
            } else {
                printf("\nTransmitter temp request failed\n");
            }
            break;
        }
        case testAX5043:{
            int i;
            for(i=0;i<=5;i++){
                ax5043WriteReg((AX5043Device)i,AX5043_SCRATCH,i);

                printf("AX5043 #%d: Revision=0x%x, scratch=%d\n",i,
                       ax5043ReadReg((AX5043Device)i,AX5043_SILICONREVISION),
                       ax5043ReadReg((AX5043Device)i,AX5043_SCRATCH));
            }
            break;
        }
        case getax5043:{
            uint8_t devb = parseNumber(afterCommand);
            if(devb >= InvalidAX5043Device){
                printf("Give a device number between 0 and 4\n");
                break;
            }
            AX5043Device dev = (AX5043Device)devb;
            printf("AX5043 dev %d\n",dev);
            printf(" FIFOSTAT: %02x\n", ax5043ReadReg(dev, AX5043_FIFOSTAT));
            printf(" PWRMODE:: %02x\n", ax5043ReadReg(dev, AX5043_PWRMODE));
            printf(" XTALCAP: %d\n", ax5043ReadReg(dev, AX5043_XTALCAP));
            printf(" PLLLOOP: %02.2x\n", ax5043ReadReg(dev, AX5043_PLLLOOP));
            printf(" PLLCPI: %02.2x\n", ax5043ReadReg(dev, AX5043_PLLCPI));
            printf(" PLLVCOI: %02.2x\n", ax5043ReadReg(dev, AX5043_PLLVCOI));
            printf(" PLLRANGINGA: %02.2x\n", ax5043ReadReg(dev, AX5043_PLLRANGINGA));
            printf(" PLLVCODIV: %02.2x\n", ax5043ReadReg(dev, AX5043_PLLVCODIV));
//            printf(" FREQ: %x", ax5043ReadReg(dev, AX5043_FREQA0));
//            printf(" %x", ax5043ReadReg(dev, AX5043_FREQA1));
//            printf(" %x", ax5043ReadReg(dev, AX5043_FREQA2));
//            printf(" %x\n", ax5043ReadReg(dev, AX5043_FREQA3));
            uint32_t val = ax5043ReadReg(dev, AX5043_FREQA0)
        + (ax5043ReadReg(dev, AX5043_FREQA1) << 8)
        + (ax5043ReadReg(dev, AX5043_FREQA2) << 16)
        + (ax5043ReadReg(dev, AX5043_FREQA3) << 24);
            uint32_t freq = axradio_conv_freq_tohz(val);
            printf(" FREQ %d Hz\n", freq);
            printf(" MODULATION: %x\n", ax5043ReadReg(dev, AX5043_MODULATION));
            printf(" TXPWRCOEFFB0: %x\n", ax5043ReadReg(dev, AX5043_TXPWRCOEFFB0));
            printf(" TXPWRCOEFFB1: %x\n", ax5043ReadReg(dev, AX5043_TXPWRCOEFFB1));

            // Tx on VHF (typically)
            dev = TX_DEVICE;
            printf("\nAX5043 dev %d Tx:\n",dev);
            printf(" FIFOSTAT: %02x\n", ax5043ReadReg(dev, AX5043_FIFOSTAT));
            printf(" PWRMODE: %02x\n", ax5043ReadReg(dev, AX5043_PWRMODE));
            printf(" XTALCAP: %d\n", ax5043ReadReg(dev, AX5043_XTALCAP));
            printf(" PLLLOOP: %02.2x\n", ax5043ReadReg(dev, AX5043_PLLLOOP));
            printf(" PLLCPI: %02.2x\n", ax5043ReadReg(dev, AX5043_PLLCPI));
            printf(" PLLVCOI: %02.2x\n", ax5043ReadReg(dev, AX5043_PLLVCOI));
            printf(" PLLRANGINGA: %02.2x\n", ax5043ReadReg(dev, AX5043_PLLRANGINGA));
            printf(" PLLVCODIV: %02.2x\n", ax5043ReadReg(dev, AX5043_PLLVCODIV));
//            printf(" FREQ: %x", ax5043ReadReg(dev, AX5043_FREQA0));
//            printf(" %x", ax5043ReadReg(dev, AX5043_FREQA1));
//            printf(" %x", ax5043ReadReg(dev, AX5043_FREQA2));
//            printf(" %x\n", ax5043ReadReg(dev, AX5043_FREQA3));
            val = ax5043ReadReg(dev, AX5043_FREQA0)
        + (ax5043ReadReg(dev, AX5043_FREQA1) << 8)
        + (ax5043ReadReg(dev, AX5043_FREQA2) << 16)
        + (ax5043ReadReg(dev, AX5043_FREQA3) << 24);
            freq = axradio_conv_freq_tohz(val);
            printf(" FREQ %d Hz\n", freq);
            printf(" MODULATION: %x\n", ax5043ReadReg(dev, AX5043_MODULATION));
            printf(" TXPWRCOEFFB0: %x\n", ax5043ReadReg(dev, AX5043_TXPWRCOEFFB0));
            printf(" TXPWRCOEFFB1: %x\n", ax5043ReadReg(dev, AX5043_TXPWRCOEFFB1));

            break;
        }

        case getRSSI:{
            uint8_t devb = parseNumber(afterCommand);
            if(devb >= InvalidAX5043Device){
                printf("Give a device number between 0 and 4\n");
                break;
            }
            AX5043Device dev = (AX5043Device)devb;
            int rssi = get_rssi(dev);
            printf("AX5043 Dev: %d RSSI is %02x = %d dBm\n",dev,rssi,((int16_t)rssi) - 255);
            break;
        }
        case testRxFreq: {
             //This is so we can find what the receive frequency is on first build
            {
                uint8_t devb = parseNumber(afterCommand);
                if(devb >= InvalidAX5043Device){
                    printf("Give a device number between 0 and 4\n");
                    break;
                }
                AX5043Device device = (AX5043Device)devb;
                uint32_t freq = 145835000;

               if (device == AX5043Dev2) {
                   printf("Testing TX for AX5043 Dev: %d with single ended antenna\n",device);
                   test_rx_freq(device, freq, ANT_SINGLE_ENDED); // only AX5043Dev2 is single ended
               } else {
                   printf("Testing TX for AX5043 Dev: %d with differential antenna\n",device);
                   test_rx_freq(device, freq, ANT_DIFFERENTIAL);
               }
            }
            break;
        }

        case testPLLrange:{
            AX5043Device dev = AX5043Dev1;
            printf("Testing the PLL range for device: %d\n",dev);

            test_pll_2m_range(dev, RATE_9600); // test the range of the receiver on 2m
            break;
        }

#ifdef DEBUG
        /* G0KLA TEST ROUTINES
         * This is a subset of the self tests.  They do not cause a transmission
         * It does create a test file in the directory that needs to be removed manually
         */
        case testPacsat:{

            if(! pb_test_callsigns()) {  debug_print("### Callsign TEST FAILED\n"); break; }
            if(!test_ax25_util_decode_packet()) {  debug_print("### Packet Decode TEST FAILED\n"); break; }
            if(! pb_test_list()) {  debug_print("### pb list TEST FAILED.  ** was PB Enabled?? Use 'open pb' to enable it ** \n"); break; }
            if(! pb_clear_list()) {  debug_print("### pb list clear TEST FAILED\n"); break; }
            if(! tx_test_make_packet()) {  debug_print("### tx make packet TEST FAILED\n"); break; }
            if(! test_pfh()) {  debug_print("### pfh TEST FAILED\n"); break; }
            if(! test_pfh_file()) {  debug_print("### pfh TEST FILE FAILED\n"); break; }

            if(! test_ftl0_upload_table()) {  debug_print("### FTL0 Upload Table TEST FAILED\n"); break; }


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
            pb_send_status();
            break;
        }
        case testPbList:{
            bool rc = pb_test_list();
            if (rc != TRUE) debug_print (".. was PB Enabled?? Use 'open pb' to enable it \n");
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
        case testPfhFile:{
            bool rc = test_pfh_file();
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
        case testDecode:{
            bool rc = test_ax25_util_decode_packet();
            break;
        }
        case sendUplinkStatus:{
            ax25_send_status();
            break;
        }
        case testRetransmission:{
            bool rc = test_ax25_retransmission();
            break;
        }
        case testUploadTable:{
            bool rc = test_ftl0_upload_table();
            break;
        }
        case listUploadTable:{
            bool rc = ftl0_debug_list_upload_table();
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
            WriteMRAMBoolState(StatePbEnabled,false);
            printf("PB SHUT\n");
            break;
        }
        case pbOpen:{
            WriteMRAMBoolState(StatePbEnabled,true);
            printf("PB OPEN\n");
            break;
        }
        case uplinkShut:{
            WriteMRAMBoolState(StateUplinkEnabled,false);
            printf("UPLINK SHUT\n");
            break;
        }
        case uplinkOpen:{
            WriteMRAMBoolState(StateUplinkEnabled,true);
            printf("UPLINK OPEN\n");
            break;
        }
        case digiShut:{
            WriteMRAMBoolState(StateDigiEnabled,false);
            printf("Digipeater Disabled\n");
            break;
        }
        case digiOpen:{
            WriteMRAMBoolState(StateDigiEnabled,true);
            printf("Digipeater Enabled\n");
            break;
        }
        case mkdirFS: {
            int numSpace=0;
            char *srchStrng;
            while(afterCommand[numSpace] == ' ') numSpace++;
            srchStrng = &afterCommand[numSpace];
            if(strlen(srchStrng)== 0){
                printf("Usage: mkdir <file name with path>\n");
                break;
            }
            int32_t rc = red_mkdir(srchStrng);
            if (rc == -1) {
                printf("Unable make dir %s: %s\n", srchStrng, red_strerror(red_errno));
            }
            break;
        }

        case rmdirFS: {
            int numSpace=0;
            char *srchStrng;
            while(afterCommand[numSpace] == ' ') numSpace++;
            srchStrng = &afterCommand[numSpace];
            if(strlen(srchStrng)== 0){
                printf("Usage: rmdir <file name with path>\n");
                break;
            }
            int32_t rc = red_rmdir(srchStrng);
            if (rc == -1) {
                if (red_errno == RED_EBUSY)
                    printf("Unable remove dir that contains other files or directories: %s\n", srchStrng, red_strerror(red_errno));
                else
                    printf("Unable remove dir %s: %s\n", srchStrng, red_strerror(red_errno));
            }
            break;
        }


        case mramHxd: {
            int numSpace=0;
            char *srchStrng;
            while(afterCommand[numSpace] == ' ') numSpace++;
            srchStrng = &afterCommand[numSpace];
            if(strlen(srchStrng)== 0){
                printf("Usage: hxd <file name with path>\n");
                break;
            }
            char read_buffer[512];
            int32_t fp = red_open(srchStrng, RED_O_RDONLY);
            if (fp != -1) {
                int32_t numOfBytesRead = red_read(fp, read_buffer, sizeof(read_buffer));
                printf("Read returned: %d\n",numOfBytesRead);
                if (numOfBytesRead == -1) {
                    printf("Unable to read file: %s\n", red_strerror(red_errno));
                } else {
                    int q;
                    for (q=0; q< numOfBytesRead; q++) {
                        printf("%02x ", read_buffer[q]);
                        if (q != 0 && q % 20 == 0 ) printf("\n");
                    }
                }

                int32_t rc = red_close(fp);
                if (rc != 0) {
                    printf("Unable to close file: %s\n", red_strerror(red_errno));
                }
            } else {
                printf("Unable to open %s for reading: %s\n", srchStrng, red_strerror(red_errno));
            }
            printf("\n");


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
            dir_debug_print(NULL); // pass NULL to print from the head of the list
            break;
        }
        case getNextFileNumber:{
            uint32_t next_file_id = dir_next_file_number();
            printf("Next File Number from the Dir will be: %04x\n",next_file_id);
            break;
        }
        case resetNextFileNumber:{
            WriteMRAMHighestFileNumber(0);
            printf("Next file number reset to zero\n");
            break;
        }

        case heapFree:{
            printf("Free heap size: %d\n",xPortGetFreeHeapSize());
            break;
        }

        case getUnxTime:{
            printf("Unix time in secs: %d\n",getUnixTime());
            break;
        }

        case setUnxTime:
        case setRtc: {
            static char *nextNum;
            uint32_t t = (uint32_t)strtol(afterCommand,&nextNum,0);
            printf("Setting unix time to: %d\n",t);
            setUnixTime(t);
            //printf("Setting RTC\n");
            if (RTCIsOk()) {
                bool set = SetRtcTime31331(&t);
                if (set) {
                    printf("Setting RTC\n");
                } else {
                    printf("Failed to set RTC\n");
                }
            }
            break;
        }

        case setRate1200:{
            printf("Setting Radio to 1200bps.\n");
            WriteMRAMBoolState(StateAx25Rate9600, RATE_1200);

            ax5043StopRx(AX5043Dev0);
            ax5043StopRx(AX5043Dev1);
            ax5043StopRx(AX5043Dev2);
            ax5043StopRx(AX5043Dev3);
            ax5043StopTx(AX5043Dev4);

            ax5043StartRx(AX5043Dev0, ANT_DIFFERENTIAL);
            ax5043StartRx(AX5043Dev1, ANT_DIFFERENTIAL);
            ax5043StartRx(AX5043Dev2, ANT_SINGLE_ENDED);
            ax5043StartRx(AX5043Dev3, ANT_DIFFERENTIAL);
            ax5043StartTx(AX5043Dev4, ANT_DIFFERENTIAL);
            break;
        }

        case setRate9600:{
            printf("Setting radio to 9600bps.\n");
            WriteMRAMBoolState(StateAx25Rate9600, RATE_9600);
            ax5043StopRx(AX5043Dev0);
            ax5043StopRx(AX5043Dev1);
            ax5043StopRx(AX5043Dev2);
            ax5043StopRx(AX5043Dev3);
            ax5043StopTx(AX5043Dev4);

            ax5043StartRx(AX5043Dev0, ANT_DIFFERENTIAL);
            ax5043StartRx(AX5043Dev1, ANT_DIFFERENTIAL);
            ax5043StartRx(AX5043Dev2, ANT_SINGLE_ENDED);
            ax5043StartRx(AX5043Dev3, ANT_DIFFERENTIAL);
            ax5043StartTx(AX5043Dev4, ANT_DIFFERENTIAL);
            break;
        }

        case getRtc:{
            bool rc = false;

            uint32_t time;
            rc = GetRtcTime31331(&time);
            if (rc == FALSE)
                 printf(" Error, time unavailable\n");
             else {
                 printf("Time: %d vs IHU Unix time %d\n",time, getUnixTime());
             }
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
