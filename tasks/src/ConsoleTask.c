/*
 * ConsoleTask.c
 *
 *  Created on: Mar 4, 2019
 *      Author: burns
 */
/*
 * Standard headers for Golf
 */
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
#include "ax5043-70cm-PSK.h"
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
#include "telemetryCollectionInterface.h"
#include "gpioDriver.h"
#include "DownlinkControl.h"
#include "LinearInterp.h"
#include "Buzzer.h"
#include "keyfile.h"

//Extern definition
extern uint8_t SWCmdRing[SW_CMD_RING_SIZE],SWCmdIndex;


static char commandString[COM_STRING_SIZE]; /* Must be long enough for worst case: Uploadtest20xxxxxx  which is 86 */
static uint32_t DCTTxFreq,DCTRxFreq;


extern bool InSafeMode,InScienceMode,InHealthMode;
extern bool TransponderEnabled,onOrbit,SimDoppler;
extern resetMemory_t SaveAcrossReset;
extern uint16_t WODFrequency,NumWODSaved;
extern int32_t WODHkStoreIndex,WODSciStoreIndex,WODSciFetchIndex,WODHkFetchIndex;
extern int32_t WODRagStoreIndex,WODRagFetchIndex;
extern char *ErrMsg[];

bool TestPlayDead=false;

static const uint16_t one=1,zero=0;

enum {
    nada=0
    ,getTemp
    ,getDeploy
    ,com1test
    ,com2test
    ,reset
    ,resetBoth
    ,resetOther
    ,resetAll
    ,resetLIHU
    ,reason4reset
    ,startWD
    ,loop
    ,preflight
    ,enTransp
    ,disTransp
    ,time
    ,showOtherCoord
    ,takeControl
    ,dropControl
    ,version
    ,endianTest
    ,clrMinMax
    ,getI2cState
    ,internalWDTimeout
    ,externalWDTimeout
    ,telem0
    ,telem1
    ,telem2
    ,telem3
    ,telem5
    ,telemDiag
    ,setTimeouts
    ,pollI2c
    ,ShowWODRange
    ,ShowWODContents
    ,changeWODSaved
    ,readMRAMsr
    ,writeMRAMsr
    ,swapBus
    ,dropBus
    ,getMode
    ,healthMode
    ,safeMode
    ,scienceMode
    ,inhibitTx
    ,eclipseAction
    ,getState
    ,toneTx
    ,noToneTx
    ,testLED
    ,testAlert
    ,fastWOD
    ,defaultWOD
    ,standardWOD
    ,restartCAN
    ,resetWODStore
    ,turnOnVUC
    ,initMRAM
    ,DeployUHF1
    ,DeployUHF2
    ,DeployVHF
    ,DeployMultiband
    ,DeployPanelL
    ,DeployPanelR
    ,DeployDeorbit
    ,DeployAll
    ,DeployInitial
    ,SetInOrbit
    ,ClearInOrbit
    ,TestBuzzer
    ,enbCanPrint
    ,dsbCanPrint
    ,EnableComPr
    ,DisableComPr
    ,EnableTimePr
    ,DisableTimePr
    ,EnableEclipsePr
    ,DisableEclipsePr
    ,corModeOn
    ,corModeOff
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
    ,GRadiationTlm
    ,DisablePA
    ,EnablePA
    ,StartADC1
    ,StartADC2
    ,ReadADC1
    ,ReadADC2
    ,TestOneWire
    ,TestMemScrub
    ,SetFakeEclipse
    ,GetCommands
    ,GetGpios
    ,getax5043
    ,getRSSI
    ,simDoppler
    ,stopDoppler
    ,deployBusCAN
    ,deployBusI2c
    ,deployBusAgree
    ,deployBusEither
    ,testRxFreq
    ,testPLLrange
    ,testEttusCAN
    ,testShutdownEttusCAN
    ,testFakeDead
    ,HelpAll
    ,HelpDevo
    ,HelpSetup
    ,Help
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
                                ,{"test com1","Output to com1",com1test}
                                ,{"test com2","Output to com2",com2test}
                                ,{"test internal wd","Force internal watchdog to reset CPU",internalWDTimeout}
                                ,{"test external wd","Force external watchdog to reset CPU",externalWDTimeout}
                                ,{"test set timeouts","Set orbit time timeouts",setTimeouts}
                                ,{"test leds","Flash the LEDs in order",testLED}
                                ,{"test alert","Try beeping at different rates",testAlert}
                                ,{"test buzzer","Tweaks the alert for 2 seconds at specified pitch",TestBuzzer}
                                ,{"test adc1","Read group 1 of ADC",StartADC1}
                                ,{"test adc2","Read group 2 of ADC",StartADC2}
                                ,{"test onewire","Check for a presence response to a reset",TestOneWire}
};
commandPairs debugCommands[] = {
                                 {"test scrub","Run the memory scrub routine once",TestMemScrub}
                                ,{"test eclipse","0 for sun, 1 for eclipse",SetFakeEclipse}
                                ,{"test pll", "test ax5043 PLL frequency range",testPLLrange}
                                ,{"test ettus can","Send a CAN message to the Ettus",testEttusCAN}
                                ,{"test ettus shutdown","Shutdown the Ettus via CAN",testShutdownEttusCAN}
                                ,{"test endian","Output a structure in hex",endianTest}
                                ,{"test play dead","Make this IHU play dead (no CAN)",testFakeDead}
                                ,{"simulate doppler","Slowly change the telem transmit frequency",simDoppler}
                                ,{"stop doppler","Start fixed frequency again",stopDoppler}
                                ,{"init mram","Initializes MRAM state,WOD,Min/max but does not clear InOrbit--testing only",initMRAM}
                                ,{"load key","Load an authorization key for uplink commands",LoadKey}
                                ,{"reset ihu","Reset this processor",reset}
                                ,{"reset both","Reset both primary and secondary processor",resetBoth}
                                ,{"reset other","Reset the other processor from the one we are on",resetOther}
                                ,{"reset all","Reset all processors including LIHU",resetAll}
                                ,{"reset lihu","Reset only the LIHU",resetLIHU}
                                ,{"reset can","Reset the CAN devices",restartCAN}
                                ,{"set cor mode","Disable RF Control for autonomous RxTx",corModeOn}
                                ,{"clear cor mode","Enable RF Control for controlled RxTx",corModeOff}
                                ,{"poll i2c","Poll to see which I2c devices are there",pollI2c}
                                ,{"inhibit tx","Simulate FCC command to turn off tx",inhibitTx}
                                ,{"get mram sr","Get the MRAM status register",readMRAMsr}
                                ,{"get downlink size","Debug-get sizes of downlink payloads and frames",showDownlinkSize}
                                ,{"get temp","Get RT-IHU board temperature",getTemp}
                                ,{"set bus can","Set the bus used for the deploy command",deployBusCAN}
                                ,{"set bus i2c","Set the bus used for the deploy command",deployBusI2c}
                                ,{"set bus agree","I2c and CAN must agree for the deploy command",deployBusAgree}
                                ,{"set bus either","Use either I2c or CAN for the deploy command",deployBusEither}
};
commandPairs commonCommands[] = {
                                 {"get deploy","Get i2c deploy status",getDeploy}
                                 ,{"get reset","Why did the processor reset?",reason4reset}
                                 ,{"get i2c","What I2c devices are working?",getI2cState}
                                 ,{"get time","Display reset number and seconds",time}
                                 ,{"get lihu state","Get the LIHU's current IHU Coordination State",showOtherCoord}
                                 ,{"get status","Get some general status info",telem0}
                                 ,{"get rssi","Get the current RSSI reading from the AX5043 Rx",getRSSI}
                                 ,{"get realtime telem","Display a health payload",telem1}
                                 ,{"get max telem","Display a max payload",telem2}
                                 ,{"get min telem","Display a min payload",telem3}
                                 ,{"get rad telem","Display a Vanderbilt experiment payload",GRadiationTlm}
                                 ,{"get wod range","Get the indices for WOD",ShowWODRange}
                                 ,{"get wod contents","Get the full WOD contents",ShowWODContents}
                                 ,{"get wod telemetry","Get a WOD payload",telem5}
                                 ,{"get diag telemetry","Get a diagnostic payload",telemDiag}
                                 ,{"get mode","Get the current mode",getMode}
                                 ,{"get state","Mainly for debug: Get the current state of the downlink state machine",getState}
                                 ,{"get version","Get the software version number and build time",version}
                                 ,{"get rf power","Print the RF power settings",GetRfPower}
                                 ,{"get commands","Get a list the last 4 s/w commands",GetCommands}
                                 ,{"get gpios","Display the values of all GPIOS",GetGpios}
                                 ,{"get ax","Get ax5043 status",getax5043}
                                 ,{"health mode","Set health mode",healthMode}
                                 ,{"safe mode","Set safe mode",safeMode}
                                 ,{"science mode","Set science mode",scienceMode}
                                 ,{"set eclipse action","Set the action taken entring an eclipse",eclipseAction}
                                 ,{"set dct drive power","Set drive power for high and low power",SetDCTDrivePower}
                                 ,{"select dct power","Set rf power used in safe or normal modes",SelectRFPowerLevels}
                                 ,{"set mram sr","Set the MRAM status register",writeMRAMsr}
                                 ,{"set wod saved","Set the number of WOD records to be saved",changeWODSaved}
                                 ,{"set wod fast","Set the WOD collection frequency fast for debugging",fastWOD}
                                 ,{"set wod normal","Set the WOD collection frequency to standard",standardWOD}
                                 ,{"set pa on","Turn on the RF power amp",EnablePA}
                                 ,{"set pa off","Turn off the RF power amp",DisablePA}
                                 ,{"set inorbit flag","Use after preflight init to avoid delay",SetInOrbit}
                                 ,{"start watchdog","Start the watchdog",startWD}
                                 ,{"loop","Loop without resetting watchdog",loop}
                                 ,{"preflight init","Initialize MRAM etc for flight",preflight}
                                 ,{"clear minmax","Clear the min, max, and CIU counts",clrMinMax}
                                 ,{"init wod store","Clear all the WOD info",resetWODStore}
                                 ,{"take control","Take control from the LIHU",takeControl}
                                 ,{"drop control","Drop control to let the LIHU take over",dropControl}
                                 ,{"swap bus","Simulate a commmand to swap the bus to the opposite processor",swapBus}
                                 ,{"drop bus","Assert a fail briefly so the bus switches drop out",dropBus}
                                 ,{"ignore umbilical","Allow radios to go on with umbilical connected",ignoreUmb}
                                 ,{"heed umbilical","Reverse effects of ignore umbilical",noticeUmb}
                                 ,{"enable canprint","Debug can--number follows",enbCanPrint}
                                 ,{"disable canprint","Debug can--number follows",dsbCanPrint}
                                 ,{"enable comprint","Show uplink commands",EnableComPr}
                                 ,{"disable comprint","Turn off uplink command print",DisableComPr}
                                 ,{"enable timeprint","Turn on print of time updates 1=infrequent, 2=frequent",EnableTimePr}
                                 ,{"disable timeprint","Turn off print of time updates 1=infrequent, 2=frequent",DisableTimePr}
                                 ,{"enable eclipseprint","Turn on print of action on entering eclipse",EnableEclipsePr}
                                 ,{"disable eclipseprint","Turn on print of action on entering eclipse",DisableEclipsePr}
                                 ,{"enable vuc","make sure the experiment can turn on",turnOnVUC}
                                 ,{"enable transponder","Let transponder turn on when not in safe mode",enTransp}
                                 ,{"disable transponder","Prevent turning on when not in safe mode",disTransp}
                                 ,{"deploy uhf1","Deploy UFH1 Tx Antenna",DeployUHF1}
                                 ,{"deploy uhf2","Deploy UHF antenna",DeployUHF2}
                                 ,{"deploy vhf","",DeployVHF}
                                 ,{"deploy multi","",DeployMultiband}
                                 ,{"deploy panel left" ,"", DeployPanelL}
                                 ,{"deploy panel right" ,"", DeployPanelR}
                                 ,{"deploy deorbit" ,"", DeployDeorbit}
                                 ,{"deploy initial","Only the antennas that deploy autonomously",DeployInitial}
                                 ,{"deploy all","Does not do deorbit--can add 'override'",DeployAll}
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
        case testFakeDead:{
            int test = parseNumber(afterCommand);
            if(test == 0){
                printf("1 = play dead; 2 = come to life\n");
            }
            if(test==1){
                TestPlayDead = true;
                SWIDoBusSwitch();
            }
            else
                TestPlayDead = false;
        }
        case simDoppler:{
            SimDoppler = true;
            WriteMRAMTelemFreq(DCT_DEFAULT_TX_FREQ+10000);
            break;
        }
        case stopDoppler:{
            SimDoppler=false;
            WriteMRAMTelemFreq(DCT_DEFAULT_TX_FREQ);
        }
        case GetGpios:{
            int i;
            char *gpioNames[NumberOfGPIOs]={
               "LED1","LED2","LED3","DCTPower","DCTFlag","DCTInterrupt","PAPower","PAFlag","Experiment1Enable",
               "IHUCoordR0","IHUCoordL0",
               "IHUCoordR1","IHUCoordL1","CommandStrobe","CommandBits","WhoAmI","I2c1Reset","I2c2Reset",
               "BusDisabled","PBEnable","UmbilicalAttached","IhuRfControl","WatchdogFeed","Alert","SSPA10GHz","OneWire",
               "RfPower","EttusSwitch","ChargerAttached"
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
        case endianTest:{
#if 1 /* For testing diag downlink*/
            infrequentDownlink_t test;
            uint32_t *test32 = (uint32_t *)&test;
            uint8_t *test8 = (uint8_t *)&test;
            test32[1] = 0xffffffff;
            test.UTCHours = 0x16;
            test.UTCMinutes = 0x3f;
            test.UTCSeconds = 0x3f;
            test.UTCDay = 0x0;
            test.UTCMonth = 0;
            test.UTCYear = 0x0;
            test.UTCValid = 1;
            printf("32 bit version of struct is %08x bytes low to high are %x %x %x %x\n",
                   test32[2],test8[8],test8[9],test8[10],test8[11]);
#else
            struct xxx {
                unsigned int a0:1;
                unsigned int a1:1;
                unsigned int a2:1;
                unsigned int a3:1;
                unsigned int a4:1;
                unsigned int a5:1;
                unsigned int a6:2;
            } test;
            uint8_t *testbyte = (uint8_t *)&test;
            test.a0=1;
            test.a1=0;
            test.a2=1;
            test.a3=0;
            test.a4=1;
            test.a5=0;
            test.a6 = 3;

            printf("Value is 0x%x\n",*testbyte);
#endif
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
        case EnablePA:{
            GPIOSetOn(PAPower);
            break;
        }
        case DisablePA:{
            GPIOSetOff(PAPower);
            break;
        }
        case RaiseTxFreq:{
            int number = parseNumber(afterCommand);
            DCTTxFreq += number;
            printf("TxFreq=%d\n",DCTTxFreq);
            quick_setfreq(DCTTxFreq);
            break;
        }
        case LowerTxFreq:{
            int number = parseNumber(afterCommand);
            DCTTxFreq -= number;
            printf("TxFreq=%d\n",DCTTxFreq);
            quick_setfreq(DCTTxFreq);
            break;
        }
        case RaiseRxFreq:{
             int number = parseNumber(afterCommand);
             DCTRxFreq += number;
             printf("RxFreq=%d\n",DCTRxFreq);
             quick_setfreq(DCTRxFreq);
             break;
         }
         case LowerRxFreq:{
             int number = parseNumber(afterCommand);
             DCTRxFreq -= number;
             printf("RxFreq=%d\n",DCTRxFreq);
             quick_setfreq(DCTRxFreq);
             break;
         }
        case SaveFreq:{
            printf("Saving Rx frequency %d and Tx frequency %d to MRAM\n",DCTRxFreq,DCTTxFreq);
            WriteMRAMTelemFreq(DCTTxFreq);
            WriteMRAMCommandFreq(DCTRxFreq);
            break;
        }
        case initSaved:
            IHUInitSaved();
            DCTTxFreq = ReadMRAMTelemFreq();
            DCTRxFreq = ReadMRAMCommandFreq();
            break;
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
            printf("Safe Rf Power Level is %s\n",GetSafeRfPowerLevel()?"HIGH":"LOW");
            printf("Normal Rf Power Level is %s\n",GetNormalRfPowerLevel()?"HIGH":"LOW");
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
        case eclipseAction:{
            uint16_t arg;
            arg = parseNumber(afterCommand);
            printf("Set eclipse action to %d\n",arg);
            SimulateSwCommand(SWCmdNSSpaceCraftOps,SWCmdOpsSetEclipseAction,&arg,1);
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


        case corModeOn:{
            GPIOSetOff(IhuRfControl);
            break;
        }
        case corModeOff:{
            GPIOSetOn(IhuRfControl);
            break;
        }
        case initMRAM:{
            MRAMInit();
            WriteMRAMBoolState(StateInOrbit,true); // Don't get confused by in orbit state!
            break;
        }
        case TestBuzzer:{
            int pitch;
            pitch = parseNumber(afterCommand);
            Buzzer(SECONDS(2),pitch);
            break;
        }
        case SetInOrbit:{
            WriteMRAMBoolState(StateInOrbit,true);
            break;
        }
        case ClearInOrbit:{
            WriteMRAMBoolState(StateInOrbit,false);
            break;
        }
        case ignoreUmb:
            OverrideUmbilical(true);
            break;
        case noticeUmb:
            OverrideUmbilical(false);
            break;
        case resetWODStore:
            /* Clear out all the WOD and reset the indices */
            WriteMRAMWODSciDownlinkIndex(0);
            WriteMRAMWODHkDownlinkIndex(0);
            WriteMRAMWODRagDownlinkIndex(0);
            WriteMRAMWODHkStoreIndex(0);
            WriteMRAMWODSciStoreIndex(0);
            WriteMRAMWODRagStoreIndex(0);
            WODSciStoreIndex = 0;
            WODRagStoreIndex = 0;
            WODHkStoreIndex = 0;
            MRAMInitWOD();
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
        case fastWOD:{
            WODFrequency=4;
            printf("Frequency set to collection rate/4\n");
            break;
        }
        case standardWOD:{
            WODFrequency = ReadMRAMWODFreq();
            printf("Frequency set to collection rate/%d",WODFrequency);
            break;
        }
        case testLED:{
            GPIOSetOn(LED1);
            GPIOSetOn(LED2);
            GPIOSetOn(LED3);
            vTaskDelay(SECONDS(2));
            GPIOSetOff(LED1);
            vTaskDelay(SECONDS(2));
            GPIOSetOn(LED1);
            GPIOSetOff(LED2);
            vTaskDelay(SECONDS(2));
            GPIOSetOn(LED2);
            GPIOSetOff(LED3);
            vTaskDelay(SECONDS(2));
            GPIOSetOn(LED3);
            break;
        }

        case getState:{
             char * stateNames[]=
                 {"None","Health","Safe","SafeBcon","Autosafe","AutoSfBcon","TransmitInhibit"};
            printf("Current downlink state is %s\n",stateNames[GetCurrentDownlinkState()]);
            break;
        }
        case enTransp:{
            SimulateSwCommand(SWCmdNSSpaceCraftOps,SWCmdOpsEnableTransponder,&one,1);
            break;
        }
        case disTransp:{
            SimulateSwCommand(SWCmdNSSpaceCraftOps,SWCmdOpsEnableTransponder,&zero,1);
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
        case EnableTimePr:{
            int timeType = parseNumber(afterCommand);
            EnableTimePrint(true,timeType);
            break;
        }
        case DisableTimePr:{
            int timeType = parseNumber(afterCommand);
            EnableTimePrint(false,timeType);
            break;
        }
        case getMode:{
            if(InSafeMode){
                char * autoStr;
                autoStr = InAutoSafeMode()?" (auto)":InEclipseSafeMode()?" (eclipse)":"";
                 printf("Satellite is in%s safe mode\n",autoStr);
             } else if (InScienceMode) {
                 printf("Satellite is in science mode\n");
             } else if (InHealthMode){
                 printf("Satellite is in health mode\n");
             } else {
                 printf("Satellite is in UNKNOWN MODE\n");
             }
            break;
        }
        case healthMode:{
            SimulateSwCommand(SWCmdNSSpaceCraftOps,SWCmdOpsHealthMode,NULL,0);
            break;
        }
        case safeMode:{
            SimulateSwCommand(SWCmdNSSpaceCraftOps,SWCmdOpsSafeMode,NULL,0);
            break;
        }
        case scienceMode:{
            uint16_t time = parseNumber(afterCommand);
            if(time==0)time=5;
            SimulateSwCommand(SWCmdNSSpaceCraftOps,SWCmdOpsScienceMode,&time,1);
            break;
        }
#define primaryProc 1
#define secondaryProc 2
#define swapProc 0
        case dropBus:{
            SWIDoBusSwitch();
            vTaskDelay(2); // Give the bus time to switch before doing anything else
            break;
        }

        case ShowWODRange:{
            DisplayWODRange(false);
            break;
        }
        case ShowWODContents:{
            DisplayWODRange(true); //Special type to show the range
            break;
        }
        case changeWODSaved:{
            uint16_t size = parseNumber(afterCommand);
            ChangeWODSaved(size);
            break;
        }
        case readMRAMsr:{
            printf("MRAM status is %x\n",ReadMRAMStatus());
            break;
        }
        case writeMRAMsr:{
            uint8_t stat = parseNumber(afterCommand);
            WriteMRAMStatus(stat);
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
        case telem1:{
            DisplayTelemetry(RT_HK_PAYLOAD);
            break;
        }
        case telem2:{
            DisplayTelemetry(MAX_VALS_PAYLOAD);
            break;
        }
        case telem3:{
            DisplayTelemetry(MIN_VALS_PAYLOAD);
            break;
        }
        case GRadiationTlm:{
            DisplayTelemetry(RT_RAD_PAYLOAD);
            break;
        }
        case telem5:{
            DisplayTelemetry(WOD_HK_PAYLOAD);
            break;
        }
        case telemDiag:{
            DisplayTelemetry(DIAGNOSTIC_PAYLOAD);
            break;
        }

        case clrMinMax:{
            ClearMinMax();
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
                printf("Turning off tone; telemetry enabled\n");
                AudioSetMixerSource(MixerSilence);
                DisableTelemetry(false);
                BeaconOff();
                break;
        case toneTx: {
                printf("Sending a tone\n");
                AudioSetMixerSource(MixerSilence);  //Stop everything
                BeaconOff();
                AudioSetMixerSource(MixerTone);
                DisableTelemetry(true);
                BeaconOn();
                break;
            }
        }
        case version:{
            printID();
            break;
        }
        case time:{
            logicalTime_t time;
            uint32_t timeouts[MaxNumberOfTimeouts];
            getTime(&time);
            printf("IHU time:  Resets=%i,seconds=%i\n",time.IHUresetCnt,time.METcount);
            getTimestamp(&time);
            printf("Timestamp time:  Epoch=%i,seconds=%i\n",time.IHUresetCnt,time.METcount);
            printf("Poweron Time since preflight: %d seconds\n",getSecondsInOrbit());
            GetTimeoutTimes(timeouts);
            printf("Command timeout=%d, sw timecheck timeout=%d, time to minmax clear=%d\n",
                   timeouts[NoCommandTimeout],timeouts[NoTimeCommandTimeout],timeouts[MinMaxResetTimeout]);

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
        case loop:{
            while(1){};
        }
        case startWD:{
            SWISetWDTimer();
            dwdReset();
            SWIStartWatchdog();
            break;
        }
        case resetAll:{
            uint16_t args[]={1,1,1};
            SimulateSwCommand(SWCmdNSSpaceCraftOps,SWCmdOpsResetSpecified,args,3);
            vTaskDelay(CENTISECONDS(50));
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
        case resetLIHU:{
            uint16_t args[]={1,0,0};
            SimulateSwCommand(SWCmdNSSpaceCraftOps,SWCmdOpsResetSpecified,args,3);
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
            printf("AX5043_FIFOSTAT: %02x\n", ax5043ReadReg(AX5043_FIFOSTAT));
            printf("AX5043_PWRMODE:: %02x\n", ax5043ReadReg(AX5043_PWRMODE));
            printf("AX5043_XTALCAP: %d\n", ax5043ReadReg(AX5043_XTALCAP));
            printf("AX5043_PLLLOOP: %02.2x\n", ax5043ReadReg(AX5043_PLLLOOP));
            printf("AX5043_PLLCPI: %02.2x\n", ax5043ReadReg(AX5043_PLLCPI));
            printf("AX5043_PLLVCOI: %02.2x\n", ax5043ReadReg(AX5043_PLLVCOI));
            printf("AX5043_PLLRANGINGA: %02.2x\n", ax5043ReadReg(AX5043_PLLRANGINGA));
            printf("AX5043_PLLVCODIV: %02.2x\n", ax5043ReadReg(AX5043_PLLVCODIV));
            printf("AX5043_FREQA0: %x\n", ax5043ReadReg(AX5043_FREQA0));
            printf("AX5043_FREQA1: %x\n", ax5043ReadReg(AX5043_FREQA1));
            printf("AX5043_FREQA2: %x\n", ax5043ReadReg(AX5043_FREQA2));
            printf("AX5043_FREQA3: %x\n", ax5043ReadReg(AX5043_FREQA3));
            printf("AX5043_MODULATION: %x\n", ax5043ReadReg(AX5043_MODULATION));
            printf("AX5043_TXPWRCOEFFB0: %x\n", ax5043ReadReg(AX5043_TXPWRCOEFFB0));
            printf("AX5043_TXPWRCOEFFB1: %x\n", ax5043ReadReg(AX5043_TXPWRCOEFFB1));
            break;
        }

        case getRSSI:{
//            if(IAmStandbyCPU()){
//                int rssi = get_rssi();
//                printf("RSSI is %d\n",((int16_t)rssi) - 255);
//            } else {
//                printf("Only standby CPU has RSSI on the RT-IHU (for now)\n");
//            }

            break;
        }
#if 0
        case testRxFreq: {
            // This is so we can find what the receive frequency is on first build
            {
                printf("I am not in control\n");
                int freq = 145835000;

                ax5043PowerOn();

                printf("Transmitting on receive freq: %d\n", freq);

                uint8_t retVal = axradio_init_2m(freq);
                printf("axradio_init_2m: %d\n",retVal);


                retVal = mode_tx_2m();
                printf("mode_tx_2m: %d\n",retVal);

                ax5043WriteReg(AX5043_PWRMODE, AX5043_PWRSTATE_FULL_TX);
                printf("Powerstate is FULL_TX\n");

                printf("AX5043_XTALCAP: %d\n", ax5043ReadReg(AX5043_XTALCAP));

                fifo_repeat_byte(0xAA, 100, 0);
                fifo_repeat_byte(0xAA, 100, 0);
                fifo_repeat_byte(0xAA, 100, 0);
                fifo_repeat_byte(0xAA, 100, 0);
                fifo_repeat_byte(0xAA, 100, 0);
                fifo_repeat_byte(0xAA, 100, 0);
                fifo_repeat_byte(0xAA, 100, 0);
                fifo_repeat_byte(0xAA, 100, 0);
                fifo_repeat_byte(0xAA, 100, 0);
                fifo_repeat_byte(0xAA, 100, 0);
                fifo_repeat_byte(0xAA, 100, 0);
                fifo_repeat_byte(0xAA, 100, 0);
                fifo_repeat_byte(0xAA, 100, 0);
                fifo_repeat_byte(0xAA, 100, 0);

                fifo_commit();
            }
            break;
        }
#endif

        case testPLLrange:{
            test_pll_range();
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
