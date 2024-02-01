/*
 * Command.c
 *
 *  Created on: Mar 12, 2019
 *      Author: burns
 */
#include <pacsat.h>
#include "FreeRTOS.h"
#include "os_task.h"

//Flight software headers
#include "CommandTask.h"
#include "gpioDriver.h"
#include "serialDriver.h"
#include "het.h"
#include "gpioDriver.h"
#include "spiDriver.h"
#include "MET.h"
#include "nonvolManagement.h"
#include "UplinkCommands.h"
#include "TMS570Hardware.h"
#include "hmac_sha256.h"
#include "ao_fec_rx.h"
#include "inet.h"
#include "ax5043_access.h"
#include "Max31331Rtc.h"
#include "ADS7828.h"
#include "redposix.h"

#define command_print if(PrintCommandInfo)printf

extern bool InSafeMode,InScienceMode,InHealthMode;
extern bool JustReleasedFromBooster;
extern uint16_t NumWODSaved;
extern int32_t WODHkStoreIndex,WODSciStoreIndex;
extern rt1Errors_t localErrorCollection;

static uint8_t SWCmdCount,HWCmdCount;

/*
 * Forward Routines
 */
static void DecodeHardwareCommand(UplinkCommands);
static bool TlmSWCommands(CommandAndArgs *comarg);
static bool OpsSWCommands(CommandAndArgs *comarg);
static bool DispatchSoftwareCommand(SWCmdUplink *uplink,bool local);
bool AuthenticateSoftwareCommand(SWCmdUplink *uplink);
bool CommandTimeOK(SWCmdUplink *uplink);


static bool PrintCommandInfo = false,CommandTimeEnabled = false;
uint8_t SWCmdRing[4] = {0,0,0,0};

//#define SPECIAL_RANDOM_NUMBER 49093  /* For little endian */
#define SPECIAL_RANDOM_NUMBER 50623 /* For big endian */

#define INSERT_IN_PAIR_RING_BUFFER(shortinfo,longinfo)\
        {\
    SWCmdRing[3] = SWCmdRing[2];\
    SWCmdRing[2] = SWCmdRing[1];\
    SWCmdRing[1] = SWCmdRing[0];\
    SWCmdRing[0] =(((shortinfo & 7)<< 5) | (longinfo & 0x1f));\
        }


void CommandTask(void *pvParameters)
{
    bool FallbackTimerActive = true;

    /*
     * This is a task which sits around waiting for a message either from the the hardware or the software
     * command decoder to tell it that a command has been received.  When that happens, it acts on the
     * command.
     *
     * It is also used to update time time-based items that can't be done from the MET module because
     * they must not be done from an interrupt routine.
     */


    //Intertask_Message message;
    vTaskSetApplicationTaskTag((xTaskHandle) 0, (pdTASK_HOOK_CODE)CommandWD );
    ReportToWatchdog(CommandWD);

    //RestartUplinkDecode(); // Initialize software command decoder
    SWCmdCount = 0;
    HWCmdCount = 0;

    InitInterTask(ToCommand, 10);
    LoadMRAMKey();
    ReportToWatchdog(CommandWD);
    CommandTimeEnabled = ReadMRAMBoolState(StateCommandTimeCheck);
    debug_print("Waiting for command ...\n");
    while (1) {
        ReportToWatchdog(CommandWD);
        taskYIELD();
        bool gotSomething;
        Intertask_Message msg;
        ReportToWatchdog(CommandWD);
        gotSomething=WaitInterTask(ToCommand,SECONDS(2), &msg);
        /*
         * Since this task is not terribly busy, we will also use it to trigger things that
         * run on a timer.
         */
        ReportToWatchdog(CommandWD);
        if(gotSomething){
            /* Here we actually received a message, so there is a command */
            //GPIOSetOff(LED3); // THere was a command.  Tell a human for ground testing
            JustReleasedFromBooster = false;
            if(FallbackTimerActive){
                /*
                 * Remember we have received a command and turn off the fallback
                 * timer.
                 */
                WriteMRAMCommandReceived(true);
                FallbackTimerActive = false;
            }
            if(msg.MsgType == CmdTypeRawSoftware){
                debug_print("COMMAND VIA MESSAGE NOT SUPPORTED!!\n");
            } else if(msg.MsgType == CmdTypeHardware){
                int fixedUp=0;
                //
                // The data lines are not attached to adjacent pins
                // in order so we have to straighten them out.
                if(msg.argument & 2){
                    fixedUp |= 8;
                }
                if(msg.argument & 1){
                    fixedUp |= 1;
                }
                DecodeHardwareCommand((UplinkCommands)fixedUp);
            }
        } else {
            // Just another timed thing not really related to this task
            //GPIOSetOn(LED3); // Turn the light off (so command light was on about 2 seconds)
        }
    }
}

/*******************************************************************
 * Here are some utility routines.
 *
 * Fake command is called from the debug task and from below.
 * CheckCommandTimeout is called from the command task.
 *
 ********************************************************************/
void SimulateHWCommand(UplinkCommands cmd){
    Intertask_Message msg;
    msg.MsgType = CmdTypeHardware;
    msg.argument = cmd;
    //debug_print("Fake command %d\n",cmd);
    NotifyInterTask(ToCommand,WATCHDOG_SHORT_WAIT_TIME,(Intertask_Message *)&msg);
}
void SimulateSwCommand(uint8_t namesp, uint16_t cmd, const uint16_t *args,int numargs){
    Intertask_Message msg;
    int i;
    msg.MsgType = CmdTypeValidatedSoftwareLocal;
    msg.argument = namesp;
    msg.argument2 = cmd;
    if(numargs > sizeof(msg.data))numargs=sizeof(msg.data);
    for(i=0;i<numargs;i++){
        msg.data[i] = args[i];
    }
    //debug_print("Fake command %d\n",cmd);
    NotifyInterTask(ToCommand,WATCHDOG_SHORT_WAIT_TIME,(Intertask_Message *)&msg);
}
static void DecodeHardwareCommand(UplinkCommands command){

    /*
     * This is where we decode a hardware command.  In other words, this is where
     * we get when the hardware device on the receiver finds a command and sends it
     * to the IHU.  No alt sequence decoding on Golf.
     */
    if(command & CMD_TX_OFF) {
        uint16_t args[1] = {1};
        command_print("Hw Cmd: Transmit off\n");
        SimulateSwCommand(SWCmdNSSpaceCraftOps,SWCmdOpsDCTTxInhibit,args,1); //Send to others

    }
}

bool DispatchSoftwareCommand(SWCmdUplink *uplink,bool local){
    uint8_t nameSpace;

    /*
     * Ok, we have a command that has been error corrected, unwhitened, CRC checked, unconvolved, timestamped,
     * etc.
     *
     * Next we check other stuff that has to be right.  Specifically, we check the satellite address
     * and then authenticate, and also check the timestamp for something reasonable.
     */
    nameSpace = uplink->namespaceNumber;
    if(uplink->address != OUR_ADDRESS){
        command_print("Wrong address %x\n\r",uplink->address);
        return FALSE;
    }
    if(local && (nameSpace != SWCmdNSInternal)){
        //Enter in the telemetry ring buffer only if it originated on this
        //CPU (and don't report internal commands)
        INSERT_IN_PAIR_RING_BUFFER(nameSpace,uplink->comArg.command);
    }
    /*
     * If Command Time Checking was enabled and we got a command, we know it is working
     * so we can turn off the timeout function
     */
    if(CommandTimeEnabled) {  // - is this missing { }??  Otherwise it applies to the command_print?? - { } added by G0KLA
        //SetInternalSchedules(NoTimeCommandTimeout,TIMEOUT_NONE);
    }

    command_print("Namespace=%d,command=%d,arg=%d\n",nameSpace,uplink->comArg.command,
                  uplink->comArg.arguments[0]);

    switch(nameSpace){
    case SWCmdNSSpaceCraftOps: {
        return OpsSWCommands(&uplink->comArg);
    }
    case SWCmdNSTelemetry:{
        return TlmSWCommands(&uplink->comArg);
    }
    default:
        printf("Unknown namespace %d\n\r",nameSpace);
        localErrorCollection.DCTCmdFailNamespaceCnt++;
        return FALSE;

    }
}

/*
 * Just for initial pacsat code most of the operations that commands do are commented out.  We just
 * print the command so the uplink can be tested.
 */

bool OpsSWCommands(CommandAndArgs *comarg){

    switch((int)comarg->command){

    // This first group is intended to write states into the MRAM

    case SWCmdOpsDisableAutosafe:
        command_print("Disable Autosafe Command\n\r");
        //DisableAutosafe();
        break;
    case SWCmdOpsEnableAutosafe:
        command_print("Enable Autosafe Command\n\r");
        //EnableAutosafe();
        break;
    case SWCmdOpsSetAutosafeVoltages:{
        uint16_t into=comarg->arguments[0],outof=comarg->arguments[1];
        command_print("Set autosafe voltages %d %d\n",into,outof);
        //SetAutosafe(into,outof);
        break;
    }

    //Now we get to other stuff

    case SWCmdOpsSafeMode:{
        command_print("Safe mode command\n");
        //SendSafeModeMsg(false); // False forced it to not be autosafe
        break;
    }
    case SWCmdOpsHealthMode:
        command_print("Health mode \n");
        //SendHealthModeMsg();
        break;
    case SWCmdOpsScienceMode:{
        int timeout = comarg->arguments[0];
        if(timeout<=0)timeout = 1; // Just in case
        //SendScienceModeMsg(timeout);
        break;
    }
    case SWCmdOpsClearMinMax:
        command_print("Clear minmax\n");
        //ClearMinMax();
        break;

    case SWCmdOpsNoop:
        command_print("No-op command\n\r");
        break;
    case SWCmdOpsEnablePb: {
        bool turnOn;
        turnOn = (comarg->arguments[0] != 0);
        if(turnOn){
            command_print("Enable PB\n\r");

        } else {
            command_print("Disable PB\n\r");
        }
        WriteMRAMBoolState(StatePbEnabled,turnOn);
        break;
    }
    /* This command will wipe the file system but not reset the file numbers.  So a ground
     * station will be able to continue receiving files without having to reset its local
     * directory*/
    case SWCmdOpsFormatFs: {
        if (red_umount("/") == -1) {
            debug_print("Unable to unmount filesystem: %s\n",
                               red_strerror(red_errno));
            return FALSE;
        }
        if (red_format("/") == -1) {
            debug_print("Unable to format filesystem: %s\n",
                   red_strerror(red_errno));
            return FALSE;
        }
        if (red_mount("/") == -1) {
            debug_print("Mount after format failed, filesystem broken: %s\n",
                   red_strerror(red_errno));
            return FALSE;
        }
        break;
    }
    case SWCmdOpsEnableUplink: {
        bool turnOn;
        turnOn = (comarg->arguments[0] != 0);
        if(turnOn){
            command_print("Enable Uplink\n\r");

        } else {
            command_print("Disable Uplink\n\r");
        }
        WriteMRAMBoolState(StateUplinkEnabled,turnOn);
        break;
    }
    case SWCmdOpsSetTime:{
        uint32_t time = comarg->arguments[0] + (comarg->arguments[1] << 16);
        command_print("Set time to %d\n\r",time);
        setUnixTime(time);
        if (RTCIsOk()) {
            bool set = SetRtcTime31331(&time);
            if (set) {
                command_print("Setting RTC\n");
            } else {
                command_print("Failed to set RTC\n");
            }
        }
        break;
    }
    case SWCmdOpsResetSpecified:{
            command_print("Reset IHU\n");
            // This will execute when we return to the PB task
        break;
    }
    case SWCmdOpsEnableCommandTimeCheck:{
        EnableCommandTimeCheck(comarg->arguments[0] != 0);
        if(CommandTimeEnabled){
            command_print("Enable command time check\n\r");

        } else {
            command_print("Disable command time check\n\r");
        }
        break;
    }
    case SWCmdOpsDCTTxInhibit:
        if(comarg->arguments[0] != 0) { // True means to inhibit it
            command_print("SW:Inhibit transmitting\n");
        } else {
            command_print("SW:Uninhibit transmitting\n");
        }
        break;
    case SWCmdOpsSelectDCTRFPower: {
        int myCpuIndex = 0;//ThisProcessorIsPrimary()?0:1;
        bool safePowerHigh = comarg->arguments[myCpuIndex]; // Arg 0 and 1 are for safe
        bool normalPowerHigh = comarg->arguments[myCpuIndex+2]; //Arg 2 and 3 are for normal
        command_print("Select Power Level; for this DCT, safe=%s,normal=%s\n",safePowerHigh?"high":"low",
                normalPowerHigh?"high":"low");
        //SetSafeRfPowerLevel(safePowerHigh);
        //SetNormalRfPowerLevel(normalPowerHigh);
#if 0
        WriteMRAMTelemLowPower();
        ax5043_set_frequency(freq);
        ax5043_set_power(power);
        normalModeHighPower=ReadMRAMBoolState(StateNormalRfPowerLevel);
#endif

        break;
    }
     default:
        localErrorCollection.DCTCmdFailCommandCnt++;
        command_print("Unknown Ops Command %d\n\r",comarg->command);
        return FALSE;
    }
    return TRUE;
}
void EnableCommandPrint(bool enable){
    PrintCommandInfo = enable;
}
void EnableCommandTimeCheck(bool enable){
    CommandTimeEnabled = enable;
    if(enable){
        //SetInternalSchedules(NoTimeCommandTimeout,SW_COMMAND_TIME_TIMEOUT);
    } else {
        //SetInternalSchedules(NoTimeCommandTimeout,TIMEOUT_NONE);
    }
    WriteMRAMBoolState(StateCommandTimeCheck,CommandTimeEnabled);
}
bool TlmSWCommands(CommandAndArgs *comarg){
    switch(comarg->command){
    case SWCmdTlmWODSaveSize:
        command_print("Change WOD size to %d\n\r",comarg->arguments[0]);
        //ChangeWODSaved(comarg->arguments[0]);
        break;

    case SWCmdTlmLegacyGain:
        command_print("Tlm legacy gain to %d\n\r",comarg->arguments[0]);
        break;

    case SWCmdTlmDCTDrivePwr:{
        int myCpuIndex = 0;
        uint16_t lowPower = comarg->arguments[myCpuIndex]; // Arg 0 and 1 are for low
        uint16_t highPower = comarg->arguments[myCpuIndex+2]; //Arg 2 and 3 are for high
        command_print("Drive power reg for this proc are Low: %d, high: %d\n",lowPower,highPower);
        //SetDCTDriveValue(lowPower,highPower);
        break;
    }

    default:
        localErrorCollection.DCTCmdFailCommandCnt++;
        printf("Unknown Tlm Command\n\r");
        return FALSE;
    }
    return TRUE;
}

bool DecodeSoftwareCommand(SWCmdUplink *softwareCommand) {

    if(AuthenticateSoftwareCommand(softwareCommand)){
        command_print("\n\rCommand Authenticated!\n");

        softwareCommand->comArg.arguments[0] = ttohs(softwareCommand->comArg.arguments[0]);
        softwareCommand->comArg.arguments[1] = ttohs(softwareCommand->comArg.arguments[1]);
        softwareCommand->comArg.arguments[2] = ttohs(softwareCommand->comArg.arguments[2]);
        softwareCommand->comArg.arguments[3] = ttohs(softwareCommand->comArg.arguments[3]);

        /*
         * Here we have a command that was received on the uplink and ready to act on.
         */

        bool rc = DispatchSoftwareCommand(softwareCommand,true);

        return rc;
    } else {
        command_print("\n\rCommand does not authenticate\n");
    }
    return FALSE;

}

bool AuthenticateSoftwareCommand(SWCmdUplink *uplink){
    uint8_t localSecureHash[32];
    bool shaOK;

    hmac_sha256(hmac_sha_key, AUTH_KEY_SIZE,
                (uint8_t *) uplink, SW_COMMAND_SIZE,
                localSecureHash, sizeof(localSecureHash));
    shaOK = (memcmp(localSecureHash, uplink->AuthenticationVector, 32) == 0);
    if (PrintCommandInfo) {
        command_print("Local: ");
        int i;
        for (i=0; i<sizeof(localSecureHash);i++)
            command_print("%x ", localSecureHash[i]);
        command_print("\nUplink: ");
        for (i=0; i<sizeof(uplink->AuthenticationVector);i++)
            command_print("%x ", uplink->AuthenticationVector[i]);
        command_print("\n");
    }
    if(shaOK){
        uplink->comArg.command = ttohs(uplink->comArg.command); // We might have to look to determine if authenticated
        return CommandTimeOK(uplink);
    } else {
        localErrorCollection.DCTCmdFailAuthenticateCnt++;
        return false;
    }

}
bool CommandTimeOK(SWCmdUplink *uplink){
    logicalTime_t timeNow;
    static int lastCommandTime = 0;
    int secondsOff,uplinkSeconds;
    uint16_t uplinkEpoch;
    bool goodTime = true;

    /*
     * Here we are checking the time included in the command to make sure it is within the appropriate
     * range of the satellite time.  This is to avoid replay attacks.
     *
     * The algorithm is as follows:
     *
     * 1) The reset epoch in the command must match the satellite exactly
     * 2) The seconds in the command MUST be greater than the seconds in the last command
     * 3) The seconds in the command has to be within a tolerance (specified in FoxConfig) of the satellite time
     */
    uplinkEpoch = ttohs(uplink->resetNumber);
    uplinkSeconds = ttoh24(uplink->secondsSinceReset);

    if(CommandTimeEnabled){
        /*
         * If command time is not enabled, always return true, i.e. it's ok.
         * Also as a safety valve, allow us to disable the time check if we can't seem to get it right by
         * putting in a specific random number as the 4th argument for the disable command
         */
        if((uplink->namespaceNumber == SWCmdNSSpaceCraftOps) &&
                (uplink->comArg.command == SWCmdOpsEnableCommandTimeCheck)  &&
                (uplink->comArg.arguments[3] == SPECIAL_RANDOM_NUMBER)
        ) return true;
        getTimestamp(&timeNow);
        secondsOff = uplinkSeconds - timeNow.METcount;
        command_print("UplinkTime,delta %d/%d,%d\n\r",uplink->resetNumber,uplink->secondsSinceReset,secondsOff);
        if(
                /*
                 * The reset epoch in the command HAS to match the satellite, and the satellite time must not
                 * be greater than the "expiration time" in the command.
                 *
                 * In addition, to avoid a reply attack, the command HAS to be newer than the last command received.
                 * And to avoid totally circumventing this whole check, the transmitted time can't be too far behind
                 * or too far ahead of the current satellite time.
                 */

                (uplinkEpoch != timeNow.IHUresetCnt)   || // The uplinked reset count is wrong
                (uplinkSeconds <= lastCommandTime)  || // The uplinked second time is less or same as that of last command
                (uplinkSeconds < timeNow.METcount) || // The uplinked command has expired
                (secondsOff < SECONDS_AUTHENTICATION_MAX_BEHIND) || // Uplinked time too far behind sat time
                (secondsOff > SECONDS_AUTHENTICATION_MAX_AHEAD)  // They uplink had an expiration time too far ahead
        ){
            goodTime = false;
            localErrorCollection.DCTCmdFailTimeCnt++;
        }
        if(goodTime)lastCommandTime = uplinkSeconds; // Do not allow the next command to have the same or earlier
    }
    return goodTime;
}


/*********************************************************************************
 * These routines are called from different tasks
 **********************************************************************************/



/*
 * Here are the timeout callbacks relating to commands
 */

void NoCommandTimeoutCallback(void){
    printf("No command timeout\n");
    SimulateSwCommand(SWCmdNSSpaceCraftOps,SWCmdOpsHealthMode,NULL,0);
    //SetInternalSchedules(NoCommandTimeout,TIMEOUT_NONE); // Set this timeout to never
}
void NoTimedSWCommandTimeoutCallback(void){
    printf("No command after setting time check\n");
    EnableCommandTimeCheck(false);
    //SetInternalSchedules(NoTimeCommandTimeout,TIMEOUT_NONE); // Set this timeout to never
}

/*
 * Here are some external calls
 */
uint8_t GetHWCmdCount(void){
    return HWCmdCount;
}
uint8_t GetSWCmdCount(void){
    return SWCmdCount;
}

//void test_auth() {
//    uint8_t uplink[18] = {0xc8,0x34,0x8a,0xc5,0x8f,0x6c,0x78,0x17,0x94,0x2b,0xc,0xd3,0x74,0x6d,0x53,0xab,0xa6,0x44};
//    int i;
////    for (i=0; i<18;i++)
////        uplink[i] = 0xAB;
//
//    uint32_t localSecureHash32[8],uplinkSecureHash32[8];
//    SHA(uplink, SW_COMMAND_SIZE, localSecureHash32);
//
//    uint8_t *bptr = (uint8_t *)&localSecureHash32[0];
//    int p;
//    for (p=0; p<32; p++)
//        printf("%02x ",*bptr++);
//    printf("\n");
//
//    uint8_t cypher[] = {
//                        0x52,0xfb,0x80,0xb4,0xad,0xaf,0x20,0x40,0xe5,0x58,0x97,0x1d,0xbf,0xe1,0x39,0x23,0xce,0x59,0x0,0xf1,0x54,0xdb,0xd4,0xfa,0x2e,0xb8,0xc7,0x6c,0x1e,0xd3,0x52,0x34
//    };
//    for(i=0;i<8;i++){
//        cypher[i] = ttohl(cypher[i]);
//            }
//    DeCipher32(cypher,(uint8_t *)uplinkSecureHash32);
//    for(i=0;i<8;i++){
//            uplinkSecureHash32[i] = ttohl(uplinkSecureHash32[i]);
//        }
//    bptr = (uint8_t *)&uplinkSecureHash32[0];
//    for (p=0; p<32; p++)
//            printf("%02x ",*bptr++);
//        printf("\n");
//
//}
