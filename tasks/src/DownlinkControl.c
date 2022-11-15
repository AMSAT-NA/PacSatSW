/*
 * downlinkControl.c
 *
 *  Created on: Jan 4, 2020
 *      Author: bfisher, WB1FJ
 *
 *  Modified severely from the Fox version of this module:
 *
 *  Created on: Dec 18, 2012
 *      Author: Burns Fisher, AMSAT-NA
 *
 *  This is the Fox satellite downlink control module.  It manages what is being
 *  transmitted by the satellite based on message passed to it by other tasks and
 *  timers.
 */

#include <pacsat.h> /* Must precede stdio */
#include "DownlinkControl.h"
#include "nonvolManagement.h"
#include "interTaskNotify.h"
#include "downlink.h"
#include "MET.h"
#include "watchdogSupport.h"
#include "nonvolManagement.h"
#include "TelemEncoding.h"
#include "TelemetryBufferMgmt.h"
#include "TelemetryRadio.h"
#include "crc32.h"
#include "ax5043_access.h"
#include "ax5043.h"
#include "inet.h"
#include "CommandTask.h"
#include "TelemetryCollection.h"
/* Standard headers */
#include "stdio.h"
#include "string.h"

/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "os_task.h"
#include "os_queue.h"
#include "os_timer.h"

/* Fox Driver headers */
#include "gpioDriver.h"

/*
 * Global (extern) variables definitions
 *
 * These mode variables are checked in other modules to find out how to behave.
 * They do not necesarily indicate WHY we are in that mode.  That inforamtion is
 * kept locally below.
 */

bool InSafeMode=1,InScienceMode=0,InHealthMode=0;  // These are the global modes that are externs
bool OnOrbit;
/*
 * These are local booleans that describe subsets of the major modes, which in many cases specifies
 * how we got to a particular mode.
 */
static bool transponderEnabled,allowAutoSafeMode,inAutoSafeMode;
static bool inCommandedSafeMode, inEclipseSafeMode;
static bool transponderInhibited=false;
static bool halfDuplexTimerRunning = false;


/*
 * Forward routine declarations
 */
static void IdleTimerCallback ( xTimerHandle xTimer );
static void BeaconTimerCallback ( xTimerHandle xTimer );
static void ScienceModeTimerCallback(xTimerHandle xTimer);
static void SetTelemetryMode(TelemetryMode mode);
static void StartContinuousDownlink(void);
static void SetTransmitterNormalRfPower(void);
static void SetTransmitterSafeRfPower(void);
static void HalfDupRxTimerCallback(xTimerHandle x);

extern bool JustReleasedFromBooster;
extern rt1Errors_t localRtErrors;

/*
 * Global (within this module) variables
 */

static xTimerHandle IdleTimer=0,BeaconTimer=0,ScienceTimer=0,halfDuplexRxTimer;
static bool TransponderTurnedOn = false,ReturnToSafeFromScience=false;
static bool safeModeHighPower=false,normalModeHighPower=false;
static bool overrideUmbilical = false, forceTxOff = false;

;
static int safeFramesRemaining=0;
static uint16_t EnterAutosafeVoltage,ExitAutosafeVoltage;

/*
 * State variables.  We keep track of whether we think that we have a timer
 * running so that we can can ignore any extra timer notifies that we might get.
 */
static bool BeaconActive=false,ShortBeacon=true,IdleTimerActive=false;

#define TELEMETRY_ACTIVE_FALSE 0
#define TELEMETRY_ACTIVE_TRUE 1
#define END_OF_FRAME_LIST -1
#define END_OF_PAYLOAD_LIST -1

static uint32_t TelemetryActive=TELEMETRY_ACTIVE_FALSE; /*0 means no, 1 means yes, other means the time when it changed*/

/**************************************************************************************
 * DownlinkControl is implemented as a state machine.  This section defines the states
 * the transitions, and the routine for entering new states.
 **************************************************************************************/


#define NUM_MESSAGES 8



/*
 *  Here is the actual state table.  If you are in the state defined by a row, and
 *  a message comes in which matches a column, the intersection of row and column tells
 *  what state you go to next.
 */

static const uint8_t StateTable[NUM_STATES][DWNLNK_NUMBER_OF_STATE_MESSAGES] =
{

 ///////////todo:  Fix This table for eclipse--one more column, and fix the eclips rows

 //IdleTimOut BeaconTO TelmDone EntrSafe EntrHelth EntrAutoSf InhibitTx      EnableTx EntrSci    EntrEClSf
 //1           2          3         4      5         6           7              8        9          10
 {Unexpctd,  Unexpctd, TurnOnRx,  Safe,  Health,  AutoSafe,TransmitInhibit, NoChange,ScienceMode,EclipSafe},//1(Health) Health Mode
 {SafeBcon,  NoChange, Safe,      Safe,  Health,  Unexpctd,TransmitInhibit, NoChange,ScienceMode,NoChange},//2(Safe)SafeMode no carrier
 {Unexpctd,  Safe,     Safe,      Safe,  Health,  Unexpctd,TransmitInhibit, NoChange,ScienceMode,NoChange},//3(SafeBcon) Silent Carrier then 1 frame tlm at Start of Beacon
 {AutoSfBcon,NoChange, Unexpctd,  Safe,  Health,  AutoSafe,TransmitInhibit, NoChange,ScienceMode,NoChange},//4(AutoSafe)SafeMode no carrier
 {Unexpctd,  AutoSafe, AutoSafe,  Safe,  Health,  AutoSafe,TransmitInhibit, NoChange,ScienceMode,NoChange},//5(AutoSafeBcon) Silent Carrier then 1 frame tlm at Start of Beacon
 {EclipSfBcon,NoChange,Unexpctd,  Safe,  Health,  AutoSafe,TransmitInhibit, NoChange,ScienceMode,NoChange},//6(EclipseSafe)SafeMode no carrier
 {Unexpctd,  EclipSafe,EclipSafe, Safe,  Health,  AutoSafe,TransmitInhibit, NoChange,ScienceMode,NoChange},//7(EclipseSafeBcon) Silent Carrier then 1 frame tlm at Start of Beacon
 {NoChange,  NoChange, NoChange,NoChange,NoChange,NoChange,    NoChange,    Safe,      NoChange, NoChange}, //8 (Transmit Inhibit)
 {NoChange,  NoChange, TurnOnRx,  Safe,  Health,  AutoSafe,TransmitInhibit, NoChange , NoChange, EclipSafe} //9 (Science)

 //IdleTimOut BeaconTO TelmDone EntrSafe EntrHelth EntrSci  EntrAutoSf
};
/*
 *   This variable holds the current state
 */

static DownlinkStates CurrentState = Safe; /* Starts safe mode */
static bool DoNotSendTelemetry=false; /* This is used for testing*/
/*
 * This routine is called if the state changes (i.e. there was a message which does
 * not result in "no change".  The new state might actually be the same as the old one,
 */

DownlinkStates StateChangeTo(DownlinkStates new);





typedef enum
{
    RestartStandardTelem,
    SafeBeaconTelem,
    ScienceBeaconTelem
} FrameTypeMode;

/*
 * Routines used by the state machine
 */

void GetTelemetryData();
void SetTelemetryFrameType(FrameTypeMode type);

/************************************End of state machine declarations*********************************/

/*
 * For PacSat, these are just filler routines at first to allow it to compile.  We probably
 * don't need at least some of this
 */
bool CreateHeader(int x,void *y){
return true;
};
bool CreatePayload(int x, void *y){
return true;
}

/*
 * The Downlink Control Task itself
 */

portTASK_FUNCTION_PROTO(DownlinkCtrlTask, pvParameters){
    bool status;
    Intertask_Message msgReceived;

    vTaskSetApplicationTaskTag((xTaskHandle) 0, (pdTASK_HOOK_CODE)ControlWD );

    /*
     * Init the timers we will be using
     */
    // Beacon timer:  How long the beacon can stay on.  Changes after launch from 10 seconds to more
    BeaconTimer = xTimerCreate("Beacon",SHORT_BEACON_TIMEOUT,FALSE,(void *)1,BeaconTimerCallback);
    // How long is the silence between beacons
    IdleTimer = xTimerCreate("Idle",IDLE_TIMEOUT,pdTRUE,(void *)2,IdleTimerCallback);
    // How long does science and camera mode last
    ScienceTimer = xTimerCreate("Science",(SCIENCE_TIMEOUT*MINUTES(1)),pdFALSE,(void *)4,ScienceModeTimerCallback);
    // How long does the receiver stay on if we are in half duplex mode (i.e. the standby processor has failed)
    halfDuplexRxTimer = xTimerCreate("HalfDupRx",HALF_DUPLEX_RX_TIME,false,(void *)1,HalfDupRxTimerCallback);
    /*
     * Init the intertask communication we will be using
     */
    InitInterTask(ToDownlinkControl,10);

    /*
     * These inits might have taken a little time.  Let's reset the watchdogs once.
     */
    ResetAllWatchdogs();

    /*
     * We have booted, and are ready to run.
     * Time to say hello and then do our relay-sat job.
     * Queue up some messages and init the state variables
     * to get things going.
     */

    /*
     * Initialize the state variables.  When the state machine gets a message, it will
     * set the correctly.
     */
    CurrentState = Safe;
    InSafeMode = true;
    inEclipseSafeMode = false;  // Eclipse is never remembered across resets
    SetAutosafeVoltages();

    // Initialize state variables from MRAM
    transponderEnabled = ReadMRAMBoolState(StateTransponderEnabled);
    safeModeHighPower=ReadMRAMBoolState(StateSafeRfPowerLevel);
    normalModeHighPower=ReadMRAMBoolState(StateNormalRfPowerLevel);
    allowAutoSafeMode = ReadMRAMBoolState(StateAutoSafeAllow);
    if(ReadMRAMBoolState(StateTransmitInhibit)){
        CommandInhibitTransmitting();
    } else {
        // This block is just to get a temporary variable autosafe and avoid
        // confusing it with the global variable InAutoSafe.
        bool autosafe,commandSafe;
        autosafe = ReadMRAMBoolState(StateAutoSafe) && allowAutoSafeMode;
        commandSafe = ReadMRAMBoolState(StateCommandedSafeMode);
        if(commandSafe || autosafe){
            // Only one of these should be set.  Commanded overrides auto.  They should
            // never be both set, but just in case, we will force it.
            if(commandSafe){
                WriteMRAMBoolState(StateAutoSafe,false);
                autosafe = false;
            } // Else not required because the logic above says commandSafe is already
              // correct if we executed an else.
            SendSafeModeMsg(autosafe);
        } else {
            SendHealthModeMsg(); // This, plus a ground command, is the only way to get into health mode
        }
    }
    ReportToWatchdog(ControlWD);

    while(true) {               /* Here is the main control loop */
        /*
         * Wait for something to do
         *
         *  This task is mostly a state machine that runs on message passing.  Messages are sent by a
         *  timer or another task. The the "other tasks" are either Radio (whose messages relate
         *  to telemetry needs or Command (which is the task that gets notified when explicit
         *  commands are uplinked).  They also come from "periodicRoutines" where we check for timed or
         *  eclipse-based actions.
         *
         *  Timers are BeaconTimer (the maximum amount of time the beacon may
         *  be on...a launch requirement), and IdleTimer (how long do we stay silent between beacons
         *  in safe mode if there is no other uplinked command).  Remember that timer messages may be
         *  in the queue before their respective timer is canceled, so we only want
         *  to act on them if the appropriate variable is set.
         *
         *  There are some states that are not managed by modes and the state machine.  I will call these
         *  "out of band".  Such things including turning the transponder on (which can only happen in health
         *  mode, but does not have to) and turning the SDR on/off.
         *
         */
        ReportToWatchdog(ControlWD);
        status = WaitInterTask(ToDownlinkControl,WATCHDOG_MAX_WAIT_TIME,&msgReceived);
        ReportToWatchdog(ControlWD);
        /*
         * Ok, now this is a convenient time to check the bus voltage and go into safe mode if
         * it is too low.  Come out if it is high enough after we went into safe because of low
         * power.
         */
#if 0
        if(IsStabilizedAfterBoot() && allowAutoSafeMode){
            uint8_t battery;
            if(getBusVoltage(&battery)){ //Only do if battery reading is valid
                if (!InSafeMode && (battery < EnterAutosafeVoltage)){
                    // Go into auto safe mode because battery is too low
                    debug_print("Going into autosafe mode at battery=%d\n\r",battery);
                    SimulateSwCommand(SWCmdNSInternal,SWCmdIntAutosafeMode,NULL,0);
                }
                if(InSafeMode && (battery > ExitAutosafeVoltage)
                        && inAutoSafeMode) {
                    debug_print("Coming out of autosafe mode at battery=%d\n\r",battery);
                    SimulateSwCommand(SWCmdNSSpaceCraftOps,SWCmdOpsHealthMode,NULL,0);
                }
            }
        }
#endif
        if(status==true && !DoNotSendTelemetry){
            DownlinkStates newState;
            IntertaskMessageType message = msgReceived.MsgType;
            /*
             * Here we got a message.  Act on it.
             */
            //debug_print("Message received is %d\n\r",message);
            if(message == DwnlnkCollectTelemetry){
                /*
                 * This is the only message that does not change state.  There is not
                 * even a state table column for it.  Just do it.
                 */
                GetTelemetryData();
                /*
                 * This message is sent from Audio when it has freed up a buffer so that it can be
                 * filled.  We get here if there is no special case.  All we have to do is to fill
                 * the buffer.
                 */

                ReportToWatchdog(ControlWD);
            } else {

                /*
                 * Here we may have a state change.  Call routines to enter a new state.
                 * If you want to re-enter the same state, that's ok.  You can specify
                 * that and it will call StateChange again (for example to reset the
                 * hang timer if you see a new 67Hz tone.  NoChange does nothing except return
                 * NoChange, and thus makes (duh) NoChange.  StateChangeTo may also return
                 * NoChange even if that is not what was passed in.
                 */
                //debug_print("In state %d, rcvd msg %d, ",CurrentState,message);
                newState = StateChangeTo((DownlinkStates)StateTable[CurrentState-1][(int)message-1]); // Everything is 1-based but the array
                //debug_print("new state %d\n\r",newState);

                if ((newState != NoChange) && (newState != Unexpctd)){
                    CurrentState = newState;
                }
            }
        }

    } /*While*/

}


DownlinkStates StateChangeTo(DownlinkStates newState) {
    DownlinkStates returnState;
    /*
     * When we change to a new state, we come here to do the stuff
     * required to actually enter that state.  States are defined
     * such that it does not matter what state you are coming from.
     * The work is the same.  Mostly.
     */
    returnState = newState; //Default to the return state being the passed in state.
    switch(newState){
    case TurnOnRx: {
        /*
         * When the state table sends us here, it means that we were in a mode that normally would not
         * see the telemetry stop, but the radio DID tell us that the telemetry ended.  This happens
         * when Downlink command has sent it a frame with the "lastFrame" variable set, and this happens
         * because we have come to a "wait for rx" frame type and the other processor is dead.  Thus this
         * all means we are in half duplex mode and we are going to turn on the receiver for a while.
         */
        if(StartReceiveTimer(false)){
            // False argument means "start" as opposed to "restart".
            // Returning false means that the timer did not start for some reason so we will not
            // start the receiver
            printf("Starting Receiver\n");
            ax5043StartRx();
        } else {
            // Receive timer did not start, so we did not start the receiver, but let's restart telemetry
            // by making a fake timer call
            HalfDupRxTimerCallback(halfDuplexRxTimer);
        }
        returnState = NoChange; // This one does not make any change in the main state
        break;
    }
    case Health: {
        /*
         * We can get here either from safe mode (with a command), from
         * science mode (either via a command or a timeout) or from one of
         * the substates that has the idle timer or beacon timer active.
         * Turn all that stuff off.
         */
        SetTelemetryMode(ModeHealth);
        StartContinuousDownlink();
        break;
    }
    case ScienceMode: {
        /*
         * We can get here either from most modes (with a command), and
         * exit either via the timer or another command.  On initting after
         * reset, we never go here.
         */
        SetTelemetryMode(ModeScience);
        StartContinuousDownlink();
        break;

    }

    case AutoSfBcon:
    case SafeBcon: {
        /* Here we are in safe mode.  Our idle timer has kicked off so
         * it is time to send /unmodulated carrier/2 telem frames/unmodulated carrier
         */
        //AudioSetMixerSource(MixerSquare); // Silent carrier needed to sync FoxTelem
        //assert(IdleTimerActive)
        /*
         *  Now we now sending a silent carrier
         *  Get some telemetry queued up and ready to send
         */
        safeFramesRemaining = FRAMES_IN_SAFE_MODE;
        SetTelemetryFrameType(SafeBeaconTelem); // Set it to do the frames in beacon order
        InitRestartTelemetryQueues();   // Allow buffers to be filled
        GetTelemetryData();             // Now go get the first batch of telemetry
        ReportToWatchdog(ControlWD);
        AudioSetMixerSource(MixerTone);  // Just send a carrier (off center a bit)
        BeaconActive=true;
        BeaconOn(); // "Silent beacon" for a few seconds
        if(newState==SafeBcon && JustReleasedFromBooster){

            /*
             * For safe mode we must start the timer right now so maximum beacon time include the
             * silence
             */
            if(xTimerStart(BeaconTimer, WATCHDOG_MAX_WAIT_TIME) != pdTRUE) {
                ReportError(ControlTimerNotStarted,true,CharString,(int)"BeaconTimer");
            }

            vTaskDelay(BEACON_SILENCE);     // But don't send telemetry yet.  Wait for silent carrier time */
        } else {
            /*
             * After a command, we can exceed the 10 second limit, so give a longer time of blank
             * carrier before starting the timer.  Also if this is the first time here, set the beacon
             * timeout to a bit longer to be sure both packets are fully sent.
             */

            ReportToWatchdog(ControlWD);
            if(ShortBeacon){

                if(xTimerChangePeriod( BeaconTimer,
                                       BEACON_TIMEOUT,
                                       WATCHDOG_MAX_WAIT_TIME)
                        == pdTRUE){
                    ShortBeacon = false;
                }
                ReportToWatchdog(ControlWD);
            }

            vTaskDelay(BEACON_SILENCE);     // But don't send telemetry yet.  Wait for silent carrier time */
            ReportToWatchdog(ControlWD);
            if(xTimerStart(BeaconTimer, WATCHDOG_MAX_WAIT_TIME) != pdTRUE) {
                ReportError(ControlTimerNotStarted,true,CharString,(int)"BeaconTimer");
            }
        }
        BeaconOn(); // Make really sure it is on.
        AudioSetMixerSource(MixerData);// Now start the data sequence
        ReportToWatchdog(ControlWD);
        xTimerReset(IdleTimer, WATCHDOG_MAX_WAIT_TIME); //Make sure we are more-or-less in sync with the beacon timer.
        break;
    }
    case EclipSafe:
    case AutoSafe:
    case Safe: {
        /*
         * Here we have entered safe mode either after a beacon or from a ground command or from
         * some problem like low voltage or entering eclipse
         */
        if(!allowAutoSafeMode && (returnState ==AutoSafe)){
            // Auto might have just been disabled.  Get out of the auto-safe path
            returnState = Safe;
        }
        TransponderOff();
        BeaconOff();
        StopTelemetryProcessing();
        AudioSetMixerSource(MixerSilence);

        vTaskDelay(CENTISECONDS(10)); // Make sure everything is done.
        if(returnState == AutoSafe) { // Do all the appropriate stuff for each of the possible safe modes
            SetTelemetryMode(ModeAutosafe);
        } else if (returnState == EclipSafe){
            SetTelemetryMode(ModeEclipseSafe);
        } else {
            SetTelemetryMode(ModeSafe);
        }

        if(BeaconActive){
            xTimerStop(BeaconTimer,WATCHDOG_MAX_WAIT_TIME);
            BeaconActive=false;
        }
        if(!Can2PartnerIsOk())ax5043StartRx(); // If other proc is dead, do receive on this one (1/2 duplex)
        if(!IdleTimerActive){
            int status;
            status = xTimerStart(IdleTimer, WATCHDOG_MAX_WAIT_TIME);
            if(status != pdTRUE){
                ReportToWatchdog(ControlWD); // We have waited the max time.  Don't let it timeout.  We want a different error!
                ReportError(ControlTimerNotStarted,true,CharString,(int)"IdleTimer");;
            }
            IdleTimerActive=true;
        }

        break;
    }
    case TransmitInhibit:{
        // FCC no transmit command.  Turn off all the timers.  Turn off all the transmission from the DCT
        // and anything else we have control of.
        if(BeaconActive){
            xTimerStop(BeaconTimer,WATCHDOG_MAX_WAIT_TIME);
            BeaconActive=false;
        }
        if(IdleTimerActive) {
            xTimerStop(IdleTimer,WATCHDOG_MAX_WAIT_TIME);
            IdleTimerActive = false;
        }
        TransponderOff();
        BeaconOff();
        StopTelemetryProcessing();
        AudioSetMixerSource(MixerSilence);
        //todo:  Need to turn off SDR
    }
    case Unexpctd:{
        /* Just in case we got the beacon timer on accidentally
         * we make sure it is off here.  I'm doing this because of
         * a 2-minute glitch in downlink data.  I'd think it would be
         * pretty unusual for this case to actually happen.
         */
        if(IdleTimerActive) {
            xTimerStop(IdleTimer,WATCHDOG_MAX_WAIT_TIME);
            IdleTimerActive = false;
        }

        break;
    }
    default: /* NoChange comes here */
        break;
    }
    return returnState;

}
static uint8_t LowSpeedTelemTypeIndex=0, SafeTelemTypeIndex=0,ScienceTelemTypeIndex=0;

void SetTelemetryFrameType(FrameTypeMode frameTypes){

    StopTelemetryProcessing();
    if (frameTypes == RestartStandardTelem){
        /* Restart means to start again with a type 1 frame; make all buffers ready to use */
        InitRestartTelemetryQueues();     // Make all the buffers ready to fill
        LowSpeedTelemTypeIndex = 0;
        ScienceTelemTypeIndex=0;
    }
}
static void CreateFrame(allFrames_t *frameToFill,int frameType){
    static const int PayloadOrder[TYPES_OF_FRAME][MAX_PAYLOADS_PER_FRAME+1]={
            //0 REALTIME_MIN_FRAME
            {HEADER_PAYLOAD,RT_HK_PAYLOAD,RT_RAD_PAYLOAD,RT_RAGNAROK_PAYLOAD,MIN_VALS_PAYLOAD,WOD_HK_PAYLOAD,
                    WOD_RAD_PAYLOAD,WOD_RAGNAROK_PAYLOAD,FILLER_PAYLOAD,END_OF_PAYLOAD_LIST},

            //1 REALTIME_MAX_FRAME
            {HEADER_PAYLOAD,RT_HK_PAYLOAD,RT_RAD_PAYLOAD,RT_HK_PAYLOAD,MAX_VALS_PAYLOAD,WOD_HK_PAYLOAD,WOD_RAD_PAYLOAD,
                    WOD_RAGNAROK_PAYLOAD,FILLER_PAYLOAD,END_OF_PAYLOAD_LIST},

            //2 ALL_WOD1_FRAME
            {HEADER_PAYLOAD,WOD_HK_PAYLOAD,WOD_HK_PAYLOAD,WOD_HK_PAYLOAD,WOD_RAD_PAYLOAD, WOD_RAD_PAYLOAD,
                    WOD_RAGNAROK_PAYLOAD,WOD_RAGNAROK_PAYLOAD,FILLER_PAYLOAD,END_OF_PAYLOAD_LIST},

            //3 ALL_WOD2_FRAME
            {HEADER_PAYLOAD,WOD_HK_PAYLOAD,WOD_HK_PAYLOAD,WOD_RAD_PAYLOAD, WOD_RAD_PAYLOAD,WOD_RAD_PAYLOAD,
                    WOD_RAGNAROK_PAYLOAD, WOD_RAGNAROK_PAYLOAD,FILLER_PAYLOAD,END_OF_PAYLOAD_LIST},

            //4 ALL_WOD2_FRAME
            {HEADER_PAYLOAD,WOD_HK_PAYLOAD,WOD_HK_PAYLOAD,WOD_RAD_PAYLOAD, WOD_RAD_PAYLOAD, WOD_RAGNAROK_PAYLOAD,
                    WOD_RAGNAROK_PAYLOAD,WOD_RAGNAROK_PAYLOAD,FILLER_PAYLOAD,END_OF_PAYLOAD_LIST},
            //5 SAFE_DATA1_FRAME
            {HEADER_PAYLOAD,RT_HK_PAYLOAD,RT_RAGNAROK_PAYLOAD,MIN_VALS_PAYLOAD, MAX_VALS_PAYLOAD, WOD_HK_PAYLOAD,
                    WOD_HK_PAYLOAD,WOD_RAGNAROK_PAYLOAD,FILLER_PAYLOAD,END_OF_PAYLOAD_LIST},

            //6 SAFE_DATA2_FRAME
            {HEADER_PAYLOAD,RT_HK_PAYLOAD,RT_RAGNAROK_PAYLOAD,MIN_VALS_PAYLOAD, MAX_VALS_PAYLOAD, WOD_HK_PAYLOAD,
                WOD_RAGNAROK_PAYLOAD,WOD_RAGNAROK_PAYLOAD,FILLER_PAYLOAD,END_OF_PAYLOAD_LIST},

            //7 SAFE_WOD_FRAME
            {HEADER_PAYLOAD,WOD_HK_PAYLOAD,WOD_HK_PAYLOAD,WOD_HK_PAYLOAD, WOD_RAGNAROK_PAYLOAD, WOD_RAGNAROK_PAYLOAD,
                WOD_RAGNAROK_PAYLOAD,FILLER_PAYLOAD,END_OF_PAYLOAD_LIST},

            //8 OCCASIONAL_FRAME
            {HEADER_PAYLOAD,RT_HK_PAYLOAD,RT_RAGNAROK_PAYLOAD,WOD_HK_PAYLOAD,WOD_HK_PAYLOAD, WOD_HK_PAYLOAD,
                DIAGNOSTIC_PAYLOAD,FILLER_PAYLOAD,END_OF_PAYLOAD_LIST},

    };

#define memberOffset(type, member) (((uint32_t)(&(((type *)0)->member)))&0xFFFF)

    const uint16_t framePayloadOffsets[TYPES_OF_FRAME][MAX_PAYLOADS_PER_FRAME] = {
            { // REALTIME_MIN
                    memberOffset(realTimeMinFrame_t,header),
                    memberOffset(realTimeMinFrame_t,rtHealth),
                    memberOffset(realTimeMinFrame_t,rtRad),
                    memberOffset(realTimeMinFrame_t,rtRag),
                    memberOffset(realTimeMinFrame_t,minVals),
                    memberOffset(realTimeMinFrame_t,HKWod[0]),
                    memberOffset(realTimeMinFrame_t,radWod[0]),
                    memberOffset(realTimeMinFrame_t,ragWod[0]),
                    memberOffset(realTimeMinFrame_t,filler)
            },
            {//REALTIME_MAX
                    memberOffset(realTimeMaxFrame_t,header),
                    memberOffset(realTimeMaxFrame_t,rtHealth),
                    memberOffset(realTimeMaxFrame_t,rtRad),
                    memberOffset(realTimeMaxFrame_t,rtRag),
                    memberOffset(realTimeMaxFrame_t,maxVals),
                    memberOffset(realTimeMaxFrame_t,HKWod[0]),
                    memberOffset(realTimeMaxFrame_t,radWod[0]),
                    memberOffset(realTimeMaxFrame_t,ragWod[0]),
                    memberOffset(realTimeMaxFrame_t,filler)
            },
            {//WOD1
                    memberOffset(allWOD1Frame_t,header),
                    memberOffset(allWOD1Frame_t,HKWod[0]),
                    memberOffset(allWOD1Frame_t,HKWod[1]),
                    memberOffset(allWOD1Frame_t,HKWod[2]),
                    memberOffset(allWOD1Frame_t,radWod[0]),
                    memberOffset(allWOD1Frame_t,radWod[1]),
                    memberOffset(allWOD1Frame_t,ragWod[0]),
                    memberOffset(allWOD1Frame_t,ragWod[1]),
                    memberOffset(allWOD1Frame_t,filler)
            },
            {//WOD2
                    memberOffset(allWOD2Frame_t,header),
                    memberOffset(allWOD2Frame_t,HKWod[0]),
                    memberOffset(allWOD2Frame_t,HKWod[1]),
                    memberOffset(allWOD2Frame_t,radWod[0]),
                    memberOffset(allWOD2Frame_t,radWod[1]),
                    memberOffset(allWOD2Frame_t,radWod[2]),
                    memberOffset(allWOD2Frame_t,ragWod[0]),
                    memberOffset(allWOD2Frame_t,ragWod[1]),
                    memberOffset(allWOD2Frame_t,filler)
            },
            {//WOD3
                    memberOffset(allWOD3Frame_t,header),
                    memberOffset(allWOD3Frame_t,HKWod[0]),
                    memberOffset(allWOD3Frame_t,HKWod[1]),
                    memberOffset(allWOD3Frame_t,radWod[0]),
                    memberOffset(allWOD3Frame_t,radWod[1]),
                    memberOffset(allWOD3Frame_t,ragWod[0]),
                    memberOffset(allWOD3Frame_t,ragWod[1]),
                    memberOffset(allWOD3Frame_t,ragWod[2]),
                    memberOffset(allWOD3Frame_t,filler)
            },
            { // SAFE_DATA1
                     memberOffset(safeData1Frame_t,header),
                     memberOffset(safeData1Frame_t,rtHealth),
                     memberOffset(safeData1Frame_t,rtRag),
                     memberOffset(safeData1Frame_t,minVals),
                     memberOffset(safeData1Frame_t,maxVals),
                     memberOffset(safeData1Frame_t,HKWod[0]),
                     memberOffset(safeData1Frame_t,HKWod[1]),
                     memberOffset(safeData1Frame_t,ragWod[0]),
                     memberOffset(safeData1Frame_t,filler)
             },
             { // SAFE_DATA2
                     memberOffset(safeData2Frame_t,header),
                     memberOffset(safeData2Frame_t,rtHealth),
                     memberOffset(safeData2Frame_t,rtRag),
                     memberOffset(safeData2Frame_t,minVals),
                     memberOffset(safeData2Frame_t,maxVals),
                     memberOffset(safeData2Frame_t,HKWod[0]),
                     memberOffset(safeData2Frame_t,ragWod[0]),
                     memberOffset(safeData2Frame_t,ragWod[1]),
                     memberOffset(safeData2Frame_t,filler)
             },
             {//SAFE_WOD
                     memberOffset(safeWODFrame_t,header),
                     memberOffset(safeWODFrame_t,HKWod[0]),
                     memberOffset(safeWODFrame_t,HKWod[1]),
                     memberOffset(safeWODFrame_t,HKWod[2]),
                     memberOffset(safeWODFrame_t,ragWod[0]),
                     memberOffset(safeWODFrame_t,ragWod[1]),
                     memberOffset(safeWODFrame_t,ragWod[2]),
                     memberOffset(safeWODFrame_t,filler)
             },
            {//INFREQUENT
                    memberOffset(diagFrame_t,header),
                    memberOffset(diagFrame_t,rtHealth),
                    memberOffset(diagFrame_t,rtRag),
                    memberOffset(diagFrame_t,HKWod[0]),
                    memberOffset(diagFrame_t,HKWod[1]),
                    memberOffset(diagFrame_t,HKWod[2]),
                    memberOffset(diagFrame_t,diags),
                    memberOffset(diagFrame_t,filler)
            },

    };


    int i,plType;
    uint16_t plOffset;
    uint8_t *frameAddress;
    anyPayload_t *payload;
    frameAddress = (uint8_t *)frameToFill;
    //printf("Filling frame type %d\n",frameType);
    for(i=0;i<MAX_PAYLOADS_PER_FRAME;i++){
        plType = PayloadOrder[frameType][i];
        plOffset = framePayloadOffsets[frameType][i];
        //printf("Payload Type=%d, offset=%d\n",plType,plOffset);
        payload = (anyPayload_t *)(frameAddress + plOffset);
        if(plType<0)break;
        switch(plType){
        case HEADER_PAYLOAD:{
            //printf("Header\n");
            CreateHeader(frameType,(header_t *)payload);
            break;
        }
        case FILLER_PAYLOAD:{
            //printf("Filler=%d\n",sizeof(allFrames_t)-plOffset);
            memset(payload,0,sizeof(allFrames_t)-plOffset);
            break;
        }
        default:{
            CreatePayload(plType,payload);
            break;
        }
        }
    }
}

void GetTelemetryData(void){
    /*
     * This routine gets the next of the double buffers, gets telemetry data, and does the FEC
     * and run encoding into the double buffer used by the Audio routine.
     */

    /*
     * This array defines the order of low speed telemetry payloads within a given frame type
     */
#define END_OF_FRAME_LIST -1
    //todo: See if we want a command to switch to less Ragnarok and less Rad data
    static const int HealthModeFrameOrder[] = {
                                               INFREQUENT_FRAME,
                                               REALTIME_MIN_FRAME,
                                               ALL_WOD1_FRAME,
                                               ALL_WOD2_FRAME,
                                               ALL_WOD3_FRAME,
                                               PAUSE_FOR_RX,
                                               REALTIME_MAX_FRAME,
                                               ALL_WOD1_FRAME,
                                               ALL_WOD2_FRAME,
                                               ALL_WOD3_FRAME,
                                               PAUSE_FOR_RX,
                                               REALTIME_MIN_FRAME,
                                               ALL_WOD1_FRAME,
                                               ALL_WOD2_FRAME,
                                               ALL_WOD3_FRAME,
                                               PAUSE_FOR_RX,
                                               REALTIME_MAX_FRAME,
                                               ALL_WOD1_FRAME,
                                               ALL_WOD2_FRAME,
                                               ALL_WOD3_FRAME,
                                               PAUSE_FOR_RX,
                                               REALTIME_MIN_FRAME,
                                               ALL_WOD1_FRAME,
                                               ALL_WOD2_FRAME,
                                               ALL_WOD3_FRAME,
                                               PAUSE_FOR_RX,
                                               REALTIME_MAX_FRAME,
                                               ALL_WOD1_FRAME,
                                               ALL_WOD2_FRAME,
                                               ALL_WOD3_FRAME,
                                               PAUSE_FOR_RX,
                                               END_OF_FRAME_LIST
    };

    static const int SafeModeFrameOrder[] = {
                                             REALTIME_MIN_FRAME,
                                             REALTIME_MAX_FRAME,
                                             INFREQUENT_FRAME,
                                             END_OF_FRAME_LIST
    };
    static const int ScienceModeFrameOrder[] = {
                                                REALTIME_MIN_FRAME,
                                                ALL_WOD1_FRAME,
                                                ALL_WOD2_FRAME,
                                                ALL_WOD3_FRAME,
                                                PAUSE_FOR_RX,
                                                REALTIME_MAX_FRAME,
                                                ALL_WOD1_FRAME,
                                                ALL_WOD2_FRAME,
                                                ALL_WOD3_FRAME,
                                                PAUSE_FOR_RX,
                                                END_OF_FRAME_LIST
    };
    //todo: Science and safe mode frames need fixing.  Science gets what? ADAC? Safe does not get any rad

    allFrames_t frame;
    uint32_t *downlinkFrameToFill;
    uint8_t thisFrameType;
    bool workingOnLastFrame,needNewFrameNumber=true;
    /*
     * Here get the buffer and figure out if there is anything to do.
     *
     * It is important to remember that we are FILLING the buffers here
     * asynchronously to when we EMPTY to buffers in the audio task.  That's
     * because the audio task is time critical, where as we have maybe 3 or 4
     * seconds to fill a buffer and get it all fixed up with FEC and everything.
     * But that means that, for example, on a beacon which sends only 2 frames
     * we get called a third time to start filling a buffer, even though it never
     * gets used.  Hence what may seem like a quirky way to decide what type of
     * data to put in it.
     *
     */
    downlinkFrameToFill = GetFECBufferToFill();
    if(downlinkFrameToFill != (void *)NO_BUFFERS){
        while(needNewFrameNumber){
            /*
             * If 0, telemetry is turned off or full; don't get data.  Otherwise,
             * do the following:
             */
            int i,j=0;
            static int32_t line_state = 1,thisCRC32;
            uint8_t *thisCRCByte;
            thisCRCByte = (uint8_t *)&thisCRC32;
            uint8_t parities[CODE_WORDS_PER_FRAME][PARITY_BYTES_PER_CODEWORD],inputByte;

            /* Now figure out what frame type to send */

            if(InSafeMode){
                thisFrameType = SafeModeFrameOrder[SafeTelemTypeIndex++];
                if(SafeModeFrameOrder[SafeTelemTypeIndex] == END_OF_FRAME_LIST){
                    SafeTelemTypeIndex = 0;
                }
                if(safeFramesRemaining-- == 1){
                    workingOnLastFrame = true;
                } else {
                    workingOnLastFrame = false;
                }
            } else if(InScienceMode){
                workingOnLastFrame = false;
                thisFrameType = ScienceModeFrameOrder[ScienceTelemTypeIndex++];
                if(SafeModeFrameOrder[ScienceTelemTypeIndex] == END_OF_FRAME_LIST){
                    ScienceTelemTypeIndex = 0;
                }

            } else {
                /* Health telem just follows the telem order */
                workingOnLastFrame = false;
                thisFrameType = HealthModeFrameOrder[LowSpeedTelemTypeIndex++];
                if(HealthModeFrameOrder[LowSpeedTelemTypeIndex] == END_OF_FRAME_LIST){
                    LowSpeedTelemTypeIndex = 0;
                }
            }
            /*
             * Here we have the frame type in "thisFrameType" and we are creating a
             * frame by adding a header and then the payloads to it.
             */
            if(thisFrameType == PAUSE_FOR_RX){
                if(StandbyReceiverOk()){
                    // Assuming the other processor is doing Rx, we will not do it.  Go around the get frame
                    // type loop again to get another frame.
                    continue;
                } else {
                    // Otherwise, we report a null frame and "last frame" to radio task, which will stop transmitting
                    // at the end of the last frame we have, and send "telem done" which will turn on the receiver
                    // because of the state table.
                    ReportFrameReadyToSend(0,true); //0 length--no more telem; true=tell me when you have finished
                    //ax5043StopTx();
                    //ax5043StartRx();
                    ReportToWatchdog(ControlWD);
                    return;
                }
            } else {
                needNewFrameNumber=false;  // The frame we have is good enough
                //printf("Header size=%d, payload size=%d\n",sizeof(header_t),sizeof(anyPayload_t));
                CreateFrame(&frame,thisFrameType);
            }
            /*
             * Make sure that the timestamp is unique
             */
#if 0
            if((thisFrameType == -1)){ //todo:  We might want something like this for Ragnarok
                CreatePayload(CAN_FULL_FRAME7,&frame.payload[0]);
            } else
#endif
                thisCRC32=~0U; //Initialize the CRC
            /*
             * The following is as specified by Phil Karn for FEC.
             */
            if((frame.rtFrame1.header.resetCnt == 0) && (frame.rtFrame1.header.uptime == 0)){
                memset(parities,0x5a,sizeof(parities));
                //printf("Starting with bogus FEC\n");

            } else {
                memset(parities,0,sizeof(parities));
            }

            /*
             * Update the forward error correction blocks (parity blocks or codewords)
             * and then pack the data bytes into the buffer.  Remember we are alternating
             * which codeword the next data byte contributes to
             */
            j=0;
            Put10bInBuffer(
                    downlinkFrameToFill,
                    j++,
                    SYNC10B);

            for(i=0; i < LOW_SPEED_FRAME_SIZE-4; i++){ // Do not count the CRC
                uint16_t data;
                if(i%20==0)taskYIELD();
                inputByte = (((uint8_t *)&frame)[i]);

                //printf("0x%02x ", inputByte); //DEBUG

                data = encode_8b10b(&line_state,(int32_t)inputByte);
                thisCRC32 = crc32Single(thisCRC32,inputByte);
                update_rs(parities[(j-1)%CODE_WORDS_PER_FRAME],inputByte);
                Put10bInBuffer(
                        downlinkFrameToFill,
                        j++,
                        data);
            }

            // This allows us to try different frame sizes relatively easily.  Not required when we are done
            // deciding
#ifdef UNDEFINE_BEFORE_FLIGHT
            {
                int k;
                inputByte = 0;
                k=(TOTAL_FRAME_SIZE-4) - (j-1); // J is where we are now.  Subtract crc size (to come) sync (already in)
                for(i=0;i<k;i++){
                    uint16_t data;
                    data = encode_8b10b(&line_state,inputByte);
                    thisCRC32 = crc32Single(thisCRC32,inputByte);
                    update_rs(parities[(j-1)%CODE_WORDS_PER_FRAME],inputByte);
                    Put10bInBuffer(
                            downlinkFrameToFill,
                            j++,
                            data);


                }
            }
#endif
            thisCRC32 = thisCRC32 ^ ~0U; // Finish the CRC
            thisCRC32 = htotl(thisCRC32);  //Switch to telemetry endian if required

            for(i=0;i<sizeof(thisCRC32);i++){
                uint8_t thisByte = thisCRCByte[i];
                update_rs(parities[(j-1)%CODE_WORDS_PER_FRAME],thisByte);
                Put10bInBuffer(
                        downlinkFrameToFill,
                        j++,
                        encode_8b10b(&line_state,(int32_t)thisByte)
                );

            }

            for(i=0;i < LOW_SPEED_PARITY_SIZE;i++){ // Put the parity bytes in the buffer, interleaving the two blocks
                Put10bInBuffer(
                        downlinkFrameToFill,
                        j++,
                        encode_8b10b(&line_state,(int32_t)parities[0][i]));
                Put10bInBuffer(
                        downlinkFrameToFill,
                        j++,
                        encode_8b10b(&line_state,(int32_t)parities[1][i]));
                Put10bInBuffer(
                        downlinkFrameToFill,
                        j++,
                        encode_8b10b(&line_state,(int32_t)parities[2][i]));

                //printf("0x%02x ", parities[0][i]); //DEBUG
                //printf("0x%02x ", parities[1][i]); //DEBUG
                //printf("0x%02x ", parities[2][i]); //DEBUG


            }

            if(workingOnLastFrame){
                Put10bInBuffer(
                        downlinkFrameToFill,
                        j++,
                        SYNC10B);
            }
            ReportFrameReadyToSend(j,workingOnLastFrame); ////workingOnLastFrame is getting overwritten.  Stack size probs?
        }

        ReportToWatchdog(ControlWD);
    }

}
void SetTelemetryMode(TelemetryMode mode){
    /*
     * This is a bit tricky.  There are a number of transponder modes, each of which has an
     * associated boolean:
     *
     *     InSafeMode, InAutosafeMode (If we are in autosafe mode, we are also in safe mode.)
     *     InHealthMode, InScienceMode (full time telemetry of various frame types)
     *
     *     Safe and Autosafe also have bits in NVRAM.  Unlike the booleans, though, these bits in NVRAM are
     *     mutually exclusive.  That hopefully avoids the Fox bug where occasionally a reset can switch from
     *     autosafe to full safe.
     *
     *     If we go into housekeeping/health mode, the NV safe bits are cleared.  If we go into science/camera
     *     they are unchanged.  Thus if there is a reboot while in science/camera mode, we return to the mode we were
     *     in when science/camera started.
     *
     *     Note that we have separate "modes" for entering autosafe, eclipse safe,  and manual safe for the
     *     first time.  That will be when we write stuff in MRAM (or not).  We don't write it nor change the
     *     autoness of the safe mode normally.
     */
    if(IdleTimerActive) {
        xTimerStop(IdleTimer,WATCHDOG_MAX_WAIT_TIME);
        IdleTimerActive = false;
    }
    if(BeaconActive){
        xTimerStop(BeaconTimer,WATCHDOG_MAX_WAIT_TIME);
        BeaconActive=false;
    }
    if(InScienceMode){
        xTimerStop(ScienceTimer,WATCHDOG_MAX_WAIT_TIME);
        InScienceMode = false;
    }

    switch(mode){
    case ModeHealth:{
        InHealthMode = true;
        InSafeMode = false;
        InScienceMode = false;
        inAutoSafeMode = false;
        inCommandedSafeMode = false;
        inEclipseSafeMode = false;
        WriteMRAM2BoolState(StateCommandedSafeMode,false,StateAutoSafe,false);
        SetTransmitterNormalRfPower();
        break;
    }
    case ModeScience:{
        InSafeMode = false;
        InHealthMode = false;
        InScienceMode = true;
        ReturnToSafeFromScience = InSafeMode; // Remember where to go back to on timeout
        inAutoSafeMode = false;
        inCommandedSafeMode = false;
        inEclipseSafeMode = false;
        SetTransmitterNormalRfPower();
        break;
    }
    case ModeEclipseSafe:{
        /*
         * We have come here due to eclipse.  Eclipse is not saved in NVRam.  We'll find out again
         * as soon as we reboot!
         */
        if(!inCommandedSafeMode && !inAutoSafeMode){
            /*
             * We should not get here if we are in either of those modes, but just to check
             * if there is a race condition of some sort.  If these are not true, we should not
             * have to clear their bits in NVRam either.
             */
            InSafeMode = true;
            InHealthMode = false;
            InScienceMode = false;
            inAutoSafeMode = false;
            inEclipseSafeMode = true;
            inCommandedSafeMode = false;
            inEclipseSafeMode = true;
            SetTransmitterSafeRfPower();
        }
        break;
    }
    case ModeAutosafe:{
        InSafeMode = true;
        InHealthMode = false;
        InScienceMode = false;
        inAutoSafeMode = true;
        inCommandedSafeMode = false;
        inEclipseSafeMode = false;

        WriteMRAM2BoolState(StateAutoSafe,true,StateCommandedSafeMode,false);
        SetTransmitterSafeRfPower();
        break;

    }
    case ModeSafe:{
        InSafeMode = true;
        InHealthMode = false;
        InScienceMode = false;
        inAutoSafeMode = false;
        inCommandedSafeMode = true;
        inEclipseSafeMode = false;

        WriteMRAM2BoolState(StateAutoSafe,false,StateCommandedSafeMode,true);
        SetTransmitterSafeRfPower();
        break;
    }
    default:
        break;

    }
}
void StartContinuousDownlink(){
    /*
     * This routine is called by health or science to start the transmitter going as appropriate.
     * We may here or somewhere else choose the right downlink frame types.
     */

    /*
     * Send a tone to let FoxTelem find the frequency
     */
    AudioSetMixerSource(MixerTone);
    BeaconOn();
    vTaskDelay(BEACON_SILENCE);

    SetTelemetryFrameType(RestartStandardTelem);
    GetTelemetryData();               // Queue up the telemetry and restart with regular data order
    TransponderOn(); // Conditionally depending on how the variable is set
    AudioSetMixerSource(MixerData);
}
//todo: Fix to 8-bit voltage
uint8_t GetAutosafeEnterVoltage(){
    return EnterAutosafeVoltage;
}
uint8_t GetAutosafeExitVoltage(){
    return ExitAutosafeVoltage;
}

void SetAutosafeVoltages(void){
    EnterAutosafeVoltage = ReadMRAMEnterAutosafe();
    ExitAutosafeVoltage = ReadMRAMExitAutosafe();
    // Can not be initialized in "Error" since that is to early to read the MRAM
 }

DownlinkStates GetCurrentDownlinkState(void){
    return CurrentState;
}

/*
 * Here are the timer callback and other message-sending routines.  They all just
 * send messages to Downlink Control.  In general these are intended to be called by
 * different tasks even though they are in the same code module with DownlinkControl.
 */
void SendEclipseSafeModeMsg(void){
    Intertask_Message message;
    bool status;
    message.MsgType = DwnlnkEnterEclipseSafeMode;
    status = NotifyInterTask(ToDownlinkControl, 0, &message);
    if(!status) {
        ReportError(ControlQueueOverflow,true,CharString,(int)"Eclipse SafeMode");
    }
}
void SendSafeModeMsg(bool autoSafe){
    Intertask_Message message;
    bool status;
    if(autoSafe){
        message.MsgType = DwnlnkEnterAutoSafeMode;
    } else {
        message.MsgType = DwnlnkEnterSafeMode;
    }
    status = NotifyInterTask(ToDownlinkControl, 0, &message);
    if(!status) {
        ReportError(ControlQueueOverflow,true,CharString,(int)"SafeMode");
    }
}
void SendHealthModeMsg(){
    Intertask_Message message={DwnlnkEnterHealthMode,0};
    bool status;
    //debug_print("Send transponder msg\n\r");
    status = NotifyInterTask(ToDownlinkControl, 0, &message);
    if(!status) {
        ReportError(ControlQueueOverflow,true,CharString,(int)"HealthMode");
    }
}
void SendBeaconModeMsg(){
    Intertask_Message message;
    bool status;
    message.MsgType = DwnlnkBeaconTimeout;
    status = NotifyInterTask(ToDownlinkControl, 0, &message);
    if(!status) {
        ReportError(ControlQueueOverflow,true,CharString,(int)"BeaconTimer");
    }
}
void SendScienceModeMsg(int minutes){
    Intertask_Message message;
    portTickType timeout;
    debug_print("Change science mode minutes to %d\n\r",minutes);
    timeout = minutes * MINUTES(1);
    xTimerChangePeriod( ScienceTimer,
                        timeout,
                        WATCHDOG_MAX_WAIT_TIME);
    message.MsgType = DwnlnkEnterScienceMode;
    while(NotifyInterTask(ToDownlinkControl,RETRY_TIMEOUT, &message)!=pdTRUE){};

}
void CommandInhibitTransmitting(void){
    /*
     * This is in case we get an official order from the FCC to cease transmitting.  The totally hardwired
     * way to do this is to send a hardware command 0x0A, i.e. with bits 3 and 1 set.  This will power off
     * both the legacy RxTx, the SDR, and the RTIHU, including the DCTs.  But we would much rather send just
     * bit 3 (i.e. 8) which turns of the legacy RxTx and SDR by hardwire, but send this command here so that
     * we can turn off only the DCT that is currently being used to transmit.  That we we leave on the DCT
     * that is currently used to receive.
     */
    Intertask_Message message;
    WriteMRAMBoolState(StateTransmitInhibit,true); // This is if the FCC orders a shutdown
    message.MsgType = DwnlnkInhibitTransmit;
    while(NotifyInterTask(ToDownlinkControl,RETRY_TIMEOUT, &message)!=pdTRUE){};
}
void CommandResumeTransmitting(void){
    Intertask_Message message;
    WriteMRAMBoolState(StateTransmitInhibit,false); // FCC lifts the shutdown order
    message.MsgType = DwnlnkEnableTransmit;
    while(NotifyInterTask(ToDownlinkControl,RETRY_TIMEOUT, &message)!=pdTRUE){};
}
void IdleTimerCallback ( xTimerHandle xTimer ) {
    Intertask_Message message;
    bool status;
    message.MsgType = DwnlnkIdleTimeout;
    status = NotifyInterTask(ToDownlinkControl, 0, &message);
    if(!status) {
        ReportError(ControlQueueOverflow,true,CharString,(int)"IdleTimer");
    }
}

void ScienceModeTimerCallback(xTimerHandle xTimer){

    Intertask_Message message;
    bool status;
    if(!InScienceMode)return; // Just in case
    if(ReturnToSafeFromScience){
        message.MsgType = DwnlnkEnterSafeMode;
    } else {
        message.MsgType = DwnlnkEnterHealthMode;
    }
    ReturnToSafeFromScience=false;
    status = NotifyInterTask(ToDownlinkControl, 0, &message);
    if(!status) {
        ReportError(ControlQueueOverflow,true,CharString,(int)"ScienceTimer");
    }
}

void BeaconTimerCallback ( xTimerHandle xTimer ) {
    Intertask_Message message;
    bool status;
    message.MsgType = DwnlnkBeaconTimeout;
    status = NotifyInterTask(ToDownlinkControl, 0, &message);
    if(!status) {
        ReportError(ControlQueueOverflow,true,CharString,(int)"BeaconTimer");
    }
}
void EnableTransponder(bool toOn){
    /*
     * This is a ground (or typed) command that sets a non-volatile bit to allow the transponder to come
     * on whenever the mode is right (i.e. it is not in safe mode)
     */
    transponderEnabled = toOn;
    WriteMRAMBoolState(StateTransponderEnabled,toOn);
    if(toOn  && !InSafeMode){
        TransponderOn();
    } else {
        TransponderOff();
    }

}
void InhibitTransponder(bool inhibit){
    /*
     * Inhibit transponder comes from eclipse entry and is volatile.  It temporarily prevents the
     * transponder from being turned on under any circumstances.  It is always false on boot.
     */
    transponderInhibited = inhibit;
    if(inhibit)TransponderOff();
    else if(transponderEnabled && !InSafeMode){
        TransponderOn();
    }
}
void TransponderOn(void){
    /*
     * Turn on the transmitter, assuming that we are not getting powered by the umbilical.
     */
    if (IsTxAllowedOn() && transponderEnabled && !transponderInhibited){
        GPIOSetOn(PBEnable);
        TransponderTurnedOn=true;
    } else {
        GPIOSetOff(PBEnable); /* Just make double sure */
    }
}
void TransponderOff(void){
    /*
     * Turn off the transponder
     */

    GPIOSetOff(PBEnable);
    TransponderTurnedOn = false;
}
void DisableTelemetry(bool disable){
    /*
     * This is used for testing only -- it allows the console task to set up the transmit something else
     */
    DoNotSendTelemetry = disable;
}
void BeaconOn(void){
    /*
     * Turn on the transmitter, assuming that we are not getting powered by the umbilical.
     */
    if (IsTxAllowedOn()){
        uint32_t rfPower;
        rfPower = ReadMRAMDCTDriveLowPower();
        ax5043StartTx();
        ax5043_set_power(rfPower);
        GPIOSetOn(PAPower);
    } else {
        ax5043StopTx(); //Also stops PA
    }
    if(TelemetryActive == TELEMETRY_ACTIVE_FALSE)
        TelemetryActive = getSeconds(); /* Remember when we turned it on */
}
void BeaconOff(void){
    /*
     * Turn off the beacon
     */
    ax5043StopTx(); // Also stops the PA
    TelemetryActive = 0; /* Not on */
}
bool TransponderIsOn(void){
    return TransponderTurnedOn;
}

bool TransmitterIsOnAndStable(void){
    /*
     * On this particular satellite, the transmitter is on iff
     * the telemetry is on.  We could turn on the passband without
     * the telemetry, but we don't do that here.
     *
     * We make sure we do not say it is on for 5 seconds since when it first comes
     * on it is sending a sine wave which gives an unreasonable power value.
     */
    bool retVal;
#define TRANSMIT_STABLE_TIME 5 /* How many seconds before we are willing to read the power*/
    if(TelemetryActive == TELEMETRY_ACTIVE_FALSE)retVal = false; //It is not active
    else if(TelemetryActive == TELEMETRY_ACTIVE_TRUE) retVal = true; // It was already determined to be stable
    else if((getSeconds() - TelemetryActive)>TRANSMIT_STABLE_TIME){
        TelemetryActive = TELEMETRY_ACTIVE_TRUE; // It was not stable before but it is now
        retVal = true;
    } else
        retVal = false; // It is not yet stable
    return retVal;
}
bool GetSafeRfPowerLevel(void){
    return safeModeHighPower;
}
bool GetNormalRfPowerLevel(void){
    return normalModeHighPower;
}
void SetDCTDriveValue(uint16_t lowRegValue, uint16_t highRegValue){
    if(highRegValue != 0){
        if(highRegValue == 0xFFFF)
            highRegValue = DCT_DEFAULT_HIGH_POWER;
        WriteMRAMDCTDriveHighPower(highRegValue);
    }
    if(lowRegValue != 0){
        if(lowRegValue == 0xFFFF)
            lowRegValue = DCT_DEFAULT_LOW_POWER;
        WriteMRAMDCTDriveLowPower(lowRegValue);
    }
    if(InSafeMode)SetTransmitterSafeRfPower();
    else SetTransmitterNormalRfPower();
}
void SetSafeRfPowerLevel(bool isHigh){
    safeModeHighPower = isHigh;
    WriteMRAMBoolState(StateSafeRfPowerLevel,isHigh);
    if(InSafeMode)SetTransmitterSafeRfPower();
}
void SetNormalRfPowerLevel(bool isHigh){
    normalModeHighPower = isHigh;
    WriteMRAMBoolState(StateNormalRfPowerLevel,isHigh);
    if(!InSafeMode)SetTransmitterNormalRfPower();
}
static void SetTransmitterSafeRfPower(void){
    uint16_t power;
    if(safeModeHighPower){
        power = (uint16_t)ReadMRAMDCTDriveHighPower();
        ax5043_set_power(power);
    } else {
        power = (uint16_t)ReadMRAMDCTDriveLowPower();
        ax5043_set_power(power);
    }
}
static void SetTransmitterNormalRfPower(void){
    uint16_t power;
    if(normalModeHighPower){
        power = (uint16_t)ReadMRAMDCTDriveHighPower();
        ax5043_set_power(power);
    } else {
        power = (uint16_t)ReadMRAMDCTDriveLowPower();
        ax5043_set_power(power);
    }
}
static OverrideTxState(bool override, bool forceOff){
    /*
     * This is part of what used to be "override Umbilical".  But now there are two
     * override states, the old override Umbilical, and forceOff.  With override
     * umbilical true, the transmitter is allowed to turn on even if the umbilical
     * is plugged in.  With forceOff true, the transmitter will not come on regardless
     * of the umbilical state.
     *
     * This is a single routine to set both so we only have a single place to send
     * the message from.  To change only one state, call one of the two routines
     * below
     */
    Intertask_Message message;
    overrideUmbilical = override;
    forceTxOff = forceOff;
    DisableTelemetry(false); // Otherwise we won't see the message being sent below
    message.MsgType = DwnlnkControlChange;
    NotifyInterTask(ToDownlinkControl, 0, &message);
}
void OverrideUmbilical(bool override){
    /*
     * True means do not let the umbilical being plugged in prevent
     * us from transmitting.
     */
    OverrideTxState(override,forceTxOff);
}
void ForceTransmittersOff(bool forceOff){
    /*
     * This is pretty much the opposite of Override Umbilical in
     * the sense that if the umbilical state allows transmit, this
     * disallows it.
     */
    OverrideTxState(overrideUmbilical,forceOff);
}
/**************************************************************************************
 * This is all about 1/2 duplex (meaning we are the active processor and the standby
 * processor (which normally receives commands) is dead.  We switch to receive on the
 * processor for a limited time and then the timer calls this routine to go back to
 * transmit telemetry.
 ***************************************************************************************/

bool StartReceiveTimer(bool restart){
    // This routine can be used either to start the timer if it is stopped or to reset the expire
    // time if, for example, a command comes in.
    //
    // If restart is true, we can only do the restart if the timer is running.  If restart is
    // false, we can do it in any case.
    //printf("Start Rx Timer: restart=%d, isRunning=%d...",restart,halfDuplexTimerRunning);
    if(halfDuplexTimerRunning || !restart){
        if (xTimerReset(halfDuplexRxTimer, WATCHDOG_MAX_WAIT_TIME) == pdTRUE){
            halfDuplexTimerRunning = true;
            //printf("Timer started\n");
            return true;
        } else {
            //printf("Return false because timer (%x) did not start\n",halfDuplexRxTimer);
            return false;
        }
    }
    //printf("Return false because restart requested but not running\n");
    return false;
}

static void HalfDupRxTimerCallback(TimerHandle_t x){
    Intertask_Message message;
    halfDuplexTimerRunning=false;
    // Time to turn the Rx off and continue transmitting
    DisableTelemetry(false); // Otherwise we won't see the message being sent below
    message.MsgType = DwnlnkControlChange;
    NotifyInterTask(ToDownlinkControl, 0, &message);
}

/*********************************************************************************
 * Here are a bunch of routines that just allow one to find out various states
 *********************************************************************************/
bool IsTxAllowedOn(void){
    if(forceTxOff)return false;
    if(overrideUmbilical)return true;
    else return !GPIOIsOn(UmbilicalAttached);
}
bool IsTransponderEnabled(void){
    return transponderEnabled;
}
bool InCommandedSafeMode(void){
    return inCommandedSafeMode;
}
bool IsAutoSafeAllowed(void){
    return allowAutoSafeMode;
}
void SetAllowAutoSafe(bool allow){
    allowAutoSafeMode = allow;
    WriteMRAMBoolState(StateAutoSafeAllow,allow);
    if(!allow && inAutoSafeMode){
        // If we disallow autosafe while we are IN autosafe, then get out ot safe mode.
        // Remember autosafe and commandSafe a mutually exclusive.
        WriteMRAMBoolState(StateAutoSafe,false);
        SendHealthModeMsg();
    }
}
bool InAutoSafeMode(void){
    return inAutoSafeMode;
}
bool InEclipseSafeMode(void){
    return inEclipseSafeMode;
}
bool StandbyReceiverOk(void){
    bool can2Ok = Can2PartnerIsOk();
    int16_t RSSI = 0;//GetStandbyRSSI();
    return can2Ok && (RSSI > MIN_RSSI_OK);
}
