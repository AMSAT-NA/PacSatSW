/* AMSAT Fox-1
 Mission Elapsed Time (MET) Interface
 12/16/2012 Mike McCann KB2GHZ
 1/18/2014 Semaphore Synchronization with the Telemetry Collection task added
*/

#include <nonvolManagement.h>
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "os_semphr.h"
#include "os_task.h"
#include "os_timer.h"
#include "MET.h"
#include "MRAMmap.h"
#include "nonvol.h"
#include <stdint.h>
#include "errors.h"
#include "gpioDriver.h"
#include "CommandTask.h"
#include "watchdogSupport.h"

extern bool SimDoppler;
static bool TelemetryReady = false;
static Intertask_Message telemMsg;

//static int timeType;



static uint32_t METcount = 0;  /* 1 second time periods since the last IHU start */
static uint32_t stableStart = 0xffffffff;
static portTickType TicksAtLastSecond = 0; /* The tick count from FreeRTOS when when METcount was incremented. */
                                              /* This allows us to offer a high resolution time. */
/* IHU reset event counter's current value */
static uint16_t IHUreset;
static uint32_t clockPhase = 0;

/* Here is the telemetry timestamp */
static uint16_t timestampEpoch=0;
static uint32_t timestampSeconds;

/* Here is the current number of seconds in orbit */
static uint32_t secondsInOrbit;

/* Here is the current Unix time */
static uint32_t unixTime;

/*
 * Here are timestamps and callbacks for things that must be
 * tested against the time in orbit
 */

static void METupdate(xTimerHandle x);

/*
 * This module deals with most aspects of the system's time.  The main time is a combination of
 * the reset count and seconds since the last reset.  So in general when we say "time" that is what
 * we mean.  There is also "timeStamp" which is used for telemetry, including the downlink packet,
 * the min/max change timestamp, and the WOD timestamp.  This value is coordinated among all the processors
 * so that as a whole the satellite timestamp always increases.  The timestamp consists of an "Epoch"
 * (rather than a reset count) and seconds since the Epoch.  The Epoch changes when there is an event
 * that might cause one of the IHUs to loose track, for example a reset, or a change of control. And
 * finally there is "seconds in orbit", which is a kind-of absolute time reference that can be used
 * with various type of scheduled events.  Seconds in orbit is also coordinated among all the processors.
 */


/*
 * Initialization-related routines for this module
 */
void initMET() {
    xTimerHandle handle;
    volatile portBASE_TYPE timerStatus;
    int pvtID = 0;

    /* create a RTOS software timer - 1 second period */
    handle = xTimerCreate( "MET", SECONDS(1), TRUE, &pvtID, METupdate);
    /* start the timer */
    timerStatus = xTimerStart(handle, 0);
    if (timerStatus != pdPASS){
        ReportError(RTOSfailure, FALSE, ReturnAddr, (int) initMET); /* failed to create the RTOS timer */
    }

    ResetAllWatchdogs(); // We have spent some time; better make sure the WDs are happy

    /* update the IHU reset event-counter */
    IHUreset = ReadMRAMResets();
    WriteMRAMResets(IHUreset+1);

    /*
     * Now update the telemetry timestamp counter.  See UpdateTelemEpoch for description,
     * but note that this is a default if we are the only one up.  Otherwise this will be
     * updated by Coordination when we hear from someone else.
     */
    UpdateTelemEpoch(ReadMRAMTimestampResets()+1);

    /* And get the time on orbit */
    secondsInOrbit = ReadMRAMSecondsOnOrbit();
    telemMsg.MsgType = TacCollectMsg; // Just set up this message for later use
    ResetAllWatchdogs();
}

void initSecondsInOrbit(void){ // Called only from preflight init
    secondsInOrbit = 0;
    WriteMRAMSecondsOnOrbit(secondsInOrbit);
}
void InitResetCnt() {
    /* update the IHU reset event-counter */
    WriteMRAMResets(0); //Start the reset count at 0
    timestampEpoch = 0;
    WriteMRAMTimestampResets(0);
}

void METTelemetryReady() {  /* Ok to start sending telemetry messages */
    TelemetryReady = true;
}
void StartStableCount(void){
    /*
     * Start counting stable time
     */
    stableStart = METcount;
}

bool IsStabilizedAfterBoot(){
    return (((int)METcount-(int)stableStart) > MET_STABLE_TIME);
}

/*
 * Here is the actual major function of this module.  It gets called by a FreeRTOS Timer every second
 */

static void METupdate(xTimerHandle x) {
#ifndef LAUNCHPAD_HARDWARE
#ifdef DEBUG
    //static int LEDnum=0;
    //static Gpio_Use led=LED1;
    //const Gpio_Use whichLED[]={LED1,LED2,LED3,NumberOfGPIOs};
    //GPIOToggle(led);
    //Change to blink LED3 only   N5BRG 240516
    //led = whichLED[++LEDnum];
    //if(led==NumberOfGPIOs){LEDnum=0;led=LED1;}
    GPIOToggle(LED3); //240521 N5BRG

#endif
#endif
    METcount++;
    timestampSeconds++;
    secondsInOrbit++;

    unixTime++; // TODO - this is a quick implementation before we have a real time clock
    TicksAtLastSecond = xTaskGetTickCount();
    if(METcount > MET_STABLE_TIME*2) {  // No longer a short boot.
        ClearShortBootFlag();
    }

    /*
     * This section kicks the telemetry update to collect data.
     */
    if((clockPhase & TELEM_COLLECT_MASK)==0) {
        //uint32_t freq;
        /* This is every 4 seconds */
        if(TelemetryReady){//TelemCollect
            NotifyInterTaskFromISR(ToTelemetryAndControl,&telemMsg);
        }
    }
    clockPhase++;  // It's ok if this wraps.  We only care about the bottom few bits
}

/**
 * This returns the time based on the Unix epoch, or seconds since
 * Jan 1 1970 UTC.
 *
 */
uint32_t getUnixTime(void) {
    return unixTime;
}

void setUnixTime(uint32_t time_in_seconds) {
    unixTime = time_in_seconds;
}

/* These are to get the time for the individual satellite */
void getTime(logicalTime_t *timeRecord) {
	timeRecord->IHUresetCnt = IHUreset;
	timeRecord->METcount = METcount;
}
uint16_t getResets(void){
    return IHUreset;
}
uint32_t getSeconds(void){
    return METcount;
}
/*
 * This is currently not used, but it can get the time in hundredths of a second.  I'm
 * not sure this makes sense (we should subtract ticksatlastsecond from current ticks?
 */

void getHighResolutiontime(highResolutionTime_t *hrt) {
    getTime(&(hrt->LogicalTime));
    hrt->TicksAtLastSecond = TicksAtLastSecond;
}

/*
 * This is to get the coordinated timestamp for the entire satellite.  Coordination keeps track
 * of whether it is synchronized, and a routine TimeIsSynchronized lets a user find out.
 */

void getTimestamp(logicalTime_t *timeRecord) {
    timeRecord->IHUresetCnt = timestampEpoch;
    timeRecord->METcount = timestampSeconds;
}
 uint16_t getEpoch(void){
     return timestampEpoch;
}

/*
 * Here we get a constantly increasing number indicating the (approximate) number of seconds since we booted up
 * the first time in orbit.
 */

uint32_t getSecondsInOrbit(void){
    return secondsInOrbit;
}

void SaveSecondsInOrbit(void){
    // This gets called every few seconds (from a task, not from an interrupt routine like
    // UpdateMET;  Do not update if timestamp epoch is 0.  That means we just did preflight init
    //(!preflightInitInhibit && TimeInOrbitSynchronized()){
    //  WriteMRAMSecondsOnOrbit(secondsInOrbit);

}



void UpdateTelemEpoch(uint16 resets){
    /*
     * This always gets called on a reboot with the reset/epoch number 1 greater than
     * the last one that we knew about.  If we start out immediately as the in-control
     * cpu, that newly incremented epoch is that one that gets used since the in-control
     * cpu is the time master.
     *
     * Only the time-master sends (or at least only the non-masters pay attention to) timestamp
     * updates.  So that is why the epoch changes for the whole satellite.  OTOH, if we are NOT
     * the master, then we will soon get an update from the actual master
     *
     * Update the MRAM if resets have changed or if we are right at the start of
     * a boot (when resets have not yet been initted)
     */
    {
        if (resets != timestampEpoch){
            timestampEpoch = resets;
            WriteMRAMTimestampResets(timestampEpoch);
            timestampSeconds = 0;
        }
    }
}




