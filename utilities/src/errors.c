
/*
 * errors.c
 *
 *  Created on: Mar 7, 2019
 *      Author: burns
 */
#define ERRORS_C /* This is the actual errors routine */

#include <pacsat.h>
#include "stdio.h"
#include "errors.h"
#include "TMS570Hardware.h"
#include "downlink.h"

resetMemory_t SaveAcrossReset; // These will not be defined in errors.h
bool ErrorInProgress = false;

/*
 * errors.c
 *
 *  Created on: Mar 18, 2013
 *  Updated to Golf, Nov 30, 2019
 *      Author: WB1FJ
 */


#include <pacsat.h>
#include "config.h"
#include "stdio.h"
#include "watchdogSupport.h"
#include "FreeRTOS.h"
#include "os_task.h"
#include "gpioDriver.h"
#include "errors.h"
#include <nonvolManagement.h>

#ifdef UNDEFINE_BEFORE_FLIGHT
#define DEBUG_AIDS
#endif

/* Debugging error messages */
#ifdef DEBUG_AIDS
char *ErrMsg[EndOfErrors]={
                            " Unspecified  "
                           ,"  PowerCycle  "
                           ," Int Watchdog "
                           ,"Software Reset"
                           ,"External Reset"
                           ,"Oscillator Failure"
                           ,"StackOverflow"
                           ,"NMI Exception"
                           ,"SPIInUse"
                           ,"SPIOperationTimeout"
                           ,"SPIMramTimeout"
                           ,"UnexpectedBehavior"
                           ,"SemaphoreFail"
                           ,"USARTError"
                           ,"DMAInUseTimeout"
                           ,"Illegal GPIO Output"
                           ,"Illegal GPIO Input"
                           ,"Illegal GPIO Wait"
                           ,"MRAMcrc"
                           ,"MRAMread"
                           ,"MRAMwrite"
                           ,"RTOS failure"
                           ,"I2C In Use"
                           ,"I2C1 failure"
                           ,"I2C2 failure"
                           ,"ControlQueueOverflow"
                           ,"ControlTimerNotStarted"
                           ,"Coordination Timer Not Started"
                           ,"CAN write timeout"
                           ,"Experiment Failure"
                           ,"Debug Startup "
                           ,"TX dropped packet"
                           ,"RX dropped packet"
};
char *LIHUErrMsg[EndOfErrors]={
                                " Int Watchdog "
                               ,"  PowerCycle  "
                               ,"Software Reset"
                               ,"StackOverflow "
                               ,"   MCUFault   "
                               ," USBHighPrio  "
                               ,"  SPIInUse    "
                               ,"SPIOperationTimeout"
                               ,"SPIMramTimeout"
                               ,"UnexpectedBehavior"
                               ,"SemaphoreFail "
                               ,"USARTError"
                               ,"DMAInUseTimeout"
                               ,"IllegalGPIOOutput"
                               ,"IllegalGPIOInput"
                               ,"IllegalGPIOWait"
                               ,"MRAMcrc"
                               ,"MRAMread"
                               ,"MRAMwrite"
                               ,"RTOSfailure"
                               ,"ADCTimeout"
                               ,"ADCDACSync"
                               ,"I2C1failure"
                               ,"I2C2failure"
                               ,"ControlQueueOverflow"
                               ,"ControlTimerNotStarted"
                               ,"FlashCRCfaulty"
                               ," I2C1 In Use  "
                               ," I2C2 In Use  "
                               ,"ExperimentFailure"
                               ,"IHUStateChange"
};

/* These task names need to correspond to the id returned by xTaskGetApplicationTaskTag(0)
 * These need to be in the same order as the WdReporters_t enum in watchdogSupport.h
 * as that is the enum used to set the task id */
char *TaskNames[]={
                   "Unspecified",
                   "Telemetry & Control",
                   "Rx",
                   "Tx",
                   "Ax25",
                   "Uplink",
                   "PB",
                   "Command",
                   "CAN Support",
                   "Idle",
                   "Interrupt",
                   "Console"
};
#endif


// Error Condition Event-Counter

extern rt1Errors_t localErrorCollection; // Here is where we collect the errors

/*
 * Debugging Macros and Structures for errors and unexpected interrupts
 */

#ifdef DEBUG_AIDS

volatile DebugTaskHandle_t *stackInfo;
void * __builtin_return_address (unsigned int level);


#endif


/*
 * Here are the interrupt service routines and error hooks
 */

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed portCHAR *pcTaskName ){
#ifdef DEBUG_AIDS
    stackInfo = (struct _stack *)pxTask;
#endif
    ReportError(StackOverflow,TRUE,CharString,(int)pcTaskName);
}

/*
 * And here are routines that are called externally
 */
static bool OKToWriteSaveAcrossReset = false;
void InitErrors(void){
    /*
     * First job is to save in a downlink packet what was saved in memory across the
     * last reset.
     *
     * LocalErrorCollection is what we will downlink.  Set to 0 initially, then set the
     * fields that relate to the previous boot.  If it was a power cycle, then nothing in
     * the previous boot is valid except the power cycle code itself so make sure both
     * saveAcrossReset and localErrorCollection is cleared.
     */
    memset(&localErrorCollection,0,sizeof(localErrorCollection));
    if(SaveAcrossReset.fields.errorCode == PowerCycle){
        localErrorCollection.errorCode = PowerCycle;
    } else {
        localErrorCollection.earlyResetCount = SaveAcrossReset.fields.earlyResetCount;
        localErrorCollection.wasStillEarlyInBoot = SaveAcrossReset.fields.wasStillEarlyInBoot;
        localErrorCollection.previousTask = SaveAcrossReset.fields.previousTask;
        localErrorCollection.taskNumber = SaveAcrossReset.fields.taskNumber;
        localErrorCollection.errorCode = SaveAcrossReset.fields.errorCode;
        localErrorCollection.wdReports = SaveAcrossReset.fields.wdReports;
        localErrorCollection.errorData = SaveAcrossReset.fields.errorData;
    }
    /*
     * Now we can initialize the memory that will be saved during the next reset.
     * Zero it all and then restore the couple things we want to be there.
     */
    memset(&SaveAcrossReset,0,sizeof(SaveAcrossReset));
    SaveAcrossReset.fields.errorCode = Unspecified; //Nothing specified yet
    SaveAcrossReset.fields.earlyResetCount = localErrorCollection.earlyResetCount; //We saved this above
    SaveAcrossReset.fields.wasStillEarlyInBoot = true; //So it will be there on the next reset
    OKToWriteSaveAcrossReset = true;

    /*
     * Here either SaveAcross memory has not changed, or local error collection has been zeroed anyway.
     * Check for whether we know that the last "up" was really short.  If so count it,
     * and if too many, we power cycle with the external watchdog.
     */
    if (localErrorCollection.wasStillEarlyInBoot){
        /*
         * Remember at this point we are still looking at info about the LAST boot.
         * So this means that whatever caused us to reboot and get here happened
         * soon after the boot before that.  The info was copied to localErrorCollection
         * but we want to keep count of early resets so they will be here on the next boot.
         */
        SaveAcrossReset.fields.earlyResetCount++;  // Increment the early reset count
        if (SaveAcrossReset.fields.earlyResetCount > RESETS_BEFORE_POWER_CYCLE){
            //If too many quick reboots, we want to power cycle.
#ifdef WATCHDOG_ENABLE
            ForceExternalWatchdogTrigger();
#else
            SaveAcrossReset.fields.earlyResetCount = 0; // This is only for debug with wd disabled
            ProcessorReset();
#endif
        }
    }

    // Init some more things in the diagnostic packet


    localErrorCollection.SWVersion[0] = VERSION[0];
    localErrorCollection.SWVersion[1] = VERSION[1];
}

void ReportError(ErrorType_t code, bool fatal, ErrorInfoType_t infoType, int info)
{

#define I2C_RETRY_RESET_MASK 0x3 /*Must be (power of 2) - 1 */
#define SOFT_ERROR_FATAL_COUNT 0x0f

    if(!fatal){
        uint8_t nonfatalCount;
        localErrorCollection.nonFatalCnt++;
        switch(code){
        case I2C1failure:{
            // I2C1 is for the radiation experiment
            nonfatalCount = localErrorCollection.I2C1RetryCnt;
            if((localErrorCollection.I2C1RetryCnt & I2C_RETRY_RESET_MASK) == I2C_RETRY_RESET_MASK){
                //todo: What to do here? I2cInit(I2c1Port); /* Restart after too many errors */
            }
            break;
        }
        case I2C2failure:{
            nonfatalCount = localErrorCollection.I2C2RetryCnt++;
            if((localErrorCollection.I2C2RetryCnt & I2C_RETRY_RESET_MASK) == I2C_RETRY_RESET_MASK){
                //todo: What to do here? I2cInit(I2c2Port); /* Restart after too many errors */
            }
            break;
        }
        case MRAMcrc:
            nonfatalCount = localErrorCollection.MramCRCCnt++; /* This it for downlinking */
            break;
        case MRAMread:
            nonfatalCount = localErrorCollection.MramRdErrorCnt++; /* This it for downlinking */
            break;
        case MRAMwrite:
            nonfatalCount = localErrorCollection.MramWtErrorCnt++; /* This it for downlinking */
            break;
        case TxPacketDropped:
            nonfatalCount = localErrorCollection.TxDroppedPkts++; /* This it for downlinking */
            break;
        default:
            nonfatalCount = 0;
            break;
        }

        /*
         * If we count non-fatal errors exceeding the limit, reboot the system by reporting
         * the same error but making it fatal.
         */
        if(nonfatalCount == SOFT_ERROR_FATAL_COUNT){
            ReportError(code,true,infoType,info);
        }
    } else {
        /* Save the reason we are going to crash */
        SaveAcrossReset.fields.errorCode = code;
        SaveAcrossReset.fields.taskNumber = (int)xTaskGetApplicationTaskTag(0);
        SaveAcrossReset.fields.errorData = info; /* Get bottom 8 bits of info */
    }

#ifdef DEBUG_AIDS
    printf("\r\n\n!!!!Error %d reported from task %s;'%s'\n\r",
           code,
           TaskNames[(int)xTaskGetApplicationTaskTag(0)],
           ErrMsg[code]   );
    //#pragma GCC diagnostic push
    //#pragma GCC diagnostic ignored "-Wswitch" // Do not report that I have not used EndOfErrorTypes
    switch (infoType){
    //#pragma GCC diagnostic pop
    case PortNumber:
        printf("Error reporter gave port number %d\n",info);
        break;
    case CharString:
        printf("Error reporter gave character string '%s'\n", (char *)info);
        break;
    case ReturnAddr:
        printf("Error reporter said its return address as %x\n",info);
        break;
    case ErrorBits:
        printf("Error value reported is %x\n",info);
        break;
    case TaskNumber:
        printf("Task reported is %s\n",TaskNames[info]);
    }
     if(fatal){
        printf("Fatal error, rebooting...\n\r");
#ifdef ENABLE_WATCHDOG
        while(--time){
            ResetAllWatchdogs();
            vTaskDelay(3);
        }
#else
        vTaskDelay(100);
        ProcessorReset();
#endif
    }
#endif

    if(fatal){
        /* Oops.  About to die.  Do we need to do anything?  e.g. Switch to direct link Rx->Tx */
        ProcessorReset();
    }
    else {
        ReportToWatchdog(CurrentTaskWD);
    }
}
#ifdef UNDEFINE_BEFORE_FLIGHT
char * ErrorMessageString(ErrorType_t code){
    return ErrMsg[code];
}
#endif
void ClearShortBootFlag(){
    // This gets called after we have been up long enough that
    // we don't count this as a short boot, (which forces a power
    // cycle if it happens enough
    SaveAcrossReset.fields.wasStillEarlyInBoot = false;
    SaveAcrossReset.fields.earlyResetCount = 0;
}
void RecordNewTask(uint32_t task){
    if(OKToWriteSaveAcrossReset){
//        bool saveShort = SaveAcrossReset.fields.wasStillEarlyInBoot;
//        uint8_t saveShortCount = SaveAcrossReset.fields.earlyResetCount;
        SaveAcrossReset.fields.previousTask = SaveAcrossReset.fields.taskNumber;
        SaveAcrossReset.fields.taskNumber = task;
//        SaveAcrossReset.fields.wasStillEarlyInBoot = saveShort;
//        SaveAcrossReset.fields.earlyResetCount = saveShortCount;
//        saveShort = 0;
    }
}

void ReportInterruptRoutine(InterruptRoutine intRout){
    // This might be a very good thing to have after other stuff is debugged.
    // Save the interrupt number in the saved RAM so we will have it
    // in case of a Watchdog reset
    intRout <<=1; //Don't use the bottom bit;
    SaveAcrossReset.fields.errorData &= 1; // Clear the top 7 bits
    SaveAcrossReset.fields.errorData |= (uint8_t)(intRout); // Or in the interrupt number
}
void EndInterruptRoutine(void){
    SaveAcrossReset.fields.errorData = (uint8_t)Int_NONE;
}

