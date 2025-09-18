
/*
 * AMSAT Golf I2c Driver
 * Burns Fisher, AMSAT-NA
 * 2012,2014,2019
 *
 * Ported from Fox and STM32L software and based on the SPI driver
 */

/* Includes ------------------------------------------------------------------*/

/* Golf Includes */
#include <pacsat.h>
#include "errors.h"
#include "config.h"
#include "hardwareConfig.h"
#include "het.h"
#include "i2cDriver.h"
#include "gpioDriver.h"

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "os_queue.h"
#include "os_semphr.h"
#include "os_task.h"

/* HALCoGen Includes */
#include "i2c.h"


/*
 * I2c bus data are kept in a structure and in arrays indexed by the bus number.  This allows multiple
 * I2c busses to be used.  We only have a single I2c bus on the TMS570, but another one might be added
 * vis the HET
 */

/*
 * Per-Bus data
 */

#define HW_I2C_BUS I2C1
#define EMU_I2C_BUS I2C2

typedef struct _BusData {
    xSemaphoreHandle I2cDoneSemaphore;
    xSemaphoreHandle I2cInUseSemaphore;
    uint32_t RxBytes, TxBytes;
    uint32_t SlaveAddress;
    int RxIndex,TxIndex;
    uint8_t *RxBuffer,*TxBuffer;
    bool resendAddress;
}I2cBusData;

// Here are the per-bus data structures:

static I2cBusData bus1Data,bus2Data;

// These are all indexed by the bus number

static I2cBusData *I2cBuses[]={&bus1Data,&bus2Data};
static i2cBASE_t *busAddress[]={i2cREG1,0};
static ErrorType_t I2cError[] = {I2C1failure,I2C2failure};
static volatile bool waitFlag[] = {true,true};
static volatile bool successFlag[] = {true,true};
static volatile bool majorFailure[] = {false,false}; // This is for stuff that looks like the bus has come loose or something
static volatile bool waitingForSemaphore[] = {false,false};
static volatile bool firstInit[] = {true,true};

/* Forward (i.e. internal only) routine declarations */

static inline bool DoIO(void);

/*
 * Here are the externally callable routines (in i2cdriver.h)
 */

void I2cInit(I2cBusNum thisBusNumber) {
    /*
     * This routine gets called after the OS starts up to get
     * the OS support part set up, turned on, initialized.  Note that on TMS570,
     * for the hardware I2c (I2C1), the routine i2cInit (the HalCoGen init)
     * must be called first for I2c1 and for the I2c emulator, hetInit (HalCoGen)
     * and HetI2cInit (the emulator code) must be called first.  I did not call these
     * from here because if you call those routines, then you can use I2cSendCommand
     * (below) before the OS is started.
     */
    bool firstTime = firstInit[thisBusNumber]; // Allows us to use this as part of resetting a failed bus
    I2cBusData *thisBusData = I2cBuses[thisBusNumber];


    // Init the data counts
    thisBusData->RxBytes = 0;
    thisBusData->TxBytes = 0;
    waitingForSemaphore[thisBusNumber] = false;  // Don't free the semaphore when we are not waiting!
    // Init the I2c Lines
    if(thisBusNumber==I2C1){
        //GPIOInit(I2c1Reset,NO_TASK,NO_MESSAGE);

    } else if (thisBusNumber==I2C2){
        //GPIOInit(I2c2Reset,NO_TASK,NO_MESSAGE);
    }


    /*
     * Set up the semaphores for use later.  I2cDoneSemaphore is used to make the caller
     * wait until the IO is done.  I2cInUse is a mutex that prevents multiple tasks from
     * using the same I2c bus simultaneously.
     */
    if(firstTime){
        (void)(vSemaphoreCreateBinary(thisBusData->I2cDoneSemaphore));
        thisBusData->I2cInUseSemaphore = xSemaphoreCreateMutex(); // In use wants a mutex to get priority inheretance
        if((thisBusData->I2cDoneSemaphore == NULL) || (thisBusData->I2cInUseSemaphore == NULL)){
            ReportError(SemaphoreFail,true,CharString,(int)"I2cAllocSema");
        }
    } else {
        // Not the first time so we are resetting.  Make sure the semaphores are released before we
        // try to take them again.  This will fail if the semaphore is already released but that is ok
        xSemaphoreGive(thisBusData->I2cDoneSemaphore);
        xSemaphoreGive(thisBusData->I2cInUseSemaphore);
    }
    if(thisBusData->I2cDoneSemaphore != NULL){
        /*
         * We want the 'done' semaphore to be taken by default
         * The interrupt routine will give it back (and unblock us)
         * when it is done.
         */
        if (xSemaphoreTake(thisBusData->I2cDoneSemaphore,0)!= pdTRUE){
            ReportError(SemaphoreFail,true,CharString,(int)"I2cTakeSema");
        };
    }
    // All ready.  Do not create semaphores again
    firstInit[thisBusNumber]= false;

    return;
}

void I2cResetBus(uint32_t busNum,bool isError){
    I2cBusData *thisBusData = I2cBuses[busNum];
    /*
     * First make sure the bus is not holding the in-use semaphore.
     */
    xSemaphoreGive(thisBusData->I2cInUseSemaphore);

    if(busNum == I2C1){
        static int bus1ResetsRemaining = 5;
        if(bus1ResetsRemaining <= 0){
            ReportError(I2C1failure,true,PortNumber,busNum);
        }
        //GPIOSetOff(I2c1Reset);
        i2cInit();
        i2cRxError(i2cREG1);
        I2cInit(I2C1);
        //GPIOSetOn(I2c1Reset);
        if(isError)bus1ResetsRemaining--;
        else bus1ResetsRemaining = 5;
    } else {
        static int bus2ResetsRemaining = 5;
        if(bus2ResetsRemaining <= 0){
            ReportError(I2C2failure,true,PortNumber,busNum);
        }
        //GPIOSetOff(I2c2Reset);
        I2cInit(I2C2);
        //GPIOSetOn(I2c1Reset);
        if(isError)bus2ResetsRemaining--;
        else bus2ResetsRemaining = 5;
    }
}

/*
 * This routine performs an I/O.  When the OS is started and I2cInit has been called it will use the
 * OS semaphores to wait the caller until the IO is done.  Otherwise, it will loop until the IO is
 * done.  Before calling I2cInit, it can still be called; this is intended to be used before the OS
 * is running.
 */
bool I2cSendCommand (I2cBusNum busNum, uint32_t address, void *sndBuffer,uint16_t sndLength,
                     void *rcvBuffer,uint16_t rcvLength)
{
    I2cBusData *thisBusData = I2cBuses[busNum];
    static int taskWithSemaphore = 0;
    bool retVal = true;
    if(sndLength == 0)return false; // We must send something.  We need not receive though.

    /*
     * The driver code is not reentrant.  Block here if another task is using it already
     */
    ReportToWatchdog(CurrentTaskWD);
    if(xSemaphoreTake(thisBusData->I2cInUseSemaphore,WATCHDOG_SHORT_WAIT_TIME)!= pdTRUE){
        ReportToWatchdog(CurrentTaskWD);

        /* If we can't get it within a few seconds...trouble */
        ReportError(I2CInUse,false,TaskNumber,taskWithSemaphore);
        taskWithSemaphore = -1;
        return false;

    }
    ReportToWatchdog(CurrentTaskWD);
    taskWithSemaphore = (((uint32_t)xTaskGetApplicationTaskTag(0)));
    /*
     * Set up the bus data structure with info about the IO.  This might not be needed except for
     * the semaphore. Everything else could be local variables, but let's not mess with a generally
     * good thing.
     */
    thisBusData->SlaveAddress = address;
    thisBusData->RxBuffer = rcvBuffer;
    thisBusData->TxBuffer = sndBuffer;
    thisBusData->RxBytes=rcvLength;
    thisBusData->TxBytes=sndLength;
    retVal = DoIO();
    xSemaphoreGive(thisBusData->I2cInUseSemaphore);
    taskWithSemaphore = 0;
    return retVal;

}

static inline bool DoIO(){
    int busNum = HW_I2C_BUS;
    I2cBusData *thisBusData = I2cBuses[busNum];
    i2cBASE_t *thisBus = busAddress[busNum];
    /*
     * Here is where we actually call the HalCoGen routines to do the I/O.  This could
     * be part of I2cSendCommand for the current version.  However, we will leave it like
     * this in case we need to do something more complex.
     */

    /*
     * Here we do an initial send.  Any I2c must start with a transmit with this driver.
     *
     * First just make sure that the semaphore is taken.  It might
     * have been freed by an error or something.
     * xSemaphoreTake(thisBusData->I2cDoneSemaphore,0);
     *
     * "Success" will be false if there is a NACK interrupt
     * "MajorFailure" will be false if there is a AL interrupt.
     * Note that AL on the satellite probably means that the I2c bus is
     * disconnected, i.e. not pulled up.
     */
    successFlag[busNum] = true;
    majorFailure[busNum] = false;
    {
        uint32_t MDRreg = I2C_MASTER | I2C_TRANSMITTER | I2C_RESET_OUT | I2C_START_COND;
        thisBus->CNT = thisBusData->TxBytes;
        thisBus->SAR = thisBusData->SlaveAddress;
        if(thisBusData->RxBytes == 0){
            //No receive, so we want a stop state after this data is sent
            MDRreg |= I2C_STOP_COND;
        }
        thisBus->MDR = MDRreg;
    }
    waitingForSemaphore[busNum] = true;
    i2cSend(thisBus,thisBusData->TxBytes,thisBusData->TxBuffer);
    if(xSemaphoreTake(thisBusData->I2cDoneSemaphore,I2C_TIMEOUT)!=pdTRUE){
        vPortEnterCritical();
        if(xSemaphoreTake(thisBusData->I2cDoneSemaphore,0)!=pdTRUE) {
            waitingForSemaphore[busNum] = false; //Ignore interrupts that might free the semaphore
            vPortExitCritical();
            //ReportError(I2cError[busNum],false,CharString,(int)"SemaSend");
            // This is just a timeout here
            return false;
        }
        // If we are here, the semaphore was released just before we were going to call the error
        // routine.  It took a while, but all is well now.
        vPortExitCritical();
    }
    // If we are NOT in control, chances are good the in-control CPU poked at the I2c at the same time
    // we tried to get the local temp.  Ignore it.  Otherwise, some other sort of major problem.  Reset
    // the bus.
    if(majorFailure[busNum]){
        ReportError(I2cError[busNum],false,CharString,(int)"ArbitrationFailure");
        I2cResetBus(busNum,true); //Try to reset--call it an error
        return false;
    }


    // Here we do the read.  Similarly, this can accommodate a no-read transaction.
    if (successFlag[busNum] && thisBusData->RxBytes != 0){
        {
            uint32_t MDRreg = I2C_MASTER | I2C_RECEIVER | I2C_RESET_OUT | I2C_START_COND | I2C_STOP_COND;
            thisBus->CNT = thisBusData->RxBytes;
            thisBus->SAR = thisBusData->SlaveAddress;
            thisBus->MDR = MDRreg;
        }

        waitingForSemaphore[busNum] = true; //Ok, time to pay attention to the semaphore
        i2cReceive(thisBus, thisBusData->RxBytes, thisBusData->RxBuffer);
        if(!majorFailure[busNum]){
            if(xSemaphoreTake(thisBusData->I2cDoneSemaphore,SECONDS(5)/*SHORT_WAIT_TIME*/)!=pdTRUE){
                waitingForSemaphore[busNum] = false;
                ReportError(I2cError[busNum],false,CharString,(int)"Timeout");
            }
        }
        if(majorFailure[busNum]){
            // If we are NOT in control, chances are good the in-control CPU poked at the I2c at the same time
            // we tried to get the local temp.  Ignore it.  Otherwise, some other sort of major problem.  Reset
            // the bus.
            ReportError(I2cError[busNum],false,CharString,(int)"MajorFail");
            I2cResetBus(busNum,true); //Try to reset--call it an error

            return false;
        }

    }
    if(!successFlag[busNum] && (i2cIsStopDetected(thisBus)==0)){
        // If it failed, and bus is not in the stopped condition...
        waitingForSemaphore[busNum] = true; //Ok, time to pay attention to the semaphore
        i2cSetStop(thisBus);    // Stop it
        // And wait for the interrupt saying it has stopped
        if(xSemaphoreTake(thisBusData->I2cDoneSemaphore,SHORT_WAIT_TIME)!=pdTRUE){
            waitingForSemaphore[busNum] = false;
            ReportError(I2cError[busNum],false,TaskNumber,(int)__builtin_return_address(0));
            return false;
        }
    }
    return successFlag[busNum];
}

/*
 * Here is the interrupt Handler--actually the notification routine given by HalCoGen.
 * We only use the SCD (Stop Condition Detect) interrupt.  It would be good to be able to
 * use other interrupts and recover from bad stuff.
 *
 */
void i2cNotification(i2cBASE_t *i2cDev,uint32_t interruptType)
{
    uint32_t thisBusIndex = (i2cDev == i2cREG1) ? 0 : 1;
    bool giveSemaphore = false;
    switch(interruptType){
    case I2C_SCD_INT:{
        /*
         * Here we detected a stop condition.  Of course we are the master so we generated it, but
         * it still means that it is the end of a transfer.
         */
        giveSemaphore = true;
        break;
    }
    case I2C_ARDY_INT:{
        i2cDev->STR |= I2C_ARDY_INT; //Write to status bit to clear it
        giveSemaphore = true;
    }
    case I2C_TX_INT:
    case I2C_RX_INT:
        /*
         * We need to enable these interrupts so that the HalCoGen driver will push out the next
         * byte.  We only get called here when the last byte has been sent or is available.  But we
         * don't want to know that here really.  We use stop condition detected to know when the whole
         * thing is done.
         */
        break;
    case I2C_AL_INT:
        majorFailure[thisBusIndex] = true;
        giveSemaphore = true;
        break;
    case I2C_NACK_INT:{
        /*
         * Here we had some sort of failure.  Probably no response to the address
         */
        successFlag[thisBusIndex] = false;
        giveSemaphore = true;
        break;
    }
    default:
        break;

    }
    /*
     * If we got an interrupt that says we are done (NAK or SCD) free the caller's semaphore or
     * mark the flag for pre-OS work.
     */
    if(giveSemaphore){
        //Only free give the semaphore if someone is waiting for it.  Otherwise the interrupt was not relevant
            I2cBusData *thisBusData = I2cBuses[thisBusIndex];
            BaseType_t higherPrioTaskWoken;
            (void)(xSemaphoreGiveFromISR(thisBusData->I2cDoneSemaphore,&higherPrioTaskWoken));
            waitingForSemaphore[thisBusIndex] = false;
    }
}

