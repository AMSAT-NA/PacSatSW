
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
#include "i2cEmulator.h"
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
static bool busUsingOS[]={false,false};
static volatile bool waitFlag[] = {true,true};
static volatile bool successFlag[] = {true,true};
static volatile bool majorFailure[] = {false,false}; // This is for stuff that looks like the bus has come loose or something
static volatile bool waitingForSemaphore[] = {false,false};


/* Forward (i.e. internal only) routine declarations */

static inline bool DoIO(void),DoEmulatedIO(void);

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
    bool firstTime = true; // Allows us to use this as part of resetting a failed bus
    I2cBusData *thisBusData = I2cBuses[thisBusNumber];
    if(busUsingOS[thisBusNumber]) firstTime = false; //This bus is already initted


    // Init the data counts
    thisBusData->RxBytes = 0;
    thisBusData->TxBytes = 0;
    waitingForSemaphore[thisBusNumber] = false;  // Don't free the semaphore when we are not waiting!
    // Init the I2c Lines
    if(thisBusNumber==I2C1){
        GPIOInit(I2c1Reset,NO_TASK,NO_MESSAGE,None);

    } else if (thisBusNumber==I2C2){
        GPIOInit(I2c2Reset,NO_TASK,NO_MESSAGE,None);
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
    // All ready.  Not just set this boolean so the rest will use semaphores instead of looping
    busUsingOS[thisBusNumber]= true;

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
        GPIOSetOff(I2c1Reset);
        i2cInit();
        i2cRxError(i2cREG1);
        I2cInit(I2C1);
        GPIOSetOn(I2c1Reset);
        if(isError)bus1ResetsRemaining--;
        else bus1ResetsRemaining = 5;
    } else {
        static int bus2ResetsRemaining = 5;
        if(bus2ResetsRemaining <= 0){
            ReportError(I2C2failure,true,PortNumber,busNum);
        }
        GPIOSetOff(I2c2Reset);
        I2cInit(I2C2);
        GPIOSetOn(I2c1Reset);
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
#define MAX31725_ADDR (0x90 >> 1)
    if(sndLength+rcvLength == 0)return false;

    if(busUsingOS[busNum]){
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
    }
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
    if(busAddress[busNum != 0]){
        retVal = DoIO();
    } else {
        retVal = DoEmulatedIO();
    }
    if(busUsingOS[busNum]){
        xSemaphoreGive(thisBusData->I2cInUseSemaphore);
        taskWithSemaphore = 0;
    }
    return retVal;

}


static inline bool DoIO(){
    int loopCount,delayCount;
    int busNum = HW_I2C_BUS;
    I2cBusData *thisBusData = I2cBuses[busNum];
    i2cBASE_t *thisBus = busAddress[busNum];
    /*
     * Here is where we actually call the HalCoGen routines to do the I/O.  This could
     * be part of I2cSendCommand for the current version.  However, we will leave it like
     * this in case we need to do something more complex.
     */

    /*
     * Here we do an initial send.  I'm not sure if any I2c I/O does not have a send at first,
     * but we can accommodate that.
     */
    if(busUsingOS[busNum]){
        // First just make sure that the semaphore is taken.  It might
        // have been freed by an error or something.
        xSemaphoreTake(thisBusData->I2cDoneSemaphore,0);
    }
    /*
     * "Success" will be false if there is a NACK interrupt
     * "MajorFailure" will be false if there is a AL interrupt.
     * Note that AL on the satellite probably means that the I2c bus is
     * disconnected, i.e. not pulled up.
     */
    successFlag[busNum] = true;
    majorFailure[busNum] = false;
    if (thisBusData->TxBytes != 0){
        i2cSetSlaveAdd(thisBus, thisBusData->SlaveAddress);
        i2cSetDirection(thisBus, I2C_TRANSMITTER);
        i2cSetCount(thisBus, thisBusData->TxBytes);
        i2cSetMode(thisBus, I2C_MASTER);
        i2cSetStop(thisBus);
        i2cSetStart(thisBus);
        waitingForSemaphore[busNum] = true;
        i2cSend(thisBus,thisBusData->TxBytes,thisBusData->TxBuffer);
        if(busUsingOS[busNum]){
            // If the OS is running wait using a semaphore (released below by the interrupt routine)
            // Note that the semaphore timeout must be shorter than the I2C Mutex
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
        } else {
            //If no OS, loop waiting for the flag (set below by the interrupt routine)
            while(waitFlag[busNum]){};
            waitFlag[busNum] = true;
        }
         // If we are NOT in control, chances are good the in-control CPU poked at the I2c at the same time
         // we tried to get the local temp.  Ignore it.  Otherwise, some other sort of major problem.  Reset
         // the bus.
        if(majorFailure[busNum]){
            ReportError(I2cError[busNum],false,CharString,(int)"ArbitrationFailure");
            I2cResetBus(busNum,true); //Try to reset--call it an error
            return false;
        }
    }

    // Here we do the read.  Similarly, this can accommodate a no-read transaction.
    if (successFlag[busNum] && thisBusData->RxBytes != 0){
        loopCount=10,delayCount=10;
        while(i2cIsMasterReady(thisBus) != true){
            loopCount--;
            if((loopCount <= 0) && (busUsingOS[busNum])){
                vTaskDelay(2);
                loopCount = 10;
                delayCount--;
                if(delayCount<0){
                    I2cResetBus(busNum,true); //Try to reset--call it an error
                    return false;
                }
            }
        }
        i2cSetSlaveAdd(thisBus, thisBusData->SlaveAddress);
        i2cSetDirection(thisBus, I2C_RECEIVER);
        i2cSetCount(thisBus, thisBusData->RxBytes);
        i2cSetMode(thisBus, I2C_MASTER);
        i2cSetStop(thisBus);
        i2cSetStart(thisBus);
        waitingForSemaphore[busNum] = true; //Ok, time to pay attention to the semaphore
        i2cReceive(thisBus, thisBusData->RxBytes, thisBusData->RxBuffer);
        if(busUsingOS[busNum]){
            if(!majorFailure[busNum]){
                if(xSemaphoreTake(thisBusData->I2cDoneSemaphore,SECONDS(5)/*SHORT_WAIT_TIME*/)!=pdTRUE){
                    waitingForSemaphore[busNum] = false;
                    ReportError(I2cError[busNum],false,CharString,(int)"Timeout");
                }
            }
        } else {
            while(waitFlag[busNum]){};
            waitFlag[busNum] = true;
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
        if(busUsingOS[busNum]){ // And wait for the interrupt saying it has stopped
            if(xSemaphoreTake(thisBusData->I2cDoneSemaphore,SHORT_WAIT_TIME)!=pdTRUE){
                waitingForSemaphore[busNum] = false;
                ReportError(I2cError[busNum],false,TaskNumber,(int)__builtin_return_address(0));
                return false;
            }
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
        if(busUsingOS[thisBusIndex] && waitingForSemaphore[thisBusIndex]){
            I2cBusData *thisBusData = I2cBuses[thisBusIndex];
            BaseType_t higherPrioTaskWoken;
            (void)(xSemaphoreGiveFromISR(thisBusData->I2cDoneSemaphore,&higherPrioTaskWoken));
            waitingForSemaphore[thisBusIndex] = false;
        } else {
            waitFlag[thisBusIndex] = false;
        }
    }
}

/////////////////////////////////////This is for the N2HET-emulated I2c master////////////////////
static inline bool DoEmulatedIO(void){
    int busNum = EMU_I2C_BUS;
    char firstOp,firstByteCount=0;
    bool doAddrInterrupt = true,sendStopBit=true;
    I2cBusData *thisBusData = I2cBuses[busNum];
    /*
     * Here is where we actually call the emulator routines.  It is simpler
     * than the hardware I2c "DoIO" routine above, I suppose because the emulator
     * is less flexible and has fewer calls.
     *
     * Start off initializing the bus data
     */
    successFlag[busNum] = true;
    thisBusData->TxIndex = 0;
    thisBusData->RxIndex = 0;
    /*
     * Here we do an initial send.  We always send an address first, but the way the emulator
     * works, we need to know what we are doing next also.
     */
    if (thisBusData->TxBytes != 0){
        /*
         * Here we know we are sending at least one byte after the address, so the op that
         * we tell the address routine is write and the number of bytes is, well, the number
         * of bytes to write.
         */
        firstOp = I2C_EMU_WRITE;
        firstByteCount = (char)thisBusData->TxBytes;
        doAddrInterrupt = true; // We need an interrupt after the address
        if(thisBusData->RxBytes != 0){
            /*
             * We also need to figure out what will happen AFTER all those writes are complete.  In this cae
             * we will want to receive, so flag that after the address and its writes, no stop bit and that we
             * will re-send the address for the read.
             */
            thisBusData->resendAddress = true; // We are sending AND receiving.  Need to resend the address for the Rx.
            sendStopBit = false; // No stop bit between send and receive
        } else {
            // Here we sent address (and maybe data) but we are not receiving anything
            thisBusData->resendAddress = false;
        }
    } else if (thisBusData->RxBytes != 0){
        /*
         * And here is where we have nothing to send other than the address, only read.  So no interrupt required
         * after the address.  We will get an Rx interrupt and that tells us to get the incoming data.
         */
        firstOp = I2C_EMU_READ;
        firstByteCount = thisBusData->RxBytes;
        doAddrInterrupt = false; // If we are only receiving, no interrupt required on the address transmit
    }
    // Start it up.  The reset of the transfer happens in the interrupt routine.
    HetI2CPutAddr((char)thisBusData->SlaveAddress,firstOp,firstByteCount,
                  (char)doAddrInterrupt?1:0,(char)sendStopBit?1:0);


    if(busUsingOS[busNum]){
        // If the OS is running wait using a semaphore (released below by the interrupt routine)
        if(xSemaphoreTake(thisBusData->I2cDoneSemaphore,SHORT_WAIT_TIME)!=pdTRUE){
            ReportError(I2cError[busNum],false,ReturnAddr,(int)__builtin_return_address(0));
            successFlag[busNum] = false;
        }
        //xSemaphoreGive(thisBusData->I2cInUseSemaphore);
    } else {
        //If no OS, loop waiting for the flag (set below by the interrupt routine)
        while(waitFlag[busNum]){};
        waitFlag[busNum] = true;
    }
    /*
     * This section is intended to reset the I2c master hardware if the bus hangs.  I don't think that is
     * required for the emulator because if the emulator times out (which is why it would say there was an
     * error in the first place) I think it inits itself.  But if needed, we COULD call the het and emulator
     * init routines again to re-copy the microcode (including state data).
     */

    if(!successFlag[busNum]){
        HetI2CInit();
        gioSetBit(I2c2_HET_Port,I2c2_HET_SCL_Pin,1);
        gioSetBit(I2c2_HET_Port,I2c2_HET_SDA_Pin,1);
    }
    return successFlag[busNum];
}
/*
 * Following is the interrupt routine for N2HET2 on the RT-IHU.  It is called from the HETInterrupt module
 * which contains the HalCoGen notify routine for both N2HET2 and N2HET1 and figures out which one to call.
 */
void i2cHETInterrupt(uint32_t offset){
    int busNum = I2C2; // We know this is the emulated bus, which is bus 2
    I2cBusData *thisBusData = I2cBuses[busNum];
    bool giveSemaphore = false;
    switch (offset)
    {
    case 11: {/*Transmit interrupt */
        int txBytes = thisBusData->TxBytes,txIndex = thisBusData->TxIndex++;
        if(txIndex < txBytes){
            HetI2CPutData(thisBusData->TxBuffer[txIndex],1);//Send the next byte of data
        } else if(txIndex == txBytes && thisBusData->RxBytes > 0){ //Index == Bytes means we have not sent the address
            //Done transmit, but there is some to receive, so send addr again.  No interrupt required for the address
            // transmit since the microcode will go on and read back immediately.  But we do want a stop bit.

            HetI2CPutAddr((char)thisBusData->SlaveAddress,I2C_EMU_READ,thisBusData->RxBytes,0,1);
        } else {
            giveSemaphore = true;
        }
        break;
    }
    case 15:{ /* Rx interrupt */
        char data;
        int rxBytes = thisBusData->RxBytes,rxIndex = thisBusData->RxIndex++;
        if(!HetI2CGetChar(&data)){
            // This should never fail because we got an interrupt saying it was ready
            // So something is hosed, probably relating to switching from one processor
            // to the other.
            majorFailure[EMU_I2C_BUS] = true;
            break;
        }
        thisBusData->RxBuffer[rxIndex++] = data;
        if(rxIndex >= rxBytes){
            giveSemaphore = true;
        }
        break;
    }
    case 29:{
        // This one is NAK
        successFlag[I2C2] = false;
        giveSemaphore = true;
        break;
    }
    case 8:
    case 2:
    default:
        successFlag[I2C2] = false; //Failure of some sort
        giveSemaphore = true;
        // Restart the Het Emulator as much as possible
    }
    if(giveSemaphore){
        // Here we have completed all ops or else failed.  Release the semaphore show mainline can continue

        if(busUsingOS[I2C2]){
            I2cBusData *thisBusData = I2cBuses[I2C2];
            BaseType_t higherPrioTaskWoken;
            (void)(xSemaphoreGiveFromISR(thisBusData->I2cDoneSemaphore,&higherPrioTaskWoken));
        } else {
            waitFlag[I2C2] = false;
        }

    }
}
