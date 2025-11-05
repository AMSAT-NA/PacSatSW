
/*
 * AMSAT Golf SPI Driver
 * Burns Fisher, AMSAT-NA
 * 2012,2014,2019
 *
 * Ported from Fox and STM32L software
 */

/* Includes ------------------------------------------------------------------*/

/* Golf Includes */
#include <pacsat.h>
#include <spi-replacement.h>
#include "errors.h"
#include "config.h"
#include "spiDriver.h"
#include "hardwareConfig.h"

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "os_queue.h"
#include "os_semphr.h"
#include "os_task.h"

/* HALCoGen Includes */
#include "spi.h"
#include "gio.h"
#include "het.h"

typedef enum {
    DoneState,
    SendCommandState,
    SendDataState,
    ReceiveDataState
} SPIState;


/*
 * Per-Bus data
 */
typedef struct _BusData {
    const spiDAT1_t *thisDat1;
    ByteToWord cmd; // Has to be here to allow multiple buses to run at once
    xSemaphoreHandle SPIDoneSemaphore;
    xSemaphoreHandle SPIInUseSemaphore;
    uint16_t RxBytes, TxBytes, CmdBytes;
    uint8_t *RxBuffer,*TxBuffer;
    SPIState State;
    bool biDirectional;
    bool busInitted;
} SPIBusData;

// Per device read-only info

typedef struct _SPIDevInfo {
    spiBASE_t *thisBus;
    Gpio_Use selGPIO;
    spiDAT1_t thisDat1;
    SPIBusData *thisBusData;
} SPIDevInfo;



/* Global or routine-wide data */


// Here are the per-bus data structures:

SPIBusData bus1Data,bus2Data,bus3Data,bus4Data,bus5Data;

/*
 * The Device info structures are the read-only data for each individual device
 * on the SPI bus.  Not used yet.
 */

#if PACSAT_MAX_MRAMS != 4
#error Work required here
#endif

static SPIDevInfo SPIMram0Device={
    .thisBus     = SPI_MRAM_Reg,
    .selGPIO     = MRAM0_Sel,
    .thisDat1    = {.WDEL = false, .DFSEL = SPI_MRAM_Data_Format},
    .thisBusData = &bus1Data,
};

static SPIDevInfo SPIMram1Device={
    .thisBus     = SPI_MRAM_Reg,
    .selGPIO     = MRAM1_Sel,
    .thisDat1    = {.WDEL = false, .DFSEL = SPI_MRAM_Data_Format},
    .thisBusData = &bus1Data,
};

static SPIDevInfo SPIMram2Device={
    .thisBus     = SPI_MRAM_Reg,
    .selGPIO     = MRAM2_Sel,
    .thisDat1    = {.WDEL = false, .DFSEL = SPI_MRAM_Data_Format},
    .thisBusData = &bus1Data,
};

static SPIDevInfo SPIMram3Device={
    .thisBus     = SPI_MRAM_Reg,
    .selGPIO     = MRAM3_Sel,
    .thisDat1    = {.WDEL = false, .DFSEL = SPI_MRAM_Data_Format},
    .thisBusData = &bus1Data,
};

static SPIDevInfo SPIRx1AX5043Device={
    .thisBus     = SPI_AX5043_Reg,
    .selGPIO     = AX5043_Rx1_Sel,
    .thisDat1    = {.WDEL = false, .DFSEL = SPI_AX5043_Data_Format},
    .thisBusData = &bus3Data,
};

static SPIDevInfo SPITxAX5043Device={
    .thisBus     = SPI_AX5043_Reg,
    .selGPIO     = AX5043_Tx_Sel,
    .thisDat1    = {.WDEL = false, .DFSEL = SPI_AX5043_Data_Format},
    .thisBusData = &bus3Data,
};

#ifdef LAUNCHPAD_HARDWARE

static const SPIDevInfo *SPIDevInfoStructures[] = {
    &SPIMram0Device, &SPIMram1Device, &SPIMram2Device, &SPIMram3Device,
    &SPIRx1AX50430Device, &SPITxAX5043Device
};

#else

static SPIDevInfo SPIRx2AX5043Device={
    .thisBus     = SPI_AX5043_Reg,
    .selGPIO     = AX5043_Rx2_Sel,
    .thisDat1    = {.WDEL = false, .DFSEL = SPI_AX5043_Data_Format},
    .thisBusData = &bus3Data,
};

static SPIDevInfo SPIRx3AX5043Device={
    .thisBus     = SPI_AX5043_Reg,
    .selGPIO     = AX5043_Rx3_Sel,
    .thisDat1    = {.WDEL = false, .DFSEL = SPI_AX5043_Data_Format},
    .thisBusData = &bus3Data,
};

static SPIDevInfo SPIRx4AX5043Device={
    .thisBus     = SPI_AX5043_Reg,
    .selGPIO     = AX5043_Rx4_Sel,
    .thisDat1    = {.WDEL = false, .DFSEL = SPI_AX5043_Data_Format},
    .thisBusData = &bus3Data,
};

static const SPIDevInfo *SPIDevInfoStructures[] = {
    &SPIMram0Device, &SPIMram1Device, &SPIMram2Device, &SPIMram3Device,
    &SPIRx1AX5043Device, &SPIRx2AX5043Device, &SPIRx3AX5043Device, &SPIRx4AX5043Device,
    &SPITxAX5043Device
};

#endif

/*
 * Key to structures.
 *
 * The device name indexes SPIDevInfoStructures, which yields the SPIDevInfo for the device
 * SPIDevInfo.myBus is the bus structure used by HALCoGen code
 * SPIDevInfo.dat1 is the data format structure used by HALCoGen
 * SPIDevInfo.chipSelect is the number of the bus's chip select line that selects this device
 * SPIDevInfo.busData points to an SPIBusData structure for the bus.  Records in this structure
 *    are largely used by this driver
 *
 */



portTickType FramDeselectTime = 0;

static bool SPIIsInitted = false;

/* Forward (i.e. internal only) routine declarations */

static bool StartIO(SPIBusData *thisBusData, const SPIDevInfo *thisDevInfo);
static void CompleteIO(SPIBusData *thisBusData, const SPIDevInfo *thisDevInfo);

/*
 * Here are the externally callable routines (in spi.h)
 */

void SPIInit(SPIDevice thisDeviceNumber) {
    /*
     * This routine gets called early in boot to get
     * things set up, turned on, initialized.  Note that on TMS570,
     * SPIInit must be called first.  I did not include it here so
     * that one could use SPI outside of this routine before the OS
     * was started.
     */
    const SPIDevInfo *thisDevInfo = SPIDevInfoStructures[thisDeviceNumber];
    SPIBusData *thisBusData = thisDevInfo->thisBusData;

    GPIOSetOff(thisDevInfo->selGPIO); // Make sure it is disabled initially

    if(!SPIIsInitted){
        /*
         * If this is the first time we have called init for any SPI device, init the bus data
         * arrays to say that there are no semaphores and then call the HALCoGen spi init routine
         */
        bus1Data.busInitted = false;
        bus3Data.busInitted = false;
        bus5Data.busInitted = false;
    } else if(thisBusData->busInitted){
        return;
    }

    // Set up the data counts
    thisBusData->RxBytes = 0;
    thisBusData->TxBytes = 0;


    /* Set up the semaphores for use later */
    (void)(vSemaphoreCreateBinary(thisBusData->SPIDoneSemaphore));
    thisBusData->SPIInUseSemaphore = xSemaphoreCreateMutex(); // In use wants a mutex to get priority inheretance
    if((thisBusData->SPIDoneSemaphore == NULL) || (thisBusData->SPIInUseSemaphore == NULL)){
        ReportError(SemaphoreFail,true,CharString,(int)"SPIAllocSema");
    }
    if(thisBusData->SPIDoneSemaphore != NULL){
        /*
         * We want the 'done' semaphore to be taken by default
         * The interrupt routine will give it back (and unblock us)
         * when it is done.
         */
        if (xSemaphoreTake(thisBusData->SPIDoneSemaphore,0)!= pdTRUE){
            ReportError(SemaphoreFail,true,CharString,(int)"SPITakeSema");
        };
    }
    thisBusData->busInitted = true;
    return;
}

/*
 * These routines starts an I/O
 */
static inline bool SPISendCommandInternal (SPIDevice device, uint32_t command,uint8_t comLength,
                                           void *sndBuffer,uint16_t sndLength,
                                           void *rcvBuffer,uint16_t rcvLength,bool rxWhileTx)
{
    const SPIDevInfo *thisDevInfo = SPIDevInfoStructures[device];
    SPIBusData *thisBusData = thisDevInfo->thisBusData;
    bool retVal = true;
    if(comLength+sndLength+rcvLength == 0)return false;

    /*
     * We send whatever is in 'command' (1 to 4 bytes), and then send what is in sndBuffer (could be
     * 0 length, and finally receive rcvLength bytes (which also could be 0).
     */

    /*
     * The driver code is not reentrant.  Block here if another task is using it already
     */
    if(xSemaphoreTake(thisBusData->SPIInUseSemaphore,SHORT_WAIT_TIME)!= pdTRUE){
        /* If we can't get it within a few seconds...trouble */
        ReportError(SPIInUse,false,ReturnAddr,(int)__builtin_return_address(0));
    }
    /* Don't let someone request too many bytes of command */
    if (comLength>sizeof(ByteToWord)){
        comLength = sizeof(ByteToWord);
    }
    thisBusData->RxBuffer = rcvBuffer;
    thisBusData->TxBuffer = sndBuffer;
    thisBusData->RxBytes=rcvLength;
    thisBusData->TxBytes=sndLength;
    thisBusData->CmdBytes = comLength;
    thisBusData->cmd.word = command;
    thisBusData->biDirectional = rxWhileTx;
    thisBusData->thisDat1 = &thisDevInfo->thisDat1;
    if(StartIO(thisBusData,thisDevInfo)){
        // If SPIStartIO returns false, no IO was started so we don't wait here
        if(xSemaphoreTake(thisBusData->SPIDoneSemaphore,SHORT_WAIT_TIME)!=pdTRUE){
            ReportError(SPIOperationTimeout,false,ReturnAddr,(int)__builtin_return_address(0));
            retVal = false;
        }
    }
    CompleteIO(thisBusData,thisDevInfo);
    xSemaphoreGive(thisBusData->SPIInUseSemaphore);
    return retVal;

}
/*
 * This one sends and receives the same number of bytes simultaneously.  This is mostly useful for
 * the AX5043 where the first transmitted byte is the command and the first received byte is the status.
 * In any case, the txBuffer and the rxBuffer can be the same (but of course the command will be over-written
 * by the status.
 */

bool SPIBidirectional(SPIDevice device, void *txBuffer, void *rxBuffer, uint16_t length){
    return SPISendCommandInternal(device,0,0,txBuffer,length,rxBuffer,0,true);
}

bool SPISendCommand(SPIDevice device, uint32_t command,uint8_t comLength, void *sndBuffer,uint16_t sndLength,
                    void *rcvBuffer,uint16_t rcvLength)
{
    return SPISendCommandInternal(device,command,comLength,sndBuffer,sndLength,rcvBuffer,rcvLength,false);
}
static void CompleteIO(SPIBusData *thisBusData, const SPIDevInfo *thisDevInfo)
{
    GPIOSetOff(thisDevInfo->selGPIO);
}
static bool StartIO(SPIBusData *thisBusData, const SPIDevInfo *thisDevInfo)
{
    GPIOSetOn(thisDevInfo->selGPIO);

    /*
     * Now we start the I/O in the first state that exists.  The
     * interrupt routines take care of switching states from then on.
     */
    if(thisBusData->CmdBytes != 0){
        //Initialize the state machine
        thisBusData->State = SendCommandState;
        //Now start the data transfer
        if(thisBusData->biDirectional){
            spiSendAndGetDataByte(thisDevInfo->thisBus, &thisDevInfo->thisDat1,
                                  thisBusData->CmdBytes,thisBusData->cmd.byte,thisBusData->RxBuffer);
        } else {
            spiSendDataByte(thisDevInfo->thisBus, &thisDevInfo->thisDat1,
                            thisBusData->CmdBytes, thisBusData->cmd.byte);
        }
    } else if (thisBusData->TxBytes != 0){
        thisBusData->State = SendDataState;
        if(thisBusData->biDirectional){
            spiSendAndGetDataByte(thisDevInfo->thisBus, &thisDevInfo->thisDat1,
                                  thisBusData->TxBytes,thisBusData->TxBuffer,thisBusData->RxBuffer);
        } else {
            spiSendDataByte(thisDevInfo->thisBus, &thisDevInfo->thisDat1,thisBusData->TxBytes,thisBusData->TxBuffer);
        }
    } else if (thisBusData->RxBytes != 0){
        thisBusData->State = ReceiveDataState;
        spiGetDataByte(thisDevInfo->thisBus, &thisDevInfo->thisDat1,thisBusData->RxBytes,thisBusData->RxBuffer);
    } else {
        return false;
    }
    /*
     * Everything else happens in the interrupt routines.  When they
     * are done, the release the semaphore.  Wait until that happens or
     * time out
     */
    return true;

}
/*
 * The following routines are all part of the state machine and interrupt routines that run the whole
 * SPI mechanism, either per-character interrupt or DMA.
 */


/* Interrupt Handlers */
void spiEndNotification(spiBASE_t *spiDev)
{
    SPIBusData *thisBusData = (spiDev == spiREG1) ? &bus1Data :((spiDev==spiREG2) ? &bus2Data : ((spiDev==spiREG3) ? &bus3Data:
            ((spiDev==spiREG4) ? &bus4Data:&bus5Data)));

    //void SPIStateMachine(const SPIBusInfo *thisBusInfo){
    /*
     * The SPI peripheral is set up as "Two-wire, full duplex".
     * That means it always sends and receives at the same time (and in fact MUST
     * do both together).  To get the clock (SCK) to go, you need to send something
     * even if all you want is receive data.  In addition, when you send good data,
     * you still get junk coming into the receive buffer.
     *
     * The driver is implemented as a state machine with states as follows:
     *	DoneState,
     *	SendCommandState,
     *	SendDataState,
     *	ReceiveDataState
     *
     * DoneState = Done. No more interrupts expected
     * SendCommandState = The ISR is entered because sending commands is completed
     * SendDataState = The ISR is entered because sending data is completed
     * ReceiveDataState = The ISR is entered because receiving data is completed
     *
     * We can use DMA or interrupt-per-character, and only asking for interrupts on the receive channel.  Thus
     * when we get a receive interrupt, we know that all the transmits are also done (since
     * the transmitter fetches a new byte just as the old one starts shifting out.  The
     * receiver writes a new byte (and interrupts when done) AFTER the shift out/in is done.
     *
     * When we are receiving junk that we don't care about we use the 1-byte buffer called "black
     * hole" and for DMA disable memory increment so the DMA keeps going there.  Similarly when
     * we are transmitting junk.
     *
     * Sometimes, we need to skip an interrupt.  That's why this whole thing is enclosed in a
     * loop; if we need to skip a state, we set the next state, and then set the variable to loop
     * around and go to the next state.
     */


    /*
     * Find out what we were doing by checking the state.  We use if, not if/else or switch
     * because sometimes we might skip a state.  For example, if we are in state 1, we have just
     * finished sending the command.  But if there is no send buffer to send, we immediately
     * drop into the ==2 handler rather than waiting for another interrupt.
     */
    bool repeatSwitch = true;
    while(repeatSwitch){
        switch (thisBusData->State){
        case SendCommandState: {
            /*
             * This was an interrupt saying we just finished sending the command
             * buffer, let's set up for the data if there is any to do.
             */
            thisBusData->State = SendDataState;     /* State for sending from Tx buffer */
            if (thisBusData->TxBytes != 0){
                // We have sent the command.  Here we have data to send also.
                //void spiSendDataByte(spiBASE_t *spi, const spiDAT1_t *dataconfig_t, uint32 blocksize, uint8 * srcbuff);

                spiSendDataByte(spiDev, thisBusData->thisDat1,thisBusData->TxBytes,thisBusData->TxBuffer);
                repeatSwitch = false;
            }
            /* If there is no transmit data, repeatSwitch stays true, so we try again on the next state */
            break;
        }
        case SendDataState: {
            /*
             * Here we have sent the command as well as the transmit buffer (or it was null).  The
             * next state is state 3 (receive data).  If there are no bytes to receive, great.  Drop
             * through and enter the state=3 completion section.  Otherwise, set up to receive and
             * return to wait for another interrupt.
             */
            thisBusData->State = ReceiveDataState;
            if(thisBusData->RxBytes!=0){
                uint8_t *rxBuffer = thisBusData->RxBuffer;
                if(thisBusData->biDirectional){
                    // If biDirectional, we read CmdBytes (or TxBytes) into the buffer already, so skip
                    // ahead of that
                    rxBuffer+=(thisBusData->CmdBytes + thisBusData->TxBytes);
                    thisBusData->biDirectional = false;  // We are done with special stuff for bidirectional
                }
                spiGetDataByte(spiDev, thisBusData->thisDat1,thisBusData->RxBytes,rxBuffer);
                repeatSwitch=false;
                /* Exit and wait for next interrupt */
            } // Else go on to completion
            break;
        }
        case ReceiveDataState: {
            BaseType_t higherPrioTaskWoken;
            /*
             * This came from the Rx DMA done or possibly we just dropped through from
             * "end of Tx and nothing to Rx" section in state 2 above.
             * Whatever the case, we are done when we get here. Wake up the caller.
             */
            repeatSwitch = false;
            thisBusData->State = DoneState;
            (void)(xSemaphoreGiveFromISR(thisBusData->SPIDoneSemaphore,&higherPrioTaskWoken));
            break;
        }
        case DoneState:
        default:
        {
            repeatSwitch = false;
            /* We should not get these states */
            ReportError(UnexpectedBehavior,true,ReturnAddr,(int)__builtin_return_address(0));
            break;
        }
        }
    }
}
