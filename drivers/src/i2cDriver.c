
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

struct i2c_data;

struct i2c_funcs {
    ErrorType_t I2cError;
    void (*init)(struct i2c_data *i2c_bus);
    int (*io)(struct i2c_data *i2c_bus,
	      uint32_t SlaveAddress,
	      uint8_t *TxBuffer, uint32_t TxBytes,
	      uint8_t *RxBuffer, uint32_t RxBytes);
};

struct i2c_data {
    const struct i2c_funcs *funcs;
    unsigned int bus_num;
    xSemaphoreHandle I2cDoneSemaphore;
    xSemaphoreHandle I2cInUseSemaphore;
    volatile bool successFlag;
    volatile bool majorFailure;
    int busResetsRemaining;
};

static void I2cInitBus(struct i2c_data *i2c_bus);
static inline bool DoIO(struct i2c_data *i2c_bus,
			uint32_t SlaveAddress,
			uint8_t *TxBuffer, uint32_t TxBytes,
			uint8_t *RxBuffer, uint32_t RxBytes);

static const struct i2c_funcs main_i2c_funcs = {
    .I2cError = I2C1failure,
    .init = I2cInitBus,
    .io = DoIO,
};

// Here are the per-bus data structures:

static struct i2c_data i2cBuses[NUM_I2C_BUSSES] = {
    {
	.funcs = &main_i2c_funcs,
	.busResetsRemaining = 5,
    },
};

/*
 * Here are the externally callable routines (in i2cdriver.h)
 */

void i2c_init(void)
{
    unsigned int i;

    for (i = 0; i < NUM_I2C_BUSSES; i++) {
	struct i2c_data *i2c_bus = &i2cBuses[i];

	i2c_bus->funcs->init(i2c_bus);
    }
}

static void I2cInitBus(struct i2c_data *i2c_bus)
{
    (void)(vSemaphoreCreateBinary(i2c_bus->I2cDoneSemaphore));
    // In use wants a mutex to get priority inheritance
    i2c_bus->I2cInUseSemaphore = xSemaphoreCreateMutex();
    if ((i2c_bus->I2cDoneSemaphore == NULL)
	|| (i2c_bus->I2cInUseSemaphore == NULL)) {
	ReportError(SemaphoreFail, true, CharString, (int)"I2cAllocSema");
    }

    if (i2c_bus->I2cDoneSemaphore != NULL) {
	/*
	 * We want the 'done' semaphore to be taken by default
	 * The interrupt routine will give it back (and unblock us)
	 * when it is done.
	 */
	if (xSemaphoreTake(i2c_bus->I2cDoneSemaphore, 0) != pdTRUE)
	    ReportError(SemaphoreFail, true, CharString,
			(int)"I2cTakeSema");
    }
}

/*
 * This routine performs an I/O.  When the OS is started and I2cInit
 * has been called it will use the OS semaphores to wait the caller
 * until the IO is done.  Otherwise, it will loop until the IO is
 * done.  Before calling I2cInit, it can still be called; this is
 * intended to be used before the OS is running.
 */
bool I2cSendCommand(I2cBusNum busNum, uint32_t address,
		    void *sndBuffer, uint16_t sndLength,
		    void *rcvBuffer, uint16_t rcvLength)
{
    struct i2c_data *i2c_bus = &i2cBuses[busNum];
    static int taskWithSemaphore = 0;
    bool retVal = true;

    if (sndLength == 0)
	return false; // We must send something.  We need not receive though.

    /*
     * The driver code is not reentrant.  Block here if another task
     * is using it already
     */
    ReportToWatchdog(CurrentTaskWD);
    if (!xSemaphoreTake(i2c_bus->I2cInUseSemaphore,
		       WATCHDOG_SHORT_WAIT_TIME)) {
        ReportToWatchdog(CurrentTaskWD);

        /* If we can't get it within a few seconds...trouble */
        ReportError(I2CInUse, false, TaskNumber, taskWithSemaphore);
        taskWithSemaphore = -1;
        return false;
    }
    ReportToWatchdog(CurrentTaskWD);
    taskWithSemaphore = (((uint32_t)xTaskGetApplicationTaskTag(0)));
    /*
     * Set up the bus data structure with info about the IO.  This
     * might not be needed except for the semaphore. Everything else
     * could be local variables, but let's not mess with a generally
     * good thing.
     */
    retVal = i2c_bus->funcs->io(i2c_bus, address, sndBuffer, sndLength,
				rcvBuffer, rcvLength);
    xSemaphoreGive(i2c_bus->I2cInUseSemaphore);
    taskWithSemaphore = 0;

    return retVal;
}

static void I2cResetBus(struct i2c_data *i2c_bus, bool isError)
{
    /*
     * First make sure the bus is not holding the in-use semaphore.
     */
    xSemaphoreGive(i2c_bus->I2cInUseSemaphore);

    if (i2c_bus->busResetsRemaining <= 0) {
	ReportError(I2C1failure, true, PortNumber, i2c_bus->bus_num);
    }
    i2cInit();
    i2cRxError(i2cREG1);

    if (isError)
	i2c_bus->busResetsRemaining--;
    else
	i2c_bus->busResetsRemaining = 5;
}

static inline bool DoIO(struct i2c_data *i2c_bus,
			uint32_t SlaveAddress,
			uint8_t *TxBuffer, uint32_t TxBytes,
			uint8_t *RxBuffer, uint32_t RxBytes)
{
    i2cBASE_t *thisBus = i2cREG1;
    uint32_t MDRreg = (I2C_MASTER | I2C_TRANSMITTER | I2C_RESET_OUT
		       | I2C_START_COND);

    /*
     * Here is where we actually call the HalCoGen routines to do the
     * I/O.  This could be part of I2cSendCommand for the current
     * version.  However, we will leave it like this in case we need
     * to do something more complex.
     */

    /*
     * Here we do an initial send.  Any I2c must start with a transmit
     * with this driver.
     *
     * First just make sure that the semaphore is taken.  It might
     * have been freed by an error or something.
     * xSemaphoreTake(i2c_bus->I2cDoneSemaphore,0);
     *
     * "Success" will be false if there is a NACK interrupt
     * "MajorFailure" will be false if there is a AL interrupt.
     * Note that AL on the satellite probably means that the I2c bus is
     * disconnected, i.e. not pulled up.
     */
    i2c_bus->successFlag = true;
    i2c_bus->majorFailure = false;

    thisBus->CNT = TxBytes;
    thisBus->SAR = SlaveAddress;
    if (RxBytes == 0) {
	// No receive, so we want a stop state after this data is sent
	MDRreg |= I2C_STOP_COND;
    }
    thisBus->MDR = MDRreg;

    i2cSend(thisBus, TxBytes, TxBuffer);
    if(!xSemaphoreTake(i2c_bus->I2cDoneSemaphore, I2C_TIMEOUT)) {
        vPortEnterCritical();
        if (!xSemaphoreTake(i2c_bus->I2cDoneSemaphore,0)) {
            vPortExitCritical();
            // ReportError(I2cError[busNum],false,CharString,(int)"SemaSend");
            // This is just a timeout here
            return false;
        }
        // If we are here, the semaphore was released just before we
        // were going to call the error routine.  It took a while, but
        // all is well now.
        vPortExitCritical();
    }
    // If we are NOT in control, chances are good the in-control CPU
    // poked at the I2c at the same time we tried to get the local
    // temp.  Ignore it.  Otherwise, some other sort of major problem.
    // Reset the bus.
    if (i2c_bus->majorFailure) {
        ReportError(i2c_bus->funcs->I2cError, false, CharString,
		    (int)"ArbitrationFailure");
        I2cResetBus(i2c_bus, true); //Try to reset--call it an error
        return false;
    }

    // Here we do the read.  Similarly, this can accommodate a no-read transaction.
    if (i2c_bus->successFlag && RxBytes != 0) {
	uint32_t MDRreg = (I2C_MASTER | I2C_RECEIVER | I2C_RESET_OUT
			   | I2C_START_COND | I2C_STOP_COND);

	thisBus->CNT = RxBytes;
	thisBus->SAR = SlaveAddress;
	thisBus->MDR = MDRreg;

	//Ok, time to pay attention to the semaphore
        i2cReceive(thisBus, RxBytes, RxBuffer);
        if (!i2c_bus->majorFailure) {
            if (!xSemaphoreTake(i2c_bus->I2cDoneSemaphore,
				SECONDS(5)/*SHORT_WAIT_TIME*/)) {
                ReportError(i2c_bus->funcs->I2cError, false, CharString,
			    (int)"Timeout");
            }
        }
        if (i2c_bus->majorFailure) {
            // If we are NOT in control, chances are good the
            // in-control CPU poked at the I2c at the same time we
            // tried to get the local temp.  Ignore it.  Otherwise,
            // some other sort of major problem.  Reset the bus.
            ReportError(i2c_bus->funcs->I2cError, false,
			CharString, (int)"MajorFail");
            I2cResetBus(i2c_bus, true); //Try to reset--call it an error

            return false;
        }

    }
    if (!i2c_bus->successFlag && (i2cIsStopDetected(thisBus) == 0)) {
        // If it failed, and bus is not in the stopped condition...

        i2cSetStop(thisBus);    // Stop it
        // And wait for the interrupt saying it has stopped
        if(!xSemaphoreTake(i2c_bus->I2cDoneSemaphore, SHORT_WAIT_TIME)) {
            ReportError(i2c_bus->funcs->I2cError, false, TaskNumber,
			(int)__builtin_return_address(0));
            return false;
        }
    }
    return i2c_bus->successFlag;
}

/*
 * Here is the interrupt Handler--actually the notification routine
 * given by HalCoGen.  We only use the SCD (Stop Condition Detect)
 * interrupt.  It would be good to be able to use other interrupts and
 * recover from bad stuff.
 */
void i2cNotification(i2cBASE_t *i2cDev, uint32_t interruptType)
{
    struct i2c_data *i2c_bus = &i2cBuses[I2C1];
    bool giveSemaphore = false;

    if (i2cDev == i2cREG1)
	return; /* Shouldn't be possible. */

    switch(interruptType) {
    case I2C_SCD_INT: {
        /*
         * Here we detected a stop condition.  Of course we are the
         * master so we generated it, but it still means that it is
         * the end of a transfer.
         */
        giveSemaphore = true;
        break;
    }

    case I2C_ARDY_INT: {
        i2cDev->STR |= I2C_ARDY_INT; //Write to status bit to clear it
        giveSemaphore = true;
    }

    case I2C_TX_INT:
    case I2C_RX_INT:
        /*
         * We need to enable these interrupts so that the HalCoGen
         * driver will push out the next byte.  We only get called
         * here when the last byte has been sent or is available.  But
         * we don't want to know that here really.  We use stop
         * condition detected to know when the whole thing is done.
         */
        break;

    case I2C_AL_INT:
        i2c_bus->majorFailure = true;
        giveSemaphore = true;
        break;

    case I2C_NACK_INT: {
        /*
         * Here we had some sort of failure.  Probably no response to
         * the address.
         */
        i2c_bus->successFlag = false;
        giveSemaphore = true;
        break;
    }

    default:
        break;
    }

    /*
     * If we got an interrupt that says we are done (NAK or SCD) free
     * the caller's semaphore or mark the flag for pre-OS work.
     */
    if (giveSemaphore) {
        //Only free give the semaphore if someone is waiting for it.
        //Otherwise the interrupt was not relevant
	BaseType_t higherPrioTaskWoken;

	(void)(xSemaphoreGiveFromISR(i2c_bus->I2cDoneSemaphore,
				     &higherPrioTaskWoken));
    }
}
