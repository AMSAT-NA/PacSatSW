
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
#include "i2cDriver.h"
#include "acp.h"

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "os_queue.h"
#include "os_semphr.h"
#include "os_task.h"

/* HALCoGen Includes */
#include "i2c.h"

/*
 * I2C Errors
 */
#define I2C_ERR_SUCCESS 0
#define I2C_ERR_NACK 1 /* Got a NACK, probably not device at addr. */
#define I2C_ERR_BUSY 2 /* Another master was probably using the bus. */
#define I2C_ERR_HUNG 3 /* Bus is hung, line is probably pulled down. */
#define I2C_ERR_OTHER 4 /* Some other error. */

struct i2c_data;

struct i2c_funcs {
    void (*init)(struct i2c_data *i2c_bus);
    bool (*io)(struct i2c_data *i2c_bus,
	       uint32_t SlaveAddress,
	       uint8_t *TxBuffer, uint32_t TxBytes,
	       uint8_t *RxBuffer, uint32_t RxBytes);
};

struct i2c_data {
    const struct i2c_funcs *funcs;
    unsigned int bus_num; /* Particular bus number on the driver. */
    ErrorType_t I2cError;
    xSemaphoreHandle I2cDoneSemaphore;
    xSemaphoreHandle I2cInUseSemaphore;
    int busResetsRemaining;
    uint8_t status;

    /* FIXME - move these to acp specific place. */
    unsigned int rx_len;
    uint8_t curr_addr;
    uint8_t rxbuffer[32];
};

static void I2cInitBus(struct i2c_data *i2c_bus);
static bool DoIO(struct i2c_data *i2c_bus,
		 uint32_t SlaveAddress,
		 uint8_t *TxBuffer, uint32_t TxBytes,
		 uint8_t *RxBuffer, uint32_t RxBytes);

static const struct i2c_funcs main_i2c_funcs = {
    .init = I2cInitBus,
    .io = DoIO,
};

static void acp_i2c_init(struct i2c_data *i2c_bus);
static bool acp_i2c_DoIO(struct i2c_data *i2c_bus,
			 uint32_t SlaveAddress,
			 uint8_t *TxBuffer, uint32_t TxBytes,
			 uint8_t *RxBuffer, uint32_t RxBytes);

static const struct i2c_funcs acp_i2c_funcs = {
    .init = acp_i2c_init,
    .io = acp_i2c_DoIO,
};

// Here are the per-bus data structures:

static struct i2c_data i2cBuses[NUM_I2C_BUSSES] = {
    {
	.funcs = &main_i2c_funcs,
	.I2cError = I2C1failure,
	.busResetsRemaining = 5,
	.I2cError = I2C1failure,
    },
    {
	.funcs = &acp_i2c_funcs,
	.I2cError = I2C2failure,
	.bus_num = 0,
	.busResetsRemaining = 5,
	.I2cError = I2C2failure,
    },
    {
	.funcs = &acp_i2c_funcs,
	.I2cError = I2C2failure,
	.bus_num = 1,
	.busResetsRemaining = 5,
	.I2cError = I2C2failure,
    },
    {
	.funcs = &acp_i2c_funcs,
	.I2cError = I2C2failure,
	.bus_num = 2,
	.busResetsRemaining = 5,
	.I2cError = I2C2failure,
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
	if (i2c_bus->funcs->init)
	    i2c_bus->funcs->init(i2c_bus);
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

    /* Have the low-level driver do the actual operation. */
    retVal = i2c_bus->funcs->io(i2c_bus, address, sndBuffer, sndLength,
				rcvBuffer, rcvLength);

    taskWithSemaphore = 0;
    xSemaphoreGive(i2c_bus->I2cInUseSemaphore);

    return retVal;
}

/*
 * Functions for the ACP I2C busses.
 */

/*
 * Got am I2C response message from the ACP.
 */
static void acp_i2c_rsp(unsigned char *msg)
{
    unsigned int i;

    for (i = 0; i < NUM_I2C_BUSSES; i++) {
	struct i2c_data *i2c_bus = &i2cBuses[i];

	if (i2c_bus->funcs != &acp_i2c_funcs
		|| i2c_bus->bus_num != msg[1])
	    continue;

	if (msg[3] != i2c_bus->rx_len)
	    break;
	if (msg[4] != i2c_bus->curr_addr)
	    break;

	switch (msg[2]) {
	case I2C_ACP_STATUS_SUCCESS:
	    memcpy(i2c_bus->rxbuffer, &msg[5], msg[3]);
	    break;

	case I2C_ACP_STATUS_ADDR_NACK:
	case I2C_ACP_STATUS_DATA_NACK:
	    i2c_bus->status = I2C_ERR_NACK;
	    break;
	    
	case I2C_ACP_STATUS_ARB_LOST:
	case I2C_ACP_STATUS_BUS_BUSY:
	    i2c_bus->status = I2C_ERR_BUSY;
	    break;

	default:
	    i2c_bus->status = I2C_ERR_OTHER;
	    break;
	}

	xSemaphoreGive(i2c_bus->I2cDoneSemaphore);
	break;
    }
}

static void acp_i2c_init(struct i2c_data *i2c_bus)
{
    acp_handlers[ACP_I2C_RSP] = acp_i2c_rsp;
}

static bool acp_i2c_DoIO(struct i2c_data *i2c_bus,
			 uint32_t SlaveAddress,
			 uint8_t *TxBuffer, uint32_t TxBytes,
			 uint8_t *RxBuffer, uint32_t RxBytes)
{
    uint8_t msg[ACP_MSG_SIZE];
    int rv;

    if (TxBytes >= MAX_I2C_ACP_MSG_SIZE)
	return false;
    if (RxBytes >= MAX_I2C_ACP_MSG_SIZE)
	return false;

    i2c_bus->status = 0;
    i2c_bus->curr_addr = SlaveAddress;
    i2c_bus->rx_len = RxBytes;

    msg[0] = ACP_I2C_CMD;
    msg[1] = i2c_bus->bus_num;
    msg[2] = TxBytes;
    msg[3] = RxBytes;
    msg[4] = SlaveAddress;
    memcpy(&msg[5], TxBuffer, TxBytes);

    rv = acp_send(msg);
    if (rv) {
	ReportError(I2C2failure, false, PortNumber, 0);
	return false;
    }

    if (!xSemaphoreTake(i2c_bus->I2cDoneSemaphore, SHORT_WAIT_TIME)) {
	ReportError(I2C2failure, false, PortNumber, i2c_bus->bus_num);
	return false;
    }

    if (i2c_bus->status) {
	return false;
    }

    memcpy(RxBuffer, i2c_bus->rxbuffer, RxBytes);
    return true;
}

static void I2cInitBus(struct i2c_data *i2c_bus)
{
}

/*
 * Functions for the main TMS570 I2C bus.
 */

static void I2cResetBus(struct i2c_data *i2c_bus, bool isError)
{
    /*
     * First make sure the bus is not holding the in-use semaphore.
     */
    xSemaphoreGive(i2c_bus->I2cInUseSemaphore);

    if (i2c_bus->busResetsRemaining <= 0) {
	ReportError(I2C1failure, true, PortNumber, 0);
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
     */
    i2c_bus->status = 0;

    thisBus->CNT = TxBytes;
    thisBus->SAR = SlaveAddress;
    if (RxBytes == 0)
	/* No receive, so we want a stop state after this data is sent. */
	MDRreg |= I2C_STOP_COND;
    thisBus->MDR = MDRreg;

    i2cSend(thisBus, TxBytes, TxBuffer);

    if(!xSemaphoreTake(i2c_bus->I2cDoneSemaphore, I2C_TIMEOUT)) {
	ReportError(i2c_bus->I2cError, false, CharString, (int)"TxTimeout");
	goto recover_sem_timeout;
    }

    if (i2c_bus->status == I2C_ERR_HUNG)
	goto recover_hung;

    /*
     * Here we do the read.  Similarly, this can accommodate a no-read
     * transaction.
     */
    if (i2c_bus->status == 0 && RxBytes != 0) {
	uint32_t MDRreg = (I2C_MASTER | I2C_RECEIVER | I2C_RESET_OUT
			   | I2C_START_COND | I2C_STOP_COND);

	thisBus->CNT = RxBytes;
	thisBus->SAR = SlaveAddress;
	thisBus->MDR = MDRreg;

        i2cReceive(thisBus, RxBytes, RxBuffer);

	if (!xSemaphoreTake(i2c_bus->I2cDoneSemaphore, SHORT_WAIT_TIME)) {
	    ReportError(i2c_bus->I2cError, false, CharString, (int)"RxTimeout");
        }

        if (i2c_bus->status == I2C_ERR_HUNG)
	    goto recover_hung;
    }

    if (!i2c_bus->status == 0 && i2cIsStopDetected(thisBus) == 0) {
        // If it failed, and bus is not in the stopped condition...

        i2cSetStop(thisBus);    // Stop it
        // And wait for the interrupt saying it has stopped
        if(!xSemaphoreTake(i2c_bus->I2cDoneSemaphore, SHORT_WAIT_TIME)) {
            ReportError(i2c_bus->I2cError, false, TaskNumber,
			(int)__builtin_return_address(0));
	    goto recover_sem_timeout;
        }
    }

    return i2c_bus->status == 0;

 recover_sem_timeout:
    /*
     * If we never get the semaphore, then something went wrong,
     * probably in the hardware.  Reset the device.  After the
     * reset we don't get a give from the semaphore, so it's safe
     * to release it then.
     */
    I2cResetBus(i2c_bus, true);

    /* Make sure the semaphore is not posted. */
    if(!xSemaphoreTake(i2c_bus->I2cDoneSemaphore, 0))
	xSemaphoreGive(i2c_bus->I2cDoneSemaphore);
    return false;

 recover_hung:
    /*
     * Something fairly bad happened, likely the bus hung for some reason.
     * Try to reset the hardware to recover.
     */
    ReportError(i2c_bus->I2cError, false, CharString, (int)"MajorFail");
    I2cResetBus(i2c_bus, true); //Try to reset--call it an error
    return false;
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

    if (i2cDev != i2cREG1)
	return; /* Shouldn't be possible. */

    switch(interruptType) {
    case I2C_SCD_INT:
        /*
         * Here we detected a stop condition.  Of course we are the
         * master so we generated it, but it still means that it is
         * the end of a transfer.
         */
        giveSemaphore = true;
        break;

    case I2C_ARDY_INT:
        i2cDev->STR |= I2C_ARDY_INT; //Write to status bit to clear it
        giveSemaphore = true;
	break;

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
        i2c_bus->status = I2C_ERR_HUNG;
        giveSemaphore = true;
        break;

    case I2C_NACK_INT:
        /*
         * Here we had some sort of failure.  Probably no response to
         * the address.
         */
        i2c_bus->status = I2C_ERR_NACK;
        giveSemaphore = true;
        break;

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
