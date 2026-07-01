/*
 * PacSat Antenna Control Processor interface.
 */

#include <pacsat.h>
#include <errors.h>
#include <spiDriver.h>
#include <gpioDriver.h>

#include "acp.h"

volatile bool acp_failed;

static xSemaphoreHandle acp_sem;

void (*acp_rx_msg_handler)(const unsigned char msg[ACP_MSG_SIZE]);

void acp_init(void)
{
    acp_sem = xSemaphoreCreateMutex();
    if (!acp_sem)
        ReportError(SemaphoreFail,true,CharString,(int)"acp_sem");
}

static void acp_reset(void)
{
#if 0
    GPIOSetOff(Ant_Power);
    vTaskDelay(CENTISECONDS(1));
    GPIOSetOn(Ant_Power);
    vTaskDelay(CENTISECONDS(1));
#endif
    acp_failed = false;
}

int acp_send(const unsigned char msg[ACP_MSG_SIZE])
{
    unsigned char rxbuffer[ACP_MSG_SIZE];
    unsigned int retries_left = 5;
    int rv = 0;

    if (!xSemaphoreTake(acp_sem, SHORT_WAIT_TIME))
        return ACP_ERR_MUTEX_TIMEOUT;

 retry:
    if (!SPIBidirectional(SPIACPDev, msg, rxbuffer, ACP_MSG_SIZE)) {
        ReportToWatchdog(CurrentTaskWD);
        acp_reset();
        retries_left--;
        if (retries_left > 0)
            goto retry;
        rv = ACP_ERR_START_TIMEOUT;
        goto out;
    }

    if (acp_failed) {
        ReportToWatchdog(CurrentTaskWD);
        acp_reset();
        retries_left--;
        if (retries_left > 0)
            goto retry;
        rv = ACP_ERR_END_TIMEOUT;
        goto out;
    }

    if (rxbuffer[0] != ACP_MSG_ID_INVALID) {
        unsigned int i;

        printf("Got rx message:  \n");
        for (i = 0; i < ACP_MSG_SIZE; i++)
            printf(" %2.2x", rxbuffer[i]);
        printf("\n");
    }
    if (rxbuffer[0] != ACP_MSG_ID_INVALID && acp_rx_msg_handler)
        acp_rx_msg_handler(rxbuffer);

 out:
    xSemaphoreGive(acp_sem);

    return rv;
}

/*
 * This is run by the CANTask code when it is messaged by
 * Ant_Interrupt handler in the spiDriver code if it gets an
 * Ant_Interrupt when a transaction is not in progress.
 */
void acp_runner(void)
{
    static const unsigned char txmsg[ACP_MSG_SIZE] = { ACP_MSG_ID_INVALID };

    /* Send a dummy message to pick up the receive message. */
    acp_send(txmsg);
}
