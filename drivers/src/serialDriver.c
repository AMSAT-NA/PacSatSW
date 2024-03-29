/*
 * Burns Fisher WB1FJ
 * AMSAT-NA
 * February 14, 2013
 * February 2019
 *
 * Vastly modified so as to be almost unrecognizable from an example include
 * with FreeRTOS.  Here is the header that originally came with it:

 FreeRTOS V7.0.2 - Copyright (C) 2011 Real Time Engineers Ltd.
 Heavily modified by WB1FJ


 ***************************************************************************
 *                                                                       *
 *    FreeRTOS tutorial books are available in pdf and paperback.        *
 *    Complete, revised, and edited pdf reference manuals are also       *
 *    available.                                                         *
 *                                                                       *
 *    Purchasing FreeRTOS documentation will not only help you, by       *
 *    ensuring you get running as quickly as possible and with an        *
 *    in-depth knowledge of how to use FreeRTOS, it will also help       *
 *    the FreeRTOS project to continue with its mission of providing     *
 *    professional grade, cross platform, de facto standard solutions    *
 *    for microcontrollers - completely free of charge!                  *
 *                                                                       *
 *    >>> See http://www.FreeRTOS.org/Documentation for details. <<<     *
 *                                                                       *
 *    Thank you for using FreeRTOS, and thank you for your support!      *
 *                                                                       *
 ***************************************************************************


 This file is part of the FreeRTOS distribution.

 FreeRTOS is free software; you can redistribute it and/or modify it under
 the terms of the GNU General Public License (version 2) as published by the
 Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
 >>>NOTE<<< The modification to the GPL is included to allow you to
 distribute a combined work that includes FreeRTOS without being obliged to
 provide the source code for proprietary components outside of the FreeRTOS
 kernel.  FreeRTOS is distributed in the hope that it will be useful, but
 WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 more details. You should have received a copy of the GNU General Public
 License and the FreeRTOS license exception along with FreeRTOS; if not it
 can be viewed here: http://www.freertos.org/a00114.html and also obtained
 by writing to Richard Barry, contact details for whom are available on the
 FreeRTOS WEB site.

 1 tab == 4 spaces!

 http://www.FreeRTOS.org - Documentation, latest information, license and
 contact details.

 http://www.SafeRTOS.com - A version that is certified for use in safety
 critical systems.

 http://www.OpenRTOS.com - Commercial support, development, porting,
 licensing and training services.
 */

#include <pacsat.h>
#include <string.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "os_queue.h"
#include "os_semphr.h"
#include "os_task.h"
/* HALCoGen includes */
#include "het.h"
#include "rti.h"
#include "sci.h"

/* Golf Include */
#include "hardwareConfig.h"
#include "serialDriver.h"
#include "errors.h"
/*
 * This is for before the OS version is initted
 */
static volatile unsigned int sw_serial_trig = 0;
static unsigned char newChar;
static bool charReady=false;

/*-----------------------------------------------------------*/

/*
 * Queues and semaphore create.  Here is what they are for:
 * - xRxedChars - A queue holding characters received on the com port and
 *                read via a per-character interrupt.  (Not involved in DMA)
 * -usartRxInUseSemaphore
 * -usartTxInUseSemaphore - Semaphores taken at the start of an Rx or Tx op to
 *                prevent multiple tasks from using the COM port at the same time
 *
 */

// All the following arrays are indexed on the COM port index

static xQueueHandle xRxedChars[numCOM] = { 0, 0, 0 };
static xQueueHandle xCharsToTx[numCOM] = { 0, 0, 0 };
static xSemaphoreHandle usartTxDoneSemaphore[numCOM] = { 0, 0, 0 };
static volatile bool txIntsDisabled[numCOM] = { TRUE, TRUE, TRUE };
static xSemaphoreHandle usartTxInUseSemaphore[numCOM] = { 0, 0, 0 };
//static sciBASE_t
static void *serialDevice[numCOM] = {(void *)COM1_Port,(void *)COM2_Port,(void *)COM3_Port};
static bool RxInUse[numCOM]={false,false,false};

/*
 * Initialize a structure for each com port.  COM1_ and COM2_xxx defined in hardwareDefinitions.h
 */


/*
 * Call this routine before using the specified COM port.
 */

bool SerialInitPort(COM_NUM comPort, unsigned int baud, unsigned int qLengthRx,
                    unsigned int qLengthTx)
{

    if (usartTxInUseSemaphore[comPort] != 0)
        return FALSE;  // Already initted


    /* Create the queues used to hold Rx/Tx characters. */

    if (qLengthRx)
    {
        xRxedChars[comPort] = xQueueCreate(
                qLengthRx,
                ( unsigned portBASE_TYPE ) sizeof( signed portCHAR ));
        if (xRxedChars[comPort] == NULL)
            return FALSE;
    }
    xCharsToTx[comPort] = xQueueCreate(
            qLengthTx, ( unsigned portBASE_TYPE ) sizeof( signed portCHAR ));
    vSemaphoreCreateBinary(usartTxDoneSemaphore[comPort]);
    usartTxInUseSemaphore[comPort] = xSemaphoreCreateMutex();

    /*
     * Set up baud rate and tell it to use interrupts
     */
    { // Assume an SCI device
        sciSetBaudrate(serialDevice[comPort], baud);
        sciEnableNotification(serialDevice[comPort],SCI_TX_INT|SCI_RX_INT);
        return (xSemaphoreTake(usartTxDoneSemaphore[comPort],SHORT_WAIT_TIME)==pdTRUE);
    }
//    return true;
}
/*-----------------------------------------------------------*/

bool SerialGetChar(COM_NUM com, char *rxedChar, portTickType timeout)
{
    /* Get the next character from the buffer.  Return false if no characters
     are available, or arrive before xBlockTime expires. */
    bool OSRunning = !(usartTxInUseSemaphore[com] == 0);
    bool bitBang = (serialDevice[com]==0);
    bool retVal = true;;
    //This test and set should be atomic, but we likely don't need this anyway!
    if(RxInUse[com])return false;
    RxInUse[com] = true;
    //
    if(OSRunning){
        /*
         * After the OS is started, both the bit-bang and the UART interfaces put received
         * characters into a queue
         */
        if (xQueueReceive(xRxedChars[com], rxedChar, timeout))
            retVal = TRUE;
        else
            retVal = FALSE;
    } else if(bitBang){
        /*
         * If the OS is not started we have to do this with bitBang...
         */
        charReady = false;
        while(!charReady){};
        *rxedChar = newChar;
    } else {
        /*
         * ...and this with the UART
         */
        sciReceiveByte(serialDevice[com]);
    }
    RxInUse[com] = false;
    return retVal;
}

void SerialPutStringInternal(COM_NUM com, const char* string, int length)
{

    //const UARTInfo *thisUART = ComInfoStructures[com];
    /*
     * Send a character string.  Note that some COM ports use DMA, and others do
     * not (they do per-character interrupts).  And one COM port is USB and branches off
     * to a completely different place.  And finally, there is the case of when we are
     * reporting an error.  In this case we don't want to use any OS features like queues and
     * waiting, so we just loop on the UART ready bit...no interrupts.
     *
     * Here is how you determine each of these conditions:
     *
     * 1) USB:  Does not have a thisUART structure.
     * 2) Does not use DMA;  DMA Channel is 0
     * 3) Error in progress:  "ErrorInProgress" global variable is true.
     */
    bool OSRunning = !(xCharsToTx[com] == 0);
    //const char *ourString = string;
    sciBASE_t *ourDevice = serialDevice[com];
    if (length == 0)
        length = strlen(string);

    if((unsigned int)ourDevice > 10){
        /*
         * Here we have a real UART.  We are going to use the HALCoGen routine.
         */
        sciSend(ourDevice, length, (uint8_t *)string);
        // In the bit-bang case, we wait after each character
        if(OSRunning){
            if(xSemaphoreTake(usartTxDoneSemaphore[com],SHORT_WAIT_TIME)!=pdTRUE){
                ReportError(SPIOperationTimeout,false,ReturnAddr,(int)__builtin_return_address(0));
            }
        }

    }
    /* Send each character in the string, one at a time. */
    else {

    }
}

void SerialPutString(COM_NUM com, const char* string, int length)
{

    //const UARTInfo *thisUART = ComInfoStructures[com];
    /*
     * Send a character string.  Note that some COM ports use DMA, and others do
     * not (they do per-character interrupts).  And one COM port is USB and branches off
     * to a completely different place.  And finally, there is the case of when we are
     * reporting an error.  In this case we don't want to use any OS features like queues and
     * waiting, so we just loop on the UART ready bit...no interrupts.
     *
     * Here is how you determine each of these conditions:
     *
     * 1) USB:  Does not have a thisUART structure.
     * 2) Does not use DMA;  DMA Channel is 0
     * 3) Error in progress:  "ErrorInProgress" global variable is true.
     */
    bool OSRunning = !(xCharsToTx[com] == 0);
    /*
     * Make sure someone else is not using the output.  If so, we can wait a few seconds, else pffft!
     */
    if(OSRunning){
        if (xSemaphoreTake(usartTxInUseSemaphore[com],SHORT_WAIT_TIME) != pdTRUE)
        {
            return;
        }
    }
    SerialPutStringInternal(com,string,length);
    if(OSRunning){
        xSemaphoreGive(usartTxInUseSemaphore[com]);
    }
}

/*-----------------------------------------------------------*/

bool SerialPutChar(COM_NUM comNum, char c, portTickType xBlockTime)
{
    char internalChar = c;
    sciBASE_t *ourDevice = serialDevice[comNum];
    bool OSRunning = !(xCharsToTx[comNum] == 0);
    if(OSRunning){
        if (xSemaphoreTake(usartTxInUseSemaphore[comNum],SHORT_WAIT_TIME) != pdTRUE){
            return false;
        }
    }

    {
        /*
         * Use the HALCoGen routine for a real UART.  It works both with and w/o the OS
         */
        if(OSRunning){
            SerialPutStringInternal(comNum,(const char *)&internalChar,1);
        } else {
            sciSend(ourDevice,1,(uint8 *)&internalChar);
        }
    }
    if(OSRunning){
        xSemaphoreGive(usartTxInUseSemaphore[comNum]);
    }
    return true;
}


/*
 * Here are the interrupt service routines.  For bitbang, it is the
 * RTI interrupt (interrupt per bit!).  For UART, it is the notification
 * routine from the HALCoGen support.
 */



/* Here is the timer interrupt for bitbang: */

__interrupt void RTI1Interrupt(void)
{
    rtiREG1->INTFLAG = 0x00000002U;
}


/*
 * Here is a callback from the HALCoGen sci support code for Rx
 */
void SerialRxCharacterInterrupt(sciBASE_t *sci,uint8_t byte){
    COM_NUM com = (sci==COM2_Port)?COM2:(sci==COM3_Port)?COM3:COM1;
    portBASE_TYPE higherPrioWoken;
    uint8_t thisChar = byte;
    xQueueSendFromISR(xRxedChars[com],&thisChar,&higherPrioWoken);
}

/*
 * Here is the notify routine from the HALCoGen sci support code for Tx
 * characters
 */

void sciNotification(sciBASE_t *sci, uint32 flags){
    COM_NUM com = (sci==COM2_Port)?COM2:(sci==COM3_Port)?COM3:COM1;
    portBASE_TYPE higherPrioWoken;

    xSemaphoreGiveFromISR(usartTxDoneSemaphore[com],&higherPrioWoken);

}
