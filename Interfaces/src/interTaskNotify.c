/*
 * interTaskNotify.c
 *
 *  Created on: Dec 27, 2012
 *      Author: Burns Fisher, AMSAT NA
 */
#include <pacsat.h>
#include "FreeRTOS.h"
#include "interTaskNotify.h"

/*
 * Global within this module
 */
static xQueueHandle MessageQueues[NumberOfDestinations]= { 0, 0 };

void InitInterTask(DestinationTask type,int length)
{
    MessageQueues[type] = xQueueCreate(length, sizeof(Intertask_Message));
}

bool WaitInterTask(DestinationTask type, int timeout, Intertask_Message *data)
{
    return xQueueReceive(MessageQueues[type], data, timeout);
}

int WaitingInterTask(DestinationTask type)
{
    return uxQueueMessagesWaitingFromISR(MessageQueues[type]);
}

bool NotifyInterTask(DestinationTask type, int timeout, Intertask_Message *data)
{
    /*
     * Send one piece of data to the waiting task.
     */
    return xQueueSend(MessageQueues[type],data,timeout);
}

bool NotifyInterTaskFromISR(DestinationTask type, Intertask_Message *data)
{
	signed portBASE_TYPE higherPrioTaskWoken;

	/*
	 * Send one piece of data to the waiting task called from an ISR.
	 */
	return xQueueSendFromISR(MessageQueues[type], data,
				 &higherPrioTaskWoken);
}
