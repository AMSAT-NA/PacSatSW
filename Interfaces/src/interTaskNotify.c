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
static xQueueHandle MessageQueues[NumberOfDestinations]={0,0};

void InitInterTask(DestinationTask type,int length){
	MessageQueues[type] = xQueueCreate(length, ( unsigned portBASE_TYPE )sizeof(Intertask_Message));
}
bool WaitInterTask(DestinationTask type, int timeout, Intertask_Message *data){
	int status;
	//Intertask_Message readData;
	//status = (bool)xQueueReceive(MessageQueues[type], &readData, timeout );
	//*data = readData;
    status = (bool)xQueueReceive(MessageQueues[type], data, timeout );
	return status; // Return false for failure
}
int WaitingInterTask(DestinationTask type){
    return uxQueueMessagesWaitingFromISR(MessageQueues[type]);
}
bool NotifyInterTask(DestinationTask type, int timeout, Intertask_Message *data){
	/*
	 * Send one piece of data to the waiting task.
	 */
	bool status;
	status = xQueueSend(MessageQueues[type],data,timeout);
	return status;
}
bool NotifyInterTaskFromISR(DestinationTask type, Intertask_Message *data){
	signed portBASE_TYPE higherPrioTaskWoken;
	/*
	 * Send one piece of data to the waiting task called from an ISR.
	 */
	return xQueueSendFromISR(MessageQueues[type],data,&higherPrioTaskWoken);
}
