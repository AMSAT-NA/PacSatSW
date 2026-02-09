/*
 * interTaskNotify.h
 *
 *  Created on: Dec 27, 2012
 *      Author: fox
 */

#ifndef INTERTASKNOTIFY_H_
#define INTERTASKNOTIFY_H_
/*
 * FreeRTOS includes
 */
#include "FreeRTOS.h"
#include "os_semphr.h"
#include "os_queue.h"
#include "can.h"
#include "intertaskMessageDefinitions.h"
#include "UplinkCommands.h"

typedef enum {
    ToNone=-1,
    ToDownlinkControl=0,
    ToCommand,
    ToTelemetryAndControl,
    ToCANTask,
    // New message destinations go above here
    ToCoordinate,
    NumberOfDestinations
} DestinationTask;


/*
 * Here are the message types for each destination
 */


typedef struct {
    IntertaskMessageType MsgType;
    uint16_t argument;
    uint16_t argument2;
    uint16_t data[4];
}Intertask_Message;


void InitInterTask(DestinationTask type, int length);
bool WaitInterTask(DestinationTask type, int timeout, Intertask_Message *data);
bool NotifyInterTask(DestinationTask type,int timeout, Intertask_Message *data);
bool NotifyInterTaskFromISR(DestinationTask type, Intertask_Message *data);
int WaitingInterTask(DestinationTask type);

#endif /* INTERTASKNOTIFY_H_ */
