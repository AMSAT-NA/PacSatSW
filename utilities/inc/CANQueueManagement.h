/*
 * CANQueueManagement.h
 *
 *  Created on: May 23, 2018
 *      Author: fox
 */

#ifndef CANQUEUEMANAGEMENT_H_
#define CANQUEUEMANAGEMENT_H_

/*
 * CANQueueManagement.c
 *
 *  Created on: May 23, 2018
 *      Author: fox
 */
#include <pacsat.h>
#include "stdint.h"
#include "stdio.h"
#include "string.h"

#include "FreeRTOS.h"
#include "os_task.h"
#include "os_semphr.h"

#include "interTaskNotify.h"
#include "gpioDriver.h"
#include "CANSupport.h"


#define CAN_COORD_QSIZE 5
#define CAN_TELEM_QSIZE 50
#define CAN_UPLINK_QSIZE 5
/*
 * Routines to manage queues of CAN messages.  There are two: telem and WOD.
 * Both the "read" and "write" routines return the address of a CAN packet to
 * copy from or to.  "Write" automatically increments the pointers.  "Read"
 * requires you to call CANIncrementQReadPointer when you have determined that
 * you are actually going to use the CAN message that you have gotten.
 */
typedef enum {
    CAN_Coordination_Queue=0,
    CAN_Uplink_Queue
}CANQType;

/*
 * Here is the queue itself
 */
typedef struct _CanQ_ {
	CANPacket_t *Packets;
	int QWriteIndex;
	int QReadIndex;
	int QSize;
	bool QIsEmpty;
	bool QHasOverflowed;
} CanQ_t;

void CANInitQueue(CANQType qNum);
void CANIncrementQReadPointer(CANQType qNum);
CANPacket_t *CANGetQPacketToRead(CANQType qNum,bool *hasOverflowed);
CANPacket_t *CANGetQPacketToWrite(CANQType qNum,bool overwrite);
// These are for debugging
CanQ_t *CANGetQPtr(CANQType qNum);
int CANGetNumberOfPackets(CANQType qNum);
int CANGetReadIndex(CANQType qnum);
int CANGetWriteIndex(CANQType qnum);


#endif /* CANQUEUEMANAGEMENT_H_ */
