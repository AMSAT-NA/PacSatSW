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
#include "canDriver.h"
#include "CANQueueManagement.h"

/*
 * Here is an array of pointers to the sizes (indexed by CANQType)
 */
static int QueueSizes[] = {CAN_COORD_QSIZE,CAN_UPLINK_QSIZE};
/*
 * Here are the queue data arrays and array of pointers to them (indexed by CANQType)
 */
static CANPacket_t CoordArray[CAN_COORD_QSIZE],UplinkArray[CAN_UPLINK_QSIZE];
static CANPacket_t *PacketArrays[]={CoordArray,UplinkArray};
/*
 * Here are the queue headers and an array of pointers to them (indexed by CANQType)
 */
static CanQ_t CanCoordQueue,CanUplinkQueue;
static CanQ_t *CANQueues[] = {&CanCoordQueue,&CanUplinkQueue};



/*
 * A queue to hold CAN packets received from the rest of the spacecraft.
 * Implemented as an array treated as a circular buffer.  When start and
 * end indices are the same, the queue is full or empty, indicated by the
 * boolean "IsEmpty"
 */
int CANGetReadIndex(CANQType qnum){
	return CANQueues[qnum]->QReadIndex;
}
int CANGetWriteIndex(CANQType qnum){
	return CANQueues[qnum]->QWriteIndex;
}

int CANGetNumberOfPackets(CANQType qNum){
	CanQ_t *queue = CANQueues[qNum];
	int read=queue->QReadIndex;
	int write=queue->QWriteIndex;
	if(queue->QIsEmpty)
		return 0;
	if(read<write)return write-read;
	return (queue->QSize -(read-write));
}
CANPacket_t *CANGetQPacketToRead(CANQType qNum,bool *hasOverflowed){
	CanQ_t *queue = CANQueues[qNum];
	if(queue->QIsEmpty){
	   queue->QHasOverflowed = false;
	   *hasOverflowed = false;
		return NULL;
	} else {
		int index = queue->QReadIndex;
	    *hasOverflowed = queue->QHasOverflowed;
		return &queue->Packets[index];
	}
}
void CANIncrementQReadPointer(CANQType qNum){
	CanQ_t *queue = CANQueues[qNum];
	queue->QReadIndex++;
	queue->QHasOverflowed = false;
	if ((queue->QReadIndex) >= queue->QSize) {
		queue->QReadIndex = 0;
	}
	if(queue->QReadIndex == queue->QWriteIndex){
		queue->QIsEmpty = true;
	}

}
CANPacket_t *CANGetQPacketToWrite(CANQType qNum,bool reportOverflow){
	CanQ_t *queue = CANQueues[qNum];
	bool queueFull = ((!queue->QIsEmpty) && (queue->QReadIndex == queue->QWriteIndex));

	if(reportOverflow && queueFull){
		queue->QHasOverflowed = true;
		debug_print("Queue overflowed with read=%d,write=%d,full=%d,queue=%d,size=%d\n\r",
				queue->QReadIndex,queue->QWriteIndex,queueFull,qNum,queue->QSize);
	}
	{
		int index = queue->QWriteIndex++;
		if ((queue->QWriteIndex) >= queue->QSize) {
			queue->QWriteIndex = 0;
		}
		queue->QIsEmpty = false;
		if(queueFull){
			/*
			 * Here we are overwriting the oldest entry in the queue, so
			 * it WILL be the newest.  Increment the read pointer to point
			 * at the new oldest entry
			 */
			queue->QReadIndex++;
			if(queue->QReadIndex >= queue->QSize){
				queue->QReadIndex = 0;
			}
		}
		return &queue->Packets[index];
	}
}
void CANInitQueue(CANQType qNum){
	CanQ_t *queue = CANQueues[qNum];
	queue->Packets = PacketArrays[qNum];
	queue->QHasOverflowed = false;
	queue->QIsEmpty = true;
	queue->QReadIndex = 0;
	queue->QWriteIndex = 0;
	queue->QSize = QueueSizes[qNum];

}
CanQ_t *CANGetQPtr(CANQType qNum){
	return CANQueues[qNum];
}

