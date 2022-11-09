/*
 * CANSupport.h
 *
 *  Created on: Apr 27, 2018
 *      Author: fox
 */

#ifndef CANSUPPORT_H_
#define CANSUPPORT_H_
#include "canID.h"
#include "canDriver.h"
#include "can.h"

typedef struct _LongCanBitmap {
    uint64_t  bitmap;
    uint32_t lastUseTime;
}CANLongBitmap;


void CANTask(void *pvParameters);


#define MAX_CAN_PACKET_SIZE 12 /* 12 bytes including the ID)*/

#define CAN_MAX_DATA 8
#define CAN_PRIORITY_HIGH 0
#define CAN_PRIORITY_MEDIUM 16
#define CAN_PRIORITY_LOW 31


#define CAN_MAX_DATA 8

#define CAN_MRAM_MAX_BYTES 200

#define CAN_PRIORITY_HIGH 0
#define CAN_PRIORITY_MEDIUM 16
#define CAN_PRIORITY_LOW 31

int CANPrintMessage(uint8_t *message);
void CANPrintPayload(uint8_t *payload, int maxSize);

void CANResetAndSetup(uint8_t groundClass);
bool CANInterruptCallback(void *);
void CANGPIOsInitialize(void);
void CANSendLongMessage(CANBus bus,CanID1 ID, CanIDType type, uint8_t priority,uint8_t *bytes,uint32_t size);
//void CANSendLongMessage2Input(CANBus bus,uint16_t ID, CanIDType type, uint8_t priority,
//                              uint8_t *bytes0,uint8_t size0,uint8_t *bytes,uint32_t size);

bool CANReceiveLongMessage(CANPacket_t *packetToAdd,uint8_t *messageBytes,CANLongBitmap *bitmap,uint32_t structSize);
void CANInitLongMessage(CANLongBitmap *);
#define CAN_LONG_MESSAGE_TIMEOUT 3 /* Seconds */

#define CAN_WRITE_TRY2(canID,pPacket,priority) if(!CANWriteMessage(canID,pPacket,priority)){ \
    ReportToWatchdog(CurrentTaskWD);\
    CANWriteMessage(canID,pPacket,priority);\
}


#ifndef MAIN_FUNCTION
extern bool CANPrintTelemetry,CANPrintCoord,CANPrintCommands,CANPrintAny,CANPrintCount,CANPrintErrors,CANPrintEttus;
#endif

#endif /* CANSUPPORT_H_ */
