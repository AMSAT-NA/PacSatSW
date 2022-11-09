/*
 * canDriver.h
 *
 *  Created on: Jun 28, 2019
 *      Author: bfisher
 */

#ifndef DRIVERS_INC_CANDRIVER_H_
#define DRIVERS_INC_CANDRIVER_H_
#include "can.h"
#include "canID.h"
#include "interTaskNotify.h"

// This is used internally to represent a CAN packet.  To downlink, the dlc
// is represented as 3 bits in the top of the ID and 0-7 in those 3 bits represents
// a dlc of 1-8.  Here the dlc is the actual number

typedef struct  _CANPacket_t {
    CanID ID;
    uint8_t dlc;
    uint8_t data[8];
} CANPacket_t;

typedef enum _canBus{
    CAN1=0,
    CAN2,
    CAN3,
    NumberOfCANBuses
} CANBus;

#define NO_TASK (DestinationTask)-1
#define CAN_Q_LENGTH 5
bool CANInit(CANBus bus,DestinationTask task, IntertaskMessageType msg);
bool CANWriteMessage(CANBus bus,CANPacket_t *packet,bool highPrio);
bool CANReadMessage(CANBus bus,uint8_t messageBox,CANPacket_t *readData);
void CANRestart(void);
CANBus CANRegToBus(canBASE_t *bus);
void Can2TimeoutTick(void); //Call from timer to allow a timeout on CAN2.
bool Can2PartnerIsOk(void); // Call to determine if CAN2 partner is sending stuff.


#endif /* DRIVERS_INC_CANDRIVER_H_ */
