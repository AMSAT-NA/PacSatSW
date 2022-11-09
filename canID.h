/*
 * GolfCanID.h
 *
 *  Created on: Apr 24, 2020
 *      Author: bfisher
 */

#ifndef CANID_H_
#define CANID_H_


typedef enum _CanIDNodes {
    Any_Node = 0,
    RTIHU_Active=1,
    RTIHU_Primary=1,
    RTIHU_Secondary,
    LIHU,
    Ragnarok,
    CIU,
    Ettus
} CanIDNode;
typedef enum _CanIDType {
    NoIDType = 0,
    Test=1,
    Telemetry,
    Deployment,
    Coordination,
    UplinkCommandInternal,
    EttusCmd=6,
    UplinkCommandOps,
    UplinkCommandTelem,
    UplinkCommandExp1,
    UplinkCommandRagnarok

} CanIDType;

typedef enum _CanID1Type {
    CANId1NoSerial = 0,
    CANId1InterCPUTelem_Short = 1,
    CANId1InterCPUTelem_HkWOD = 2,
    CANId1InterCPUTelem_RadWOD = 3,
    CANId1InterCPUTelem_RagWOD = 4,
    CANId1InterCPU_RT1Diag = 5,
    CANId1InterCPU_RT2Diag = 6,
    CANId1InterCPU_LDiag = 7,
    CANID1EttusFileData = 8
} CanID1;

typedef enum _CanID2Type {
    CANId2EttusEcho = 1,
    CANId2EttusStartSDR = 2,
    CANId2EttusStopSDR = 3,
    CANId2EttusShutdown = 4,
    CANId2OpenCloseFile = 5,
    CANId2EttusOperationComplete = 6, // This is to indicate startup or shutdown complete
    CANId2EttusFileAcknowledge = 7, //Acknowledge a file open, close, or data acceptance
    CANId2EttusTelemUTCTemp = 8,
    CANID2EttusTelemIMU = 9
} CanID2;

typedef union _ID2Serial {
    uint8_t serial;
    CanID2 ID2;
} CanID2Serial;

typedef struct _CanIDBits_ {
#ifdef THIS_IS_LIHU /* This is for little endian and the C compiler used for the LIHU*/
    CanID2Serial serialID2;
    CANID1Type ID1:4;
    CanIDNode dest:4;
    CanIDType type:4;
    CanIDSource source:4;
    unsigned int priorityBits:5;
    unsigned int pad:3; /* To make 32 bits */

#else /*This is for big endian and the C compiler used by CCS*/
unsigned int pad:3; /* To make 32 bits */
unsigned int priorityBits:5;
CanIDNode source:4;
CanIDType type:4;
CanIDNode dest:4;
CanID1 ID1:4;
CanID2Serial serialID2;
#endif
} CanIDBits_t;

typedef union _CanID_ {
    CanIDBits_t bits;
    uint32_t fullWord;
    uint8_t byteArray[4];
} CanID;
#ifndef CAN_SUPPORT_MODULE
extern CanIDNode MyLocalCanID,PartnerLocalCanID;
#else
CanIDNode MyLocalCanID,PartnerLocalCanID;
#endif

#endif /* CANID_H_ */
