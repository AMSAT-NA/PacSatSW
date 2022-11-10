
/*
 * CANSupport.c
 *
 *      Author: Burns Fisher
 */
#define CAN_SUPPORT_MODULE
#include <pacsat.h>
#include "stdint.h"
#include "stdio.h"
#include "string.h"
#include "inet.h"

#include "FreeRTOS.h"
#include "os_task.h"
#include "os_semphr.h"
#include "can.h"
#include "interTaskNotify.h"
#include "gpioDriver.h"
#include "CANSupport.h"
#include "canDriver.h"
#include "CANQueueManagement.h"
#include "MET.h"
#include "TelemetryCollection.h"
#include "CommandTask.h"
#include "canID.h"
extern bool AllTasksStarted,CoordinationMessageReceived;

// Forward routines
static void CANDispatchMessage(CANPacket_t *readData);


/*
 * Note:
 * This task is in some ways a driver--it is specific to the particular CAN device being used.  This
 * version is configured for the TMS570.  It has some expectations for the HALCoGen setup.  In particular,
 * it assumes that the message boxes 1-8 are set up for sending, with the DLC set for the number of the
 * message box.  So to send one data byte, use MB1, for 2, use MB2, etc.
 */

portTASK_FUNCTION_PROTO(CANTask, pvParameters)
{
    /*
     * Set up the software watchdog for the CAN task
     */
    vTaskSetApplicationTaskTag((xTaskHandle) 0, (pdTASK_HOOK_CODE)CANSupportWD );
    ReportToWatchdog(CurrentTaskWD);

    InitInterTask(ToCAN,20);
    CANInit(CAN1,ToCAN, CANMessageReceived);
    CANInit(CAN2,ToCAN, CANMessageReceived);

    /*
     * Now set up the CAN controller--interrupts, filters, etc--gets set up
     * by a change for satellite mode, which will surely happen at boot time.
     */
    CANInitQueue(CAN_Coordination_Queue);
    CANInitQueue(CAN_Uplink_Queue);

    /*
     * Loop forever
     */
    while(TRUE)
    {
        Intertask_Message msg;
        bool status;
        //CanIDNode myNodeName;
        //myNodeName = BusEnabled?RTIHU_Active:RTIHU_Standby;
        /*
         * empty the command queue... this acknowledges the request for more data
         */
        ReportToWatchdog(CANSupportWD);
        status = WaitInterTask(ToCAN, WATCHDOG_SHORT_WAIT_TIME, &msg);
        ReportToWatchdog(CANSupportWD);
        if(!status){
            continue;
        }
        switch(msg.MsgType){ // There is only one type for now
        default:
            break;

        case CANMessageReceived:{
            /*
             * The message is really just used as a doorbell to wake up this task.
             * We have to actually fetch the message
             */
            CANPacket_t readData;

            ReportToWatchdog(CurrentTaskWD);
            CANReadMessage((CANBus)msg.argument2,msg.argument,&readData); // Arg tells which bus and which message box
            if(((CANBus)msg.argument2 == CAN2) && (readData.ID.bits.source == MyLocalCanID)) {
                printf("Ignoring self message\n");
                return; // Ignore message to ourselves if we are standby
            }
            //printf("Box,ser=%d,%d ",msg.argument,readData.ID.bits.serialID2);
            CANDispatchMessage(&readData);
            break;
        }
        }
    }
}
void CANDispatchMessage(CANPacket_t *readData){
    CANPacket_t *queuePacket;
    Intertask_Message msg;
    DestinationTask destinationTask = ToNone;
    //CanID id;
    //id.fullWord = readData->ID.fullWord;

    queuePacket = NULL; //Init
    //	if(id.bits.source == CIU){
    //	    /*
    //	     * When we have the ID correct, this will go under telemetry
    //	     */
    //	    collectCIUData(readData);
    //	    return;
    //	}
    if(CANPrintAny){
        int j=0;
        printf("CAN Msg ID=0x%08x,DLC=%d,data=",readData->ID.fullWord,readData->dlc);
        while(j<readData->dlc){
            printf("0x%02x ",readData->data[j++]);
        }
        printf("\n");
    }
    switch(readData->ID.bits.type){
    case Telemetry:{
        /*
         * Telemetry comes in serial packets.  incomingCANTelemetry is in TelemetryCollection
         * but it actually calls back to CANReceiveLongMessage below.
         */
        incomingCANTelemetry(readData);
        ReportToWatchdog(CANSupportWD);
        break;
    }
    case Coordination: {
        //        logicalTime_t remTime;
        //        int i;
        //        char *pTimeBytes = (char *)&remTime;
        if(AllTasksStarted){
            queuePacket = CANGetQPacketToWrite(CAN_Coordination_Queue,true);
            destinationTask = ToCoordinate;
            msg.MsgType = CoordRxCANMsg;
            msg.argument = (uint32_t)queuePacket;
            ReportToWatchdog(CANSupportWD);
        } else {
            /*
             * This is really only useful on first boot in orbit.  For the standby processor
             * it tells us we have received a message from the active processor and we don't
             * have to be poised to take over the antenna opening anymore
             */
            CoordinationMessageReceived = true;
        }
        break;
    }
    case UplinkCommandOps:
    case UplinkCommandTelem:
    case UplinkCommandExp1:
    case UplinkCommandInternal:
    {
        uint16_t *wordData = (uint16_t *)&readData->data[0];
        destinationTask = ToCommand;
        msg.MsgType = CmdTypeValidatedSoftwareCAN;
        if(readData->ID.bits.type == UplinkCommandOps) msg.argument = SWCmdNSSpaceCraftOps;
        else if(readData->ID.bits.type == UplinkCommandTelem)msg.argument = SWCmdNSTelemetry;
        else if(readData->ID.bits.type == UplinkCommandExp1)msg.argument = SWCmdNSExperiment1;
        else if(readData->ID.bits.type == UplinkCommandInternal)msg.argument = SWCmdNSInternal;
        msg.argument2 = (int)(readData->ID.bits.serialID2.serial) | (int)((readData->ID.bits.ID1))<<8;
        msg.data[0] = ntohs(wordData[0]); // Make sure endianness is correct
        msg.data[1] = ntohs(wordData[1]);
        msg.data[2] = ntohs(wordData[2]);
        msg.data[3] = ntohs(wordData[3]);
        if(CANPrintCommands){
            printf("--Cmd with ID %x originating from node %d (we are %d)\n",readData->ID.fullWord,
                   readData->ID.bits.source,MyLocalCanID);
        }
        {
            if(readData->ID.bits.source == LIHU){
                // We are the active processor, and this started from the LIHU
                // Pass it to the standby processor.
                readData->ID.bits.dest = PartnerLocalCanID;
                if(CANPrintCommands){
                    printf("++Cmd to Standby RTIHU (%x)\n",readData->ID.fullWord);
                }
                CAN_WRITE_TRY2(CAN2,readData,false);
            } else {
                // We are active and this was NOT from the LIHU.  Must have
                // been from the standby, so send it to the LIHU.
                readData->ID.bits.dest = LIHU;
                if(CANPrintCommands){
                    printf("++Cmd to LIHU (ID=%x)\n",readData->ID.fullWord);
                }
                readData->ID.bits.dest = LIHU;
                CAN_WRITE_TRY2(CAN1,readData,false);
                ReportToWatchdog(CANSupportWD);

            }
        }
        break;
    }

    default:
    {
        debug_print("Unknown CAN msg: ID=%x,type=%d,Source=%d,serialID2=%d,ID1=%x,prio=%d,dlc=%d\n",
                    (unsigned int)readData->ID.fullWord,readData->ID.bits.type,readData->ID.bits.source,
                    readData->ID.bits.serialID2.serial,
                    readData->ID.bits.ID1,readData->ID.bits.priorityBits,readData->dlc);
        break;
    }
    }
    if(queuePacket != NULL){
        memcpy(&queuePacket->data,&readData->data,readData->dlc);
        queuePacket->dlc = readData->dlc;
        queuePacket->ID = readData->ID;
    }
    if(destinationTask != ToNone){
        NotifyInterTask(destinationTask,WATCHDOG_SHORT_WAIT_TIME, &msg);
    }
}

int CANPrintMessage(uint8_t *message){
    /*
     * Print out a single CAN message
     */
    CANPacket_t *canmsg;
    int size,i;
    canmsg = (CANPacket_t *)message;
    size=canmsg->dlc;
    if( canmsg->ID.fullWord != 0) {

        printf("CAN size=%d,id=%08x, data=",
               size,canmsg->ID.fullWord);
        for (i=0; i< size; i++){
            printf("%02x ",message[4+i]);
        }
        printf("]\n\r");
    }
    return size;
}

void CANInitLongMessage(CANLongBitmap *bitmap){
    bitmap->lastUseTime = 0;
    bitmap->bitmap = 0;
}

bool CANReceiveLongMessage(CANPacket_t *packetToAdd,uint8_t *messageBytes,CANLongBitmap *messageInfo,uint32_t structSize){
    /*
     * This is how we receive a multi-CAN message data packet.  We keep adding new packets until we return true,
     * meaning that all the expected packets have arrived.
     *
     * messageBytes is an array into which the data is placed after it is collected
     * bitmap is a 64-bit word which is opaque to the caller but is used to keep track of which packets have arrived
     * expectedSize is passed in by the caller to tell us the size of the array it is giving us.  It will be returned
     *    as the number of bytes actually sent.
     */
    int packetIndex,messageStart,i;
    int numPackets;   // Number of full 8-byte CAN packets
    int bytesInPacket = packetToAdd->dlc;
    uint64_t bitMapMax,oring;
    uint32_t timeNow = getSeconds();
    /*
     * First, in case of a previous error, we start over again if a long message took more than 3 seconds
     */
    if((timeNow - messageInfo->lastUseTime) > CAN_LONG_MESSAGE_TIMEOUT){
        messageInfo->bitmap = 0;
    }
    messageInfo->lastUseTime = timeNow;

    /*
     * Now figure out where in the long message this packet goes based on the serial number in
     * its ID.
     */

    packetIndex = packetToAdd->ID.bits.serialID2.serial;

    messageStart = packetIndex * 8;
    numPackets= structSize/CAN_MAX_DATA + (((structSize%CAN_MAX_DATA) != 0)?1:0);
    bitMapMax = (((uint64_t)1) << numPackets) - 1; // What will be the value of the bitmap if all packets are in

    if(packetIndex > numPackets){
        printf("***Something is screwed up\n");
        return false;
    }
    if((messageInfo->bitmap & (((uint64_t)1)) << packetIndex)!=0){
        /*
         * This serial number was already filled in without ALL serial numbers being found.
         * Clear the bitmap and start again.  (This does not fully solve the problem, but might be an improvement
         */
#ifdef UNDEFINE_BEFORE_FLIGHT
        printf("***Zeroing on previous incomplete, packetIndex = %d\n",packetIndex);
#endif
        messageInfo->bitmap = 0;
    }
    oring = (((uint64_t)1) << packetIndex); // Set the bit representing this packet
    //printf("Oring %x with %x; ",(uint32_t)(oring&0xffffffff),(uint32_t)(*bitmap&0xffffffff));
    messageInfo->bitmap = messageInfo->bitmap | oring;
    //printf(" getting %x\n",(uint32_t)(*bitmap&0xffffffff));
    for(i=0;i<bytesInPacket;i++){
        messageBytes[messageStart + i] = packetToAdd->data[i];
    }
    return ((messageInfo->bitmap) == bitMapMax);
}

void CANSendLongMessage(CANBus bus, CanID1 ID, CanIDType type, uint8_t priority,uint8_t *bytes,uint32_t size){
    /*
     * This routine takes a bunch of data longer than the max of 8 bytes allowed in a CAN
     * message and breaks it into several message.  You specify the bus, and the basic parts
     * of the ID.  This routine adds the source and a serial number (serial within this long
     * message, that is
     */
    CANPacket_t message;
    CanID messageID;
    int srcIndex,serNum,packetIndex;
    if(bus == CAN2){
        messageID.bits.source = MyLocalCanID;
        messageID.bits.dest = PartnerLocalCanID;
    } else {
        messageID.bits.source = RTIHU_Active;
        messageID.bits.dest = LIHU;
    }
    messageID.bits.ID1 = ID;
    messageID.bits.priorityBits = priority;
    messageID.bits.type = type;

    //Initialize where we are starting in the source array, and the first packet dlc
    srcIndex = 0;
    message.dlc=8;

    if(CANPrintTelemetry && (type == Telemetry)){
        char *typeString;
        messageID.bits.serialID2.serial = 0; //Just to make it print consistently
        switch (ID){
        case CANId1InterCPUTelem_Short:
            typeString = "Local Telemetry";
            break;
        case CANId1InterCPUTelem_HkWOD:
            typeString = "Housekeeping WOD";
            break;
        case CANId1InterCPUTelem_RadWOD:
            typeString = "Science WOD";
            break;
        case CANId1InterCPUTelem_RagWOD:
            typeString = "Ragnarok WOD";
            break;
        case CANId1InterCPU_RT1Diag:
            typeString = "primary CPU diagnostics";
            break;
        case CANId1InterCPU_RT2Diag:
            typeString = "secondary CPU diagnostics";
            break;
        default:
            typeString = "Unknown";
        }
        printf("Tx %s data, bus=%d, ID=%x, length=%d\n",typeString,bus,messageID.fullWord,size);
    }
    for(serNum=0;;serNum++){
        // Each one of these is a CAN packet
        messageID.bits.serialID2.serial = serNum;
        message.ID.fullWord = messageID.fullWord;
        for(packetIndex = 0;(srcIndex<size)&&(packetIndex<8);srcIndex++,packetIndex++){
            message.data[packetIndex] = bytes[srcIndex];
        }
        if(packetIndex < 8){
            message.dlc = packetIndex;
        }
        CAN_WRITE_TRY2(bus,(CANPacket_t *)&message,true);
        vTaskDelay(2);
        if(srcIndex>=size)break;
    }

}
