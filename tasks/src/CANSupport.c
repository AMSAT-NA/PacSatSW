
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

    /*
     * These init routines set up so that an incoming interrupt from either of these devices usese
     * the inter-task message system to send a message to this task
     */

    CANInit(CAN1,ToCAN, CANMessageReceived);
    CANInit(CAN2,ToCAN, CANMessageReceived);

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
    //CanID id;
    //id.fullWord = readData->ID.fullWord;

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


    default:
    {
        debug_print("Unknown CAN msg: ID=%x,type=%d,Source=%d,serialID2=%d,ID1=%x,prio=%d,dlc=%d\n",
                    (unsigned int)readData->ID.fullWord,readData->ID.bits.type,readData->ID.bits.source,
                    readData->ID.bits.serialID2.serial,
                    readData->ID.bits.ID1,readData->ID.bits.priorityBits,readData->dlc);
        break;
    }
    }
#if 0
    if(queuePacket != NULL){
        memcpy(&queuePacket->data,&readData->data,readData->dlc);
        queuePacket->dlc = readData->dlc;
        queuePacket->ID = readData->ID;
    }
    if(destinationTask != ToNone){
        NotifyInterTask(destinationTask,WATCHDOG_SHORT_WAIT_TIME, &msg);
    }
#endif
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


