/*
 * canDriver.c
 *
 *  Created on: Jun 28, 2019
 *      Author: bfisher
 */

#include "canDriver.h"
#include "interTaskNotify.h"
#include "errors.h"

#define LOW_TX_MSG_BOX 1
#define HIGH_TX_MSG_BOX 9

#define CAN_MAX_DATA 8
#define CAN_NUMBER_OF_DLCS 9 /* Can be 0 through 9 */
static bool canFailure[NumberOfCANBuses]={false,false},can2Received = true,can2PartnerOk=true;
static xSemaphoreHandle CANTxBusySemaphore[NumberOfCANBuses][CAN_NUMBER_OF_DLCS];
static xQueueHandle CANRxMessageBoxesQ[NumberOfCANBuses];


/*
 * Send a message to the CAN bus
 */

canBASE_t *BusToBase[]={
                        canREG1,
                        canREG2,
                        canREG3
};


static  IntertaskMessageType CANMessage[NumberOfCANBuses];
static DestinationTask CANMessageDestination[NumberOfCANBuses];
static uint32_t messageBoxID[3][9]={{0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0}};
extern bool TestPlayDead; // Pretend this CPU is dead and can't send any messages

void CANRestart(){
    int j;
    /*
     * This routine resets the CAN hardware and prepares it for use
     * again after the bus has been disconnected or there is some other
     * failure.
     */
    canInit(); /* Reset the CAN hardware */
    /*
     * Make sure that all the transmit semaphores are released
     */
    for(j=0;j<CAN_NUMBER_OF_DLCS;j++){
        if(CANTxBusySemaphore[CAN1][j] != 0){
            xSemaphoreGive(CANTxBusySemaphore[CAN1][j]);   //Release the semaphore.  Will return error if already released
        }
        messageBoxID[CAN1][j] = 0;
    }
    for(j=0;j<CAN_NUMBER_OF_DLCS;j++){
        if(CANTxBusySemaphore[CAN2][j] != 0){
            xSemaphoreGive(CANTxBusySemaphore[CAN2][j]);   //Release the semaphore.  Will return error if already released
        }
        messageBoxID[CAN2][j] = 0;
    }
    canEnableErrorNotification(canREG1);
    canEnableErrorNotification(canREG2);
    canFailure[CAN1] = false;
    canFailure[CAN2] = false;
}
bool CANInit(CANBus bus,DestinationTask task, IntertaskMessageType msg){
    bool status=true;
    static bool isInitted=false;
    int j;
    CANBus i;
    if(!isInitted){
        // This stuff only gets done once on the first time any bus is initialized
        canInit();
        for(j=0;j<NumberOfCANBuses;j++){
            i = (CANBus)j;
            CANMessage[i] = (IntertaskMessageType)0;
            CANMessageDestination[i] = (DestinationTask)0;
            CANTxBusySemaphore[i][0] = 0;
            CANRxMessageBoxesQ[i] = 0;

        }
        isInitted = true;
    }
    if(task != NO_TASK){
        /*
         * Here we are a asking for a task message to be sent when a CAN message comes in.
         */
        CANMessage[bus] = msg;
        CANMessageDestination[bus] = task;
    } else {
        CANRxMessageBoxesQ[bus] =
                xQueueCreate(CAN_Q_LENGTH, (unsigned portBASE_TYPE)sizeof(uint32_t));
    }
    for(j=0;j<CAN_NUMBER_OF_DLCS;j++){
        // Make a semaphore for each possible DLC since which DLC goes into a different
        // hardware message box
        vSemaphoreCreateBinary(CANTxBusySemaphore[bus][j]);
    }
    canEnableErrorNotification(BusToBase[bus]);
    canFailure[bus] = false;
    can2Received = true;
    return status;
}
bool Can2PartnerIsOk(void){
    return can2PartnerOk;
}
void Can2TimeoutTick(void){
    // This routine kind of double-buffers the can2ok bit.  We have to get two ticks
    // in a row without seeing a message on CAN2 before we return failure in IsCan2Ok.
    static int count=3;
    if(can2Received){
        count=3;
        can2PartnerOk=true;
        can2Received = false; // This will be false until we get a new message
    } else {
        if(--count <= 0){
            count = 0;  // Don't keep subtracting till it wraps!
            can2PartnerOk=false;
        }
    }
}
bool CANWriteMessage(CANBus bus,CANPacket_t *packet,bool highPrio){
    bool status = false;
    uint32_t msgBox,ID;
    canBASE_t *canRegs = BusToBase[bus];
    /*
     * The msgBox number is the same as the DLC.  Hopefully the code below makes that
     * clear but the compiler will turn it into the obvious one-liner
     */
    const uint32_t messageBoxDLC[]={
                                    canMESSAGE_BOX1,
                                    canMESSAGE_BOX2,
                                    canMESSAGE_BOX3,
                                    canMESSAGE_BOX4,
                                    canMESSAGE_BOX5,
                                    canMESSAGE_BOX6,
                                    canMESSAGE_BOX7,
                                    canMESSAGE_BOX8,
                                    canMESSAGE_BOX9

    };
    int thisDlc = packet->dlc;
#ifdef DEBUG
    // We should really call the error routine
    if(thisDlc > 8){
        printf("DLC is greater than 8!  Error!\n");
    }
#endif
    if(canFailure[bus]){
        //printf("Failure type is %x\n",failVal);
        CANRestart();
        return false;
    }
#ifdef UNDEFINE_BEFORE_FLIGHT
    if(TestPlayDead){
        // We are playing dead.  Don't send any can2 messages
        if(bus==CAN2)return false;
    }
#endif
    if(BusIsDisabled()){
        // If we don't have the bus, i.e. we are the standby, we can't send on CAN1
        if(bus==CAN1)return false;
    } else {
        // If we DO have the bus, we only send to the standby processor on CAN2 if
        // it lives or if there are no messages waiting to be sent.  If we ARE the
        // standby, we always try to send to the active
        if(
                (!Can2PartnerIsOk()) &&
                (uxQueueMessagesWaiting(CANTxBusySemaphore[bus][thisDlc]) == 0)
            )return false;
    }
    msgBox = messageBoxDLC[thisDlc];
    ID = packet->ID.fullWord;
    if(ID != messageBoxID[bus][thisDlc]) {
        //Only change the ID if we have to and...
        // We have to set it to transmit and use extended id
        canUpdateID(canRegs, msgBox, (ID | 0x60000000));
        //...remember what the value is.
        messageBoxID[bus][thisDlc] = ID;
    }
    /*
     * We can probably make a separate semaphore for each message box, but let's try
     * just one first.  We do not want the HalCoGen code looping waiting for a bit to set!
     */

    if (xSemaphoreTake(CANTxBusySemaphore[bus][thisDlc],WATCHDOG_SHORT_WAIT_TIME)!= pdTRUE){
#ifdef UNDEFINE_BEFORE_FLIGHT
        printf("Can timeout--restarting\n");
#endif
            CANRestart(); // Chances are that we switched processors.  Cancel the previous message
            return false;
            //ReportError(SemaphoreFail,true,CharString,(int)"CANTakeSema");
    }
    ReportToWatchdog(CurrentTaskWD);
    status = canTransmit(canRegs,msgBox,packet->data);
    return status;
}
/*
 * This routine works not on messages passed to a task, but rather on a local
 * queue into which the driver puts the message box of a received message.
 */
bool CANWaitAnyMessage(CANBus bus,CANPacket_t *packet,portTickType timeout){
    uint8_t msgBox;
    if(CANRxMessageBoxesQ[bus]==0)
        return false;
    if (!xQueueReceive(CANRxMessageBoxesQ[bus], &msgBox, timeout))
        return false;
    return CANReadMessage(bus,msgBox,packet);

}
bool CANReadMessage(CANBus bus,uint8_t messageBox,CANPacket_t *readData){

    /*
     * We have at least one CAN message in the buffers.  Read the buffers
     * that have messages and queue them for collection by 'telemetry
     * collection' or WOD.xx
     */
    bool retVal;
    int readRet;
    canBASE_t *canRegs = BusToBase[bus];

    // extract message from message object
    // Note that canGetData is customized.
    readRet = canGetData(canRegs, messageBox, readData->data);
    if((readRet & 3) == 1){
        readData->dlc = readRet>>8;
        readData->ID.fullWord = canGetID(canRegs,messageBox);
        retVal = true;

    } else {
        // Here there is either no data, or it overflowed
        retVal = false;
    }
    return retVal;
}


void canMessageNotification(canBASE_t *node, uint32 messageBox) {
    Intertask_Message msg;
    CANBus bus;
    bus = CANRegToBus(node);
    if(messageBox>=LOW_TX_MSG_BOX && messageBox<=HIGH_TX_MSG_BOX){
        // This is reporting a transmitted message completed, so release
        // the Tx semaphore
        BaseType_t higherPrioTaskWoken;
        (void)(xSemaphoreGiveFromISR(CANTxBusySemaphore[bus][messageBox-1],&higherPrioTaskWoken));
    } else {
        // Here we know that this is reporting an incoming message
        // If the bus init requested a message, send it.  Otherwise,
        // queue the message box number
        if(bus==CAN2){ //This message came from the other IHU
            static bool isOff=false;
            if(isOff){
                GPIOSetOn(LED2);
                isOff = false;
            } else {
                GPIOSetOff(LED2);
                isOff = true;
            }
            can2PartnerOk = can2Received= true;
        }
        if(CANMessageDestination[bus]!=0){
            msg.MsgType = CANMessage[bus];
            msg.argument = messageBox;
            msg.argument2 = bus;
            NotifyInterTaskFromISR(CANMessageDestination[bus],&msg);
        } else {
            BaseType_t higherPrioWoken;
            uint8_t msgBox = (uint8_t)messageBox;
            xQueueSendFromISR(CANRxMessageBoxesQ[bus],&msgBox,&higherPrioWoken);
        }
    }
}
void canErrorNotification(canBASE_t *node, uint32_t notification){
    canFailure[CANRegToBus(node)] = true;
}

CANBus CANRegToBus(canBASE_t *bus){
    if(bus == canREG1)return CAN1;
    else if(bus == canREG2)return CAN2;
    else return CAN3;

}
