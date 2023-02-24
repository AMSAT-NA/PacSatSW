/*
 * RxTask.c
 *
 *  Created on: Dec 1, 2022
 *      Author: Chris E. Thompson, G0KLA
 */

#include "RxTask.h"
#include "FreeRTOS.h"
#include "os_task.h"

#include "ax5043_access.h"
#include "ax5043-ax25.h"
//#include "ax5043-2M-AFSK.h"
//#include "ax5043-2M-AFSK-externs.h"
#include "ax25_util.h"

/* Local variables */
static uint8_t PAPowerFlagCnt=0,DCTPowerFlagCnt=0;
static uint8_t axradio_rxbuffer[AX25_PKT_BUFFER_LEN];
static SPIDevice device = DCTDev0;


portTASK_FUNCTION_PROTO(RxTask, pvParameters)  {

    vTaskSetApplicationTaskTag((xTaskHandle) 0, (pdTASK_HOOK_CODE)RadioWD ); // TODO - just reuse the Radio task name for now
    InitInterTask(ToRxTask, 10);
    ResetAllWatchdogs();
    printf("Initializing Rx\n");

    /* This is defined in pacsat.h, declared here */
    xPbPacketQueue = xQueueCreate( PB_PACKET_QUEUE_LEN, AX25_PKT_BUFFER_LEN * sizeof( uint8_t ) );
    if (xPbPacketQueue == NULL) {
        /* The queue could not be created.  This is fatal and should only happen in test if we are short of memory at startup */
        debug_print("FATAL ERROR: Could not create PB Packet Queue\n");
    }

    /* Initialize the Radio RX */
    ax5043StartRx(device);

    while(1) {
        Intertask_Message messageReceived;
        int status = 0;
        uint8_t rssi = 0;

        ReportToWatchdog(CurrentTaskWD);
        status = WaitInterTask(ToRxTask, CENTISECONDS(10), &messageReceived);  // This is triggered when there is RX data on the FIFO
        ReportToWatchdog(CurrentTaskWD);
        rssi = get_rssi(device);
        if (rssi > 170) {
            debug_print("RSSI: %d   ",rssi);
            debug_print("FRMRX: %d   ",ax5043ReadReg(device, AX5043_FRAMING) & 0x80 ); // FRAMING Pkt start bit detected - will print 128
            debug_print("RADIO: %d\n",ax5043ReadReg(device, AX5043_RADIOSTATE) & 0xF ); // Radio State bits 0-3
        }

        if (status==1) { // We received a message
            switch(messageReceived.MsgType){
            case DCTPowerFlagMsg:
                debug_print("AX5043 Power Interrupted\n");
                DCTPowerFlagCnt++;
                break;
            case PAPowerFlagMsg:
                debug_print("Power Amp Power Interrupted\n");
                PAPowerFlagCnt++;
                break;

            case DCTInterruptMsg:

                if ((ax5043ReadReg(device, AX5043_PWRMODE) & 0x0F) == AX5043_PWRSTATE_FULL_RX) {

                    printf("Interrupt while in FULL_RX mode\n");
                    //printf("IRQREQUEST1: %02x\n", ax5043ReadReg(AX5043_IRQREQUEST1));
                    //printf("IRQREQUEST0: %02x\n", ax5043ReadReg(AX5043_IRQREQUEST0));
                    //printf("FIFOSTAT: %02x\n", ax5043ReadReg(AX5043_FIFOSTAT));

                    if ((ax5043ReadReg(device, AX5043_FIFOSTAT) & 0x01) != 1) { // FIFO not empty
                        debug_print("FIFO NOT EMPTY\n");
                        uint8_t fifo_cmd = ax5043ReadReg(device, AX5043_FIFODATA); // read command
                        uint8_t len = (fifo_cmd & 0xE0) >> 5; // top 3 bits encode payload len
                        if (len == 7)
                            len = ax5043ReadReg(device, AX5043_FIFODATA); // 7 means variable length, -> get length byte
                        fifo_cmd &= 0x1F;

                        if (fifo_cmd == AX5043_FIFOCMD_DATA) {
                            debug_print("FIFO CMD: %d LEN:%d\n",fifo_cmd,len);
                            uint8_t loc = 0;
                            while (len--) {
                                axradio_rxbuffer[loc] = ax5043ReadReg(device, AX5043_FIFODATA);
                                debug_print("%x ",axradio_rxbuffer[loc]);
                                loc++;
                            }
                            debug_print("\n");
//                            char from_callsign[AX25_CALLSIGN_LEN];
//                            char to_callsign[AX25_CALLSIGN_LEN];
//
//                            decode_call(&axradio_rxbuffer[8], from_callsign);
//                            decode_call(&axradio_rxbuffer[1], to_callsign);
//                            debug_print("%s>%s:\n",from_callsign, to_callsign);

                            /* Add to the queue and wait for 10ms to see if space is available */
                            BaseType_t xStatus = xQueueSendToBack( xPbPacketQueue, &axradio_rxbuffer, CENTISECONDS(1) );
                            if( xStatus != pdPASS ) {
                                /* The send operation could not complete because the queue was full */
                                debug_print("PB QUEUE FULL: Could not add to Packet Queue\n");
                                // TODO - we should log this error and downlink in telemetry
                            }

                        } else {
                            debug_print("FIFO MESSAGE: %d LEN:%d\n",fifo_cmd,len);

                        }
                    }

                    break;
                } else {
                    //printf("AX5043 Interrupt in pwrmode: %02x\n", ax5043ReadReg(AX5043_PWRMODE));
                }

          }
        }
    }
}


