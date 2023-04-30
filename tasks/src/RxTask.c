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
extern bool monitorPackets;

portTASK_FUNCTION_PROTO(RxTask, pvParameters)  {

    vTaskSetApplicationTaskTag((xTaskHandle) 0, (pdTASK_HOOK_CODE)RxTaskWD ); // TODO - just reuse the Radio task name for now
    InitInterTask(ToRxTask, 10);
    ResetAllWatchdogs();
//    printf("Initializing Rx\n");

    /* These are defined in pacsat.h, declared here */
    xRxPacketQueue = xQueueCreate( RX_PACKET_QUEUE_LEN, AX25_PKT_BUFFER_LEN * sizeof( uint8_t ) );
    xPbPacketQueue = xQueueCreate( PB_PACKET_QUEUE_LEN, AX25_PKT_BUFFER_LEN * sizeof( uint8_t ) );
    xUplinkPacketQueue = xQueueCreate( UPLINK_PACKET_QUEUE_LEN, AX25_PKT_BUFFER_LEN * sizeof( uint8_t ) );
    if (xRxPacketQueue == NULL) {
        /* The queue could not be created.  This is fatal and should only happen in test if we are short of memory at startup */
        debug_print("FATAL ERROR: Could not create RX Packet Queue\n");
        //TODO - log this
    }
    if (xPbPacketQueue == NULL) {
        /* The queue could not be created.  This is fatal and should only happen in test if we are short of memory at startup */
        debug_print("FATAL ERROR: Could not create PB Packet Queue\n");
        //TODO - log this
    }
    if (xUplinkPacketQueue == NULL) {
        /* The queue could not be created.  This is fatal and should only happen in test if we are short of memory at startup */
        debug_print("FATAL ERROR: Could not create UPLINK Packet Queue\n");
        //TODO - log this
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
        if (monitorPackets)
            if (rssi > 160) { // this magic value is supposed to be above the background noise, so we only see actual transmissions
                int16_t dbm = rssi - 255;
                debug_print("RSSI: %d dBm  ",dbm);
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

                    //debug_print("Interrupt while in FULL_RX mode\n");
                    //printf("IRQREQUEST1: %02x\n", ax5043ReadReg(AX5043_IRQREQUEST1));
                    //printf("IRQREQUEST0: %02x\n", ax5043ReadReg(AX5043_IRQREQUEST0));
                    //printf("FIFOSTAT: %02x\n", ax5043ReadReg(AX5043_FIFOSTAT));

                    if ((ax5043ReadReg(device, AX5043_FIFOSTAT) & 0x01) != 1) { // FIFO not empty
                        //debug_print("FIFO NOT EMPTY\n");
                        uint8_t fifo_cmd = ax5043ReadReg(device, AX5043_FIFODATA); // read command
                        uint8_t len = (fifo_cmd & 0xE0) >> 5; // top 3 bits encode payload len
                        if (len == 7)
                            len = ax5043ReadReg(device, AX5043_FIFODATA); // 7 means variable length, -> get length byte
                        fifo_cmd &= 0x1F;
                        /* Note that the length byte and header byte are not included in the length of the packet
                           but length does include the flag byte */
                        uint8_t fifo_flags = ax5043ReadReg(device, AX5043_FIFODATA); // read command
                        len--;
                        if (fifo_cmd == AX5043_FIFOCMD_DATA) {
                            //debug_print("FIFO CMD:%d LEN:%d FLAGS:%x\n",fifo_cmd,len, fifo_flags);
                            if (fifo_flags != 0x03) {
                                // TODO - log something here?  This should never happen??
                                debug_print("ERROR in received FIFO Flags\n");
                            }
                            uint8_t loc = 0;

                            /* Store the length byte in position 0 TODO - make this a struct with length and buffer */
                            axradio_rxbuffer[loc++] = len-1; // remove the flag byte from the length
                            while (len--) {
                                axradio_rxbuffer[loc] = ax5043ReadReg(device, AX5043_FIFODATA);
                                loc++;
                            }
                            if (monitorPackets) {
                                print_packet("RX", &axradio_rxbuffer[1],loc);
                            }

                            /* Add to the queue and wait for 10ms to see if space is available */
                            BaseType_t xStatus = xQueueSendToBack( xRxPacketQueue, &axradio_rxbuffer, CENTISECONDS(1) );
                            if( xStatus != pdPASS ) {
                                /* The send operation could not complete because the queue was full */
                                debug_print("RX QUEUE FULL: Could not add to Packet Queue\n");
                                // TODO - we should log this error and downlink in telemetry
                            }

                        } else {
                            //debug_print("FIFO MESSAGE: %d LEN:%d\n",fifo_cmd,len);

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


