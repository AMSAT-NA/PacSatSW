/*
 * RxTask.c
 *
 *  Created on: Dec 1, 2022
 *      Author: Chris E. Thompson, G0KLA
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 *
 */

#include "RxTask.h"
#include "FreeRTOS.h"
#include "os_task.h"

#include "ax5043_access.h"
#include "ax5043-ax25.h"
#include "Ax25Task.h"
#include "ax25_util.h"

/* Local variables */
static uint8_t PAPowerFlagCnt=0,DCTPowerFlagCnt=0;
static rx_radio_buffer_t rx_radio_buffer; // static buffer to store the channel and received bytes from the radio
static rx_radio_buffer_t EMPTY_RADIO_BUFFER;
//static uint8_t axradio_rxbuffer[AX25_PKT_BUFFER_LEN];  ******************** HERE WE ARE - REMOVING THIS
static AX5043Device device = AX5043Dev0;
extern bool monitorPackets;

portTASK_FUNCTION_PROTO(RxTask, pvParameters)  {

    vTaskSetApplicationTaskTag((xTaskHandle) 0, (pdTASK_HOOK_CODE)RxTaskWD );
    InitInterTask(ToRxTask, 10);
    ResetAllWatchdogs();
//    printf("Initializing Rx\n");

    /* These are defined in pacsat.h, declared here */
    xRxPacketQueue = xQueueCreate( RX_PACKET_QUEUE_LEN, sizeof( rx_radio_buffer ) );
    xRxEventQueue = xQueueCreate( RX_EVENT_QUEUE_LEN, sizeof( AX25_event_t ) );
    xPbPacketQueue = xQueueCreate( PB_PACKET_QUEUE_LEN, sizeof( rx_radio_buffer ) );
    xUplinkEventQueue = xQueueCreate( UPLINK_PACKET_QUEUE_LEN,  sizeof( AX25_event_t ) );

    if (xRxPacketQueue == NULL) {
        /* The queue could not be created.  This is fatal and should only happen in test if we are short of memory at startup */
        debug_print("FATAL ERROR: Could not create RX Packet Queue\n");
        //TODO - log this
    }
    if (xRxEventQueue == NULL) {
        /* The queue could not be created.  This is fatal and should only happen in test if we are short of memory at startup */
        debug_print("FATAL ERROR: Could not create RX Event Queue\n");
        //TODO - log this
    }
    if (xPbPacketQueue == NULL) {
        /* The queue could not be created.  This is fatal and should only happen in test if we are short of memory at startup */
        debug_print("FATAL ERROR: Could not create PB Packet Queue\n");
        //TODO - log this
    }
    if (xUplinkEventQueue == NULL) {
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
            if (rssi > 170) { // this magic value is supposed to be above the background noise, so we only see actual transmissions
                int16_t dbm = rssi - 255;
                debug_print("RSSI: %d dBm \n",dbm);
//                debug_print("FRMRX: %d   ",ax5043ReadReg(device, AX5043_FRAMING) & 0x80 ); // FRAMING Pkt start bit detected - will print 128
//                debug_print("RADIO: %d ",ax5043ReadReg(device, AX5043_RADIOSTATE) & 0xF ); // Radio State bits 0-3
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

                            /* Store the length byte  */
                            rx_radio_buffer.len = len-1; // remove the flag byte from the length
                            while (len--) {
                                rx_radio_buffer.bytes[loc] = ax5043ReadReg(device, AX5043_FIFODATA);
                                loc++;
                            }
                            if (monitorPackets) {
                                int i;
                                debug_print("RX Bytes: %d:",rx_radio_buffer.len);
                                for (i=0; i< rx_radio_buffer.len; i++)
                                    debug_print("%0x ", rx_radio_buffer.bytes[i]);
                                debug_print("\n");
                                print_packet("RX", &rx_radio_buffer.bytes[0],rx_radio_buffer.len);
                            }

                            // TODO - need to store the channel here - for now it is hard coded to channel A.  This should be passed by the interrupt.
                            rx_radio_buffer.channel = Channel_A;

                            /* Add to the queue and wait for 10ms to see if space is available */
                            BaseType_t xStatus = xQueueSendToBack( xRxPacketQueue, &rx_radio_buffer, CENTISECONDS(1) );
                            if( xStatus != pdPASS ) {
                                /* The send operation could not complete because the queue was full */
                                debug_print("RX QUEUE FULL: Could not add to Packet Queue\n");
                                // TODO - we should log this error and downlink in telemetry
                            }
#ifdef DEBUG
                            rx_radio_buffer = EMPTY_RADIO_BUFFER;
#endif

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


