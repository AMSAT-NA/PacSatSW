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

#include "hardwareConfig.h"
#include "RxTask.h"
#include "nonvolManagement.h"
#include "FreeRTOS.h"
#include "os_task.h"

#include "radio.h"
#include "ax5043.h"
#include "Ax25Task.h"
#include "ax25_util.h"

/* Local variables */
static uint8_t PAPowerFlagCnt = 0;
static uint8_t AX5043PowerFlagCnt = 0;

static rx_radio_buffer_t rx_radio_buffer;
extern bool monitorPackets;

/* Forward declarations */
void process_fifo(rfchan chan);

#define ADJ_RX_RSSI_THRESHOLD (RX_RSSI_THRESHOLD + 255)

portTASK_FUNCTION_PROTO(RxTask, pvParameters)
{
    rfchan chan;

    vTaskSetApplicationTaskTag((xTaskHandle) 0, (pdTASK_HOOK_CODE)RxTaskWD);
    InitInterTask(ToRxTask, 10);
    ResetAllWatchdogs();
//    printf("Initializing Rx\n");

    /* These are defined in pacsat.h, declared here */
    xRxPacketQueue = xQueueCreate(RX_PACKET_QUEUE_LEN, sizeof(rx_radio_buffer));
    xRxEventQueue = xQueueCreate(RX_EVENT_QUEUE_LEN, sizeof(AX25_event_t));
    xPbPacketQueue = xQueueCreate(PB_PACKET_QUEUE_LEN, sizeof(rx_radio_buffer));
    xUplinkEventQueue = xQueueCreate(UPLINK_PACKET_QUEUE_LEN,
                                     sizeof(AX25_event_t));

    if (xRxPacketQueue == NULL) {
        /*
         * The queue could not be created.  This is fatal and should
         * only happen in test if we are short of memory at startup
         */
        debug_print("FATAL ERROR: Could not create RX Packet Queue\n");
        //TODO - log this
    }
    if (xRxEventQueue == NULL) {
        /*
         * The queue could not be created.  This is fatal and should
         * only happen in test if we are short of memory at startup
         */
        debug_print("FATAL ERROR: Could not create RX Event Queue\n");
        //TODO - log this
    }
    if (xPbPacketQueue == NULL) {
        /*
         * The queue could not be created.  This is fatal and should
         * only happen in test if we are short of memory at startup
         */
        debug_print("FATAL ERROR: Could not create PB Packet Queue\n");
        //TODO - log this
    }
    if (xUplinkEventQueue == NULL) {
        /*
         * The queue could not be created.  This is fatal and should
         * only happen in test if we are short of memory at startup
         */
        debug_print("FATAL ERROR: Could not create UPLINK Packet Queue\n");
        //TODO - log this
    }

    /* Initialize the Radio RX */
    for (chan = FIRST_RX_CHANNEL; chan <= LAST_RX_CHANNEL; chan++) {
#ifdef BLINKY_HARDWARE
        if (chan == 1) {
            ax5043_off(1); // dev1 is broken on blinky.
            continue;
        }
#endif
        start_rx(chan, ReadMRAMFreq(chan), ReadMRAMModulation(chan));
    }

    GPIOSetOn(LED2);

    while (true) {
        Intertask_Message messageReceived;
        int status = 0;

        ReportToWatchdog(CurrentTaskWD);
        // This is triggered when there is RX data on the FIFO
        status = WaitInterTask(ToRxTask, CENTISECONDS(10), &messageReceived);
        ReportToWatchdog(CurrentTaskWD);

        if (monitorPackets) {
            uint8_t rssi;
            int16_t dbm;
            rfchan chan;

            for (chan = FIRST_RX_CHANNEL; chan <= LAST_RX_CHANNEL; chan++) {
                // this magic value is supposed to be above the background
                // noise, so we only see actual transmissions
                rssi = get_rssi(chan);
                if (rssi > ADJ_RX_RSSI_THRESHOLD) {
                    dbm = rssi - 255;
                    debug_print("RSSI-%d: %d dBm\n", chan, dbm);
                }
            }
#if 0
            // FRAMING Pkt start bit detected - will print 128
            debug_print("FRMRX: %d   ",
                        ax5043ReadReg(chan, AX5043_FRAMING) & 0x80);
            // Radio State bits 0-3
            debug_print("RADIO: %d ",
                        ax5043ReadReg(chan, AX5043_RADIOSTATE) & 0xF);
#endif
        }

        if (status == 1) { // We received a message
            //debug_print("AX5043 Message %d\n",messageReceived.MsgType);
            switch(messageReceived.MsgType) {
            case AX5043PowerFlagMsg:
                debug_print("AX5043 Power Interrupted\n");
                AX5043PowerFlagCnt++;
                break;

            case PAPowerFlagMsg:
                debug_print("Power Amp Power Interrupted\n");
                PAPowerFlagCnt++;
                break;

            case AX5043_Rx1_InterruptMsg:
                process_fifo(0);
                break;
#if NUM_RX_CHANNELS == 4
            case AX5043_Rx2_InterruptMsg:
                process_fifo(1);
                break;

            case AX5043_Rx3_InterruptMsg:
                process_fifo(2);
                break;

            case AX5043_Rx4_InterruptMsg:
                process_fifo(3);
                break;
#endif
            }
        }
    }
}

void process_fifo(rfchan chan)
{
    if ((ax5043ReadReg(chan, AX5043_PWRMODE) & 0x0F) ==
                        AX5043_PWRSTATE_FULL_RX) {
        if (monitorPackets)
            debug_print("RX channel: %d Interrupt while in FULL_RX mode\n",
                        chan);
        //printf("IRQREQUEST1: %02x\n", ax5043ReadReg(AX5043_IRQREQUEST1));
        //printf("IRQREQUEST0: %02x\n", ax5043ReadReg(AX5043_IRQREQUEST0));
        //printf("FIFOSTAT: %02x\n", ax5043ReadReg(AX5043_FIFOSTAT));

        // FIFO not empty
        if ((ax5043ReadReg(chan, AX5043_FIFOSTAT) & 0x01) != 1) {
            //debug_print("FIFO NOT EMPTY\n");
            // read command
            uint8_t fifo_cmd = ax5043ReadReg(chan, AX5043_FIFODATA);
            uint8_t fifo_flags;
            // top 3 bits encode payload len
            uint8_t len = (fifo_cmd & 0xE0) >> 5;

            if (len == 7)
                // 7 means variable length, -> get length byte
                len = ax5043ReadReg(chan, AX5043_FIFODATA);
            fifo_cmd &= 0x1F;
            /*
             * Note that the length byte and header byte are not
             * included in the length of the packet but length does
             * include the flag byte
             */
            // read command
            fifo_flags = ax5043ReadReg(chan, AX5043_FIFODATA);
            len--;
            if (fifo_cmd == AX5043_FIFOCMD_DATA) {
                uint8_t loc = 0;
                //debug_print("FIFO CMD:%d LEN:%d FLAGS:%x\n",fifo_cmd,len, fifo_flags);
                if (fifo_flags != 0x03) {
                    // TODO - log something here?  This should never happen??
                    debug_print("ERROR in received FIFO Flags\n");
                }

                /* Store the length byte  */
                // remove the flag byte from the length
                rx_radio_buffer.len = len - 1;
                while (len--) {
                    rx_radio_buffer.bytes[loc] = ax5043ReadReg(chan,
                                                               AX5043_FIFODATA);
                    loc++;
                }
                if (monitorPackets) {
//                    int i;
//                    debug_print("RX Bytes: %d:",rx_radio_buffer.len);
//                    for (i=0; i< rx_radio_buffer.len; i++)
//                        debug_print("%0x ", rx_radio_buffer.bytes[i]);
//                    debug_print("\n");
                    char rx_str[10];
                    snprintf(rx_str, sizeof(rx_str), "RX[%d]",chan);
                    print_packet(rx_str, &rx_radio_buffer.bytes[0],
                                 rx_radio_buffer.len);
                }
                GPIOSetOff(LED2);

                // Store the channel here - same as device id
                rx_radio_buffer.channel = chan;

                /*
                 * Add to the queue and wait for 10ms to see if space
                 * is available
                 */
                BaseType_t xStatus = xQueueSendToBack(xRxPacketQueue,
                                                      &rx_radio_buffer,
                                                      CENTISECONDS(1));
                if (xStatus != pdPASS) {
                    /*
                     * The send operation could not complete because
                     * the queue was full
                     */
                    debug_print("RX QUEUE FULL: Could not add to Packet Queue\n");
                    // TODO - we should log this error and downlink in telemetry
                }
            } else {
                //debug_print("FIFO MESSAGE: %d LEN:%d\n",fifo_cmd,len);
            }
        }
    } else {
        //printf("AX5043 Interrupt in pwrmode: %02x\n", ax5043ReadReg(AX5043_PWRMODE));
    }
}
