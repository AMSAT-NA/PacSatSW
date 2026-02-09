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
static rx_radio_buffer_t rx_radio_buffer;

/* Forward declarations */
static bool process_fifo(rfchan chan);
static void rx_irq_handler(void *handler_data);

#define ADJ_RX_RSSI_THRESHOLD (RX_RSSI_THRESHOLD + 255)


/*
 * Interrupt handling for each AX5043.  If an interrupt comes in,
 * rx_ready is set for that AX5043 and the RxTask is woken with
 * RxFIFOReady.
 */
static volatile uint8_t rx_ready[NUM_RX_CHANNELS];
const static struct gpio_irq_info rx_gpio_info[NUM_RX_CHANNELS] = {
    { rx_irq_handler, (void *) (uintptr_t) 0 },
    { rx_irq_handler, (void *) (uintptr_t) 1 },
    { rx_irq_handler, (void *) (uintptr_t) 2 },
    { rx_irq_handler, (void *) (uintptr_t) 3 },
};
static xSemaphoreHandle RxFIFOReady;

static void rx_irq_handler(void *handler_data)
{
    unsigned int rxnum = (uintptr_t) handler_data;
    BaseType_t higherPrioTaskWoken;

    rx_ready[rxnum] = 1;
    xSemaphoreGiveFromISR(RxFIFOReady, &higherPrioTaskWoken);
}

portTASK_FUNCTION_PROTO(RxTask, pvParameters)
{
    rfchan chan;

    vTaskSetApplicationTaskTag((xTaskHandle) 0, (pdTASK_HOOK_CODE)RxTaskWD);
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

    vSemaphoreCreateBinary(RxFIFOReady);

    GPIOInit(AX5043_Rx1_Interrupt, &rx_gpio_info[0]);
#if NUM_RX_CHANNELS == 4
    GPIOInit(AX5043_Rx2_Interrupt, &rx_gpio_info[1]);
    GPIOInit(AX5043_Rx3_Interrupt, &rx_gpio_info[2]);
    GPIOInit(AX5043_Rx4_Interrupt, &rx_gpio_info[3]);
#endif

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

    while (true) {
        ReportToWatchdog(CurrentTaskWD);
        // This is triggered when there is RX data in a FIFO
        xSemaphoreTake(RxFIFOReady, CENTISECONDS(10));
        ReportToWatchdog(CurrentTaskWD);
        GPIOSetOff(LED2);

        if (monitorRSSI) {
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

        for (chan = 0; chan < NUM_RX_CHANNELS; chan++) {
            if (rx_ready[chan]) {
                rx_ready[chan] = 0;
                while (process_fifo(FIRST_RX_CHANNEL + chan))
                    ;
            }
        }
    }
}

static void handle_fifo_data(rfchan chan, uint8_t fifo_flags, uint8_t len)
{
    uint8_t loc = 0;

    //debug_print("FIFO CMD:%d LEN:%d FLAGS:%x\n",fifo_cmd,len, fifo_flags);
    if (fifo_flags != 0x03) {
        // TODO - log something here?  This should never happen??
        debug_print("ERROR in received FIFO Flags\n");
    }

    for (loc = 0; loc < len; loc++)
        rx_radio_buffer.bytes[loc] = ax5043ReadReg(chan, AX5043_FIFODATA);

    if (len < 2) {
        /* Shouldn't happen, the CRC at the end is still there. */
        /* TODO - log this? */
        return;
    }

    rx_radio_buffer.len = len - 2; // Remove the CRC, flags are already gone.

    if (monitorRxPackets) {
        char rx_str[10];

        snprintf(rx_str, sizeof(rx_str), "RX[%d]", chan);
        print_packet(rx_str, &rx_radio_buffer.bytes[0],
                     rx_radio_buffer.len);
    }

    // Store the channel here - same as device id
    rx_radio_buffer.channel = chan;

    /*
     * Add to the queue and wait for 10ms to see if space
     * is available
     */
    BaseType_t xStatus = xQueueSendToBack(xRxPacketQueue, &rx_radio_buffer,
                                          CENTISECONDS(1));
    if (xStatus != pdPASS) {
        /*
         * The send operation could not complete because
         * the queue was full
         */
        debug_print("RX QUEUE FULL: Could not add to Packet Queue\n");
        // TODO - we should log this error and downlink in telemetry
    }
}

static bool process_fifo(rfchan chan)
{
    uint8_t fifo_cmd, fifo_flags, len;

    if (!rx_working(chan)) {
        //printf("AX5043 Interrupt in pwrmode: %02x\n",
        //       ax5043ReadReg(AX5043_PWRMODE));
        return false;
    }

    if ((ax5043ReadReg(chan, AX5043_FIFOSTAT) & 0x01) == 1)
        // FIFO empty
        return false;

    if (monitorRxPackets)
        debug_print("RX channel: %d Interrupt while in FULL_RX mode\n", chan);
    //printf("IRQREQUEST1: %02x\n", ax5043ReadReg(AX5043_IRQREQUEST1));
    //printf("IRQREQUEST0: %02x\n", ax5043ReadReg(AX5043_IRQREQUEST0));
    //printf("FIFOSTAT: %02x\n", ax5043ReadReg(AX5043_FIFOSTAT));

    fifo_cmd = ax5043ReadReg(chan, AX5043_FIFODATA);
    // top 3 bits encode payload length
    len = (fifo_cmd & 0xE0) >> 5;

    if (len == 7)
        // 7 means length in next byte
        len = ax5043ReadReg(chan, AX5043_FIFODATA);
    fifo_cmd &= 0x1F;
    /*
     * Note that the length byte and header byte are not
     * included in the length of the packet but length does
     * include the flag byte.
     */
    // read command
    fifo_flags = ax5043ReadReg(chan, AX5043_FIFODATA);
    len--;

    if (fifo_cmd == AX5043_FIFOCMD_DATA) {
        GPIOSetOn(LED2);
        handle_fifo_data(chan, fifo_flags, len);
    } else {
        //debug_print("FIFO MESSAGE: %d LEN:%d\n",fifo_cmd,len);
    }

    return true;
}
