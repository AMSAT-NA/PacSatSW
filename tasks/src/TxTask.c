/*
 * TncTask.c
 *
 *  Created on: Dec 1, 2022
 *      Author: g0kla
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

#include "ctype.h"

#include "pacsat.h"
#include "radio.h"
#include "ax5043.h"
#include "TxTask.h"
#include "FreeRTOS.h"
#include "os_task.h"
#include "ax25_util.h"
#include "nonvolManagement.h"
#include "gpioDriver.h"


static rfchan txchan = FIRST_TX_CHANNEL;

enum radio_modulation tx_modulation;

/* Global Variable that is declared here and prevents the transmitter from generating RF */
bool inhibitTransmit;

/* Test Buffer PB Empty */
//uint8_t byteBuf[] = {0xA0,0x84,0x98,0x92,0xA6,0xA8,0x00,0xA0,0x8C,0xA6,0x66,
//                     0x40,0x40,0x17,0x03,0xF0,0x50,0x42,0x3A,0x20,0x45,0x6D,0x70,0x74,0x79,0x2E,0x0D};

static void
print_raw_packet(const char *str, uint8_t *data, unsigned int len)
{
    unsigned int i;

    printf("%s RAW:", str);
    for (i = 0; i < len; i++)
        printf(" %2.2x", data[i]);
    printf("\n");
}

static xSemaphoreHandle TxFIFOReady;
static void tx_irq_handler(void *handler_data)
{
    BaseType_t higherPrioTaskWoken;

    xSemaphoreGiveFromISR(TxFIFOReady, &higherPrioTaskWoken);
}

const static struct gpio_irq_info tx_gpio_info = {
    tx_irq_handler, (void *) (uintptr_t) 0
};

portTASK_FUNCTION_PROTO(TxTask, pvParameters)
{
    /* Buffer used when data copied from tx queue */
    tx_radio_buffer_t tx_packet_buffer;
    enum radio_modulation curr_modulation = MODULATION_INVALID;

    tx_modulation = ReadMRAMModulation(txchan);

    vTaskSetApplicationTaskTag((xTaskHandle) 0, (pdTASK_HOOK_CODE) TxTaskWD);

    //    printf("Initializing TX\n");

    vSemaphoreCreateBinary(TxFIFOReady);
    GPIOInit(AX5043_Tx_Interrupt, &tx_gpio_info);

    /* This is defined in pacsat.h, declared here */
    xTxPacketQueue = xQueueCreate(TX_PACKET_QUEUE_LEN,
                                  sizeof(tx_radio_buffer_t));
    if (xTxPacketQueue == NULL) {
        /*
         * The queue could not be created.  This is fatal and should
         * only happen in test if we are short of memory at startup.
         */
        ReportError(RTOSfailure, true, CharString,
                    (int)"FATAL ERROR: Could not create TX Packet Queue");
    }

    start_tx(txchan, ReadMRAMFreq(txchan), ReadMRAMModulation(txchan));

    // Add seletable Tx power levels  N5BRG  240516
    //set_tx_power(1); // minimum power to test RF output on AX5043
    //set_tx_power(50); // midrange power to test RF output on AX5043
    //set_tx_power(txchan, 100); // maximum power to test RF output on AX5043
    set_tx_power(txchan, 5); // Low power, in case it's not plugged in.

    /* Set Power state to FULL_TX */
    ax5043WriteReg(txchan, AX5043_PWRMODE, AX5043_PWRSTATE_FULL_TX);

    //printf("Turn off TX LED1 at init\n");
    GPIOSetOff(LED1);
    GPIOSetOff(SSPAPower);
    ReportToWatchdog(CurrentTaskWD);

    /*
     * TODO - Add PA thermal management to this to keep the PA cool
     * enough.  Probably add temperature monitoring and transmit
     * limiting based on temperature.
     */
    while (1) {
        BaseType_t xStatus;

        // TODO - adjust block time vs watchdog
        xStatus = xQueueReceive(xTxPacketQueue, &tx_packet_buffer,
                                CENTISECONDS(10));
        ReportToWatchdog(CurrentTaskWD);

        if (xStatus != pdPASS)
            continue;

        if (inhibitTransmit)
            continue;

        GPIOSetOn(LED1);
        GPIOSetOn(SSPAPower);

        /* Transmit until we have no more packets. */
        while (xStatus == pdPASS) {
            /* Data was successfully received from the queue */
            uint8_t preamble_length = 32;
            unsigned int numbytesleft = tx_packet_buffer.len;
            unsigned int numbytes, flag, bytepos, retries;
            enum radio_modulation mod;
            enum fec fec;

            mod = (enum radio_modulation) tx_packet_buffer.tx_modulation;

            if (mod != curr_modulation) {
                set_modulation(txchan, mod, true);
                curr_modulation = mod;
            }

            fec = MODULATION_TO_FEC(mod);
            mod = MODULATION_TO_BASE_MODULATION(mod);

            // 10 for 1200 bps - Radio lab recommends 32 for 9600, may
            // need as much as 56.
            switch (mod) {
            case MODULATION_AFSK_1200:
                preamble_length = 10;
                break;
            case MODULATION_GMSK_9600:
                break;
            }

            if (monitorTxPackets)
                print_raw_packet("TX", tx_packet_buffer.bytes, tx_packet_buffer.len);

            //printf("FIFO_FREE 1: %d\n",fifo_free());

            // clear FIFO data & flags
            fifo_clear(txchan);

            if (fec == FEC_CONV) {
                /*
                 * With FEC enabled, you must send a single extra bit
                 * at the beginning of the preamble or the data won't
                 * line up correctly in the interleaver.
                 */
                fifo_repeat_byte(txchan, 0x02, 1,
                                 AX5043_QUEUE_RESIDUE_FLAG |
                                 AX5043_QUEUE_RAW_NO_CRC_FLAG |
                                 AX5043_QUEUE_PKTSTART_FLAG);
                fifo_repeat_byte(txchan, 0x7E, preamble_length,
                                 AX5043_QUEUE_RAW_NO_CRC_FLAG);
            } else {
                // repeat the preamble bytes
                //  TODO - no preamble for back to back packets
                fifo_repeat_byte(txchan, 0x7E, preamble_length,
                                 AX5043_QUEUE_RAW_NO_CRC_FLAG |
                                 AX5043_QUEUE_PKTSTART_FLAG);
            }

            /*
             * The AX5043 buffer is 256 bytes, a long message can
             * overflow it with the preamble and overhead.  Only allow
             * 240 bytes in at first, and put the rest in a later
             * request.  The overhead for the headers and preamble is
             * up to 10 bytes, and leave a little slack.
             */
            bytepos = 0;
            if (numbytesleft > 200) {
                numbytes = 200;
                numbytesleft -= 200;
                flag = 0;
            } else {
                numbytes = numbytesleft;
                numbytesleft = 0;
                flag = AX5043_QUEUE_PKTEND_FLAG;
            }
            fifo_queue_buffer(txchan, tx_packet_buffer.bytes, numbytes, flag);
            bytepos += numbytes;
            //       printf("FIFO_FREE 2: %d\n",fifo_free());
            fifo_commit(txchan);
            //       printf("INFO: Waiting for transmission to complete\n");

            /*
             * Handle sending more data and waiting for all data to be
             * sent.
             */
            for (retries = 0; ; ) {
                unsigned int free_room = fifo_free(txchan);
                /* TX interrupt at 150 bytes free, and at empty. */

                if (numbytesleft == 0 && free_room == 256)
                    /* FIFO empty interrupt, finished processing. */
                    break;

                if (free_room >= 150 && numbytesleft > 0) {
                    /* 150 byte free interrupt, add another 100. */
                    if (numbytesleft > 100) {
                        numbytes = 100;
                        numbytesleft -= 100;
                        flag = 0;
                    } else {
                        numbytes = numbytesleft;
                        numbytesleft = 0;
                        flag = AX5043_QUEUE_PKTEND_FLAG;
                    }
                    fifo_queue_buffer(txchan,
                                      tx_packet_buffer.bytes + bytepos,
                                      numbytes, flag);
                    bytepos += numbytes;
                    fifo_commit(txchan);
                    retries = 0;
                } else {
                    retries++;
                }
                if (retries > 50) {
                    /* No progress in 1/2 second, time to log and give up. */
                    ReportError(AX5043error, true, CharString,
                                (int)"TX packet finish never happened");
                    break;
                }
                xSemaphoreTake(TxFIFOReady, CENTISECONDS(10));
                ReportToWatchdog(CurrentTaskWD);
            }

            // See if we have another packet to send.
            xStatus = xQueueReceive(xTxPacketQueue, &tx_packet_buffer,
                                    NO_TIMEOUT);
            ReportToWatchdog(CurrentTaskWD);
        }

        //       printf("Turn off TX LED1\n");
        GPIOSetOff(LED1);
        GPIOSetOff(SSPAPower);
        //       printf("INFO: Transmission complete\n");
    }
}

/**
 * tx_make_ui_packet()
 * Create an AX25 packet
 * from and to callsigns are strings with nul termination
 * pid byte is F0, BB or BD
 * bytes is a buffer of length len that is sent in the body of the packet
 * raw_bytes must be allocated by the caller and contains the packet.  The length is
 * stored in the first byte, which will not be transmitted.
 *
 */
static bool tx_make_ui_packet(char *from_callsign, char *to_callsign,
                              uint8_t pid, uint8_t *bytes, int len,
                              enum radio_modulation modulation,
                              tx_radio_buffer_t *tx_radio_buffer)
{
    unsigned int packet_len;
    unsigned int header_len = 16;
    int i;
    unsigned char buf[7];
    int l = encode_call(to_callsign, buf, false, 0);

    if (l != true)
        return false;
    for (i = 0; i < 7; i++)
        tx_radio_buffer->bytes[i] = buf[i];
    l = encode_call(from_callsign, buf, true, 0);
    if (l != true)
        return false;
    for (i=0; i<7; i++)
        tx_radio_buffer->bytes[i+7] = buf[i];
    tx_radio_buffer->bytes[14] = BITS_UI; // UI Frame control byte
    tx_radio_buffer->bytes[15] = pid;

    if (len + 16 > sizeof(tx_radio_buffer->bytes))
        // TODO - Add a log here?
        return false;

    for (i = 0; i < len; i++)
        tx_radio_buffer->bytes[i+header_len] = bytes[i];

    packet_len = len + header_len;
    tx_radio_buffer->len = packet_len; /* Number of bytes */
    if (modulation == MODULATION_INVALID)
        tx_radio_buffer->tx_modulation = tx_modulation;
    else
        tx_radio_buffer->tx_modulation = modulation;

//    if (true) {
//        for (i=0; i< packet_len; i++) {
//            if (isprint(raw_bytes[i]))
//                printf("%c",raw_bytes[i]);
//            else
//                printf(" ");
//        }
//        for (i=0; i< packet_len; i++) {
//            printf("%02x ",raw_bytes[i]);
//            if (i%40 == 0 && i!=0) printf("\n");
//        }
//    }

    return true;

}

/**
 * tx_make_packet()
 * Make an arbitrary packet based on the packet structure
 *
 * Returns true unless there is an error
 * TODO - this should be in ax25_util and be called encode_packet()
 */
static bool tx_make_packet(AX25_PACKET *packet,
                           enum radio_modulation modulation,
                           tx_radio_buffer_t *tx_radio_buffer)
{
    unsigned int packet_len;
    unsigned int header_len = 15; // Assumes no PID
    int i;
    unsigned char buf[7];

    // set the control byte
    switch (packet->frame_type) {
    case TYPE_I:
        packet->control = ((packet->NR << 5) | (packet->PF << 4) |
                           (packet->NS << 1) | 0b00);
        header_len = 16; // make room for pid
        packet->pid = 0xF0;
        packet->command = AX25_COMMAND;
        break;

    case TYPE_S_RR:
        packet->control = ((packet->NR << 5) | (packet->PF << 4) |
                           BITS_S_RECEIVE_READY);
        break;

    case TYPE_S_RNR:
        packet->control = ((packet->NR << 5) | (packet->PF << 4) |
                           BITS_S_RECEIVE_NOT_READY);
        break;

    case TYPE_S_REJ:
        packet->control = ((packet->NR << 5) | (packet->PF << 4) |
                           BITS_S_REJECT);
        break;

    case TYPE_S_SREJ:
        packet->control = ((packet->NR << 5) | (packet->PF << 4) |
                           BITS_S_SELECTIVE_REJECT);
        break;

    case TYPE_U_SABM:
        // override this as it is always a command
        packet->command = AX25_COMMAND;
        packet->control = (packet->PF << 4) | BITS_U_SABM;
        break;

    case TYPE_U_SABME:
        // override this as it is always a command
        packet->command = AX25_COMMAND;
        packet->control = (packet->PF << 4) | BITS_U_SABME;
        break;

    case TYPE_U_DISC:
        // override this as it is always a command
        packet->command = AX25_COMMAND;
        packet->control = (packet->PF << 4) | BITS_U_DISCONNECT;
        break;

    case TYPE_U_DM:
        // override this as it is always a response
        packet->command = AX25_RESPONSE;
        packet->control = (packet->PF << 4) | BITS_U_DISCONNECT_MODE;
        break;

    case TYPE_U_UA:
        // override this as it is always a response
        packet->command = AX25_RESPONSE;
        packet->control = (packet->PF << 4) | BITS_UA;
        break;

    case TYPE_U_FRMR:
        // override this as it is always a response
        packet->command = AX25_RESPONSE;
        packet->control = (packet->PF << 4) | BITS_U_FRAME_REJECT;
        break;

    case TYPE_U_UI:
        packet->control = (packet->PF << 4) | BITS_UI;
        // room for pid, which has to be set in the packet already
        header_len = 16;
        break;

    case TYPE_U_XID:
        packet->control = (packet->PF << 4) | BITS_U_EXCH_ID;
        break;

    case TYPE_U_TEST:
        packet->control = (packet->PF << 4) | BITS_U_TEST;
        break;

    default:
        debug_print("ERR: Invalid frame type\n");
        return false;
    }

    if (header_len + packet->data_len > sizeof(tx_radio_buffer->bytes))
        // TODO - should we log this?
        return false;

    // We are using V2, so both calls encode the command bit:
    int command = packet->command & 0b1;
    int command2 = 0;
    if (command == 0)
        command2 = 1;
    int l = encode_call(packet->to_callsign, buf, false, command);
    if (l != true)
        return false;
    for (i=0; i<7; i++)
        tx_radio_buffer->bytes[i] = buf[i];
    l = encode_call(packet->from_callsign, buf, true, command2);
    if (l != true)
        return false;
    for (i=0; i<7; i++)
        tx_radio_buffer->bytes[i+7] = buf[i];

    tx_radio_buffer->bytes[14] = packet->control;

    // If we have a pid then set it here
    if (header_len == 16)
        tx_radio_buffer->bytes[15] = packet->pid;

    // If there are data bytes then add them here
     for (i = 0; i < packet->data_len; i++)
        tx_radio_buffer->bytes[i + header_len] = packet->data[i];
    packet_len = packet->data_len + header_len;
    tx_radio_buffer->len = packet_len; /* Number of bytes */
    if (modulation == MODULATION_INVALID)
        tx_radio_buffer->tx_modulation = tx_modulation;
    else
        tx_radio_buffer->tx_modulation = modulation;

//    if (true) {
//        for (i=0; i< packet_len; i++) {
//            if (isprint(raw_bytes[i]))
//                printf("%c",raw_bytes[i]);
//            else
//                printf(" ");
//        }
//        for (i=0; i< packet_len; i++) {
//            printf("%02x ",raw_bytes[i]);
//            if (i%40 == 0 && i!=0) printf("\n");
//        }
//    }

    return true;
}

/**
 * tx_send_ui_packet()
 * Create and queue an AX25 packet on the TX queue
 * from and to callsigns are strings with nul termination
 * pid byte is F0, BB or BD
 * bytes is a buffer of length len that is sent in the body of the packet
 * The TX takes are of HDLC framing and CRC
 */
bool tx_send_ui_packet(char *from_callsign, char *to_callsign, uint8_t pid,
                       uint8_t *bytes, int len, bool block,
                       enum radio_modulation modulation)
{
    tx_radio_buffer_t tmp_packet_buffer;
    //uint8_t raw_bytes[AX25_PKT_BUFFER_LEN];
    bool rc = tx_make_ui_packet(from_callsign, to_callsign, pid,
                                bytes, len, modulation, &tmp_packet_buffer);
    TickType_t xTicksToWait = 0;

    if (block)
        xTicksToWait = CENTISECONDS(20);

    BaseType_t xStatus = xQueueSendToBack(xTxPacketQueue, &tmp_packet_buffer,
                                          xTicksToWait);
    if (xStatus != pdPASS) {
        /* The send operation could not complete because the queue was full */
        debug_print("TX QUEUE FULL: Could not add UI frame to Packet Queue.\n");
        ReportError(TxPacketDropped, FALSE, CharString,
                    (int)"AX25: ERROR: UI Packet QUEUE FULL");
        ReportToWatchdog(CurrentTaskWD);
        return false;
    }

    ReportToWatchdog(CurrentTaskWD);
    return true;
}

/**
 * tx_send_packet()
 * Create and queue an AX25 packet based on the packet structure
 *
 * Returns false if there is an issue
 *
 * Channel is passed in but not yet implemented.  Only 1 TX channel is
 * assumed
 *
 * NOTE - we have a mix of errors which could return false.  If this
 * is called from iFramePops then the failure causes the I-Frame to be
 * put back on the queue.  That means logic errors will repeat in a
 * loop.  They should be logged but return true as we want to ignore
 * the bad data.  That seems to be safer than locking up the BBS with
 * a bad packet in a loop.
 */
bool tx_send_packet(AX25_PACKET *packet, bool expedited, bool block,
                    enum radio_modulation modulation)
{
    tx_radio_buffer_t tmp_packet_buffer;
    bool rc = tx_make_packet(packet, modulation, &tmp_packet_buffer);

    if (rc == false) {
        debug_print("LOGIC ERROR: Invalid packet. Packet not sent\n");
        // return true here as we do not want to repeat this bad
        // packet.  Data is dropped
        return true;
    }
    //print_packet("TX_SEND: ", &tmp_packet_buffer[1], tmp_packet_buffer[0]);

    TickType_t xTicksToWait = 0;
    BaseType_t xStatus = pdFAIL;
    if (block)
         // wait for about 255/1200 seconds, which is long enough to
         // clear a packet from TX or several packets at 9600bps.
        xTicksToWait = CENTISECONDS(20);
    if (expedited)
        xStatus = xQueueSendToFront(xTxPacketQueue, &tmp_packet_buffer,
                                    xTicksToWait);
    else
        xStatus = xQueueSendToBack(xTxPacketQueue, &tmp_packet_buffer,
                                   xTicksToWait);
    if (xStatus != pdPASS) {
        /*
         * The send operation could not complete because the queue was
         * full.  */
        ReportError(TxPacketDropped, FALSE, CharString,
                    (int)"AX25: ERROR: TX Packet QUEUE FULL");
        return false;
    }

    return true;
}

/**
 * TEST ROUTINES FOLLOW
 */

bool tx_test_make_packet(uint32_t len)
{
    bool success;
    //uint8_t raw_bytes[AX25_PKT_BUFFER_LEN]; // position 0 will hold the number of bytes
    char *from_callsign = "PACSAT-12";
    char *to_callsign = "AC2CZ-2";
    uint8_t pid = 0xbb;
    uint8_t bytes[256];
    tx_radio_buffer_t tmp_packet_buffer;
    unsigned int i;

    debug_print("## SELF TEST: tx_test_make_packet\n");

    if (len > sizeof(bytes)) {
        debug_print("## FAILED SELF TEST: packet too lage\n");
        return false;
    }

    for (i = 0; i < len; i++)
        bytes[i] = i;

    success = tx_make_ui_packet(from_callsign, to_callsign, pid, bytes, len,
                                tx_modulation, &tmp_packet_buffer);
    if (!success) {
        debug_print("## FAILED SELF TEST: packet format failed\n");
        return false;
    }

    BaseType_t xStatus = xQueueSendToBack(xTxPacketQueue, &tmp_packet_buffer,
                                          CENTISECONDS(1));
    if(xStatus != pdPASS) {
        /* The send operation could not complete because the queue was full */
        debug_print("TX QUEUE FULL: Could not add to Packet Queue\n");
        success = false;
    }

    return success;
}
