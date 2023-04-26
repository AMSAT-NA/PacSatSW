/*
 * TncTask.c
 *
 *  Created on: Dec 1, 2022
 *      Author: g0kla
 */

#include "ctype.h"

#include "pacsat.h"
#include "ax5043_access.h"
#include "ax5043-ax25.h"
#include "TxTask.h"
#include "FreeRTOS.h"
#include "os_task.h"
#include "ax25_util.h"

void radio_set_power(uint32_t regVal);

static uint8_t tx_packet_buffer[AX25_PKT_BUFFER_LEN]; /* Buffer used when data copied from tx queue */
static uint8_t tmp_packet_buffer[AX25_PKT_BUFFER_LEN]; /* Buffer used when constructing new packets. position 0 will hold the number of bytes */
static SPIDevice device = DCTDev1;
extern bool monitorPackets;

/* Test Buffer PB Empty */
//uint8_t byteBuf[] = {0xA0,0x84,0x98,0x92,0xA6,0xA8,0x00,0xA0,0x8C,0xA6,0x66,
//                     0x40,0x40,0x17,0x03,0xF0,0x50,0x42,0x3A,0x20,0x45,0x6D,0x70,0x74,0x79,0x2E,0x0D};

portTASK_FUNCTION_PROTO(TxTask, pvParameters)  {

    vTaskSetApplicationTaskTag((xTaskHandle) 0, (pdTASK_HOOK_CODE)TxTaskWD ); // TODO - just reuse the Radio task name for now
    InitInterTask(ToTxTask, 10);
    ResetAllWatchdogs();
//    printf("Initializing TX\n");

    /* This is defined in pacsat.h, declared here */
    xTxPacketQueue = xQueueCreate( TX_PACKET_QUEUE_LEN, AX25_PKT_BUFFER_LEN * sizeof( uint8_t ) );
    if (xTxPacketQueue == NULL) {
        /* The queue could not be created.  This is fatal and should only happen in test if we are short of memory at startup */
        debug_print("FATAL ERROR: Could not create TX Packet Queue\n");
    }


    ax5043StartTx(device);
    radio_set_power(0x020); // minimum power to test

    /* Set Power state to FULL_TX */
     ax5043WriteReg(device, AX5043_PWRMODE, AX5043_PWRSTATE_FULL_TX);

    while(1) {

        uint8_t pktstart_flag = 0x01;
        uint8_t pktend_flag = 0x02;
        uint8_t raw_no_crc_flag = 0x18; // Flag of 0x18 is RAW no CRC
        uint8_t preamble_length = 32; // TODO - Set programatically!!!!  10 for 1200 bps - Radio lab recommends 32 for 9600, may need as much as 56.

        ReportToWatchdog(CurrentTaskWD);
        BaseType_t xStatus = xQueueReceive( xTxPacketQueue, &tx_packet_buffer, CENTISECONDS(10) );  // TODO - adjust block time vs watchdog
        ReportToWatchdog(CurrentTaskWD);
        if( xStatus == pdPASS ) {
            /* Data was successfully received from the queue */
            int numbytes = tx_packet_buffer[0] - 1; // first byte holds number of bytes
            //        printf("FIFO_FREE 1: %d\n",fifo_free());
            ax5043WriteReg(device, AX5043_FIFOSTAT, 3); // clear FIFO data & flags
            fifo_repeat_byte(device, 0x7E, preamble_length, raw_no_crc_flag); // repeat the preamble bytes  ///  TODO - preamble length needs to be 20 for 9600
            fifo_commit(device);
            fifo_queue_buffer(device, tx_packet_buffer+1, numbytes, pktstart_flag|pktend_flag);
            //       printf("FIFO_FREE 2: %d\n",fifo_free());
            fifo_commit(device);
            if (monitorPackets)
                print_packet("TX", tx_packet_buffer+1,numbytes);
            //       printf("INFO: Waiting for transmission to complete\n");
            while (ax5043ReadReg(device, AX5043_RADIOSTATE) != 0) {
                vTaskDelay(1);
            }
            //       printf("INFO: Transmission complete\n");
        }
    }
}

/**
 * Pout = Pmax * txpwrcoeffb / (2^12 - 1)
 * Where Pmax = 0dBm
 *
 * FFF is max
 *
 */
void radio_set_power(uint32_t regVal) {
    ax5043WriteReg(device, AX5043_TXPWRCOEFFB0,regVal);
    ax5043WriteReg(device, AX5043_TXPWRCOEFFB1,regVal>>8);
}

/**
 * tx_make_packet()
 * Create an AX25 packet
 * from and to callsigns are strings will nul termination
 * pid byte is F0, BB or BD
 * bytes is a buffer of length len that is sent in the body of the packet
 * raw_bytes must be allocated by the caller and contains the packet.  The length is
 * stored in the first byte, which will not be transmitted.
 *
 */
bool tx_make_packet(char *from_callsign, char *to_callsign, uint8_t pid, uint8_t *bytes, int len, uint8_t *raw_bytes) {
    uint8_t packet_len;
    uint8_t header_len = 17;
    int i;
    unsigned char buf[7];
    int l = encode_call(to_callsign, buf, false, 0);
    if (l != true) return false;
    for (i=0; i<7; i++)
        raw_bytes[i+1] = buf[i];
    l = encode_call(from_callsign, buf, true, 0);
    if (l != true) return false;
    for (i=0; i<7; i++)
        raw_bytes[i+8] = buf[i];
    raw_bytes[15] = 0x03; // UI Frame control byte
    raw_bytes[16] = pid;

    for (i=0; i< len; i++) {
        raw_bytes[i+header_len] = bytes[i];
    }
    packet_len = len + header_len;
    raw_bytes[0] = packet_len; /* Number of bytes */

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
 * tx_send_packet()
 * Create and queue an AX25 packet on the TX queue
 * from and to callsigns are strings will nul termination
 * pid byte is F0, BB or BD
 * bytes is a buffer of length len that is sent in the body of the packet
 * The TX takes are of HDLC framing and CRC
 *
 */
bool tx_send_packet(char *from_callsign, char *to_callsign, uint8_t pid, uint8_t *bytes, int len, bool block) {
    //uint8_t raw_bytes[AX25_PKT_BUFFER_LEN];
    bool rc = tx_make_packet(from_callsign, to_callsign, pid, bytes, len, tmp_packet_buffer);
    TickType_t xTicksToWait = 0;
    if (block)
        xTicksToWait = CENTISECONDS(1);
    BaseType_t xStatus = xQueueSendToBack( xTxPacketQueue, &tmp_packet_buffer, xTicksToWait );
    if( xStatus != pdPASS ) {
        /* The send operation could not complete because the queue was full */
        debug_print("TX QUEUE FULL: Could not add to Packet Queue\n");
        // TODO - we should log this error and downlink in telemetry
    }

    return true;
}

/**
 * TEST ROUTINES FOLLOW
 */

bool tx_test_make_packet() {
    debug_print("## SELF TEST: tx_test_make_packet\n");
    bool rc = true;
    //uint8_t raw_bytes[AX25_PKT_BUFFER_LEN]; // position 0 will hold the number of bytes
    char *from_callsign = "PACSAT-12";
    char *to_callsign = "AC2CZ-2";
    uint8_t pid = 0xbb;
    uint8_t bytes[] = {0,1,2,3,4,5,6,7,8,9};
    uint8_t len = 10;
    rc = tx_make_packet(from_callsign, to_callsign, pid, bytes, len, tmp_packet_buffer);

    BaseType_t xStatus = xQueueSendToBack( xTxPacketQueue, &tmp_packet_buffer, CENTISECONDS(1) );
    if( xStatus != pdPASS ) {
        /* The send operation could not complete because the queue was full */
        debug_print("TX QUEUE FULL: Could not add to Packet Queue\n");
        rc = FALSE;
    }

    if (rc == FALSE) {
        debug_print("## FAILED SELF TEST: pb_test_status\n");
    } else {
        debug_print("## PASSED SELF TEST: pb_test_status\n");
    }
    return rc;
}

