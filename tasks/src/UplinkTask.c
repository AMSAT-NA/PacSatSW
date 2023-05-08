/*
 * UplinkTask.c
 *
 *  Created on: May 3, 2023
 *      Author: g0kla
 */
#include <strings.h>

#include "PbTask.h"
#include "TxTask.h"
#include "Ax25Task.h"
#include "UplinkTask.h"
#include "FreeRTOS.h"
#include "os_task.h"
#include "MET.h"
#include "inet.h"

#include "RxTask.h"
#include "ax25_util.h"
#include "str_util.h"


#define TRACE_FTL0
#ifdef TRACE_FTL0
#define trace_ftl0 printf
#else
#define trace_ftl0 //
#endif

void ftl0_next_state_from_primative(ftl0_state_machine_t *state, AX25_event_t *event);
void ftl0_state_uninit(ftl0_state_machine_t *state, AX25_event_t *event);
void ftl0_state_cmd_wait(ftl0_state_machine_t *state, AX25_event_t *event);
void ftl0_state_cmd_ok(ftl0_state_machine_t *state, AX25_event_t *event);
void ftl0_state_data_rx(ftl0_state_machine_t *state, AX25_event_t *event);
void ftl0_state_abort(ftl0_state_machine_t *state, AX25_event_t *event);

int ftl0_connection_received(char *from_callsign, char *to_callsign, rx_channel_t channel);
int ftl0_make_packet(uint8_t *data_bytes, uint8_t *info, int length, int frame_type);

/* Local variables */
static ftl0_state_machine_t ftl0_state_machine[NUM_OF_RX_CHANNELS];
static AX25_event_t ax25_event; /* Static storage for event */
static AX25_event_t send_event_buffer;

portTASK_FUNCTION_PROTO(UplinkTask, pvParameters)  {

    vTaskSetApplicationTaskTag((xTaskHandle) 0, (pdTASK_HOOK_CODE)UplinkTaskWD );
    ResetAllWatchdogs();
    printf("Initializing Uplink FTL0 Task\n");

    while(1) {
        BaseType_t xStatus = xQueueReceive( xUplinkEventQueue, &ax25_event, CENTISECONDS(1) );  // Wait to see if data available
        if( xStatus == pdPASS ) {
            if (ax25_event.channel >= NUM_OF_RX_CHANNELS) {
                // something is seriously wrong.  Programming error.  Unlikely to occur in flight
                debug_print("ERR: AX25 channel %d is invalid\n",ax25_event.channel);
            } else {
                trace_ftl0("Received event: %d\n",ax25_event.primative);
                ftl0_next_state_from_primative(&ftl0_state_machine[ax25_event.channel], &ax25_event);
            }
        }

    }
}

/*
 The Uplink state machine processes the AX25 events - these are the same as the Data Link SM events, just the other way around
 Mainly we process:
   DL_CONNECT_Indicate received for connection
   DL_DISCONNECT_Request - sent when we are done
   DL_DISCONNECT_Confirm - received when we are disconnected.  This can also be received because T3 expired!
   DL_DATA_Indicate - when DATA received
   DL_DATA_Request - when we send data

The FTL0 SM only processes DATA commands and has its own set of states.
*/

void ftl0_next_state_from_primative(ftl0_state_machine_t *state, AX25_event_t *event) {
    switch (state->dl_state) {
        case UL_UNINIT : {
            ftl0_state_uninit(state, event);
            break;
        }
        case UL_CMD_WAIT : {
            ftl0_state_cmd_wait(state, event);
            break;
        }
        case UL_CMD_OK : {
            ftl0_state_cmd_ok(state, event);
            break;
        }
        case UL_DATA_RX : {
            ftl0_state_data_rx(state, event);
            break;
        }
        case UL_ABORT : {
            ftl0_state_abort(state, event);
            break;
        }
        default:
            break;
    }
}

/**
 * The Uplink is idle and waiting for a connection
 */
void ftl0_state_uninit(ftl0_state_machine_t *state, AX25_event_t *event) {
    trace_ftl0("FTL0: STATE UNINIT: ");
    switch (event->primative) {
        case DL_DISCONNECT_Confirm : {
            trace_ftl0("Disconnect from Layer 2\n");
            break;
        }
        case DL_CONNECT_Indicate : {
            trace_ftl0("Connection from Layer 2\n");
            ftl0_connection_received(event->packet.from_callsign, event->packet.to_callsign, event->channel);
            break;
        }
        default : {
            trace_ftl0(".. Ignored\n");
            break;
        }

    }

}

void ftl0_state_cmd_wait(ftl0_state_machine_t *state, AX25_event_t *event) {
    trace_ftl0("FTL0: STATE CMD WAIT: ");
}

void ftl0_state_cmd_ok(ftl0_state_machine_t *state, AX25_event_t *event) {
    trace_ftl0("FTL0: STATE CMD OK: ");
}

void ftl0_state_data_rx(ftl0_state_machine_t *state, AX25_event_t *event) {
    trace_ftl0("FTL0: STATE DATA RX: ");
}

void ftl0_state_abort(ftl0_state_machine_t *state, AX25_event_t *event) {
    trace_ftl0("FTL0: STATE ABORT: ");
}

/**
 * ftl0_connection_received()
 *
When the data link is established the server transmits a LOGIN_RESP packet.

Packet: LOGIN_RESP
Information: 5 bytes
struct LOGIN_DATA{
     unsigned long login_time;
     unsigned char login_flags;
}

<login_time>  -  a 32-bit unsigned integer indicating the  number  of  seconds
since January 1, 1970.

<login_flags> - an 8-bit field.
     bit:76543210
         xxxxSHVV

Bit 3, the SelectionActive bit, will be 1 if there is already an active selec-
tion list for this client.  The SelectionActive bit will be 0 if there is no
active selection for this client already available.

Bit 2, the HeaderPFH bit, will be 1 if the server uses and requires PACSAT
File Headers on all files.  If the HeaderPFH bit is 1, the flag PFHserver used
in the following definition should be considered TRUE.

The HeaderPFH bit will be 0 if the server does not use PACSAT File Headers.
If the HeaderPFH bit is 0, the modified procedures specified in Section 7
should be used.

Bits 1 and 0 form a 2-bit FTL0 protocol version number.  The version described
here is 0.

Upon transmitting the LOGIN_RESP packet the server should initialize its state
variables to UL_CMD_WAIT

// TODO - Falconsat3 transmits a UI frame to confirm the login as well, perhaps so that
 * other stations can see who has logged in??
 *
 */
int ftl0_connection_received(char *from_callsign, char *to_callsign, rx_channel_t channel) {
    debug_print("Connection for File Upload from: %s\n",from_callsign);

    /* Add the request, which initializes their uplink state machine. At this point we don't know the
     * file number, offset or dir node */
//    int rc = ftl0_add_request(from_callsign, channel, 3);
//    if (rc != EXIT_SUCCESS){
//        /* We could not add this request, either full or already on the uplink.  Disconnect. */
//        ftl0_disconnect(from_callsign, channel);
//        return EXIT_SUCCESS;
//    } else {
//        debug_print("Added %s to uplink list\n",from_callsign);
//        ftl0_debug_print_list();
//    }

    int frame_type = LOGIN_RESP;
    FTL0_LOGIN_DATA login_data;

    uint8_t flag = 0;
    flag |= 1UL << FTL0_VERSION_BIT1;
    flag |= 1UL << FTL0_VERSION_BIT2;
    flag |= 1UL << FTL0_PFH_BIT; // Set the bit to require PFHs

    login_data.login_time = htotl(getUnixTime());
    login_data.login_flags = flag;

    bool rc = ftl0_make_packet(send_event_buffer.packet.data, (uint8_t *)&login_data, sizeof(login_data), frame_type);
    if (rc != TRUE) {
           debug_print("Could not make FTL0 LOGIN packet\n");
           return FALSE;
       }

    send_event_buffer.channel = channel;
    send_event_buffer.primative = DL_DATA_Request;
    strlcpy(send_event_buffer.packet.to_callsign, from_callsign, MAX_CALLSIGN_LEN);
    strlcpy(send_event_buffer.packet.from_callsign, BBS_CALLSIGN, MAX_CALLSIGN_LEN);
    send_event_buffer.packet.frame_type = TYPE_I;
    send_event_buffer.packet.data_len = sizeof(login_data)+2;

    BaseType_t xStatus = xQueueSendToBack( xRxEventQueue, &send_event_buffer, CENTISECONDS(1) );
    if( xStatus != pdPASS ) {
        /* The send operation could not complete because the queue was full */
        debug_print("RX Event QUEUE FULL: Could not add to Event Queue\n");
        // TODO - we should log this error and downlink in telemetry
        return FALSE;
    } else {
        trace_ftl0("Added event to Data Link SM: %d\n",send_event_buffer.primative);
    }

    if (rc != TRUE) {
        debug_print("Could not send FTL0 LOGIN packet to Data Link State Machine \n");
        return FALSE;
    }
    return TRUE;
}

/**
 * ftl0_make_packet()
 *
 * Pass the ftl0 info bytes, the length of the info bytes and frame type
 * returns packet in unsigned char *data_bytes, which must be length + 2 long
 *
Packets flow as follows:

<length_lsb><h1>[<info>...]<length_lsb><h1>[<info>...]
|----First FTL0 packet-----|----Second FTL0 packet---|

<length_lsb>  - 8 bit unsigned integer supplying the least significant 8  bits
of data_length.

<h1> - an 8-bit field.
     bits 7-5 contribute 3 most significant bits to data_length.

     bits 4-0 encode 32 packet types
 */
int ftl0_make_packet(uint8_t *data_bytes, uint8_t *info, int length, int frame_type) {
    uint8_t length_lsb = length & 0xff; // least 8 bits of length
    uint8_t h1 = (length >> 8) & 0x07; // 3 most sig bits of length
    h1 = (uint8_t)(frame_type | (h1 << 5)); // move into bits 7-5

    /* Copy the bytes into the frame */
    data_bytes[0] = length_lsb;
    data_bytes[1] = h1;
    int i;
    if (info != NULL && sizeof(info) > 0) {
        for (i=0; i<length;i++ )
            data_bytes[i+2] = info[i];
    }
    return TRUE;
}
