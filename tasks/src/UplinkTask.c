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
#include "nonvolManagement.h"

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

bool ftl0_send_event(AX25_event_t *received_event, AX25_event_t *send_event);
bool ftl0_add_request(char *from_callsign, rx_channel_t channel, uint32_t file_id);
bool ftl0_remove_request(rx_channel_t channel);
bool ftl0_connection_received(char *from_callsign, char *to_callsign, rx_channel_t channel);
bool ftl0_disconnect(char *to_callsign, rx_channel_t channel);
int ftl0_make_packet(uint8_t *data_bytes, uint8_t *info, int length, int frame_type);
int ftl0_parse_packet_type(unsigned char * data);
int ftl0_parse_packet_length(unsigned char * data);

/* Local variables */
static ftl0_state_machine_t ftl0_state_machine[NUM_OF_RX_CHANNELS];
static AX25_event_t ax25_event; /* Static storage for event */
static AX25_event_t send_event_buffer;
static rx_channel_t current_channel_on_uplink; // This is the channel we will next send data to

#ifdef DEBUG
/* This decodes the AX25 error numbers.  Only used for debug. */
char *ax25_errors_strs[] = {
"F=1 received but P=1 not outstanding.", // A,
"Unexpected DM with F=1 in states 3, 4 or 5.", // B
"Unexpected UA in states 3, 4 or 5.", // C
"UA received without F=1 when SABM or DISC was sent P=1.", // D
"DM received in states 3, 4 or 5.", // E
"Data link reset, i.e., SABM received in state 3, 4 or 5.", // F
"DISC retries exceeded.", // G
"ERROR H not defined",
"N2 timeouts: unacknowledged data.", // I
"N(r) sequence ERROR_.", // J
"ERROR K not defined",
"Control field invalid or not implemented.", // L
"Information field was received in a U or S-type frame.", // M
"Length of frame incorrect for frame type.", // N
"I frame exceeded maximum allowed length.", // O
"N(s) out of the window.", // P
"UI response received, or UI command with P=1 received.", // Q
"UI frame exceeded maximum allowed length.", // R
"I response received.", // S
"N2 timeouts: no response to enquiry.", // T
"N2 timeouts: extended peer busy condition.", // U
"No DL machines available to establish connection." // V
};
#endif


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
//                trace_ftl0("Received event: %d\n",ax25_event.primative);

                if (ax25_event.primative == DL_ERROR_Indicate) {
                    // These are just for debugging.  Another event is sent if an action is needed.
                    debug_print("FTL0[%d]: ERR from AX25: %s\n",ax25_event.channel, ax25_errors_strs[ERROR_G]);
                } else {
                    ftl0_next_state_from_primative(&ftl0_state_machine[ax25_event.channel], &ax25_event);
                }
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

If any unexpected packet or ERROR is received from Layer 2 then we send the Data Link Terminated event and
enter UL_UNINIT.  We need to make sure that event is not putting us in a loop, where we receive the same
error or packet again.

*/

void ftl0_next_state_from_primative(ftl0_state_machine_t *state, AX25_event_t *event) {
    switch (state->ul_state) {
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

        case DL_DISCONNECT_Indicate : {
            trace_ftl0("Disconnect is in progress from Layer 2\n");
            // We consider this fatal and do not wait for the confirm message
            ftl0_remove_request(event->channel);
            break;
        }
        case DL_DISCONNECT_Confirm : {
            trace_ftl0("Disconnected from Layer 2\n");
            break;
        }
        case DL_CONNECT_Indicate : {
            trace_ftl0("Connection from Layer 2\n");
            ftl0_connection_received(event->packet.from_callsign, event->packet.to_callsign, event->channel);
            break;
        }
        case DL_CONNECT_Confirm : {
            trace_ftl0("Connection from Layer 2 Confirmed\n");
            ftl0_connection_received(event->packet.from_callsign, event->packet.to_callsign, event->channel);
            break;
        }
        default : {
            trace_ftl0("FTL0[%d]: .. Ignored\n",state->channel);
            break;
        }
    }

}

void ftl0_state_cmd_wait(ftl0_state_machine_t *state, AX25_event_t *event) {
    trace_ftl0("FTL0: STATE CMD WAIT: ");
}

void ftl0_state_cmd_ok(ftl0_state_machine_t *state, AX25_event_t *event) {
    trace_ftl0("FTL0: STATE CMD OK: ");
    switch (event->primative) {

        case DL_DISCONNECT_Indicate : // considered fatal, we don't wait for confirm
        case DL_DISCONNECT_Confirm : {
            trace_ftl0("Disconnected from Layer 2\n");
            ftl0_remove_request(event->channel);
            break;
        }
        case DL_CONNECT_Indicate :
        case DL_CONNECT_Confirm : {
            trace_ftl0("Connection from Layer 2\n");
            // Perhaos the other end missed the connection and was still trying.  Send the CMD OK message again.
            ftl0_remove_request(event->channel); // remove them first
            ftl0_connection_received(event->packet.from_callsign, event->packet.to_callsign, event->channel);
            break;
        }
        case DL_DATA_Indicate : {
            trace_ftl0("Data from Layer 2\n");

//            int ftl0_type = ftl0_parse_packet_type(data);
//            if (ftl0_type > MAX_PACKET_ID) {
//                int rc = ftl0_send_err(from_callsign, channel, ER_ILL_FORMED_CMD);
//                if (rc != EXIT_SUCCESS) {
//                    /* We likely could not send the error.  Something serious has gone wrong.
//                     * Not much we can do as we are going to offload the request anyway */
//                }
//                ftl0_disconnect(uplink_list[selected_station].callsign, uplink_list[selected_station].channel);
//                ftl0_remove_request(selected_station);
//            }
//            debug_print("%s: UL_CMD_OK - %s\n",uplink_list[selected_station].callsign, ftl0_packet_type_names[ftl0_type]);
//
//            /* Process the EVENT through the UPLINK STATE MACHINE */
//            switch (ftl0_type) {
//                case UPLOAD_CMD : {
//                    /* if OK to upload send UL_GO_RESP.  We determine if it is OK by checking if we have space
//                     * and that it is a valid continue of file_id != 0
//                     * This will send UL_GO_DATA packet if checks pass
//                     * TODO - the code would be clearer if this parses the request then returns here and then we
//                     * send the packet from here.  Then all packet sends are from this level of the state machine*/
//                    int err = ftl0_process_upload_cmd(selected_station, from_callsign, channel, data, len);
//                    if (err != ER_NONE) {
//                        // send the error
//                        rc = ftl0_send_err(from_callsign, channel, err);
//                        if (rc != EXIT_SUCCESS) {
//                            /* We likely could not send the error.  Something serious has gone wrong.
//                             * But the best we can do is remove the station and return the error code. */
//                            ftl0_disconnect(uplink_list[selected_station].callsign, uplink_list[selected_station].channel);
//                            ftl0_remove_request(selected_station);
//                        }
//                        // If we sent error successfully then we stay in state UL_CMD_OK and the station can try another file
//                        return rc;
//                    }
//                    // We move to state UL_DATA_RX
//                    uplink_list[selected_station].state = UL_DATA_RX;
//                    break;
//                }
//                default: {
//                    ftl0_disconnect(uplink_list[selected_station].callsign, uplink_list[selected_station].channel);
//                    ftl0_remove_request(selected_station);
//                    return EXIT_SUCCESS; // don't increment or change the current station
//                    break;
//                }
//            }

            break;
        }
        default : {
            trace_ftl0(".. Unexpected packet or event, disconnect\n");
            ftl0_disconnect(state->callsign, event->channel);
            break;
        }
    }
}

void ftl0_state_data_rx(ftl0_state_machine_t *state, AX25_event_t *event) {
    trace_ftl0("FTL0: STATE DATA RX: ");
    switch (event->primative) {

        case DL_DISCONNECT_Indicate : {
            trace_ftl0("Disconnect is in progress from Layer 2\n");
            // We consider this fatal and do not wait for the confirm message
            ftl0_remove_request(event->channel);
            break;
        }
        case DL_DISCONNECT_Confirm : {
            trace_ftl0("Disconnected from Layer 2\n");
            ftl0_remove_request(event->channel);
            break;
        }
        case DL_DATA_Indicate : {
            trace_ftl0("Data from Layer 2\n");
            break;
        }
        default : {
            trace_ftl0(".. Unexpected packet or event, disconnect\n");
            ftl0_disconnect(state->callsign, event->channel);
            break;
        }
    }
}

void ftl0_state_abort(ftl0_state_machine_t *state, AX25_event_t *event) {
    trace_ftl0("FTL0: STATE ABORT: ");
    switch (event->primative) {

        case DL_DISCONNECT_Indicate : {
            trace_ftl0("Disconnect is in progress from Layer 2\n");
            // We consider this fatal and do not wait for the confirm message
            ftl0_remove_request(event->channel);
            break;
        }
        case DL_DISCONNECT_Confirm : {
            trace_ftl0("Disconnected from Layer 2\n");
            ftl0_remove_request(event->channel);
            break;
        }
        case DL_DATA_Indicate : {
            trace_ftl0("Data from Layer 2\n");
            break;
        }
        default : {
            trace_ftl0(".. Unexpected packet or event, disconnect\n");
            ftl0_disconnect(state->callsign, event->channel);
            break;
        }
    }
}

/**
 * Send event to the data link state machine (Layer 2)
 * The Event type and any attached packet should already be set in the send_event
 *
 */
bool ftl0_send_event(AX25_event_t *received_event, AX25_event_t *send_event) {
    send_event->channel = received_event->channel;
    strlcpy(send_event->packet.to_callsign, received_event->packet.from_callsign, MAX_CALLSIGN_LEN);
    strlcpy(send_event->packet.from_callsign, BBS_CALLSIGN, MAX_CALLSIGN_LEN);

    if (send_event->primative == DL_DATA_Request) {
        BaseType_t xStatus = xQueueSendToBack( xIFrameQueue[received_event->channel], send_event, CENTISECONDS(1) );
        if( xStatus != pdPASS ) {
            /* The send operation could not complete because the queue was full */
            debug_print("I FRAME QUEUE FULL: Could not add to Event Queue for channel %d\n",received_event->channel);
            // TODO - we should log this error and downlink in telemetry
            return FALSE;
        } else {
            trace_ftl0("FTL0: Added iframe data event to Data Link SM: %d\n",send_event->primative);
        }
    } else {
        BaseType_t xStatus = xQueueSendToBack( xRxEventQueue, send_event, CENTISECONDS(1) );
        if( xStatus != pdPASS ) {
            /* The send operation could not complete because the queue was full */
            debug_print("RX Event QUEUE FULL: Could not add to Event Queue\n");
            // TODO - we should log this error and downlink in telemetry
            return FALSE;
        } else {
            trace_ftl0("FTL0: Added event to Data Link SM: %d\n",send_event->primative);
        }
    }
    return TRUE;
}


/**
 * ftl0_add_request()
 *
 * Add a callsign and its request to the uplink
 *
 * Make a copy of all the data because the original packet will be purged soon from the
 * circular buffer
 *
 * When adding a request the variable number_on_uplink points to the next empty slot
 *
 * returns EXIT_SUCCESS it it succeeds or EXIT_FAILURE if the PB is shut or full
 *
 */
bool ftl0_add_request(char *from_callsign, rx_channel_t channel, uint32_t file_id) {
    if (!ReadMRAMBoolState(StateUplinkEnabled)) {
        trace_ftl0("FTL0: Uplink closed\n");
        return FALSE;
    }

    int i;
    /* Each station can only be on the Uplink once, so reject if the callsign is already in the list */
    for (i=0; i < NUM_OF_RX_CHANNELS; i++) {
        if (ftl0_state_machine[channel].ul_state != UL_UNINIT) {
            if (strcasecmp(ftl0_state_machine[channel].callsign, from_callsign) == 0) {
                trace_ftl0("FTL0: %s is already on the uplink\n",from_callsign);
                return FALSE; // Station is already on the Uplink
            }
        }
    }

    trace_ftl0("FTL0 Connecting %s\n",ftl0_state_machine[channel].callsign);
    strlcpy(ftl0_state_machine[channel].callsign, from_callsign, MAX_CALLSIGN_LEN);
    ftl0_state_machine[channel].ul_state = UL_CMD_OK;
    ftl0_state_machine[channel].channel = channel;
    ftl0_state_machine[channel].file_id = file_id;
    ftl0_state_machine[channel].request_time = getSeconds(); // for timeout

    return TRUE;
}

/**
 * ftl0_remove_request()
 *
 * Remove the callsign / channel
 *
 *
 * return TRUE unless there is no item to remove.
 *
 */
bool ftl0_remove_request(rx_channel_t channel) {
    uint32_t now = getSeconds();
    int duration = (int)(now - ftl0_state_machine[channel].request_time);
    trace_ftl0("FTL0 Disconnecting %s - connected for %d seconds\n",ftl0_state_machine[channel].callsign, duration);

    /* Remove the item */
    ftl0_state_machine[channel].ul_state = UL_UNINIT;
    ftl0_state_machine[channel].file_id = 0;
    ftl0_state_machine[channel].request_time = 0;

    return TRUE;
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
bool ftl0_connection_received(char *from_callsign, char *to_callsign, rx_channel_t channel) {
    trace_ftl0("FTL0: Connection for File Upload from: %s\n",from_callsign);

    /* Add the request, which initializes their uplink state machine. At this point we don't know the
     * file number, offset or dir node */
    bool rc = ftl0_add_request(from_callsign, channel, 3);
    if (rc == FALSE){
        /* We could not add this request, either full or already on the uplink.  Disconnect. */
        ftl0_disconnect(from_callsign, channel);
        return FALSE;
    } else {
        trace_ftl0("FTL0: Added %s to uplink list\n",from_callsign);
        //ftl0_debug_print_list();
    }

    int frame_type = LOGIN_RESP;
    FTL0_LOGIN_DATA login_data;

    uint8_t flag = 0;
    flag |= 1UL << FTL0_VERSION_BIT1;
    flag |= 1UL << FTL0_VERSION_BIT2;
    flag |= 1UL << FTL0_PFH_BIT; // Set the bit to require PFHs

    login_data.login_time = htotl(getUnixTime());
    login_data.login_flags = flag;

    rc = ftl0_make_packet(send_event_buffer.packet.data, (uint8_t *)&login_data, sizeof(login_data), frame_type);
    if (rc != TRUE) {
        debug_print("Could not make FTL0 LOGIN packet\n");
        return FALSE;
    }

    send_event_buffer.primative = DL_DATA_Request;
    send_event_buffer.packet.frame_type = TYPE_I;
    send_event_buffer.packet.data_len = sizeof(login_data)+2;

    rc = ftl0_send_event(&ax25_event, &send_event_buffer);

    if (rc != TRUE) {
        // TODO - log error
        // Disconnect??  Retry??
        debug_print("Could not send FTL0 LOGIN packet to Data Link State Machine \n");
        return FALSE;
    }
    return TRUE;
}

/**
 * ftl0_disconnect()
 *
 * Disconnect from the station specified in to_callsign
 *
 */
bool ftl0_disconnect(char *to_callsign, rx_channel_t channel) {
    trace_ftl0("FTL0: Disconnecting: %s\n", to_callsign);
    send_event_buffer.primative = DL_DISCONNECT_Request;

    bool rc = ftl0_send_event(&ax25_event, &send_event_buffer);

    if (rc != TRUE) {
        // TODO - log error
        // Disconnect??  Retry??
        debug_print("Could not send FTL0 Disconnect Event to Data Link State Machine \n");
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

int ftl0_parse_packet_type(unsigned char * data) {
    int type = data[1] & 0b00011111;
    return type;
}

int ftl0_parse_packet_length(unsigned char * data) {
    int length = (data[1] >> 5) * 256 + data[0];
    return length;
}
