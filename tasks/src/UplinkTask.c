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
#include "redposix.h"

#include "RxTask.h"
#include "ax25_util.h"
#include "pacsat_dir.h"
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
bool ftl0_add_request(char *from_callsign, rx_channel_t channel);
bool ftl0_remove_request(rx_channel_t channel);
bool ftl0_connection_received(char *from_callsign, char *to_callsign, rx_channel_t channel);
bool ftl0_disconnect(char *to_callsign, rx_channel_t channel);
int ftl0_send_err(char *from_callsign, int channel, int err);
int ftl0_send_ack(char *from_callsign, int channel);
int ftl0_send_nak(char *from_callsign, int channel, int err);
int ftl0_process_upload_cmd(ftl0_state_machine_t *state, uint8_t *data, int len);
int ftl0_process_data_cmd(ftl0_state_machine_t *state, uint8_t *data, int len);
int ftl0_process_data_end_cmd(ftl0_state_machine_t *state, uint8_t *data, int len);

int ftl0_make_packet(uint8_t *data_bytes, uint8_t *info, int length, int frame_type);
int ftl0_parse_packet_type(uint8_t * data);
int ftl0_parse_packet_length(uint8_t * data);
void ftl0_make_tmp_filename(int file_id, char *dir_name, char *filename, int max_len);

/* Local variables */
static ftl0_state_machine_t ftl0_state_machine[NUM_OF_RX_CHANNELS];
static AX25_event_t ax25_event; /* Static storage for event */
static AX25_event_t send_event_buffer;

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
char * ftl0_packet_type_names[] = {"DATA","DATA_END","LOGIN_RESP","UPLOAD_CMD","UL_GO_RESP","UL_ERROR_RESP","UL_ACK_RESP","UL_NAL_RESP"};


portTASK_FUNCTION_PROTO(UplinkTask, pvParameters)  {

    vTaskSetApplicationTaskTag((xTaskHandle) 0, (pdTASK_HOOK_CODE)UplinkTaskWD );
    ResetAllWatchdogs();
    printf("Initializing Uplink FTL0 Task\n");

    /* Setup a timer to send the status periodically */
    xTimerHandle uplinkStatusTimerHandle;
    volatile portBASE_TYPE timerStatus;
    int pvtUplinkStatusTimerID = 0; // timer id

    /* create a RTOS software timer - TODO period should be in MRAM and changeable from the ground using xTimerChangePeriod() */
    uplinkStatusTimerHandle = xTimerCreate( "UPLINK STATUS", SECONDS(40), TRUE, &pvtUplinkStatusTimerID, ax25_send_status); // auto reload timer
    /* start the timer */
//    timerStatus = xTimerStart(uplinkStatusTimerHandle, 0); // Block time of zero as this can not block
//    if (timerStatus != pdPASS) {
//        debug_print("ERROR: Failed in init PB Status Timer\n");
//// TODO =>        ReportError(RTOSfailure, FALSE, ReturnAddr, (int) PbTask); /* failed to create the RTOS timer */
//        // TODO - it's possible this might fail.  Somehow we should recover from that.
//    }


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
                    debug_print("FTL0[%d]: ERR from AX25: %s\n",ax25_event.channel, ax25_errors_strs[ax25_event.error_num]);
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

The FTL0 State Machine only processes DATA commands and has its own set of states.

If any unexpected packet or ERROR is received from Layer 2 then we send the Data Link Terminated event and
enter UL_UNINIT.  We need to make sure that event is not putting us in a loop, where we receive the same
error or packet again and again.

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

    trace_ftl0("FTL0[%d]: STATE UNINIT: ",state->channel);
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
    trace_ftl0("FTL0[%d]: STATE CMD WAIT: ",state->channel);
}

void ftl0_state_cmd_ok(ftl0_state_machine_t *state, AX25_event_t *event) {
    trace_ftl0("FTL0[%d]: STATE CMD OK: ",state->channel);
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
            int rc;
            int ftl0_type = ftl0_parse_packet_type(event->packet.data);
            if (ftl0_type >= MAX_PACKET_ID) {
                rc = ftl0_send_err(event->packet.from_callsign, event->channel, ER_ILL_FORMED_CMD);
                if (rc != TRUE) {
                    /* We likely could not send the error.  Something serious has gone wrong.
                     * Not much we can do as we are going to offload the request anyway */
                }
                ftl0_disconnect(state->callsign, state->channel);
                ftl0_remove_request(state->channel);
            }
            trace_ftl0("FTL0[%d]: %s: UL_CMD_OK - %s\n",state->channel, state->callsign, ftl0_packet_type_names[ftl0_type]);

            /* Process the EVENT through the UPLINK STATE MACHINE */
            switch (ftl0_type) {
                case UPLOAD_CMD : {
                    /* if OK to upload send UL_GO_RESP.  We determine if it is OK by checking if we have space
                     * and that it is a valid continue of file_id != 0
                     * This will send UL_GO_DATA packet if checks pass
                     * TODO - the code might be clearer if this parses the request then returns here and then we
                     * send the packet from here.  Then all packet sends are from this level of the state machine.
                     * But this case within case is already long ... */
                    int err = ftl0_process_upload_cmd(state, event->packet.data, event->packet.data_len);
                    if (err != ER_NONE) {
                        // send the error
                        rc = ftl0_send_err(event->packet.from_callsign, event->channel, err);
                        if (rc != TRUE) {
                            /* We likely could not send the error.  Something serious has gone wrong.
                             * But the best we can do is remove the station and return the error code. */
                            ftl0_disconnect(state->callsign, state->channel);
                            ftl0_remove_request(state->channel);
                            break;
                        }
                        // If we sent error successfully then we stay in state UL_CMD_OK and the station can try another file
                        break;
                    }
                    // We move to state UL_DATA_RX
                    state->ul_state = UL_DATA_RX;
                    break;
                }
                default: {
                    trace_ftl0("FTL0: Unknown FTL0 command %d\n",ftl0_type);
                    ftl0_disconnect(state->callsign, state->channel);
                    ftl0_remove_request(state->channel);
                    break;
                }
            }

            break; // End of DL_DATA_Indicate
        }
        default : {
            trace_ftl0(".. Unexpected packet or event, disconnect\n");
            ftl0_disconnect(state->callsign, event->channel);
            ftl0_remove_request(state->channel);
            break;
        }
    }
}

/**
 * State UL_DATA_RC
 *
   EVENT: Receive DATA packet {
        Try to store data.
        if out of storage {
             Transmit UL_NAK_RESP packet.
             Close file.
             Save file_offset and file_number.
             ul_state <- UL_ABORT
        }
        else {
             Update file_offset.
             ul_state <- UL_DATA_RX
        }
   }

   EVENT: Receive DATA_END packet {
        Close file.
        if file passes checks {
             Transmit UL_ACK_RESP packet
             ul_state <- UL_CMD_OK
        }
        else {
             file_offset <- 0
             Save file_offset and file_number
             Transmit UL_NAK_RESP packet
             ul_state <- UL_CMD_OK
        }
   }

   EVENT: DEFAULT{
        if (file_offset > 0)
             Save file_offset and file number.
        if EVENT is not "data link terminated"
              Terminate data link.
        ul_state <- UL_UNINIT
   }
 */
void ftl0_state_data_rx(ftl0_state_machine_t *state, AX25_event_t *event) {
    trace_ftl0("FTL0[%d]: STATE UL_DATA_RX: ",state->channel);
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
            //trace_ftl0("Data from Layer 2\n");
            int rc;
            int ftl0_type = ftl0_parse_packet_type(event->packet.data);
            if (ftl0_type >= MAX_PACKET_ID) {
                rc = ftl0_send_err(event->packet.from_callsign, event->channel, ER_ILL_FORMED_CMD);
                if (rc != TRUE) {
                    /* We likely could not send the error.  Something serious has gone wrong.
                     * Not much we can do as we are going to offload the request anyway */
                }
                ftl0_disconnect(state->callsign, state->channel);
                ftl0_remove_request(state->channel);
            }
            trace_ftl0("FTL0[%d]: Layer 2 Data from %s: in FTL0 Packet: %s\n",state->channel, state->callsign, ftl0_packet_type_names[ftl0_type]);

            /* Process the EVENT through the UPLINK STATE MACHINE */
            switch (ftl0_type) {
                case DATA : {
                    trace_ftl0("FTL0[%d]: %s: UL_DATA_RX - DATA RECEIVED\n",state->channel, state->callsign);
                    int err = ftl0_process_data_cmd(state, event->packet.data, event->packet.data_len);
                    if (err != ER_NONE) {
                        // send the error
                        rc = ftl0_send_err(event->packet.from_callsign, event->channel, err);
                        if (rc != TRUE) {
                            /* We likely could not send the error.  Something serious has gone wrong.
                             * But the best we can do is remove the station and return the error code. */
                            ftl0_disconnect(state->callsign, state->channel);
                            ftl0_remove_request(state->channel);
                            break;
                        }
                        // If we sent error successfully then we stay in state UL_DATA_RX and the station can send more data

                    }
                    break;
                }
                case DATA_END : {
                    trace_ftl0("FTL0[%d]: %s: UL_DATA_RX - DATA END RECEIVED\n",state->channel, state->callsign);
                    int err = ftl0_process_data_end_cmd(state, event->packet.data, event->packet.data_len);
                    if (err != ER_NONE) {
                        rc = ftl0_send_nak(event->packet.from_callsign, event->channel, err);
                    } else {
                        debug_print(" *** SENDING ACK *** \n");
                        rc = ftl0_send_ack(event->packet.from_callsign, event->channel);
                    }
                    state->ul_state = UL_CMD_OK;
                    if (rc != TRUE) {
                        ftl0_disconnect(state->callsign, state->channel);
                        ftl0_remove_request(state->channel);
                    }
                    break;
                }
                default : {
                    ftl0_disconnect(state->callsign, state->channel);
                    ftl0_remove_request(state->channel);
                    break;
                }
            }

            break; // end of DL_DATA_Indicate
        }
        default : {
            trace_ftl0(".. Unexpected packet or event, disconnect\n");
            ftl0_disconnect(state->callsign, event->channel);
            ftl0_remove_request(state->channel);
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
            ftl0_remove_request(state->channel);
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
    send_event->primative = DL_DATA_Request;
    send_event->packet.frame_type = TYPE_I;
    strlcpy(send_event->packet.to_callsign, received_event->packet.from_callsign, MAX_CALLSIGN_LEN);
    strlcpy(send_event->packet.from_callsign, BBS_CALLSIGN, MAX_CALLSIGN_LEN);

    if (send_event->primative == DL_DATA_Request) {
        // Add data events directly to the iFrame Queue
        BaseType_t xStatus = xQueueSendToBack( xIFrameQueue[received_event->channel], send_event, CENTISECONDS(1) );
        if( xStatus != pdPASS ) {
            /* The send operation could not complete because the queue was full */
            debug_print("I FRAME QUEUE FULL: Could not add to Event Queue for channel %d\n",received_event->channel);
            // TODO - we should log this error and downlink in telemetry
            return FALSE;
        }
    } else {
        // All other events are added to the event Queue
        BaseType_t xStatus = xQueueSendToBack( xRxEventQueue, send_event, CENTISECONDS(1) );
        if( xStatus != pdPASS ) {
            /* The send operation could not complete because the queue was full */
            debug_print("RX Event QUEUE FULL: Could not add to Event Queue\n");
            // TODO - we should log this error and downlink in telemetry
            return FALSE;
        } else {
            trace_ftl0("FTL0[%d]: Sending Event %d\n",send_event->channel, send_event->primative);
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
bool ftl0_add_request(char *from_callsign, rx_channel_t channel) {
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

//    trace_ftl0("FTL0 Connecting %s\n",ftl0_state_machine[channel].callsign);
    strlcpy(ftl0_state_machine[channel].callsign, from_callsign, MAX_CALLSIGN_LEN);
    ftl0_state_machine[channel].ul_state = UL_CMD_OK;
    ftl0_state_machine[channel].channel = channel;
    ftl0_state_machine[channel].file_id = 0; // Set once the UPLD packet received
    ftl0_state_machine[channel].request_time = getSeconds(); // for timeout
    ftl0_state_machine[channel].file_id = 0; // Set when UPLD packet received

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
#ifdef DEBUG
    uint32_t now = getSeconds();
    int duration = (int)(now - ftl0_state_machine[channel].request_time);
    trace_ftl0("FTL0 Disconnecting %s - connected for %d seconds\n",ftl0_state_machine[channel].callsign, duration);
#endif
    /* Remove the item */
    ftl0_state_machine[channel].ul_state = UL_UNINIT;
    ftl0_state_machine[channel].file_id = 0;
    ftl0_state_machine[channel].request_time = 0;
    ftl0_state_machine[channel].length = 0;

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
    //trace_ftl0("FTL0: Connection for File Upload from: %s\n",from_callsign);

    /* Add the request, which initializes their uplink state machine. At this point we don't know the
     * file number, offset or dir node */
    bool rc = ftl0_add_request(from_callsign, channel);
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

    send_event_buffer.packet.data_len = sizeof(login_data)+2;

    rc = ftl0_send_event(&ax25_event, &send_event_buffer);

    if (rc != TRUE) {
        // TODO - log error
        // Disconnect??  Retry??
        debug_print("Could not send FTL0 LOGIN packet to Data Link State Machine \n");
        return FALSE;
    } else {
        trace_ftl0("FTL0:[%d]: Sending FTL0 LOGIN PKT\n",channel);
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

int ftl0_send_err(char *from_callsign, int channel, int err) {
    int frame_type = UL_ERROR_RESP;
    uint8_t err_info[1];
    err_info[0] = err;

    int rc = ftl0_make_packet(send_event_buffer.packet.data, err_info, sizeof(err_info), frame_type);
    if (rc != TRUE) {
        debug_print("Could not make FTL0 ERR packet\n");
        return FALSE;
    }
    send_event_buffer.packet.data_len = sizeof(err_info)+2;

    rc = ftl0_send_event(&ax25_event, &send_event_buffer);
    if (rc != TRUE) {
        debug_print("Could not send FTL0 ERR packet to TNC \n");
        return FALSE;
    } else {
        trace_ftl0("FTL0:[%d]: Sending FTL0 ERR %d\n",channel, err);
    }
    return TRUE;
}


int ftl0_send_ack(char *from_callsign, int channel) {
    int frame_type = UL_ACK_RESP;

    int rc = ftl0_make_packet(send_event_buffer.packet.data, (uint8_t *)NULL, 0, frame_type);
    if (rc != TRUE) {
        debug_print("Could not make FTL0 ACK packet\n");
        return FALSE;
    }
    send_event_buffer.packet.data_len = 2;

    rc = ftl0_send_event(&ax25_event, &send_event_buffer);
    if (rc != TRUE) {
        debug_print("Could not send FTL0 ACK packet to TNC \n");
        return FALSE;
    }else {
        trace_ftl0("FTL0:[%d]: Sending FTL0 ACK %d\n",channel);
    }
    return TRUE;
}

int ftl0_send_nak(char *from_callsign, int channel, int err) {
    int frame_type = UL_NAK_RESP;
    uint8_t err_info[1];
    err_info[0] = err;

    int rc = ftl0_make_packet(send_event_buffer.packet.data, err_info, sizeof(err_info), frame_type);
    if (rc != TRUE) {
        debug_print("Could not make FTL0 NAK packet\n");
        return FALSE;
    }
    send_event_buffer.packet.data_len = sizeof(err_info)+2;

    rc = ftl0_send_event(&ax25_event, &send_event_buffer);
    if (rc != TRUE) {
        debug_print("Could not send FTL0 NAK packet to TNC \n");
        return FALSE;
    }else {
        trace_ftl0("FTL0:[%d]: Sending FTL0 NAK %d\n",channel, err);
    }
    return TRUE;
}


/**
 * ftl0_process_upload_cmd()
 *
 * The upload command packets data is parsed
 *
Packet: UPLOAD_CMD
Information: 8 bytes
struct {
     unsigned long continue_file_no;
     unsigned long file_length;
}

<continue_file_no> - a 32-bit unsigned integer identifying the file to contin-
ue.   Used to continue a previously-aborted upload.  Must be 0 when commencing
a new upload.

<file_length> -  32-bit unsigned integer indicating the number of bytes in the
file.

 */
int ftl0_process_upload_cmd(ftl0_state_machine_t *state, uint8_t *data, int len) {

    int ftl0_length = ftl0_parse_packet_length(data);
    if (ftl0_length != 8)
        return ER_ILL_FORMED_CMD;

    FTL0_UPLOAD_CMD *upload_cmd = (FTL0_UPLOAD_CMD *)(data + 2); /* Point to the data just past the header */

    /* Parse the file number and length from the little endian packet */
    state->file_id = ttohl(upload_cmd->continue_file_no);
    state->length = ttohl(upload_cmd->file_length);

    if (state->length == 0)
        return ER_ILL_FORMED_CMD;

    /* This is the data we are going to send */
    FTL0_UL_GO_DATA ul_go_data;

    /* Check if data is valid */
    if (state->file_id == 0) {
        /* Do we have space */
        REDSTATFS redstatfs;
        int32_t red = red_statvfs("/", &redstatfs);
        if (red != 0) {
            trace_ftl0("Unable to check disk space with statvfs: %s\n", red_strerror(red_errno));
            /* Can't check if we have space, assume an error */
            return ER_NO_ROOM;
        } else {
            uint32_t available = redstatfs.f_frsize * redstatfs.f_bfree;
            uint32_t allocated = 0;
            /* TODO - need to check all the partially uploaded files to see what remaining space they have claimed. Or
             when we first start to upload, seek to the end and size a full empty file.  But that could be very wasteful. It
             would need an expiry date and cleanup routine.
             A better method could be to check only the other files uploading right now and then check continues to
             make sure there is still space available.  Reject and delete continues if there is no space.  But that could be cruel
             for long images that were 90% uploaded, then space runs out.
             */
            trace_ftl0("File length: %d.  Disk has Free blocks: %d of %d.  Free Bytes: %d\n",state->length, redstatfs.f_bfree, redstatfs.f_blocks, available);
            if ((state->length + UPLOAD_SPACE_THRESHOLD) > available )
                return ER_NO_ROOM;
        }

        /* We have space so allocate a file number, store in uplink list and send to the station */
//        ul_go_data.server_file_no = dir_next_file_number();
        state->file_id = dir_next_file_number();
        ul_go_data.server_file_no = htotl(state->file_id);
        debug_print("Allocated file id: %04x\n",state->file_id);
        // New file so start uploading from offset 0
        ul_go_data.byte_offset = 0;
        state->offset = 0;

        /* Initialize the empty file */
        char file_name_with_path[MAX_FILENAME_WITH_PATH_LEN];
        dir_get_tmp_file_path_from_file_id(state->file_id, file_name_with_path, MAX_FILENAME_WITH_PATH_LEN);
        int32_t fp = red_open(file_name_with_path, RED_O_CREAT | RED_O_WRONLY);
        if (fp == -1) {
            debug_print("Unable to open %s for writing: %s\n", file_name_with_path, red_strerror(red_errno));
            return ER_NO_ROOM;  // TODO - is this the best error to send?  File system is unavailable
        }
        int32_t cc = red_close(fp);
        if (cc == -1) {
            printf("Unable to close %s: %s\n", file_name_with_path, red_strerror(red_errno));
        }

    } else { // File number was supplied in the Upload command
        /* Is this a valid continue? Check to see if there is a tmp file
          */
         // TODO - we also need to check the situation where we have the complete file but the ground station never received the ACK.
         //        So an atttempt to upload a finished file that belongs to this station, that has the right length, should get an ACK to finish upload off
         char file_name_with_path[MAX_FILENAME_WITH_PATH_LEN];
         dir_get_tmp_file_path_from_file_id(state->file_id, file_name_with_path, MAX_FILENAME_WITH_PATH_LEN);
         trace_ftl0("FTL0[%d]: Checking continue file: %s\n",state->channel, file_name_with_path);

         // TODO - we check that the file exists, but for now we do not check that it belongs to this station.  That allows two
         // stations to cooperatively upload the same file, but it also allows a station to corrupt someone elses file
         int32_t fp = red_open(file_name_with_path, RED_O_RDONLY);
         if (fp == -1) {
             debug_print("No such file number \n");
             return ER_NO_SUCH_FILE_NUMBER;
         }
         int rc;
         int32_t off = red_lseek(fp, 0, RED_SEEK_END);
         if (off == -1) {
             debug_print("Unable to seek %s  to end: %s\n", file_name_with_path, red_strerror(red_errno));

             rc = red_close(fp);
             if (rc != 0) {
                 debug_print("Unable to close %s: %s\n", file_name_with_path, red_strerror(red_errno));
             }
             return ER_NO_SUCH_FILE_NUMBER;
         } else {
             state->offset = off;
             trace_ftl0("FTL0[%d]: Continuing file %04x at offset %d\n",state->channel, state->file_id, state->offset);
         }
         rc = red_close(fp);
         if (rc != 0) {
             printf("Unable to close %s: %s\n", file_name_with_path, red_strerror(red_errno));
         }

         // TODO - we need to check file length with the length previously supplied
         /* if <continue_file_no> is not 0 and the <file_length> does not
             agree with the <file_length> previously associated with the file identified by
             <continue_file_no>.  Continue is not possible.*/
         // code this error check

         // TODO - do we recheck that space is still available here?  See the note above on space available

         ul_go_data.server_file_no = htotl(state->file_id);
         ul_go_data.byte_offset = htotl(state->offset); // this is the end of the file so far
     }

     int rc = ftl0_make_packet(send_event_buffer.packet.data, (uint8_t *)&ul_go_data, sizeof(ul_go_data), UL_GO_RESP);
         if (rc != TRUE) {
             debug_print("Could not make FTL0 UL GO packet \n");
             return ER_ILL_FORMED_CMD; // TODO This will cause err 1 to be sent and the station to be offloaded.  Is that right..
         }

         send_event_buffer.packet.data_len = sizeof(ul_go_data)+2;

         rc = ftl0_send_event(&ax25_event, &send_event_buffer);
         if (rc != TRUE) {
             debug_print("Could not send FTL0 UL GO packet to TNC \n");
             return ER_ILL_FORMED_CMD; // TODO This will cause err 1 to be sent and the station to be offloaded.  Is that right..
         } else {
             trace_ftl0("FTL0:[%d]: Sending FTL0 UL_GO PKT\n",state->channel);
         }
     return ER_NONE;
}

/**
 * ftl0_process_data_cmd()
 *
 * Parse and process a data command from the ground station.
 *
 * TODO
 * Reception of a data command also means that the filenumber should be saved. This is what "reserves" the
 * new file number.
 *
 */
int ftl0_process_data_cmd(ftl0_state_machine_t *state, uint8_t *data, int len) {
    int ftl0_type = ftl0_parse_packet_type(data);
    if (ftl0_type != DATA) {
        /* We should never get this */
        debug_print("ERROR: FTL0 Program logic issue.  Non data packet received in DATA function");
        return ER_ILL_FORMED_CMD;
    }
    int ftl0_length = ftl0_parse_packet_length(data);
    if (ftl0_length == 0 || ftl0_length > len-2) {
        return ER_BAD_HEADER; /* This will cause a NAK to be sent as the data is corrupt in some way */
    }

    unsigned char * data_bytes = (unsigned char *)data + 2; /* Point to the data just past the header */

    char file_name_with_path[MAX_FILENAME_WITH_PATH_LEN];
    dir_get_tmp_file_path_from_file_id(state->file_id, file_name_with_path, MAX_FILENAME_WITH_PATH_LEN);

    int32_t rc = dir_fs_write_file_chunk(file_name_with_path, data_bytes, ftl0_length, state->offset);
    if (rc == -1) {
        debug_print("FTL0[%d]:File I/O error writing chunk\n",state->channel);
        return ER_NO_ROOM; // This is most likely caused by running out of file ids or space
    }

    state->offset += ftl0_length;

    return ER_NONE;
}


int ftl0_process_data_end_cmd(ftl0_state_machine_t *state, uint8_t *data, int len) {
    static HEADER ftl0_pfh_buffer; // Static allocation of a header to use when we need to load/save the header details
    static uint8_t ftl0_pfh_byte_buffer[MAX_BYTES_IN_PACSAT_FILE_HEADER]; /* Buffer for the bytes in a PFH when we decode a received file */
    int ftl0_type = ftl0_parse_packet_type(data);
    if (ftl0_type != DATA_END) {
        /* We should never get this */
        debug_print("ERROR: FTL0 Program logic issue.  Non data end packet received in DATA_END function");
        return ER_ILL_FORMED_CMD;
    }
    int ftl0_length = ftl0_parse_packet_length(data);
    if (ftl0_length != 0) {
        return ER_BAD_HEADER; /* This will cause a NAK to be sent as the data is corrupt in some way */
    }

    char file_name_with_path[MAX_FILENAME_WITH_PATH_LEN];
    dir_get_tmp_file_path_from_file_id(state->file_id, file_name_with_path, MAX_FILENAME_WITH_PATH_LEN);

    /* We can't call dir_load_pacsat_file() here because we want to check the tmp file first, then
     * add the file after we rename it. So we validate it. */

    // Read enough of the file to parse the PFH
    int32_t rc = dir_fs_read_file_chunk(file_name_with_path, ftl0_pfh_byte_buffer, sizeof(ftl0_pfh_byte_buffer), 0);
    if (rc == -1) {
        debug_print("Error reading file: %s\n",file_name_with_path);
        return FALSE;
    }
    uint16_t size;
    bool crc_passed = FALSE;
    pfh_extract_header(&ftl0_pfh_buffer, ftl0_pfh_byte_buffer, sizeof(ftl0_pfh_byte_buffer), &size, &crc_passed);
    if (!crc_passed) {
        /* Header is invalid */
        trace_ftl0("FTL0[%d] ** Header check failed for file: %s\n",state->channel, file_name_with_path);
        int32_t fp = red_unlink(file_name_with_path);
        if (fp == -1) {
            printf("Unable to remove file: %s : %s\n", file_name_with_path, red_strerror(red_errno));
        }
        return ER_BAD_HEADER;
    }



//
//    int rc = dir_validate_file(pfh, tmp_filename);
//    if (rc != ER_NONE) {
//        free(pfh);
//        if (remove(tmp_filename) != 0) {
//            error_print("Could not remove the temp file: %s\n", tmp_filename);
//        }
//        return rc;
//    }
//
//    /* Otherwise this looks good.  Rename the file and add it to the directory. */
//    //TODO - note that we are renaming the file before we know that the ground station has received an ACK
//    char new_filename[MAX_FILE_PATH_LEN];
//    pfh_make_filename(uplink_list[selected_station].file_id, get_dir_folder(), new_filename, MAX_FILE_PATH_LEN);
//    if (rename(tmp_filename, new_filename) == EXIT_SUCCESS) {
////      char file_id_str[5];
////      snprintf(file_id_str, 4, "%d",uplink_list[selected_station].file_id);
////      strlcpy(pfh->fileName, file_id_str, sizeof(pfh->fileName));
////      strlcpy(pfh->fileExt, PSF_FILE_EXT, sizeof(pfh->fileExt));
//
//        DIR_NODE *p = dir_add_pfh(pfh, new_filename);
//        if (p == NULL) {
//            error_print("** Could not add %s to dir\n",new_filename);
//            free(pfh);
//            if (remove(tmp_filename) != 0) {
//                error_print("Could not remove the temp file: %s\n", new_filename);
//            }
//            return ER_NO_ROOM; /* This is a bit of a guess at the error, but it is unclear why else this would fail. */
//        }
//    } else {
//        /* This looks like an io error and we can't rename the file.  Send error to the ground */
//        free(pfh);
//        if (remove(tmp_filename) != 0) {
//            error_print("Could not remove the temp file: %s\n", tmp_filename);
//        }
//        return ER_NO_ROOM;
//    }

    return ER_NONE;
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

int ftl0_parse_packet_type(uint8_t * data) {
    int type = data[1] & 0b00011111;
    return type;
}

int ftl0_parse_packet_length(uint8_t * data) {
    int length = (data[1] >> 5) * 256 + data[0];
    return length;
}
