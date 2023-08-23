/*
 * UplinkTask.c
 *
 *  Created on: May 3, 2023
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
#include <strings.h>

#include "PbTask.h"
#include "TxTask.h"
#include "Ax25Task.h"
#include "UplinkTask.h"
#include "FreeRTOS.h"
#include "os_task.h"
#include "MET.h"
#include "inet.h"
#include "nonvol.h"
#include "nonvolManagement.h"
#include "redposix.h"

#include "RxTask.h"
#include "ax25_util.h"
#include "pacsat_dir.h"
#include "str_util.h"

/* Forward functions */
void ftl0_status_callback();
void ftl0_next_state_from_primitive(ftl0_state_machine_t *state, AX25_event_t *event);
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

bool ftl0_get_file_upload_record(uint32_t file_id, InProcessFileUpload_t * file_upload_record);
bool ftl0_set_file_upload_record(InProcessFileUpload_t * file_upload_record);
bool ftl0_update_file_upload_record(InProcessFileUpload_t * file_upload_record);
bool ftl0_remove_file_upload_record(uint32_t id);
bool ftl0_mram_get_file_upload_record(uint32_t id, InProcessFileUpload_t * file_upload_record);
bool ftl0_mram_set_file_upload_record(uint32_t id, InProcessFileUpload_t * file_upload_record);
bool ftl0_clear_upload_table();
int ftl0_get_space_reserved_by_upload_table();

/* Local variables */
static ftl0_state_machine_t ftl0_state_machine[NUM_OF_RX_CHANNELS];
static AX25_event_t ax25_event; /* Static storage for event */
static AX25_event_t send_event_buffer;
static Intertask_Message statusMsg; // Storage used to send messages to the telem and control task
static const MRAMmap_t *LocalFlash = (MRAMmap_t *) 0; /* Used to index the MRAM static storage where the File Upload Table is stored */

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
"ERROR K - Unexpected FRMR while connected",
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
//    debug_print("Initializing Uplink FTL0 Task\n");

    /* Setup a timer to send the status periodically */
    xTimerHandle uplinkStatusTimerHandle;
    volatile portBASE_TYPE timerStatus;
    int pvtUplinkStatusTimerID = 0; // timer id

    /* create a RTOS software timer - TODO period should be in MRAM and changeable from the ground using xTimerChangePeriod() */
    uplinkStatusTimerHandle = xTimerCreate( "UPLINK STATUS", SECONDS(40), TRUE, &pvtUplinkStatusTimerID, ftl0_status_callback); // auto reload timer
    /* start the timer */
    timerStatus = xTimerStart(uplinkStatusTimerHandle, 0); // Block time of zero as this can not block
    if (timerStatus != pdPASS) {
        debug_print("ERROR: Failed in init PB Status Timer\n");
// TODO =>        ReportError(RTOSfailure, FALSE, ReturnAddr, (int) PbTask); /* failed to create the RTOS timer */
        // TODO - it's possible this might fail.  Somehow we should recover from that.
    }


    while(1) {
        BaseType_t xStatus = xQueueReceive( xUplinkEventQueue, &ax25_event, CENTISECONDS(1) );  // Wait to see if data available
        if( xStatus == pdPASS ) {
            if (ax25_event.channel >= NUM_OF_RX_CHANNELS) {
                // something is seriously wrong.  Programming error.  Unlikely to occur in flight
                debug_print("ERR: AX25 channel %d is invalid\n",ax25_event.channel);
            } else {
//                trace_ftl0("Received event: %d\n",ax25_event.primitive);

                if (ax25_event.primitive == DL_ERROR_Indicate) {
                    // Most of these are just for debugging.  Typically another event is sent if an action is needed.
                    switch (ax25_event.error_num) {
                        case ERROR_F : {
                            trace_ftl0("FTL0[%d]: DATA LINK RESET from AX25\n",ax25_event.channel);
                            // We don't off load the callsign, we just reset the state machine
                            ftl0_state_machine[ax25_event.channel].ul_state = UL_CMD_OK;
                            ftl0_state_machine[ax25_event.channel].file_id = 0;
                            ftl0_state_machine[ax25_event.channel].request_time = 0;
                            ftl0_state_machine[ax25_event.channel].length = 0;
                            break;
                        }
                        default : {
                            debug_print("FTL0[%d]: ERR from AX25: %s\n",ax25_event.channel, ax25_errors_strs[ax25_event.error_num]);
                            break;
                        }
                    }
                } else {
                    ftl0_next_state_from_primitive(&ftl0_state_machine[ax25_event.channel], &ax25_event);
                }
            }
        }

    }
}

/**
 * ftl0_status_callback()
 *
 * This is called from a timer whenever the status of the Uplink should be sent.  The actual status is assembled and
 * sent to the TX by the Telemetry and Control task
 *
 */
void ftl0_status_callback() {
    statusMsg.MsgType = TacSendUplinkStatus;
    NotifyInterTaskFromISR(ToTelemetryAndControl,&statusMsg);
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
void ftl0_next_state_from_primitive(ftl0_state_machine_t *state, AX25_event_t *event) {
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
    switch (event->primitive) {

        /* We receive either a DISCONNECT Indicate or a Confirm.  We only receive the Confirm if we send DISC and actually received the UA
         * frame from the other side.  Otherwise if we time out or we receive a DM from the other end then we get a DISCONNECT Indicate
         * message */
        case DL_DISCONNECT_Indicate :
        case DL_DISCONNECT_Confirm : {
            trace_ftl0("Disconnected from Layer 2\n");
            ftl0_remove_request(event->channel);
            break;
        }
        /* We receive either a CONNECT Indicate or a CONNECT Confirm.  We only get the confirm if the Uplink initiated the request  */
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
    switch (event->primitive) {

        case DL_DISCONNECT_Indicate :
        case DL_DISCONNECT_Confirm : {
            trace_ftl0("Disconnected from Layer 2\n");
            ftl0_remove_request(event->channel);
            break;
        }
        case DL_CONNECT_Indicate :
        case DL_CONNECT_Confirm : {
            trace_ftl0("Connection from Layer 2\n");
            // Perhaps the other end missed the connection and was still trying.  Send the CMD OK message again.
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
                     */
                    int err = ftl0_process_upload_cmd(state, event->packet.data, event->packet.data_len);
                    if (err == ER_NONE) {
                        // All is good
                    } else {
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
                    //debug_print("FTL0[%d]: %s: UL_GO_RESP - File: %04x \n",state->channel, state->callsign, state->file_id);

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
    switch (event->primitive) {

        case DL_DISCONNECT_Indicate :
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
                    // Update the upload record in case nothing else is received
                    InProcessFileUpload_t file_upload_record;
                    if (ftl0_get_file_upload_record(state->file_id, &file_upload_record) ) {
                        file_upload_record.request_time = getUnixTime(); // this is updated when we receive data
                        file_upload_record.offset = state->offset;
                        if (!ftl0_update_file_upload_record(&file_upload_record) ) {
                            debug_print("Unable to update upload record in MRAM\n");
                            // do not treat this as fatal because the file can still be uploaded
                        }
                    }
                    break;
                }
                case DATA_END : {
                    trace_ftl0("FTL0[%d]: %s: UL_DATA_RX - DATA END RECEIVED\n",state->channel, state->callsign);
                    int err = ftl0_process_data_end_cmd(state, event->packet.data, event->packet.data_len);
                    if (err != ER_NONE) {
                        //debug_print(" FTL0[%d] SENDING %s NAK for file %04x\n",state->channel, state->callsign, state->file_id);
                        rc = ftl0_send_nak(event->packet.from_callsign, event->channel, err);
                    } else {
                        //debug_print(" FTL0[%d] SENDING %s ACK for file %04x\n",state->channel, state->callsign, state->file_id);
                        rc = ftl0_send_ack(event->packet.from_callsign, event->channel);
                    }
                    if (!ftl0_remove_file_upload_record(state->file_id))  {
                        debug_print(" FTL0[%d] Could not remove upload record for %s file id %04x\n",state->channel, state->callsign, state->file_id);
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
    switch (event->primitive) {

        case DL_DISCONNECT_Indicate :
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
    send_event->primitive = DL_DATA_Request;
    send_event->packet.frame_type = TYPE_I;
    strlcpy(send_event->packet.to_callsign, received_event->packet.from_callsign, MAX_CALLSIGN_LEN);
    strlcpy(send_event->packet.from_callsign, BBS_CALLSIGN, MAX_CALLSIGN_LEN);

    if (send_event->primitive == DL_DATA_Request) {
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
            trace_ftl0("FTL0[%d]: Sending Event %d\n",send_event->channel, send_event->primitive);
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
        if (ftl0_state_machine[i].ul_state != UL_UNINIT) {
            if (strcasecmp(ftl0_state_machine[i].callsign, from_callsign) == 0) {
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
    ftl0_state_machine[channel].request_time = getUnixTime(); // for timeout
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
    ftl0_state_machine[channel].callsign[0] = 0;

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
    send_event_buffer.primitive = DL_DISCONNECT_Request;

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

    /* Check if received data is valid */
    if (state->file_id == 0) {
        /* first check against the maximum allowed file size, TODO - which should be a configurable param from the ground */
        if (state->length > MAX_FILESIZE)
            return ER_NO_ROOM;
        /* Do we have space */
        REDSTATFS redstatfs;
        int32_t red = red_statvfs("/", &redstatfs);
        if (red != 0) {
            trace_ftl0("Unable to check disk space with statvfs: %s\n", red_strerror(red_errno));
            /* Can't check if we have space, assume an error */
            return ER_NO_ROOM;
        } else {
            uint32_t available = redstatfs.f_frsize * redstatfs.f_bfree;

            /* Need to check all the partially uploaded files to see what remaining space they have claimed. */
            uint32_t upload_table_space = ftl0_get_space_reserved_by_upload_table();

            trace_ftl0("File length: %d. Upload table: %d  Disk has Free blocks: %d of %d.  Free Bytes: %d\n",state->length, upload_table_space, redstatfs.f_bfree, redstatfs.f_blocks, available);
            if ((state->length + upload_table_space + UPLOAD_SPACE_THRESHOLD) > available )
                return ER_NO_ROOM;
        }

        /* We have space so allocate a file number, store in uplink list and send to the station */
        state->file_id = dir_next_file_number();
        if (state->file_id == 0) {
            debug_print("Unable to allocated new file id: %s\n", red_strerror(red_errno));
            return ER_NO_ROOM;  // TODO - is this the best error to send?  File system is unavailable it seems
        }
        ul_go_data.server_file_no = htotl(state->file_id);
        trace_ftl0("Allocated file id: %04x\n",state->file_id);
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
            debug_print("Unable to close %s: %s\n", file_name_with_path, red_strerror(red_errno));
        }

        /* Store in an upload table record.  The state will now contain all the details */
         InProcessFileUpload_t file_upload_record;
         strlcpy(file_upload_record.callsign,state->callsign, sizeof(file_upload_record.callsign));
         file_upload_record.file_id = state->file_id;
         file_upload_record.length = state->length;
         file_upload_record.request_time = state->request_time;
         file_upload_record.offset = state->offset;
         if (!ftl0_set_file_upload_record(&file_upload_record) ) {
             debug_print("Unable to create upload record in MRAM for file id %04x\n",state->file_id);
             // this is not fatal as we may still be able to upload the file, though a later continue may not work
         }
    } else {
        /* File number was supplied in the Upload command.  In this situation we send a GO response with the offset
         * that the station should use to continue the upload.  Space should still be available as it was allocated
         * before.  If there is an error then that is returned instead of GO.
         * If there is an upload record then that is not changed by a continue, even if there is an error. */

        // We first need to check the situation where we have the complete file but the ground station never received the ACK.
        // So If we get a continue request and the offset is at the end of the file and the file is on the disk, then we send
        // ER_FILE_COMPLETE.
        char file_name_with_path[MAX_FILENAME_WITH_PATH_LEN];
        dir_get_file_path_from_file_id(state->file_id, file_name_with_path, MAX_FILENAME_WITH_PATH_LEN);
        trace_ftl0("FTL0[%d]: Checking if file: %s already uploaded\n",state->channel, file_name_with_path);

        int32_t fp = red_open(file_name_with_path, RED_O_RDONLY);
        if (fp != -1) { // File is already on disk
            trace_ftl0("File is already on disk\n");
            int32_t off = red_lseek(fp, 0, RED_SEEK_END);
            int32_t rc = red_close(fp);
            if (rc != 0) {
                debug_print("Unable to close %s: %s\n", file_name_with_path, red_strerror(red_errno));
            }
            if (off == -1) {
                debug_print("Unable to seek %s  to end: %s\n", file_name_with_path, red_strerror(red_errno));
                return ER_NO_SUCH_FILE_NUMBER; // something is wrong with this file - tell ground station to ask for a new number
            } else {
                if (state->length == off) { // we have the full file
                    trace_ftl0("FTL0[%d]: We already have file %04x at final offset %d -- ER FILE COMPLETE\n",state->channel, state->file_id, state->offset);
                    return ER_FILE_COMPLETE;
                }
            }
        }

        /* Is this a valid continue? Check to see if there is a tmp file and an upload table entry
         */
        InProcessFileUpload_t upload_record;
        if (!ftl0_get_file_upload_record(state->file_id, &upload_record))  {
            debug_print("Could not read upload record for file id %04x - FAILED\n",state->file_id);
            return ER_NO_SUCH_FILE_NUMBER;
        } else {
            /* if <continue_file_no> is not 0 and the <file_length> does not
               agree with the <file_length> previously associated with the file identified by
               <continue_file_no>.  Continue is not possible.*/
            if (upload_record.length != state->length) {
                debug_print("Promised file length does not match - BAD CONTINUE\n");
                return ER_BAD_CONTINUE;
            }
            /* If this file does not belong to this callsign then reject */
            if (strcmp(upload_record.callsign, state->callsign) != 0) {
                debug_print("Callsign does not match - BAD CONTINUE\n");
                return ER_BAD_CONTINUE;
            }
        }

        dir_get_tmp_file_path_from_file_id(state->file_id, file_name_with_path, MAX_FILENAME_WITH_PATH_LEN);
        trace_ftl0("FTL0[%d]: Checking continue file: %s\n",state->channel, file_name_with_path);

        fp = red_open(file_name_with_path, RED_O_RDONLY);
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
            debug_print("Unable to close %s: %s\n", file_name_with_path, red_strerror(red_errno));
        }

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
 * TODO - It is possible that a ground station requests to upload a file length X and then attempts to upload a much
 * longer file.  This would fill our file system.  We should make sure that the loaded data does not exceed the
 * promised file length or the maximum size we allow for a file
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
        return ER_NO_SUCH_FILE_NUMBER;
    }
    uint16_t size;
    bool crc_passed = FALSE;
    pfh_extract_header(&ftl0_pfh_buffer, ftl0_pfh_byte_buffer, sizeof(ftl0_pfh_byte_buffer), &size, &crc_passed);
    if (!crc_passed) {
        /* Header is invalid */
        trace_ftl0("FTL0[%d] ** Header check failed for file: %s\n",state->channel, file_name_with_path);
        int32_t fp = red_unlink(file_name_with_path);
        if (fp == -1) {
            debug_print("Unable to remove tmp file: %s : %s\n", file_name_with_path, red_strerror(red_errno));
        }
        return ER_BAD_HEADER;
    }

    int err = dir_validate_file(&ftl0_pfh_buffer, file_name_with_path);
    if (err != ER_NONE) {
        trace_ftl0("FTL0[%d] ** File validation failed for file: %s\n",state->channel, file_name_with_path);
        int32_t fp = red_unlink(file_name_with_path);
        if (fp == -1) {
            debug_print("Unable to remove tmp file: %s : %s\n", file_name_with_path, red_strerror(red_errno));
        }
        return err;
    }

    /* Otherwise this looks good.  Rename the file by linking a new name and removing the old name. Then
     * add it to the directory. */
    /* Note that we are renaming the file before we know that the ground station has received an ACK
           That is OK as long as we handle the situation where the ground station tries to finish the upload
           and we no longer have the tmp file.  This is handled in process_upload_command() where ER_FILE_COMPLETE
           is sent.
     */
    char new_file_name_with_path[MAX_FILENAME_WITH_PATH_LEN];
    dir_get_file_path_from_file_id(state->file_id, new_file_name_with_path, MAX_FILENAME_WITH_PATH_LEN);

    rc = red_link(file_name_with_path, new_file_name_with_path);
    if (rc == -1) {
        debug_print("Unable to link new file: %s : %s\n", new_file_name_with_path, red_strerror(red_errno));
        return ER_NO_ROOM;
    }

    /* We pass just the filename without the path into the dir add function */
    char file_id_str[5];
    dir_get_filename_from_file_id(state->file_id, file_id_str, sizeof(file_id_str));
    DIR_NODE *p = dir_add_pfh(file_id_str, &ftl0_pfh_buffer);
    if (p == NULL) {
        debug_print("** Could not add %s to dir\n", new_file_name_with_path);
        /* Remove the file that we could not add and leave the tmp file.  This will get cleaned up faster.  We are sending
         * an error to the ground, so we are not accepting the file. */
        rc = red_unlink(new_file_name_with_path);
        if (rc == -1) {
            debug_print("Unable to remove file: %s : %s\n", new_file_name_with_path, red_strerror(red_errno));
        }
        return ER_NO_ROOM; /* This is a bit of a guess at the error, but it is unclear why else this would fail. */
    }

    /* Otherwise File added to the dir.  Remove the tmp file*/
    rc = red_unlink(file_name_with_path);
    if (rc == -1) {
        debug_print("Unable to remove tmp file: %s : %s\n", file_name_with_path, red_strerror(red_errno));
        // TODO this is not fatal to the upload, but there needs to be a routine to clean up expired upload files
    }

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

/**
 * Given a file_id, return the in process file upload record.
 * Returns true if the record exists or false if it does not exist
 *
 */
bool ftl0_get_file_upload_record(uint32_t file_id, InProcessFileUpload_t * file_upload_record) {
    int i;
    for (i=0; i < MAX_IN_PROCESS_FILE_UPLOADS; i++) {
        bool rc = ftl0_mram_get_file_upload_record(i, file_upload_record);
        if (rc == FALSE) return FALSE;
        if (file_id == file_upload_record->file_id)
            return TRUE;
    }
    return FALSE;
}

/**
 * Given an upload record with a file_id, store the in process file upload record.
 * If the table is full then the oldest record is removed and replaced.
 * Return true if it could be stored or false otherwise.
 */
bool ftl0_set_file_upload_record(InProcessFileUpload_t * file_upload_record) {
    int i;
    int oldest_id = -1;
    int first_empty_id = -1;
    uint32_t oldest_date = 0xFFFFFFFF;
    InProcessFileUpload_t tmp_file_upload_record;
    /* Check that we do not already have this record, note the first empty slot and the oldest slot */
    for (i=0; i < MAX_IN_PROCESS_FILE_UPLOADS; i++) {
        bool rc = ftl0_mram_get_file_upload_record(i, &tmp_file_upload_record);
        if (rc == FALSE) return FALSE;
        if (tmp_file_upload_record.file_id == file_upload_record->file_id) {
            /* This file id is already being uploaded, so this is an error */
            return FALSE;
        }
        if (first_empty_id == -1 && tmp_file_upload_record.file_id == 0) {
            /* This is an empty slot, store it here */
            first_empty_id = i;
        } else {
            /* Slot is occupied, see if it is the oldest slot in case we need to replace it */
            if (tmp_file_upload_record.request_time < oldest_date) {
                /* This time is older than the oldest date so far.  Make sure this is not
                 * live right now and then note it as the oldest */
                bool on_the_uplink_now = FALSE;
                int j;
                for (j=0; j < NUM_OF_RX_CHANNELS; j++) {
                    if (ftl0_state_machine[j].ul_state != UL_UNINIT) {
                        if (strcasecmp(ftl0_state_machine[j].callsign, tmp_file_upload_record.callsign) == 0) {
                            on_the_uplink_now = TRUE;                        }
                    }
                }
                if (!on_the_uplink_now) {
                    oldest_id = i;
                    oldest_date = tmp_file_upload_record.request_time;
                }
            }
        }
    }
    if (first_empty_id != -1) {
        bool rc2 = ftl0_mram_set_file_upload_record(first_empty_id, file_upload_record);
        //debug_print("Store in empty slot %d\n",first_empty_id);
        if (rc2 == FALSE) return FALSE;
        return TRUE;
    }

    /* We could not find a slot so overwrite the oldest. */
    if (oldest_id != -1) {
        //debug_print("Store in oldest slot %d\n",oldest_id);
        bool rc3 = ftl0_mram_set_file_upload_record(oldest_id, file_upload_record);
        //TODO - this has to purge the old tmp file as well or it needs to be cleaned up by maintenance func
        if (rc3 == FALSE) return FALSE;
        return TRUE;
    }
    return FALSE;
}

/**
 * Given an upload record with a file_id, update the file upload record.
 * Return true if it could be updated or false otherwise.
 */
bool ftl0_update_file_upload_record(InProcessFileUpload_t * file_upload_record) {
    int i;
    InProcessFileUpload_t tmp_file_upload_record;
    /* Find the record and update it */
    for (i=0; i < MAX_IN_PROCESS_FILE_UPLOADS; i++) {
        bool rc = ftl0_mram_get_file_upload_record(i, &tmp_file_upload_record);
        if (rc == FALSE) return FALSE;
        if (tmp_file_upload_record.file_id == file_upload_record->file_id) {
            /* Update the record in this slot */
            bool rc3 = ftl0_mram_set_file_upload_record(i, file_upload_record);
            if (rc3 == FALSE) return FALSE;
            return TRUE;
        }
    }

    return FALSE;
}


/**
 * Read a record from the file upload table slot
 */
bool ftl0_mram_get_file_upload_record(uint32_t slot, InProcessFileUpload_t * file_upload_record) {
    bool rc = readNV(file_upload_record, sizeof(InProcessFileUpload_t),NVConfigData, (int)&(LocalFlash->FileUploadsTable[slot]));
    if (!rc) {
        debug_print("MRAM File Upload table read - FAILED\n");
        return FALSE;
    }
    return TRUE;
}

/**
 * Write a record to the file upload table
 */
bool ftl0_mram_set_file_upload_record(uint32_t id, InProcessFileUpload_t * file_upload_record) {
    bool rc = writeNV(file_upload_record, sizeof(InProcessFileUpload_t),NVConfigData, (int)&(LocalFlash->FileUploadsTable[id]));
    if (!rc) {
        debug_print("MRAM File Upload table write - FAILED\n");
        return FALSE;
    }
    return TRUE;
}

/**
 * ftl0_remove_file_upload_record()
 * Remove the record from the upload table based on the file id
 * Return TRUE if it was removed or did not exist.  Otherwise return FALSE;
 */
bool ftl0_remove_file_upload_record(uint32_t id) {
    InProcessFileUpload_t tmp_file_upload_record;
    InProcessFileUpload_t tmp_file_upload_record2;
    tmp_file_upload_record.file_id = 0;
    tmp_file_upload_record.length = 0;
    tmp_file_upload_record.request_time = 0;
    tmp_file_upload_record.callsign[0] = 0;
    tmp_file_upload_record.offset = 0;

    int i;
    for (i=0; i < MAX_IN_PROCESS_FILE_UPLOADS; i++) {
        bool rc = ftl0_mram_get_file_upload_record(i, &tmp_file_upload_record2);
        if (rc == FALSE) return FALSE;
        if (tmp_file_upload_record2.file_id == id) {
            if (!ftl0_mram_set_file_upload_record(i, &tmp_file_upload_record)) {
                return FALSE;
            }
            return TRUE;
        }
    }
    return TRUE;
}

/**
 * Calculate and return the total space consumed by the upload table.  This indicates
 * how much data we are expecting to receive from uploaded files.  If we want to guarantee
 * they can be uploaded them we need to keep this amount of space free.
 *
 */
int ftl0_get_space_reserved_by_upload_table() {
    int i;
    uint32_t space_reserved = 0;
    InProcessFileUpload_t rec;

    for (i=0; i < MAX_IN_PROCESS_FILE_UPLOADS; i++) {
        if (!ftl0_mram_get_file_upload_record(i, &rec)) {
            return FALSE;
        }
        if (rec.file_id != 0) {
            space_reserved += (rec.length - rec.offset); /* We exclude the offset because that will be included in the space consumed on the disk */
        }
    }
    return space_reserved;
}

bool ftl0_clear_upload_table() {
    int i;
    InProcessFileUpload_t tmp_file_upload_record;
    tmp_file_upload_record.file_id = 0;
    tmp_file_upload_record.length = 0;
    tmp_file_upload_record.request_time = 0;
    tmp_file_upload_record.callsign[0] = 0;
    tmp_file_upload_record.offset = 0;

    for (i=0; i < MAX_IN_PROCESS_FILE_UPLOADS; i++) {
        if (!ftl0_mram_set_file_upload_record(i, &tmp_file_upload_record)) {
            return FALSE;
        }
    }
    return TRUE;
}

/**
 * ftl0_maintenance()
 * Remove expired entries from the file upload table and delete their tmp file on disk
 * Remove any orpaned tmp files on disk
 *
 */
void ftl0_maintenance() {
    //debug_print("Running FTL0 Maintenance\n");

    // First remove any expired entries in the table
    int i;
    InProcessFileUpload_t rec;
    InProcessFileUpload_t blank_file_upload_record;
    blank_file_upload_record.file_id = 0;
    blank_file_upload_record.length = 0;
    blank_file_upload_record.request_time = 0;
    blank_file_upload_record.callsign[0] = 0;
    blank_file_upload_record.offset = 0;

    for (i=0; i < MAX_IN_PROCESS_FILE_UPLOADS; i++) {
        if (!ftl0_mram_get_file_upload_record(i, &rec)) {
            // skip and keep going in case this is temporary;
        }
        // TODO - do not remove file the is currently in the upload FTL0 state machine
        if (rec.file_id != 0) {
            uint32_t now = getUnixTime();
            int32_t age = now-rec.request_time;
            if (age < 0) {
                // this looks wrong, something is corrupt.  Skip it
            } else if (age > FTL0_MAX_UPLOAD_RECORD_AGE) {
                debug_print("REMOVING RECORD: %d- File: %04x by %s length: %d offset: %d for %d seconds\n",i, rec.file_id, rec.callsign, rec.length, rec.offset, now-rec.request_time);
                if (ftl0_mram_set_file_upload_record(i, &blank_file_upload_record)) {
                    // Remove the tmp file
                    char file_name_with_path[MAX_FILENAME_WITH_PATH_LEN];
                    dir_get_tmp_file_path_from_file_id(rec.file_id, file_name_with_path, MAX_FILENAME_WITH_PATH_LEN);
                    int32_t fp = red_unlink(file_name_with_path);
                    if (fp == -1) {
                        debug_print("Unable to remove tmp file: %s : %s\n", file_name_with_path, red_strerror(red_errno));
                    }
                } else {
                    debug_print(" FTL0 Maintenance - Could not remove upload record %d\n",i);
                }
            }
        }
        vTaskDelay(CENTISECONDS(10)); // yield some time so that other things can do work
    }

    // Next remove any orphaned tmp files
    REDDIR *pDir;
    char * path = TMP_FOLDER;
    //printf("Checking TMP Directory from %s:\n",path);
    pDir = red_opendir(path);
    if (pDir == NULL) {
        debug_print("Unable to open tmp folder: %s\n", red_strerror(red_errno));
        return;
    }

    REDDIRENT *pDirEnt;
    red_errno = 0; /* Set error to zero so we can distinguish between a real error and the end of the DIR */
    pDirEnt = red_readdir(pDir);
    while (pDirEnt != NULL) {
        if (!RED_S_ISDIR(pDirEnt->d_stat.st_mode)) {
            //debug_print("Checking: %s\n",pDirEnt->d_name);
            uint32_t id = dir_get_file_id_from_filename(pDirEnt->d_name);
            if (id == 0) {
                debug_print("Could not get file id for file %s\n",pDirEnt->d_name);
            }
            // If this is not in the upload table then remove the file
            if(!ftl0_get_file_upload_record(id, &rec)) {
                // Remove the tmp file
                char file_name_with_path[MAX_FILENAME_WITH_PATH_LEN];
                dir_get_tmp_file_path_from_file_id(id, file_name_with_path, MAX_FILENAME_WITH_PATH_LEN);
                int32_t fp = red_unlink(file_name_with_path);
                if (fp == -1) {
                    debug_print("Unable to remove orphaned tmp file: %s : %s\n", file_name_with_path, red_strerror(red_errno));
                }
            }
        }
        vTaskDelay(CENTISECONDS(10)); // yield some time so that other things can do work
        pDirEnt = red_readdir(pDir);
    }
    if (red_errno != 0) {
        debug_print("*** Error reading tmp directory: %s\n", red_strerror(red_errno));
    }
    int32_t rc2 = red_closedir(pDir);
    if (rc2 != 0) {
        debug_print("*** Unable to close tmp dir: %s\n", red_strerror(red_errno));
    }

}

#ifdef DEBUG

/**
 * TEST ROUTINES
 */

bool ftl0_debug_list_upload_table() {
    int i;
    InProcessFileUpload_t rec;

    for (i=0; i < MAX_IN_PROCESS_FILE_UPLOADS; i++) {
        if (!ftl0_mram_get_file_upload_record(i, &rec)) {
            return FALSE;
        }
        if (rec.file_id != 0) {
            uint32_t now = getUnixTime();
            debug_print("%d- File: %04x by %s length: %d offset: %d for %d seconds\n",i, rec.file_id, rec.callsign, rec.length, rec.offset, now-rec.request_time);
        }
    }
    uint32_t space = ftl0_get_space_reserved_by_upload_table();
    debug_print("Total Space Allocated: %d\n",space);
    return TRUE;
}

int test_ftl0_upload_table() {
    printf("##### TEST UPLOAD TABLE:\n");
    int rc = TRUE;
    if (!ftl0_clear_upload_table()) { debug_print("Could not clear upload table - FAILED\n"); return FALSE;}

    /* Test core set/get functions */
    InProcessFileUpload_t file_upload_record;
    strlcpy(file_upload_record.callsign,"G0KLA", sizeof(file_upload_record.callsign));
    file_upload_record.file_id = 99;
    file_upload_record.length = 12345;
    file_upload_record.request_time = 1692394562;
    file_upload_record.offset = 0;

    /* Store in the middle of the table.  In a later test this will be the oldest. */
    if (!ftl0_mram_set_file_upload_record(15, &file_upload_record)) {  debug_print("Could not add record - FAILED\n"); return FALSE; }

    InProcessFileUpload_t record;
    if (!ftl0_mram_get_file_upload_record(15, &record))  {  debug_print("Could not read record - FAILED\n"); return FALSE; }

    if (record.file_id != file_upload_record.file_id)  {  debug_print("Wrong file id - FAILED\n"); return FALSE; }
    if (record.length != file_upload_record.length)  {  debug_print("Wrong length - FAILED\n"); return FALSE; }
    if (record.request_time != file_upload_record.request_time)  {  debug_print("Wrong request_time - FAILED\n"); return FALSE; }
    if (strcmp(record.callsign, file_upload_record.callsign) != 0)  {  debug_print("Wrong callsign - FAILED\n"); return FALSE; }

    uint32_t space = ftl0_get_space_reserved_by_upload_table();
    if (space != file_upload_record.length)  {  debug_print("Wrong table space - FAILED\n"); return FALSE; }

    /* Now test adding by file id */
    InProcessFileUpload_t file_upload_record2;
    strlcpy(file_upload_record2.callsign,"AC2CZ", sizeof(file_upload_record.callsign));
    file_upload_record2.file_id = 1010;
    file_upload_record2.length = 659;
    file_upload_record2.request_time = 1692394562+1;
    file_upload_record2.offset = 0;

    if (!ftl0_set_file_upload_record(&file_upload_record2) ) {  debug_print("Could not add record2 - FAILED\n"); return FALSE; }

    InProcessFileUpload_t record2;
    if (!ftl0_get_file_upload_record(1010, &record2))  {  debug_print("Could not read record2 - FAILED\n"); return FALSE; }

    if (record2.file_id != file_upload_record2.file_id)  {  debug_print("Wrong file id for record 2 - FAILED\n"); return FALSE; }
    if (record2.length != file_upload_record2.length)  {  debug_print("Wrong length for record 2 - FAILED\n"); return FALSE; }
    if (record2.offset != file_upload_record2.offset)  {  debug_print("Wrong offset for record 2 - FAILED\n"); return FALSE; }
    if (record2.request_time != file_upload_record2.request_time)  {  debug_print("Wrong request_time for record 2 - FAILED\n"); return FALSE; }
    if (strcmp(record2.callsign, file_upload_record2.callsign) != 0)  {  debug_print("Wrong callsign for record 2 - FAILED\n"); return FALSE; }

    /* Test update */
    record2.offset = 98;
    if (!ftl0_update_file_upload_record(&record2) ) {  debug_print("Error - could not update record2 - FAILED\n"); return FALSE; }

    InProcessFileUpload_t record_up;
    if (!ftl0_get_file_upload_record(1010, &record_up))  {  debug_print("Could not read record_up - FAILED\n"); return FALSE; }

    if (record_up.file_id != file_upload_record2.file_id)  {  debug_print("Wrong file id for record_up - FAILED\n"); return FALSE; }
    if (record_up.length != file_upload_record2.length)  {  debug_print("Wrong length for record_up - FAILED\n"); return FALSE; }
    if (record_up.offset != 98)  {  debug_print("Wrong offset for record_up - FAILED\n"); return FALSE; }
    if (record_up.request_time != file_upload_record2.request_time)  {  debug_print("Wrong request_time for record_up - FAILED\n"); return FALSE; }
    if (strcmp(record_up.callsign, file_upload_record2.callsign) != 0)  {  debug_print("Wrong callsign for record_up - FAILED\n"); return FALSE; }

    /* Test add duplicate file id - error */
    InProcessFileUpload_t file_upload_record3;
    strlcpy(file_upload_record3.callsign,"VE2TCP", sizeof(file_upload_record.callsign));
    file_upload_record3.file_id = 1010;
    file_upload_record3.length = 6539;
    file_upload_record3.request_time = 1692394562+2;
    file_upload_record3.offset = 0;

    if (ftl0_set_file_upload_record(&file_upload_record3) ) {  debug_print("Error - added duplicate file id for record3 - FAILED\n"); return FALSE; }

    /* Now test that we replace the oldest if all slots are full.  Currently we have added two records. */
    InProcessFileUpload_t tmp_file_upload_record;
    strlcpy(tmp_file_upload_record.callsign,"D0MMY", sizeof(file_upload_record.callsign));
    tmp_file_upload_record.length = 123;
    int j;
    for (j=0; j < MAX_IN_PROCESS_FILE_UPLOADS; j++) {
        tmp_file_upload_record.file_id = 100 + j;
        tmp_file_upload_record.request_time = 1692394562 + 3 + j;
        tmp_file_upload_record.offset = 0;
        if (!ftl0_set_file_upload_record(&tmp_file_upload_record)) {
            return FALSE;
        }
    }

    /* The last two records added above should have replaced older records, with the second to last being in slot 15 */
    InProcessFileUpload_t record4;
    if (!ftl0_mram_get_file_upload_record(15, &record4))  {  debug_print("Could not read oldest record - FAILED\n"); return FALSE; }

    if (record4.file_id != 100 + MAX_IN_PROCESS_FILE_UPLOADS-2)  {  debug_print("Wrong oldest file id - FAILED\n"); return FALSE; }
    if (record4.length != 123)  {  debug_print("Wrong length - FAILED\n"); return FALSE; }
    if (record4.request_time != 1692394562 + 3 + MAX_IN_PROCESS_FILE_UPLOADS-2)  {  debug_print("Wrong oldest request_time - FAILED\n"); return FALSE; }
    if (strcmp(record4.callsign, "D0MMY") != 0)  {  debug_print("Wrong oldest callsign - FAILED\n"); return FALSE; }

    /* Now clear an upload record as though it completes or is purged */
    if (!ftl0_remove_file_upload_record(105))  {  debug_print("Could not remove record for id 105 - FAILED\n"); return FALSE; }

    InProcessFileUpload_t record5;
    if (ftl0_get_file_upload_record(105, &record5))  {  debug_print("ERROR: Should not be able to read record5 - FAILED\n"); return FALSE; }

    /* Now add a new record and it should go exactly in that empty slot.  Record 105 was the 6th record added above
     * and onlyt slot 0 and 15 were full.  So it should be in slot 6  */
    InProcessFileUpload_t file_upload_record6;
    strlcpy(file_upload_record6.callsign,"VE2TCP", sizeof(file_upload_record.callsign));
    file_upload_record6.file_id = 9990;
    file_upload_record6.length = 123999;
    file_upload_record6.request_time = 999;
    file_upload_record6.offset = 122999;

    if (!ftl0_set_file_upload_record(&file_upload_record6) ) {  debug_print("Error - could not add record6 - FAILED\n"); return FALSE; }

    InProcessFileUpload_t record7;
    if (!ftl0_mram_get_file_upload_record(6, &record7))  {  debug_print("ERROR: Could not read slot 6 - FAILED\n"); return FALSE; }
    if (record7.file_id != 9990)  {  debug_print("Wrong file id in slot 6- FAILED\n"); return FALSE; }

    if (!ftl0_debug_list_upload_table()) { debug_print("Could not print upload table - FAILED\n"); return FALSE; }

    int reserved = 123 * 24 + 1000;
    if (ftl0_get_space_reserved_by_upload_table() != reserved) { debug_print("Wrong space reserved: %d  - FAILED\n",reserved); return FALSE; }

    /* And reset everything */
    if (!ftl0_clear_upload_table()) { debug_print("Could not clear upload table - FAILED\n"); return FALSE;}


    if (rc == TRUE)
        printf("##### TEST UPLOAD TABLE: success:\n");
    else
        printf("##### TEST UPLOAD TABLE: fail:\n");


    return rc;

}

#endif
