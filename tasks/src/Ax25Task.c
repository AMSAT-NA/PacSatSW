/*
 * AxTask.c
 *
 *  Created on: 19 Apr, 2023
 *      Author: Chris E. Thompson, G0KLA / VE2TCP
 */

#include <strings.h>

#include "PbTask.h"
#include "TxTask.h"
#include "Ax25Task.h"
#include "FreeRTOS.h"
#include "os_task.h"

#include "RxTask.h"
#include "ax25_util.h"
#include "str_util.h"

#define TRACE_AX25_DL
#ifdef TRACE_AX25_DL
#define trace_dl printf
#else
#define trace_dl //
#endif

/* Forward functions */
void ax25_process_frame(char *from_callsign, char *to_callsign, rx_channel_t channel);
void ax25_t1_expired(TimerHandle_t xTimer);
void ax25_t3_expired(TimerHandle_t xTimer);
void start_timer(TimerHandle_t timer);
void restart_timer(TimerHandle_t timer);
void stop_timer(TimerHandle_t timer);
void ax25_send_dm(rx_channel_t channel, AX25_PACKET *packet, AX25_PACKET *response_packet);
void ax25_send_ua(rx_channel_t channel, AX25_PACKET *packet, AX25_PACKET *response_packet);
void ax25_send_response(rx_channel_t channel, ax25_frame_type_t frame_type, char *to_callsign, AX25_PACKET *response_packet);
bool ax25_send_event(AX25_data_link_state_machine_t *state, AX25_primative_t prim, AX25_PACKET *packet, ax25_error_t error_num);

/* State Machine functions */
void ax25_next_state_from_primative(AX25_data_link_state_machine_t *dl_state_info, AX25_event_t *event);
void ax25_next_state_from_packet(AX25_data_link_state_machine_t *dl_state_info, AX25_PACKET *decoded_packet);
void ax25_state_disc_prim(AX25_data_link_state_machine_t *dl_state_info, AX25_event_t *event);
void ax25_state_disc_packet(AX25_data_link_state_machine_t *dl_state_info, AX25_PACKET *packet);
void ax25_state_wait_conn_prim(AX25_data_link_state_machine_t *dl_state_info, AX25_event_t *event);
void ax25_state_wait_conn_packet(AX25_data_link_state_machine_t *dl_state_info, AX25_PACKET *packet);
void ax25_state_wait_release_prim(AX25_data_link_state_machine_t *dl_state_info, AX25_event_t *event);
void ax25_state_wait_release_packet(AX25_data_link_state_machine_t *dl_state_info, AX25_PACKET *packet);
void ax25_state_connected_prim(AX25_data_link_state_machine_t *dl_state_info, AX25_event_t *event);
void ax25_state_connected_packet(AX25_data_link_state_machine_t *dl_state_info, AX25_PACKET *packet);
void ax25_state_timer_rec_prim(AX25_data_link_state_machine_t *dl_state_info, AX25_event_t *event);
void ax25_state_timer_rec_packet(AX25_data_link_state_machine_t *dl_state_info, AX25_PACKET *packet);

/* State Machine Helper functions */
void clear_exception_conditions(AX25_data_link_state_machine_t *state);
void discard_iframe_queue(AX25_data_link_state_machine_t *state);
void transmit_enquiry(AX25_data_link_state_machine_t *state, AX25_PACKET *packet);

bool uplink_closed = FALSE; // Global variable

/* Local variables */
static xTimerHandle timerT1[NUM_OF_RX_CHANNELS];
static xTimerHandle timerT3[NUM_OF_RX_CHANNELS];

static rx_radio_buffer_t ax25_radio_buffer; /* Static storage for packets from the radio */
static AX25_data_link_state_machine_t data_link_state_machine[NUM_OF_RX_CHANNELS];
static const AX25_PACKET EMPTY_PACKET; // Use this to reset packet to zero
static AX25_event_t ax25_received_event; /* Static storage for received event */
static AX25_event_t send_event_buffer; /* Static storage for event we are sending */
static AX25_event_t timer_event; /* Static storage for timer events */

// TODO These should be globals so that the Uplink can access them as well for debug messages
static char *rx_channel_names[] = {"A","B","C","D"};
char *ax25_errors_strs[] = {
"F=1 received but P=1 not outstanding.",
"Unexpected DM with F=1 in states 3, 4 or 5.",
"Unexpected UA in states 3, 4 or 5.",
"UA received without F=1 when SABM or DISC was sent P=1.",
"DM received in states 3, 4 or 5.",
"Data link reset, i.e., SABM received in state 3, 4 or 5.",
"DISC retries exceeded.",
"ERROR H not defined",
"N2 timeouts: unacknowledged data.",
"N(r) sequence ERROR_.",
"ERROR K not defined",
"Control field invalid or not implemented.",
"Information field was received in a U or S-type frame.",
"Length of frame incorrect for frame type.",
"I frame exceeded maximum allowed length.",
"N(s) out of the window.",
"UI response received, or UI command with P=1 received.",
"UI frame exceeded maximum allowed length.",
"I response received.",
"N2 timeouts: no response to enquiry.",
"N2 timeouts: extended peer busy condition.",
"No DL machines available to establish connection."
};


portTASK_FUNCTION_PROTO(Ax25Task, pvParameters)  {

    vTaskSetApplicationTaskTag((xTaskHandle) 0, (pdTASK_HOOK_CODE)Ax25TaskWD );
    ResetAllWatchdogs();
    printf("Initializing Ax25 Data Link Task\n");

    /* create a RTOS software timers for T1 and T3.  One per channel.*/
    timerT1[Channel_A] = xTimerCreate( "T1_0", AX25_TIMER_T1_PERIOD, FALSE, (void *)Channel_A, ax25_t1_expired); // single shot timer
//    timerT1[1] = xTimerCreate( "T1_1", AX25_TIMER_T1_PERIOD, FALSE, (void *)Channel_B, ax25_t1_expired); // single shot timer
//    timerT1[2] = xTimerCreate( "T1_2", AX25_TIMER_T1_PERIOD, FALSE, (void *)Channel_C, ax25_t1_expired); // single shot timer
//    timerT1[3] = xTimerCreate( "T1_3", AX25_TIMER_T1_PERIOD, FALSE, (void *)Channel_D, ax25_t1_expired); // single shot timer

    timerT3[Channel_A] = xTimerCreate( "T3_0", AX25_TIMER_T3_PERIOD, FALSE, (void *)Channel_A, ax25_t3_expired); // single shot timer

    // test
//    start_timer(timerT1[Channel_A]);
//    start_timer(timerT3[Channel_A]);

    while(1) {

        BaseType_t xStatus = xQueueReceive( xRxPacketQueue, &ax25_radio_buffer, CENTISECONDS(1) );  // Wait to see if data available
        if( xStatus == pdPASS ) {
            /* Data was successfully received from the queue */
            char from_callsign[MAX_CALLSIGN_LEN];
            char to_callsign[MAX_CALLSIGN_LEN];

            decode_call(&ax25_radio_buffer.bytes[7], from_callsign);
            decode_call(&ax25_radio_buffer.bytes[0], to_callsign);

//            print_packet(AX25_data_link_state_machine_t *dl_state_info, "AX25", ax25_packet_buffer+1, ax25_packet_buffer[0]);
            ax25_process_frame(from_callsign, to_callsign, ax25_radio_buffer.channel);

        }
        ReportToWatchdog(CurrentTaskWD);
        /* Yield some time so any packets are sent, or so that others may do some processing */
        vTaskDelay(CENTISECONDS(1));
        ReportToWatchdog(CurrentTaskWD);
        xStatus = xQueueReceive( xRxEventQueue, &ax25_received_event, CENTISECONDS(1) );  // Wait to see if data available
        if( xStatus == pdPASS ) {
            if (ax25_received_event.channel >= NUM_OF_RX_CHANNELS) {
                // something is seriously wrong.  Programming error.  Unlikely to occur in flight
                debug_print("ERR: AX25 channel %d is invalid\n",ax25_received_event.channel);
            } else {
                ax25_next_state_from_primative(&data_link_state_machine[ax25_received_event.channel], &ax25_received_event);
            }
        }

    }
}

/**
 * ax25_send_status()
 *
 * This is called from an RTOS timer to send the status periodically
 * Puts a packet with the current status of the Uplink into the TxQueue
 *
 * Returns void to be compatible with timer callbacks
 *
 * NOTE that ax25_status_buffer is declared static because allocating a buffer of this
 * size causes a crash when this is called from a timer.
 *
 * We MUST NOT BLOCK because this can be called from a timer.  If the TX queue is full then we skip sending status
 *
 */
void ax25_send_status() {
    ReportToWatchdog(CurrentTaskWD);
    if (uplink_closed) {
        // for now we send nothing.  If the uplink is shut it is silent
        return;
    } else  {

        char buffer[25];
        uint8_t len = 11; // Open ABCD.\0
        int channels_available = NUM_OF_RX_CHANNELS;
        strlcpy(buffer,"Open ", sizeof(buffer));

        int i;
        for (i=0; i < NUM_OF_RX_CHANNELS; i++) {
            if (data_link_state_machine[i].dl_state == DISCONNECTED) {
                strlcat(buffer, rx_channel_names[i], sizeof(buffer));
            } else {
                channels_available--;
                strlcat(buffer, " ", sizeof(buffer));
              }
        }

        if (channels_available) {
            debug_print("SENDING OPEN: |%s|\n",buffer);
            int rc = tx_send_ui_packet(BBS_CALLSIGN, BBSTAT, PID_NO_PROTOCOL, (uint8_t *)buffer, len, DONT_BLOCK);
        } else {
            debug_print("Uplink is Full, nothing sent\n");
        }
    }
    ReportToWatchdog(CurrentTaskWD);
    return;
}


/*
 * Called when timer T1 expires.  The channel is stored in the timer id.
 *
 */
void ax25_t1_expired(TimerHandle_t xTimer) {
    uint32_t chan = (uint32_t)pvTimerGetTimerID( xTimer ); // timer id is treated as an integer and not as a pointer
    trace_dl("AX25: Channel %d. Timer T1 Expiry Int\n", (rx_channel_t)chan);
    timer_event.primative = DL_TIMER_T1_Expire;
    timer_event.channel = (rx_channel_t)chan;
    BaseType_t xStatus = xQueueSendToBack( xRxEventQueue, &timer_event, 0 ); // Do not block as this is called from timer
    if( xStatus != pdPASS ) {
        debug_print("EVENT QUEUE FULL: Could not add T1 expire to Event Queue\n");
        // TODO - we should log this error and downlink in telemetry
    }
}

void ax25_t3_expired(TimerHandle_t xTimer) {
    uint32_t chan = (uint32_t)pvTimerGetTimerID( xTimer ); // timer id is treated as an integer and not as a pointer
    trace_dl("AX25: Channel %d. Timeout... Timer T3 Expiry Int\n", (rx_channel_t)chan);
    timer_event.primative = DL_TIMER_T3_Expire;
    timer_event.channel = (rx_channel_t)chan;
    BaseType_t xStatus = xQueueSendToBack( xRxEventQueue, &timer_event, 0 ); // Do not block as this is called from timer
    if( xStatus != pdPASS ) {
        debug_print("EVENT QUEUE FULL: Could not add T3 expire to Event Queue\n");
        // TODO - we should log this error and downlink in telemetry
    }
}

void start_timer(TimerHandle_t timer) {

    // TODO - if this is running then we should reset??
    portBASE_TYPE timerT1Status = xTimerStart(timer, 0); // Block time of zero as this can not block
    if (timerT1Status != pdPASS) {
        debug_print("ERROR: Failed in init T1 Timer\n");
// TODO =>        ReportError(RTOSfailure, FALSE, ReturnAddr, (int) PbTask); /* failed to create the RTOS timer */
        // TODO - it's possible this might fail.  Somehow we should recover from that.
    }
}

void restart_timer(TimerHandle_t timer) {

    portBASE_TYPE timerT1Status = xTimerReset(timer, 0); // Block time of zero as this can not block
    if (timerT1Status != pdPASS) {
        debug_print("ERROR: Failed to restart T1 Timer\n");
    }
}

void stop_timer(TimerHandle_t timer) {

    portBASE_TYPE timerT1Status = xTimerStop(timer, 0); // Block time of zero as this can not block
    if (timerT1Status != pdPASS) {
        debug_print("ERROR: Failed to stop T1 Timer\n");
    }
}

/**
 * ax25_process_frame()
 *
 * Process a frame received from a ground station.
 * If the state machine is inactive then we process the data.
 * If the frame received is not for the callsign connected on this channel then we ignore it.
 * This is called from the main processing loop whenever a frame is received.  We decode the bytes and store the
 * received packet in the state machine structure.
 *
 */
void ax25_process_frame(char *from_callsign, char *to_callsign, rx_channel_t channel) {
    if (strcasecmp(to_callsign, BBS_CALLSIGN) == 0) {
        ax25_decode_packet(&ax25_radio_buffer.bytes[0], ax25_radio_buffer.len, &data_link_state_machine[channel].decoded_packet);

        if (data_link_state_machine[channel].dl_state == DISCONNECTED) {
            strlcpy(data_link_state_machine[channel].callsign, from_callsign, MAX_CALLSIGN_LEN);
            data_link_state_machine[channel].channel = channel;
            ax25_next_state_from_packet(&data_link_state_machine[channel], &data_link_state_machine[channel].decoded_packet);
        } else {
            // this needs to be the same callsign for this channel, otherwise we are busy
            if (strcasecmp(data_link_state_machine[channel].callsign, from_callsign) == 0) {
                ax25_next_state_from_packet(&data_link_state_machine[channel], &data_link_state_machine[channel].decoded_packet);
            } else {
                debug_print("AX25: BUSY!\n");
                // TODO - send a DM or just ignore?
//                data_link_state_machine[channel].response_packet = EMPTY_PACKET; // zero out the packet
//                ax25_send_dm(data_link_state_machine.channel, &data_link_state_machine[channel].decoded_packet, &data_link_state_machine[channel].response_packet);
            }
        }
    } else if (strcasecmp(to_callsign, BROADCAST_CALLSIGN) == 0) {
        // this was sent to the Broadcast Callsign

        /* Add to the queue and wait for 10ms to see if space is available */
        BaseType_t xStatus = xQueueSendToBack( xPbPacketQueue, &ax25_radio_buffer, CENTISECONDS(1) );
        if( xStatus != pdPASS ) {
            /* The send operation could not complete because the queue was full */
            debug_print("AX25: PB QUEUE FULL: Could not add to Packet Queue\n");
            // TODO - we should log this error and downlink in telemetry
        }
    } else {
        debug_print("AX25: Unknown destination: %s - Packet Ignored\n",to_callsign);
    }
}



/**
 * Create a DM response packet.
 * The dl_state of PF is copied from the received packet.
 */
void ax25_send_dm(rx_channel_t channel, AX25_PACKET *packet, AX25_PACKET *response_packet) {
    // F <- P
    response_packet->PF = packet->PF & 0b1;

    strlcpy(response_packet->to_callsign, packet->from_callsign, MAX_CALLSIGN_LEN);
    strlcpy(response_packet->from_callsign, BBS_CALLSIGN, MAX_CALLSIGN_LEN);

    response_packet->frame_type = TYPE_U_DM;
#ifdef TRACE_AX25_DL
    print_decoded_packet("AX25: ", response_packet);
#endif
    bool rc = tx_send_packet(channel, response_packet, NOT_EXPEDITED, BLOCK);
    if (rc == FALSE) {
        // TODO - handle error
        debug_print("ERR: Could not queue DM response\n");
    }
}

/**
 * Create a UA response packet.
 * The dl_state of PF is copied from the received packet.
 */
void ax25_send_ua(rx_channel_t channel, AX25_PACKET *packet, AX25_PACKET *response_packet) {
    // F <- P
    response_packet->PF = packet->PF & 0b1;

    strlcpy(response_packet->to_callsign, packet->from_callsign, MAX_CALLSIGN_LEN);
    strlcpy(response_packet->from_callsign, BBS_CALLSIGN, MAX_CALLSIGN_LEN);

    response_packet->frame_type = TYPE_U_UA;
#ifdef TRACE_AX25_DL
    print_decoded_packet("AX25: ", response_packet);
#endif
    bool rc = tx_send_packet(channel, response_packet, NOT_EXPEDITED, BLOCK);
    if (rc == FALSE) {
        // TODO - handle error
        debug_print("ERR: Could not queue UA response\n");
    }
}

/**
 * Create a response packet.
 * The dl_state of command and PF are set already in the response packet
 */
void ax25_send_response(rx_channel_t channel, ax25_frame_type_t frame_type, char *to_callsign, AX25_PACKET *response_packet) {
    strlcpy(response_packet->to_callsign, to_callsign, MAX_CALLSIGN_LEN);
    strlcpy(response_packet->from_callsign, BBS_CALLSIGN, MAX_CALLSIGN_LEN);

    response_packet->frame_type = frame_type;
#ifdef TRACE_AX25_DL
    print_decoded_packet("AX25: ", response_packet);
#endif
    bool rc = tx_send_packet(channel, response_packet, NOT_EXPEDITED, BLOCK);
    if (rc == FALSE) {
        // TODO - handle error
        debug_print("ERR: Could not queue RR response\n");
    }
}

/**
 * Send event to Layer 3.
 * packet can be NULL if the event does not pass a packet
 * Set error to NO_ERROR if there is no error
 *
 */
bool ax25_send_event(AX25_data_link_state_machine_t *state, AX25_primative_t prim, AX25_PACKET *packet, ax25_error_t error_num) {
    send_event_buffer.channel = state->channel;
    send_event_buffer.primative = prim;
    send_event_buffer.error_num = error_num;
    if (packet != NULL)
        ax25_copy_packet(packet, &send_event_buffer.packet);
    BaseType_t xStatus = xQueueSendToBack( xUplinkEventQueue, &send_event_buffer, CENTISECONDS(1) );
    if( xStatus != pdPASS ) {
        /* The send operation could not complete because the queue was full */
        debug_print("UPLINK QUEUE FULL: Could not add to Event Queue\n");
        // TODO - we should log this error and downlink in telemetry
        return FALSE;
    } else {
        trace_dl("AX25: Added event to uplink: %d\n",send_event_buffer.primative);
    }
    return TRUE;
}


/**
 * DATA LINK STATE MACHINE
 */

void ax25_next_state_from_packet(AX25_data_link_state_machine_t *dl_state_info, AX25_PACKET *decoded_packet) {
    switch (dl_state_info->dl_state) {
        case DISCONNECTED : {
            ax25_state_disc_packet(dl_state_info, decoded_packet);
            break;
        }
        case AWAITING_CONNECTION : {
            ax25_state_wait_conn_packet(dl_state_info, decoded_packet);
            break;
        }
        case AWAITING_RELEASE : {
            ax25_state_wait_release_packet(dl_state_info, decoded_packet);
            break;
        }
        case CONNECTED : {
            ax25_state_connected_packet(dl_state_info, decoded_packet);
            break;
        }
        case TIMER_RECOVERY : {
            ax25_state_timer_rec_packet(dl_state_info, decoded_packet);
            break;
        }
        case AWAITING_V2_2_CONNECTION : {
            // not implemented
            break;
        }
        default:
            break;
    }
}

void ax25_next_state_from_primative(AX25_data_link_state_machine_t *dl_state_info, AX25_event_t *event) {
    switch (dl_state_info->dl_state) {
        case DISCONNECTED : {
            ax25_state_disc_prim(dl_state_info, event);
            break;
        }
        case AWAITING_CONNECTION : {
            ax25_state_wait_conn_prim(dl_state_info, event);
            break;
        }
        case AWAITING_RELEASE : {
            ax25_state_wait_release_prim(dl_state_info, event);
            break;
        }
        case CONNECTED : {
            ax25_state_connected_prim(dl_state_info, event);
            break;
        }
        case TIMER_RECOVERY : {
            ax25_state_timer_rec_prim(dl_state_info, event);
            break;
        }
        case AWAITING_V2_2_CONNECTION : {
            // not implemented
            break;
        }
        default:
            break;
    }
}

/**
 * Data Link DISCONNECTED State
 */
void ax25_state_disc_prim(AX25_data_link_state_machine_t *state, AX25_event_t *event) {
    trace_dl("AX25: STATE DISC (prim): ");
    switch (event->primative) {
        case DL_DISCONNECT_Request : {
            trace_dl("Disconnect Request from Layer 3\n");
            break;
        }
        case DL_UNIT_DATA_Request : {
            trace_dl("Send UI Request from Layer 3\n");
            break;
        }
        case DL_CONNECT_Request : {
            trace_dl("Request to initiate connection from Layer 3\n");

            break;
        }


        default : {
            trace_dl(".. Ignored\n");
            break;
        }

    }
}

void ax25_state_disc_packet(AX25_data_link_state_machine_t *state, AX25_PACKET *packet) {
    trace_dl("AX25: STATE DISC (frame): ");
    switch (packet->frame_type) {
        case TYPE_U_UA : {
            trace_dl("UA\n");
            break;
        }
        case TYPE_U_UI : {
            trace_dl("UI\n");
            break;
        }
        case TYPE_U_DISC : {
            trace_dl("UA\n");
            state->response_packet = EMPTY_PACKET; // zero out the packet
            ax25_send_dm(state->channel, packet, &state->response_packet);
            break;
        }
        case TYPE_U_SABM : {
            trace_dl("SABM\n");
            state->response_packet = EMPTY_PACKET; // zero out the packet
            // Set version 2.0 - we already have the default values of MODULO 8 and no SREJ.
            ax25_send_ua(state->channel, packet, &state->response_packet);
            clear_exception_conditions(state);
            state->VS = 0;
            state->VA = 0;
            state->VR = 0;

            // Send DL_CONNECT_Indication to Layer 3
            ax25_send_event(state, DL_CONNECT_Indicate, packet, NO_ERROR);
            // TODO - what action to take if the return code is FALSE and we can not send the event.

            // NOT IMPLEMENTED - SRT and T1V are not calculated and set

            start_timer(timerT3[state->channel]);
            state->RC = 0;
            state->dl_state = CONNECTED;

            break;
        }
        case TYPE_U_SABME : {
            trace_dl("SABME - NOT SUPPORTED YET\n");
            // We dont support v2.2 yet, so send DM and remain in disconnected dl_state.
            state->response_packet = EMPTY_PACKET; // zero out the packet
            ax25_send_dm(state->channel, packet, &state->response_packet);
            break;
        }
        default:
            // All other commands get a DM
            if (packet->command) {
                state->response_packet = EMPTY_PACKET; // zero out the packet
                ax25_send_dm(state->channel, packet, &state->response_packet);
            } else {
                trace_dl("Ignoring unexpected pkt type: %0x from %s\n",packet->frame_type, packet->from_callsign);
            }
            break;
    }
}


/**
 * Data Link Wait Connection State
 */
void ax25_state_wait_conn_prim(AX25_data_link_state_machine_t *state, AX25_event_t *event) {
}

void ax25_state_wait_conn_packet(AX25_data_link_state_machine_t *state, AX25_PACKET *packet) {

}



void ax25_state_wait_release_prim(AX25_data_link_state_machine_t *state, AX25_event_t *event) {
    trace_dl("AX25: STATE WAIT RELEASE (prim): ");
    switch (event->primative) {
        case DL_DISCONNECT_Request : {


        }




        case DL_TIMER_T1_Expire : {
             trace_dl("Timer T1 Expired\n");

             if (state->RC == AX25_RETRIES_N2) {
                 trace_dl("ERROR G: %s\n",ax25_errors_strs[ERROR_G]);
                 debug_print("\n");
                 ax25_send_event(state, DL_ERROR_Indicate, NULL, ERROR_G);
                 state->dl_state = DISCONNECTED;
             } else {
                 state->RC += 1;
                 // DISC P = 1
                 state->response_packet = EMPTY_PACKET;
                 state->response_packet.PF = 1;
                 ax25_send_response(state->channel, TYPE_U_DISC, state->callsign, &state->response_packet);

                 // Select T1 Value - is not implemented

                 // Start T1
                 start_timer(timerT1[state->channel]);

             }

             break;
         }

    }
}

void ax25_state_wait_release_packet(AX25_data_link_state_machine_t *state, AX25_PACKET *packet) {
    trace_dl("AX25: STATE WAIT RELEASE (frame): ");
    switch (packet->frame_type) {
        case TYPE_U_UA : {
            trace_dl("UA\n");
            if (packet->PF == 1) {
                //TODO - DL Disconnect Confirm sent to layer 3, but it is not processed
                ax25_send_event(state, DL_DISCONNECT_Confirm, NULL, NO_ERROR);

                // Stop T1
                stop_timer(timerT1[state->channel]);
                state->dl_state = DISCONNECTED;
            } else {
                trace_dl("ERROR D: %s\n",ax25_errors_strs[ERROR_D]);
                debug_print("\n");
                // TODO error number needs to be processed in layer 3, otherwise we are stuck
                ax25_send_event(state, DL_ERROR_Indicate, NULL, ERROR_D);
            }
            break;
        }
        default: {
            break;
        }
    }
}



void ax25_state_connected_prim(AX25_data_link_state_machine_t *state, AX25_event_t *event) {
    trace_dl("AX25: STATE CONNECTED (prim): ");
    switch (event->primative) {
        case DL_DISCONNECT_Request : {
            trace_dl("Disconnect Request from Layer 3\n");
            discard_iframe_queue(state);
            state->RC = 0;
            // DISC P = 1
            state->response_packet = EMPTY_PACKET;
            state->response_packet.PF = 1;
            ax25_send_response(state->channel, TYPE_U_DISC, state->callsign, &state->response_packet);

            // Stop T3
            stop_timer(timerT3[state->channel]);
            // Start T1
            start_timer(timerT1[state->channel]);
            // Awaiting Release
            state->dl_state = AWAITING_RELEASE;
            break;
        }
        case DL_DATA_Request : {
            trace_dl("Send DATA Request from Layer 3\n");
            print_decoded_packet("Putting I-frame on Queue: ",&(event->packet));
            // TODO - for now just send this - it should be put on the IFRAME QUEUE
            bool rc = tx_send_packet(event->channel, &(event->packet), NOT_EXPEDITED, BLOCK);
            if (rc == FALSE) {
                // TODO - handle error
                debug_print("ERR: Could not queue I frame\n");
            }

            break;
        }

        //TODO - I FRAME POPS OFF QUEUE - from LAYER 3??

        case DL_UNIT_DATA_Request : {
            trace_dl("Send UI Request from Layer 3\n");
            break;
        }
        case DL_FLOW_ON_Request : {
            trace_dl("Request FLOW ON from Layer 3\n");

            break;
        }
        case DL_FLOW_OFF_Request : {
            trace_dl("Request FLOW OFF from Layer 3\n");

            break;
        }
        case DL_CONNECT_Request : {
            trace_dl("Request to initiate connection from Layer 3\n");

            break;
        }
       case DL_TIMER_T1_Expire : {
            trace_dl("Timer T1 Expired\n");
            state->RC = 1;
            transmit_enquiry(state, &state->decoded_packet);
            state->dl_state = TIMER_RECOVERY;
            break;
        }
       case DL_TIMER_T3_Expire : {
            trace_dl("Timer T3 Expired\n");
            state->RC = 1;
            transmit_enquiry(state, &state->decoded_packet);
            state->dl_state = TIMER_RECOVERY;
            break;
        }

        default : {
            trace_dl(".. Ignored\n");
            break;
        }

    }


}

void ax25_state_connected_packet(AX25_data_link_state_machine_t *state, AX25_PACKET *packet) {

}



void ax25_state_timer_rec_prim(AX25_data_link_state_machine_t *state, AX25_event_t *event) {

}

void ax25_state_timer_rec_packet(AX25_data_link_state_machine_t *state, AX25_PACKET *packet) {

}

void clear_exception_conditions(AX25_data_link_state_machine_t *state) {
    state->peer_receiver_busy = false;
    state->own_receiver_busy = false;
    state->reject_exception = false;
    state->srej_exception = false;
    state->achnowledge_pending = false;
    discard_iframe_queue(state);
}

void discard_iframe_queue(AX25_data_link_state_machine_t *state) {
    int i;
    for (i=0; i < I_QUEUE_FRAME_LEN; i++)
        state->I_frame_queue[i] = EMPTY_PACKET;

}

void transmit_enquiry(AX25_data_link_state_machine_t *state, AX25_PACKET *packet) {
    state->response_packet = EMPTY_PACKET;
    state->response_packet.PF = 1;
    state->response_packet.command = 1;
    state->response_packet.NR = state->VR;
    if (state->own_receiver_busy) {
        // RNR
        ax25_send_response(state->channel, TYPE_S_RNR, state->callsign, &state->response_packet);

    } else {
        //RR
        ax25_send_response(state->channel, TYPE_S_RR, state->callsign, &state->response_packet);

    }
    state->achnowledge_pending = false;
    start_timer(timerT1[state->channel]);
}
