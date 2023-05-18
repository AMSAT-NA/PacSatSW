/*
 * AxTask.c
 *
 *  Created on: 19 Apr, 2023
 *      Author: Chris E. Thompson, G0KLA / VE2TCP
 *
 * This is an RTOS port of the AX25 Data Link state machine intended for use
 * in an AMSAT satellite.  It follows Version 2.0 of the specification.
 *
 * XID is not implemented as the bandwidth to the spacecraft is precious and the
 * characteristics should already be available to the groundstation.
 * SREJ is not implemented
 * FRMR is handled but never sent, as recommended in the specification.
 *
 */

#include <strings.h>

#include "PbTask.h"
#include "TxTask.h"
#include "Ax25Task.h"
#include "FreeRTOS.h"
#include "os_task.h"
#include "nonvolManagement.h"

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
void ax25_send_response(rx_channel_t channel, ax25_frame_type_t frame_type, char *to_callsign, AX25_PACKET *response_packet, bool expedited);
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
void connected_rframe_response(AX25_data_link_state_machine_t *state, AX25_PACKET *packet);
void timer_rec_rframe_response(AX25_data_link_state_machine_t *state, AX25_PACKET *packet);
void iframe_pops_off_queue(AX25_data_link_state_machine_t *state, AX25_event_t *event);
void process_iframe(AX25_data_link_state_machine_t *state, AX25_PACKET *packet, AX25_data_link_state_t final_state);
void send_rr_frame(AX25_data_link_state_machine_t *state, AX25_PACKET *packet);

/* State Machine Helper functions */
void nr_error_recovery(AX25_data_link_state_machine_t *state, AX25_PACKET *packet);
void clear_exception_conditions(AX25_data_link_state_machine_t *state);
void transmit_enquiry(AX25_data_link_state_machine_t *state, AX25_PACKET *packet);
void enquiry_response(AX25_data_link_state_machine_t *state, AX25_PACKET *packet, int F);
void invoke_retransmission(AX25_data_link_state_machine_t *state, int NR);
void check_iframes_acknowledged(AX25_data_link_state_machine_t *state, AX25_PACKET *packet);
void establish_data_link(AX25_data_link_state_machine_t *state);
void check_need_for_response(AX25_data_link_state_machine_t *state, AX25_PACKET *packet);

/* Utility functions */
void clear_layer_3_initiated(AX25_data_link_state_machine_t *state);
void set_layer_3_initiated(AX25_data_link_state_machine_t *state);
void discard_iframe_queue(AX25_data_link_state_machine_t *state);
bool VA_lte_NR_lte_VS(AX25_data_link_state_machine_t *state, int nr);
int shiftByV(int x, int VA);
int AX25_MODULO(int vs);

/* Local variables */
static xTimerHandle timerT1[NUM_OF_RX_CHANNELS];
static xTimerHandle timerT3[NUM_OF_RX_CHANNELS];

static rx_radio_buffer_t ax25_radio_buffer; /* Static storage for packets from the radio */
static AX25_data_link_state_machine_t data_link_state_machine[NUM_OF_RX_CHANNELS];
static const AX25_PACKET EMPTY_PACKET; // Use this to reset packet to zero
static AX25_event_t ax25_received_event; /* Static storage for received event */
static AX25_event_t send_event_buffer; /* Static storage for event we are sending */
static AX25_event_t timer_event; /* Static storage for timer events */
bool in_test = false;

// TODO These should be globals so that the Uplink can access them as well for debug messages
static char *rx_channel_names[] = {"A","B","C","D"};

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

    int chan;
    for (chan=0; chan < NUM_OF_RX_CHANNELS; chan++) {
        xIFrameQueue[chan] = xQueueCreate( IFRAME_QUEUE_LEN,  sizeof( AX25_event_t ) );
        if (xIFrameQueue[chan] == NULL) {
            /* The queue could not be created.  This is fatal and should only happen in test if we are short of memory at startup */
            debug_print("FATAL ERROR: Could not create IFRAME Queue for channel %d\n",chan);
            //TODO - log this
        }
    }

    // test
//    start_timer(timerT1[Channel_A]);
//    vTaskDelay(CENTISECONDS(200));
//    start_timer(timerT1[Channel_A]);
//    start_timer(timerT3[Channel_A]);
//    vTaskDelay(CENTISECONDS(1));
//    BaseType_t act = xTimerIsTimerActive(timerT1[Channel_A]);
//    debug_print("Timer 1: %d\n",act);
//    act = xTimerIsTimerActive(timerT3[Channel_A]);
//    debug_print("Timer 3: %d\n",act);

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
        /* Yield some time so any packets are sent, or so that others may do some processing */
        vTaskDelay(CENTISECONDS(1));
        ReportToWatchdog(CurrentTaskWD);

        if (!in_test) {
            // If any channels have I frames to send, then request that now
            int c;
            for (c = 0; c < NUM_OF_RX_CHANNELS; c++) {
                xStatus = xQueueReceive( xIFrameQueue[c], &ax25_received_event, CENTISECONDS(1) );  // Wait to see if data available
                if( xStatus == pdPASS ) {
//#ifdef DEBUG
//            print_decoded_packet("POPS OFF QUEUE: ",&ax25_received_event.packet);
//#endif
                    ax25_received_event.primative = DL_POP_IFRAME_Request;
                    ax25_next_state_from_primative(&data_link_state_machine[c], &ax25_received_event);
                }
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
    if (!ReadMRAMBoolState(StateUplinkEnabled)) {
        // for now we send nothing.  If the uplink is shut it is silent
        trace_dl("AX25: Uplink SHUT\n");
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

    BaseType_t act = xTimerIsTimerActive(timerT1[Channel_A]);
    if (act == pdPASS) {
        restart_timer(timer);
    } else {
        portBASE_TYPE timerT1Status = xTimerStart(timer, 0); // Block time of zero as this can not block
        if (timerT1Status != pdPASS) {
            debug_print("ERROR: Failed in init T1 Timer\n");
            // TODO =>        ReportError(RTOSfailure, FALSE, ReturnAddr, (int) PbTask); /* failed to create the RTOS timer */
            // TODO - it's possible this might fail.  Somehow we should recover from that.
        }
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
 * Create a response packet.
 * The dl_state of command and PF are set already in the response packet
 */
void ax25_send_response(rx_channel_t channel, ax25_frame_type_t frame_type, char *to_callsign, AX25_PACKET *response_packet, bool expedited) {
    strlcpy(response_packet->to_callsign, to_callsign, MAX_CALLSIGN_LEN);
    strlcpy(response_packet->from_callsign, BBS_CALLSIGN, MAX_CALLSIGN_LEN);

    response_packet->frame_type = frame_type;
#ifdef TRACE_AX25_DL
    print_decoded_packet("AX25: ", response_packet);
#endif
    bool rc = tx_send_packet(channel, response_packet, expedited, BLOCK);
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
            trace_dl("Disconnect Request from Layer 3.  Ignore, already disconnected\n");
            break;
        }
        case DL_UNIT_DATA_Request : {
            trace_dl("Send UI Request from Layer 3\n");
            debug_print("ERROR: NOT IMPLEMENTED\n");
            break;
        }
        case DL_CONNECT_Request : {
            trace_dl("Request to initiate connection from Layer 3\n");
            establish_data_link(state);
            set_layer_3_initiated(state);
            state->dl_state = AWAITING_CONNECTION;
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
            state->response_packet.PF = packet->PF & 0b1;
            ax25_send_response(state->channel, TYPE_U_DM, state->callsign, &state->response_packet, NOT_EXPEDITED);
            break;
        }
        case TYPE_U_SABM : {
            trace_dl("SABM\n");
            state->response_packet = EMPTY_PACKET; // zero out the packet
            // Set version 2.0 - we already have the default values of MODULO 8 and no SREJ.
            state->response_packet.PF = packet->PF;
            ax25_send_response(state->channel, TYPE_U_UA, state->callsign, &state->response_packet, NOT_EXPEDITED);
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
            state->response_packet.PF = packet->PF & 0b1;
            ax25_send_response(state->channel, TYPE_U_DM, state->callsign, &state->response_packet, NOT_EXPEDITED);
            break;
        }
        default:
            // All other commands get a DM
            if (packet->command) {
                state->response_packet = EMPTY_PACKET; // zero out the packet
                state->response_packet.PF = packet->PF & 0b1;
                ax25_send_response(state->channel, TYPE_U_DM, state->callsign, &state->response_packet, NOT_EXPEDITED);
            } else {
                trace_dl("Ignoring unexpected pkt type: %0x from %s\n",packet->frame_type, packet->from_callsign);
            }
            break;
    }
}


/**
 * Data Link WAIT CONNECTION State
 */
void ax25_state_wait_conn_prim(AX25_data_link_state_machine_t *state, AX25_event_t *event) {
    trace_dl("AX25: STATE WAIT CONNECTION (prim): ");
    switch (event->primative) {
        case DL_DISCONNECT_Request : {
            /* The spec says requeue the request and remain in the same state.
             * This is a request from Layer 3 to Disconnect while we are waiting for the response
             * to a connection request.  So an internal error, a command from the ground or a
             * timout has caused the disconnect.
             * The other end has not heard our SABM or it is trying to respond with its UA and we
             * have not heard it yet.  So they are in DISCONNECTED or CONNECTED state.
             * This should follow the same logic as DL_DISCONNECT_Request in the CONNECTED state
             */
            trace_dl("Disconnect Request from Layer 3\n");
            discard_iframe_queue(state);
            state->RC = 0;
            // DISC P = 1
            state->response_packet = EMPTY_PACKET;
            state->response_packet.PF = 1;
            ax25_send_response(state->channel, TYPE_U_DISC, state->callsign, &state->response_packet, NOT_EXPEDITED);
            stop_timer(timerT3[state->channel]);
            start_timer(timerT1[state->channel]);
            state->dl_state = AWAITING_RELEASE;
            break;
        }
        case DL_CONNECT_Request : {
            trace_dl("Request to initiate connection from Layer 3\n");
            discard_iframe_queue(state);
            set_layer_3_initiated(state);
            break;
        }
        case DL_UNIT_DATA_Request : {
            trace_dl("Send UI Request from Layer 3\n");
            debug_print("ERROR: NOT IMPLEMENTED\n");
            break;
        }
        case DL_DATA_Request : {
            trace_dl("Send DATA Request from Layer 3\n");
            /* We don't add the Iframe through this event.  Instead the Uplink (Layer 3) adds it directly
             * to the IFrameQueue.  Then it will know if the Queue is full and be able to throttle data */
            debug_print("ERROR: This should have been directly added to the iFrame Queue\n");
            break;
        }
        case DL_POP_IFRAME_Request : {
            trace_dl("I-frame Request or pop from Layer 3\n");
            // Is layer 3 initiated
            if (state->layer_3_initiated) {
                // TODO - This is what the spec says to do, but we will get stuck in a loop unless a new frame received or we time out?
                // Instead we could ignore the data and tell layer 3 we are disconnected.  i.e. reset it.
                BaseType_t xStatus = xQueueSendToFront( xIFrameQueue[state->channel], event, CENTISECONDS(1) );
                if( xStatus != pdPASS ) {
                    /* The send operation could not complete because the queue was full */
                    debug_print("AX25: IFRAME QUEUE FULL Channel %d: Could not push back to IFrame Queue for frame received in wrong state\n",state->channel);
                    // TODO - this can perhaps be ignored.  We are doing our best to conserve info received in the wrong state
                }
            }
            break;
        }
        case DL_TIMER_T1_Expire : {
             trace_dl("Timer T1 Expired\n");
             if (state->RC == AX25_RETRIES_N2) {
                 discard_iframe_queue(state);
                 ax25_send_event(state, DL_ERROR_Indicate, NULL, ERROR_G);
                 ax25_send_event(state, DL_DISCONNECT_Indicate, NULL, NO_ERROR);
                 state->dl_state = DISCONNECTED;
             } else {
                 state->RC += 1;
                 // SABM P = 1
                 state->response_packet = EMPTY_PACKET;
                 state->response_packet.PF = 1;
                 ax25_send_response(state->channel, TYPE_U_SABM, state->callsign, &state->response_packet, NOT_EXPEDITED);
                 // Select T1 Value - is not implemented
                 start_timer(timerT1[state->channel]);
             }
             break;
        }
        default : {
            trace_dl("Ignored event: %d \n", event->primative);
            break;
        }

    }
}

void ax25_state_wait_conn_packet(AX25_data_link_state_machine_t *state, AX25_PACKET *packet) {
    trace_dl("AX25: STATE WAIT CONNECTION (frame): ");
    switch (packet->frame_type) {
        case TYPE_U_UI : {
            trace_dl("UI\n");
            trace_dl("UI Not implemented\n");
            if (packet->PF == 1) {
                state->response_packet = EMPTY_PACKET; // zero out the packet
                state->response_packet.PF = 1;
                // The spec does not specify EXPEDITED, but we are establishing a connection and we now want to send a DM.
                // It should go ahead of further SABMs or other packets
                ax25_send_response(state->channel, TYPE_U_DM, state->callsign, &state->response_packet, EXPEDITED);
            }
            break;
        }
        case TYPE_U_SABM : {
            trace_dl("SABM\n");
            state->response_packet = EMPTY_PACKET; // zero out the packet
            state->response_packet.PF = packet->PF;
            ax25_send_response(state->channel, TYPE_U_UA, state->callsign, &state->response_packet, EXPEDITED);
            break;
        }
        case TYPE_U_SABME : {
            trace_dl("SABME\n");
            state->response_packet = EMPTY_PACKET; // zero out the packet
            state->response_packet.PF = packet->PF;
            ax25_send_response(state->channel, TYPE_U_DM, state->callsign, &state->response_packet, EXPEDITED);
            // TODO - stay in wait connection and hope they send SABM or move to DISCONNECTED?
            break;
        }
        case TYPE_U_DISC : {
            trace_dl("DISC\n");
            state->response_packet = EMPTY_PACKET; // zero out the packet
            state->response_packet.PF = packet->PF;
            ax25_send_response(state->channel, TYPE_U_DM, state->callsign, &state->response_packet, EXPEDITED);
            break;
        }
        case TYPE_U_DM : {
            trace_dl("DM\n");
            if (packet->PF == 1) {
                discard_iframe_queue(state);
                ax25_send_event(state, DL_DISCONNECT_Indicate, NULL, NO_ERROR);
                stop_timer(timerT1[state->channel]);
                state->dl_state = DISCONNECTED;
            }
            // else stay in AWAITING CONNECTION state
            break;
        }
        case TYPE_U_UA : {
            trace_dl("UA\n");
            if (packet->PF == 1) {
                if (!state->layer_3_initiated) {
                    /* In this unusual situation, how is layer 3 initiated
                     * The revised spec is wrong and confused.  The original spec says
                     * to send DL_CONNECT_Indicate to layer 3 but I interpret that is
                     * calling set_layer_3_initiated, which sends the same message
                     * The revised spec says to send DL_CONNECT_Confirm if VS != VA, but surely we
                     * are connected in this situation regardless?
                     */
                    if (state->VS != state->VA) {
                        discard_iframe_queue(state);
                    }
                    set_layer_3_initiated(state);
                }
                ax25_send_event(state, DL_CONNECT_Confirm, packet, NO_ERROR);
                stop_timer(timerT1[state->channel]);
                start_timer(timerT3[state->channel]);
                state->VS = 0;
                state->VA = 0;
                state->VR = 0;
                // Select T1 value - NOT IMPLEMEMENTED
                state->dl_state = CONNECTED;
            } else {
                ax25_send_event(state, DL_ERROR_Indicate, NULL, ERROR_D);
                // else stay in AWAITING CONNECTION state
            }
            break;
        }
        default: {
            trace_dl("Ignored packet\n");
            break;
        }

    }
}


/**
 * Data Link WAIT RELEASE State
 */
void ax25_state_wait_release_prim(AX25_data_link_state_machine_t *state, AX25_event_t *event) {
    trace_dl("AX25: STATE WAIT RELEASE (prim): ");
    switch (event->primative) {
        case DL_DISCONNECT_Request : {
            state->response_packet = EMPTY_PACKET; // zero out the packet
            state->response_packet.PF = 0;
            ax25_send_response(state->channel, TYPE_U_DM, state->callsign, &state->response_packet, EXPEDITED);
            break;
        }
        case DL_TIMER_T1_Expire : {
             trace_dl("Timer T1 Expired\n");

             if (state->RC == AX25_RETRIES_N2) {
                 ax25_send_event(state, DL_ERROR_Indicate, NULL, ERROR_G);
                 ax25_send_event(state, DL_DISCONNECT_Indicate, NULL, NO_ERROR);
                 state->dl_state = DISCONNECTED;
             } else {
                 state->RC += 1;
                 // DISC P = 1
                 state->response_packet = EMPTY_PACKET;
                 state->response_packet.PF = 1;
                 ax25_send_response(state->channel, TYPE_U_DISC, state->callsign, &state->response_packet, NOT_EXPEDITED);
                 // Select T1 Value - is not implemented
                 start_timer(timerT1[state->channel]);
             }
             break;
         }
        default : {
            trace_dl("Ignored event: %d \n", event->primative);
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
                // TODO error number needs to be processed in layer 3, otherwise we are stuck
                ax25_send_event(state, DL_ERROR_Indicate, NULL, ERROR_D);
            }
            break;
        }
        case TYPE_U_SABM : {
            trace_dl("SABM\n");
            state->response_packet = EMPTY_PACKET; // zero out the packet
            state->response_packet.PF = packet->PF;
            ax25_send_response(state->channel, TYPE_U_DM, state->callsign, &state->response_packet, EXPEDITED);
            break;
        }
        case TYPE_U_SABME : {
            trace_dl("SABME - NOT SUPPORTED YET\n");
            // We dont support v2.2 yet, so send DM and remain in dl_state.
            state->response_packet = EMPTY_PACKET; // zero out the packet
            state->response_packet.PF = packet->PF;
            ax25_send_response(state->channel, TYPE_U_DM, state->callsign, &state->response_packet, EXPEDITED);
            break;
        }
        case TYPE_U_DISC : {
            trace_dl("UA\n");
            state->response_packet = EMPTY_PACKET; // zero out the packet
            state->response_packet.PF = packet->PF;
            ax25_send_response(state->channel, TYPE_U_UA, state->callsign, &state->response_packet, EXPEDITED);
            break;
        }
        case TYPE_U_DM : {
            trace_dl("DM\n");
            if (packet->PF == 1) {
                ax25_send_event(state, DL_DISCONNECT_Confirm, packet, NO_ERROR);
                stop_timer(timerT1[state->channel]);
                state->dl_state = DISCONNECTED;
            }
            // else if F != 1 just stay in awaiting release state
            break;
        }
        case TYPE_U_UI : {
            trace_dl("UI\n");
            trace_dl("UI Not implemented\n");
            if (packet->PF == 1) {
                state->response_packet = EMPTY_PACKET; // zero out the packet
                state->response_packet.PF = 1;
                ax25_send_response(state->channel, TYPE_U_DM, state->callsign, &state->response_packet, EXPEDITED);
            }
            break;
        }
        case TYPE_I:
        case TYPE_S_RR:
        case TYPE_S_RNR:
        case TYPE_S_REJ: {
            trace_dl("I, RR, RNR or REJ\n");
            if (packet->PF == 1) {
                state->response_packet = EMPTY_PACKET; // zero out the packet
                state->response_packet.PF = 1;
                ax25_send_response(state->channel, TYPE_U_DM, state->callsign, &state->response_packet, EXPEDITED);
            }
            break;
        }
        case TYPE_S_SREJ : {
            trace_dl("SREJ\n");
            if (packet->command == AX25_COMMAND) {
                if (packet->PF == 1) {
                    state->response_packet = EMPTY_PACKET; // zero out the packet
                    state->response_packet.PF = 1;
                    ax25_send_response(state->channel, TYPE_U_DM, state->callsign, &state->response_packet, EXPEDITED);
                }
            }
            break;
        }
        default: {
            trace_dl("Ignored packet\n");
            break;
        }
    }
}


/**
 * Data Link CONNECTED State
 */
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
            ax25_send_response(state->channel, TYPE_U_DISC, state->callsign, &state->response_packet, NOT_EXPEDITED);

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
            /* We don't add the Iframe through this event.  Instead the Uplink (Layer 3) adds it directly
             * to the IFrameQueue.  Then it will know if the Queue is full and be able to throttle data */
            debug_print("ERROR: This should have been directly added to the iFrame Queue\n");
            break;
        }
        case DL_POP_IFRAME_Request : {
            iframe_pops_off_queue(state, event);
            break;
        }

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
            discard_iframe_queue(state);
            establish_data_link(state);
            set_layer_3_initiated(state);
            state->dl_state = AWAITING_CONNECTION;
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
            trace_dl("Ignored event: %d \n", event->primative);
            // TODO - what is the action here
            break;
        }

    }
}

void ax25_state_connected_packet(AX25_data_link_state_machine_t *state, AX25_PACKET *packet) {
    trace_dl("AX25: STATE CONNECTED (frame): ");
    switch (packet->frame_type) {
        case TYPE_U_SABM : {
            trace_dl("SABM\n");
            state->response_packet = EMPTY_PACKET; // zero out the packet
            state->response_packet.PF = packet->PF;
            // Set version 2.0 - we already have the default values of MODULO 8 and no SREJ.
            ax25_send_response(state->channel, TYPE_U_UA, state->callsign, &state->response_packet, NOT_EXPEDITED);
            clear_exception_conditions(state);

            // Send ERROR Messsage to Layer 3.  We are already connected.  Layer 3 can ignore this
            ax25_send_event(state, DL_ERROR_Indicate, NULL, ERROR_F);

            if (!state->VS == state->VA) {
                discard_iframe_queue(state);
                ax25_send_event(state, DL_CONNECT_Indicate, packet, NO_ERROR);
            }
            stop_timer(timerT1[state->channel]);
            start_timer(timerT3[state->channel]);
            state->VS = 0;
            state->VA = 0;
            state->VR = 0;
            state->RC = 0;
            break;
        }
        case TYPE_U_SABME : {
            trace_dl("SABME\n");
            trace_dl("NOT IMPLEMENTED\n");
            break;
        }
        case TYPE_U_FRMR : {
            trace_dl("FRMR\n");
            ax25_send_event(state, DL_ERROR_Indicate, NULL, ERROR_K);
            establish_data_link(state);
            clear_layer_3_initiated(state);
            state->dl_state = AWAITING_CONNECTION;
            break;
        }
        case TYPE_U_UA : {
            trace_dl("UA\n");
            // This would trigger us establishing the data link
            // Send ERROR Messsage to Layer 3.  We are already connected.  Layer 3 can ignore this
            ax25_send_event(state, DL_ERROR_Indicate, NULL, ERROR_C);
            establish_data_link(state);
            clear_layer_3_initiated(state);
            state->dl_state = AWAITING_CONNECTION;
            break;
        }
        case TYPE_U_UI : {
            trace_dl("UI\n");
            trace_dl("NOT IMPLEMENTED\n");
            break;
        }
        case TYPE_U_DISC : {
            trace_dl("DISC\n");
            discard_iframe_queue(state);
            state->response_packet = EMPTY_PACKET; // zero out the packet
            state->response_packet.PF = packet->PF;
            ax25_send_response(state->channel, TYPE_U_UA, state->callsign, &state->response_packet, NOT_EXPEDITED);
            ax25_send_event(state, DL_DISCONNECT_Indicate, packet, NO_ERROR);
            stop_timer(timerT3[state->channel]);
            stop_timer(timerT1[state->channel]);  // TODO - the flow diagram says START T1, but that makes no sense
            state->dl_state = DISCONNECTED;
            break;
        }
        case TYPE_U_DM : {
            trace_dl("DM\n");
            ax25_send_event(state, DL_ERROR_Indicate, packet, ERROR_E);
            ax25_send_event(state, DL_DISCONNECT_Confirm, packet, NO_ERROR);
            discard_iframe_queue(state);
            stop_timer(timerT3[state->channel]);
            stop_timer(timerT1[state->channel]);
            state->dl_state = DISCONNECTED;
            break;
        }
        case TYPE_S_RR : {
            trace_dl("RR\n");
            state->peer_receiver_busy = false;
            connected_rframe_response(state, packet);
            break;
        }
        case TYPE_S_RNR : {
            trace_dl("RNR\n");
            state->peer_receiver_busy = true;
            connected_rframe_response(state, packet);
            break;
        }

        // LM SIEZE CONFIRM - Implement if needed.  If we are able to transmit then send Eqnuiry Response if AckPending...

        case TYPE_S_SREJ : {
            trace_dl("SREJ -- NOT IMPLEMENTED\n");
            break;
        }
        /* Causes us to start resending I Frames at the specified number */
        case TYPE_S_REJ : {
            trace_dl("REJ\n");

            state->peer_receiver_busy = false;
            check_need_for_response(state, packet);
            if (VA_lte_NR_lte_VS(state, packet->NR)) {
                state->VA = packet->NR;
                stop_timer(timerT1[state->channel]);
                stop_timer(timerT3[state->channel]);
                state->achnowledge_pending = false;
                // Select T1 value
                invoke_retransmission(state, packet->NR);
            } else {
                nr_error_recovery(state, packet);
                state->dl_state = AWAITING_CONNECTION;
            }

            break;
        }
        case TYPE_I : {
            trace_dl("I FRAME\n");
            process_iframe(state, packet, CONNECTED);
            break;
        }

        default: {
            trace_dl("Ignored packet: %d \n", packet->frame_type);
            break;
        }
    }
}

/**
 * STATE TIMER RECOVERY
 */
void ax25_state_timer_rec_prim(AX25_data_link_state_machine_t *state, AX25_event_t *event) {
    trace_dl("AX25: STATE TIMER REC (prim): ");
    switch (event->primative) {
        case DL_DISCONNECT_Request : {
            trace_dl("Disconnect Request from Layer 3\n");
            discard_iframe_queue(state);
            state->RC = 0;
            // DISC P = 1
            state->response_packet = EMPTY_PACKET;
            state->response_packet.PF = 1;
            ax25_send_response(state->channel, TYPE_U_DISC, state->callsign, &state->response_packet, NOT_EXPEDITED);
            stop_timer(timerT3[state->channel]);
            start_timer(timerT1[state->channel]);
            state->dl_state = AWAITING_RELEASE;
            break;
        }
        case DL_DATA_Request : {
            trace_dl("Send DATA Request from Layer 3\n");
            /* We don't add the Iframe through this event.  Instead the Uplink (Layer 3) adds it directly
             * to the IFrameQueue.  Then it will know if the Queue is full and be able to throttle data */
            debug_print("ERROR: This should have been directly added to the iFrame Queue\n");
            break;
        }
        case DL_POP_IFRAME_Request : {
            iframe_pops_off_queue(state, event);
            break;
        }

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
            discard_iframe_queue(state);
            establish_data_link(state);
            set_layer_3_initiated(state);
            state->dl_state = TIMER_RECOVERY;
            break;
        }
       case DL_TIMER_T1_Expire : {
            trace_dl("Timer T1 Expired\n");
            if (state->RC == AX25_RETRIES_N2) {
                // MAX Retries reached, disconnect, just work out the error first
                if (state->VA == state->VS) {
                    if (state->peer_receiver_busy) {
                        ax25_send_event(state, DL_ERROR_Indicate, &event->packet, ERROR_U);
                    } else {
                        ax25_send_event(state, DL_ERROR_Indicate, &event->packet, ERROR_T);
                    }
                } else {
                    ax25_send_event(state, DL_ERROR_Indicate, &event->packet, ERROR_I);
                }
                ax25_send_event(state, DL_DISCONNECT_Confirm, &event->packet, NO_ERROR); // Tell Layer 3 link is dead
                discard_iframe_queue(state);
                // Send DM Response F=0
                state->response_packet = EMPTY_PACKET;
                state->response_packet.PF = 1;
                state->response_packet.command = AX25_RESPONSE;
                ax25_send_response(state->channel, TYPE_U_DM, state->callsign, &state->response_packet, NOT_EXPEDITED);
                state->dl_state = DISCONNECTED;
            } else {
                transmit_enquiry(state, &state->decoded_packet);
                state->dl_state = TIMER_RECOVERY;
                state->RC = state->RC + 1;
            }
            break;
        }

        default : {
            trace_dl("Ignored event: %d \n", event->primative);
            break;
        }
    }
}

void ax25_state_timer_rec_packet(AX25_data_link_state_machine_t *state, AX25_PACKET *packet) {
    trace_dl("AX25: STATE TIMER REC (frame): ");
    switch (packet->frame_type) {
        case TYPE_U_UA : {
            trace_dl("UA\n");
            // This would trigger us establishing the data link
            // Send ERROR Messsage to Layer 3.  We are already connected.  Layer 3 can ignore this
            ax25_send_event(state, DL_ERROR_Indicate, NULL, ERROR_C);
            establish_data_link(state);
            clear_layer_3_initiated(state);
            state->dl_state = AWAITING_CONNECTION;
            break;
        }
        case TYPE_U_DM : {
            trace_dl("DM\n");
            ax25_send_event(state, DL_ERROR_Indicate, packet, ERROR_E);
            ax25_send_event(state, DL_DISCONNECT_Confirm, packet, NO_ERROR);
            discard_iframe_queue(state);
            stop_timer(timerT3[state->channel]);
            stop_timer(timerT1[state->channel]);
            state->dl_state = DISCONNECTED;
            break;
        }
        case TYPE_U_SABM : {
            trace_dl("SABM\n");
            state->response_packet = EMPTY_PACKET; // zero out the packet
            state->response_packet.PF = packet->PF;
            // Set version 2.0 - we already have the default values of MODULO 8 and no SREJ.
            ax25_send_response(state->channel, TYPE_U_UA, state->callsign, &state->response_packet, NOT_EXPEDITED);
            clear_exception_conditions(state);

            // Send ERROR Messsage to Layer 3.  We are already connected.  Layer 3 can ignore this
            ax25_send_event(state, DL_ERROR_Indicate, NULL, ERROR_F);

            if (!state->VS == state->VA) {
                discard_iframe_queue(state);
                ax25_send_event(state, DL_CONNECT_Indicate, packet, NO_ERROR);
            }
            stop_timer(timerT1[state->channel]);
            start_timer(timerT3[state->channel]);
            state->VS = 0;
            state->VA = 0;
            state->VR = 0;
            state->RC = 0;
            state->dl_state = CONNECTED;
            break;
        }
        case TYPE_U_SABME : {
            trace_dl("SABME\n");
            trace_dl("NOT IMPLEMENTED\n");
            break;
        }
        case TYPE_U_FRMR : {
            trace_dl("FRMR\n");
            ax25_send_event(state, DL_ERROR_Indicate, NULL, ERROR_K);
            establish_data_link(state);
            clear_layer_3_initiated(state);
            state->dl_state = AWAITING_CONNECTION;
            break;
        }
        case TYPE_U_UI : {
            trace_dl("UI\n");
            trace_dl("NOT IMPLEMENTED\n");
            break;
        }
        case TYPE_U_DISC : {
            trace_dl("DISC\n");
            discard_iframe_queue(state);
            state->response_packet = EMPTY_PACKET; // zero out the packet
            state->response_packet.PF = packet->PF;
            ax25_send_response(state->channel, TYPE_U_UA, state->callsign, &state->response_packet, NOT_EXPEDITED);
            ax25_send_event(state, DL_DISCONNECT_Indicate, packet, NO_ERROR);
            stop_timer(timerT3[state->channel]);
            stop_timer(timerT1[state->channel]);  // TODO - the flow diagram says START T1, but that makes no sense
            state->dl_state = DISCONNECTED;
            break;
        }
        case TYPE_S_RR : {
            trace_dl("RR\n");
            state->peer_receiver_busy = false;
            timer_rec_rframe_response(state, packet);
            break;
        }
        case TYPE_S_RNR : {
            trace_dl("RNR\n");
            state->peer_receiver_busy = true;
            timer_rec_rframe_response(state, packet);
            break;
        }

        // LM SIEZE CONFIRM - Implement if needed.  If we are able to transmit then send Eqnuiry Response if AckPending...

        case TYPE_S_SREJ : {
            trace_dl("SREJ -- NOT IMPLEMENTED\n");
            break;
        }
        case TYPE_S_REJ : {
            trace_dl("REJ\n");
            state->peer_receiver_busy = false;

            if (packet->command == AX25_RESPONSE && packet->PF == 1) {
                stop_timer(timerT1[state->channel]);
                // Select T1 value
                if (VA_lte_NR_lte_VS(state, packet->NR)) {
                    state->VA = packet->NR;
                    if (state->VS == state->VA) {
                        start_timer(timerT3[state->channel]);
                        state->RC = 0;
                        state->dl_state = CONNECTED;
                        return;
                    } else {
                        invoke_retransmission(state, packet->NR);
                        stop_timer(timerT3[state->channel]);
                        start_timer(timerT1[state->channel]);
                        state->achnowledge_pending = false;
                        state->dl_state = TIMER_RECOVERY;
                        return;
                    }
                } else { // bad NR
                    nr_error_recovery(state, packet);
                    state->dl_state = AWAITING_CONNECTION;
                    return;
                }
            } else { // Not response and F = 1
                if (packet->command == AX25_COMMAND && packet->PF == 1) {
                    enquiry_response(state, packet, 1);
                }
                if (VA_lte_NR_lte_VS(state, packet->NR)) {
                    state->VA = packet->NR;
                    if (state->VS == state->VA) {
                        state->dl_state = TIMER_RECOVERY;
                        return;
                    } else {
                        invoke_retransmission(state, packet->NR);
                        stop_timer(timerT3[state->channel]);
                        start_timer(timerT1[state->channel]);
                        state->achnowledge_pending = false;
                        state->dl_state = TIMER_RECOVERY;
                        return;
                    }
                } else { // bad NR
                    nr_error_recovery(state, packet);
                    state->dl_state = AWAITING_CONNECTION;
                }
            }
            break;
        }
        case TYPE_I : {
            trace_dl("I FRAME\n");
            process_iframe(state, packet, TIMER_RECOVERY);
            break;
        }

        default: {
            trace_dl("Ignored packet: %d \n", packet->frame_type);
            break;
        }
    }
}

void connected_rframe_response(AX25_data_link_state_machine_t *state, AX25_PACKET *packet) {
    check_need_for_response(state, packet);
    if (VA_lte_NR_lte_VS(state, packet->NR)) {
        check_iframes_acknowledged(state, packet);
    } else {
        nr_error_recovery(state, packet);
        state->dl_state = AWAITING_CONNECTION;
    }
}

void timer_rec_rframe_response(AX25_data_link_state_machine_t *state, AX25_PACKET *packet) {
    check_need_for_response(state, packet);
    if (packet->command == AX25_RESPONSE && packet->PF == 1) {
        stop_timer(timerT1[state->channel]);
        // Select T1 Value
        if (VA_lte_NR_lte_VS(state, packet->NR)) {
            state->VA = packet->NR;
            if (state->VS == state->VA) {
                start_timer(timerT3[state->channel]);
                state->RC = 0;
                state->dl_state = CONNECTED;
                return;
            } else {
                invoke_retransmission(state, packet->NR);
                stop_timer(timerT3[state->channel]);
                start_timer(timerT1[state->channel]);
                state->achnowledge_pending = true;
                state->dl_state = TIMER_RECOVERY;
                return;
            }
        } else { // NR not good
            nr_error_recovery(state, packet);
            state->dl_state = AWAITING_CONNECTION;
            return;
        }

    } else { // was not response with F == 1
        if (packet->command == AX25_COMMAND && packet->PF == 1) {
            enquiry_response(state, packet, 1);
        }
        if (VA_lte_NR_lte_VS(state, packet->NR)) {
            state->VA = packet->NR;
            state->dl_state = TIMER_RECOVERY;
            return;
        } else {
            nr_error_recovery(state, packet);
            state->dl_state = AWAITING_CONNECTION;
        }
    }
}


void iframe_pops_off_queue(AX25_data_link_state_machine_t *state, AX25_event_t *event) {
    trace_dl("POP IFRAME Request\n");
    /* While this is the DL_DATA_Request event, it was already added to
     * the event queue and has now been removed.  So we process the I Frame and send it */
//            print_decoded_packet("I-frame pops off Queue: ",&(event->packet));
    if (state->peer_receiver_busy) {
        // push iframe back on queue
        BaseType_t xStatus = xQueueSendToFront( xIFrameQueue[state->channel], event, CENTISECONDS(1) );
        if( xStatus != pdPASS ) {
            /* The send operation could not complete because the queue was full */
            debug_print("AX25: SERIOUS IFRAME QUEUE FULL: Could not push back to IFrame Queue\n");
            // TODO - this must be prevented.  The Layer 3 machine should be throttled if this queue is full
            return;
        }
    } else if (state->VS == (state->VA + K)%MODULO) {
        // push iframe back on queue
        BaseType_t xStatus = xQueueSendToFront( xIFrameQueue[state->channel], event, CENTISECONDS(1) );
        if( xStatus != pdPASS ) {
            /* The send operation could not complete because the queue was full */
            debug_print("AX25: SERIOUS IFRAME QUEUE FULL: Could not push back to IFrame Queue\n");
            // TODO - this must be prevented.  The Layer 3 machine should be throttled if this queue is full
            return;
        }
    } else {
        event->packet.NS = state->VS;
        event->packet.NR = state->VR;
        event->packet.PF = 0;
        bool rc = tx_send_packet(event->channel, &event->packet, NOT_EXPEDITED, BLOCK);
        if (rc == FALSE) {
            // TODO - handle error - retry? Put back on queue? Disconnect?
            debug_print("ERR: Could not send I frame to TX queue\n");
        }
        state->I_frames_sent[state->VS] = event->packet;
        state->VS = (state->VS + 1)%MODULO;
        state->achnowledge_pending = false;
        BaseType_t act = xTimerIsTimerActive(timerT1[state->channel]);
        if (act != pdPASS) {
            stop_timer(timerT3[state->channel]);
            start_timer(timerT1[state->channel]);
        }
    }
}

/**
 * Process a received I Frame in CONNECTED and TIME RECOVERY states
 *
 * This assumes SREJ is not enabled.
 *
 */
void process_iframe(AX25_data_link_state_machine_t *state, AX25_PACKET *packet, AX25_data_link_state_t final_state) {
    print_decoded_packet("AX25: Received I-frame: ",packet);
    if (packet->command == AX25_RESPONSE) {
        // Response received, discard the iframe. i.e do nothing with it.
        ax25_send_event(state, DL_ERROR_Indicate, packet, ERROR_S);
        state->dl_state = CONNECTED;
        return;
    } else {
        // Data length is valid
        if (packet->data_len > AX25_MAX_INFO_BYTES_LEN) {
            ax25_send_event(state, DL_ERROR_Indicate, packet, ERROR_O);
            establish_data_link(state);
            clear_layer_3_initiated(state);
            state->dl_state = AWAITING_CONNECTION;
            return;
        } else {
            if (VA_lte_NR_lte_VS(state, packet->NR)) {
                check_iframes_acknowledged(state, packet);
                if (state->own_receiver_busy) {
                    // Discard iframe i.e. ignore it
                    if (packet->PF == 1) {
                        send_rr_frame(state, packet);
                    }
                    state->dl_state = final_state;
                    return;
                } else { // own receiver is not busy
                    if (packet->NS == state->VR) { // We expect frame number VR and that is what we got
                        state->VR = (state->VR + 1)%MODULO;
                        state->reject_exception = false;
                        if (state->srej_exception > 0)
                            state->srej_exception--;
                        // Send the IFrame data to Layer 3 and store in the sent iFrame Queue
                        ax25_send_event(state, DL_DATA_Indicate, packet, NO_ERROR);
                        ax25_copy_packet(packet, &state->I_frames_sent[packet->NS]);

                        if (packet->PF == 1) {
                            send_rr_frame(state, packet);
                            state->dl_state = final_state;
                            return;
                        } else {
                            if (!state->achnowledge_pending) {
                                /* LM Seize Request
                                 * This requests transmission at the next possible opportunity.  The ack_pending will go
                                 * in the next available packet.  Given we are full duplex and we do not delay, then this
                                 * will go with the next frame regardless.
                                 */
                                state->achnowledge_pending = true;
                                state->dl_state = final_state;
                                return;
                            }
                        }
                    } else { // NS does not equal VS.  Uh oh
                        if (state->reject_exception) {
                            // We already have a rejection outstanding, discard and ignore iframe
                            if (packet->PF == 1) {
                                send_rr_frame(state,packet);
                            }
                            state->dl_state = final_state;
                            return;
                        } else { // send a rejection
                            // if srej was enabled, then we do that, but it is not
                            // Discard Iframe by ignoring
                            state->reject_exception = true;
                            state->response_packet = EMPTY_PACKET;
                            state->response_packet.PF = packet->PF;
                            state->response_packet.command = AX25_RESPONSE;
                            state->response_packet.NR = state->VR;
                            ax25_send_response(state->channel, TYPE_S_REJ, state->callsign, &state->response_packet, NOT_EXPEDITED);
                            state->achnowledge_pending = false;
                            state->dl_state = final_state;
                            return;
                        }
                    }
                }
            } else { // not good NR
                nr_error_recovery(state, packet);
                state->dl_state = AWAITING_CONNECTION;
                return;
            }
        }
    }
}

void send_rr_frame(AX25_data_link_state_machine_t *state, AX25_PACKET *packet) {
    state->response_packet = EMPTY_PACKET;
    state->response_packet.PF = 1;
    state->response_packet.command = AX25_RESPONSE;
    state->response_packet.NR = state->VR;
    ax25_send_response(state->channel, TYPE_S_RR, state->callsign, &state->response_packet, NOT_EXPEDITED);
    state->achnowledge_pending = false;
}

void nr_error_recovery(AX25_data_link_state_machine_t *state, AX25_PACKET *packet) {
    ax25_send_event(state, DL_ERROR_Indicate, NULL, ERROR_J);
    establish_data_link(state);
    clear_layer_3_initiated(state);
}

void clear_exception_conditions(AX25_data_link_state_machine_t *state) {
    state->peer_receiver_busy = false;
    state->own_receiver_busy = false;
    state->reject_exception = false;
    state->srej_exception = false;
    state->achnowledge_pending = false;
    discard_iframe_queue(state);
}

void transmit_enquiry(AX25_data_link_state_machine_t *state, AX25_PACKET *packet) {
    state->response_packet = EMPTY_PACKET;
    state->response_packet.PF = 1; /* This is the only time we sent R frames with P = 1 */
    state->response_packet.command = AX25_COMMAND;
    state->response_packet.NR = state->VR;
    if (state->own_receiver_busy) {
        ax25_send_response(state->channel, TYPE_S_RNR, state->callsign, &state->response_packet, NOT_EXPEDITED);
    } else {
        ax25_send_response(state->channel, TYPE_S_RR, state->callsign, &state->response_packet, NOT_EXPEDITED);
    }
    state->achnowledge_pending = false;
    start_timer(timerT1[state->channel]);
}

void enquiry_response(AX25_data_link_state_machine_t *state, AX25_PACKET *packet, int F) {
    state->response_packet = EMPTY_PACKET; // zero out the packet
    state->response_packet.NR = state->VR;
    if (packet->PF & (packet->frame_type == TYPE_S_RR ||
            packet->frame_type == TYPE_S_RNR ||
            packet->frame_type == TYPE_I)) {
        if (state->own_receiver_busy) {
            state->response_packet.command = AX25_RESPONSE;
            ax25_send_response(state->channel, TYPE_S_RNR, state->callsign, &state->response_packet, NOT_EXPEDITED);
            state->achnowledge_pending = false;
            return;
        } else {
            // SREJ is not enabled, otherwise additional logic needed here for out of sequence frames and SREJ
        }
    }

    ax25_send_response(state->channel, TYPE_S_RR, state->callsign, &state->response_packet, NOT_EXPEDITED);
    state->achnowledge_pending = false;

}

/**
 * Retransmit frames back as far as NR, assuming we have them
 * The spec says we "push them in the iframe queue"
 *
 * If we have 3 frames and VS = 6, for example, so we have sent 3,4,5 and a frame is in the queue, which will be 6 when
 * we transmit it.  We have a reject with NR 4, which means 3 is accepted but 4 and 5 are not.
 *
 * We need to actually decrement VS because the frames are not sent right now.  They are sent as they get popped off the
 * Iframe queue.  When the first one is popped off it needs to have VS = 4 in this case.
 * The frames in the queue don't store their VS number.  This is set when it pops off the queue, but we need to get them
 * in the right order so they end up with the right VS numbers again, which is set as NS in the frame.
 *
 * We can push frames onto the head of the queue so that they will be sent before the frame that is sitting there now, so
 * that will still be sent with NS=6.  Note that 6 is in the queue (even though it is not labeled yet).  We want to put 4 and 5 on the queue, but we must
 * push 5 and then 4.  So the newest first.
 *
 */
void invoke_retransmission(AX25_data_link_state_machine_t *state, int NR) {
    //backtrack.  Put all the frames on the queue again from NR.  Everything before is confirmed and VA already set to NR.
    //We need to change VS.  VS is the next frame we were going to send. So it now needs to equal NR, it must currently be in front
    trace_dl("BACKTRACK: VS: %d to NEW NR:%d\n",state->VS, NR);
    int vs = state->VS; // VS points to next frame we will send.
    do {
        vs=AX25_MODULO(vs-1); // Start with the previous frame sent and go backwards
//        if (state->I_frames_sent[vs] != null) {
            // NS stays the same, we are re-sending the frame from before
            // but we are confirming all frames up to N(R) -1 by setting NS = VR
          if (state->I_frames_sent[vs].NS != vs) {
                // Integrity check
                trace_dl("ERROR: I_frames_sent corrupt? Wrong I frame being retransmitted VS: %d - NS:%d\n", state->VS, state->I_frames_sent[vs].NS);
          } else {
            send_event_buffer.channel = state->channel;
            send_event_buffer.primative = DL_DATA_Request;
            send_event_buffer.packet = state->I_frames_sent[vs];
            send_event_buffer.error_num = NO_ERROR;
            BaseType_t xStatus = xQueueSendToFront( xIFrameQueue[state->channel], &send_event_buffer, CENTISECONDS(1) );
            if( xStatus != pdPASS ) {
                /* The send operation could not complete because the queue was full */
                debug_print("AX25: SERIOUS IFRAME QUEUE FULL Channel %d: Could not push back to IFrame Queue for retransmission\n",state->channel);
                // TODO - this must be prevented.  The Layer 3 machine should be throttled if this queue is full
                return;
            }
#ifdef DEBUG
            print_decoded_packet("RETRANSMIT: ",&send_event_buffer.packet);
#endif
          }
        //}

    } while (vs != NR); // continue until we have pushed the requested NR
    // Now set VS to NR.  We are ready to send the rejected frames again.  We have rewound
    state->VS = NR;

}

/**
 * We have received a frame with an NR that is an ack for all frames up to that
 * point.  Or rather it says "I am ready for the next number NR", achnowledging through NR -1.
 * We set VA equal to that NR number
 * TODO:
 * Our iFramesSent[] array holds the frames that we have sent.  This means we can delete (null)
 * this frame.  This is not required, but will help to show any bugs where we try to re-send a
 * frame that was already ack'd.  Note that it could be multiple frames.
 *
 */
void check_iframes_acknowledged(AX25_data_link_state_machine_t *state, AX25_PACKET *packet) {
    if (state->peer_receiver_busy) {
        state->VA = packet->NR;
        start_timer(timerT3[state->channel]);  // TODO - revised flow chart says STOP T3. But PSGS and direwold have Start T3 per the old flow chart.
        BaseType_t act = xTimerIsTimerActive(timerT1[state->channel]);
         if (act != pdPASS) {
             start_timer(timerT1[state->channel]);
         }
    } else {
        if (packet->NR == state->VS) { // essentially this is an RR and NR is the next frame for us - normal situation
            state->VA = packet->NR;
            stop_timer(timerT1[state->channel]);
            start_timer(timerT3[state->channel]);
            //TODO - we do not  Select T1 Value
        } else {
            // then not all frames ACK'd
            if (packet->NR != state->VA) { // we got an ack for part of the frames we have sent, but not all
                // set the ack variable VA to the number we received.  Start T1 as we don't want to send any more data yet
                state->VA = packet->NR;
                start_timer(timerT1[state->channel]);
            }
            // otherwise the RR had an NR equal to the VA we already have.  A second confirmation of where we are
            // but if our VS is out in front, don't we need to resend the frame?  What if VS > NR - then our integrity check fails below
        }
    }
}

void establish_data_link(AX25_data_link_state_machine_t *state) {
    clear_exception_conditions(state);
    state->RC = 1;
    state->response_packet = EMPTY_PACKET;
    state->response_packet.PF = 1;
    // send SABM
    ax25_send_response(state->channel, TYPE_U_SABM, state->callsign, &state->response_packet, NOT_EXPEDITED);

    stop_timer(timerT3[state->channel]);
    start_timer(timerT1[state->channel]);

}

// TODO - surely this must also reset VS, VA????
void discard_iframe_queue(AX25_data_link_state_machine_t *state) {
    BaseType_t xStatus = xQueueReset(xIFrameQueue[state->channel]);
    if( xStatus != pdPASS ) {
        /* The reset operation could not complete because something is blocking the queue */
        debug_print("AX25: SERIOUS ERROR: Could not reset the IFrame Queue\n");
        // TODO - Do we rety a number of times?  Is it possible that this happens?  This is the queue that
        // Layer 3 writes into.
    }
}

void check_need_for_response(AX25_data_link_state_machine_t *state, AX25_PACKET *packet) {
    if (packet->command & packet->PF) {
        enquiry_response(state, packet, 1);
    } else if (!packet->command & packet->PF) {
        // TODO - can we take any action here, or ignore in Layer 3?  Just debug info?
        ax25_send_event(state, DL_ERROR_Indicate, packet, ERROR_A);
    }
}

void clear_layer_3_initiated(AX25_data_link_state_machine_t *state) {
    ax25_send_event(state, DL_DISCONNECT_Confirm, NULL, NO_ERROR);
    state->layer_3_initiated = false;
}

void set_layer_3_initiated(AX25_data_link_state_machine_t *state) {
    ax25_send_event(state, DL_CONNECT_Indicate, NULL, NO_ERROR);
    state->layer_3_initiated = true;
}


/**
 * Checks that V(a) <= N(r) <= V(s)
 */
bool VA_lte_NR_lte_VS(AX25_data_link_state_machine_t *state, int nr) {
    int shiftedVa = shiftByV(state->VA, state->VA);
    int shiftedNr = shiftByV(nr, state->VA);
    int shiftedVs = shiftByV(state->VS, state->VA);

//    trace_dl("AX25 Good NR? V(a) <= N(r) <= V(s) VA:%d NR: %d VS: %d\n",state->VA, nr, state->VS); // + " sVA:"+shiftedVa+" sNR:"+shiftedNr+" sVS:"+shiftedVs);

    return ((shiftedVa <= shiftedNr) && (shiftedNr <= shiftedVs));
}

/**
 * Shift by VA or VR depending on what you pass in
 * TODO - this is ported code, but all modulo should use the ax25_MODULO function
 * DireWolf implements this as ax25_MODULO(x) - S->VA
 */
int shiftByV(int x, int VA) {
    return (x - VA) & (MODULO-1);
}

int AX25_MODULO(int vs) {
    return (vs & (MODULO-1)); // uses masking rather than % so that negative numbers handled correctly
}

/**
 * TEST FUNCTIONS FOLLOW
 */

bool test_ax25_retransmission() {
    debug_print("## SELF TEST: ax25 retransmission for NR 6\n");

    in_test = TRUE;
    AX25_data_link_state_machine_t dl;
    dl.channel = Channel_A;
    dl.VA = 7;
    dl.VS = 0;
    dl.VR = 2;

    AX25_event_t send_event;
    send_event.channel = dl.channel;
    send_event.packet.frame_type = TYPE_I;
    send_event.packet.PF = 0;
    strlcpy(send_event.packet.to_callsign, "G0KLA", MAX_CALLSIGN_LEN);
    strlcpy(send_event.packet.from_callsign, BBS_CALLSIGN, MAX_CALLSIGN_LEN);
    send_event.primative = DL_DATA_Request;

    int n = 0;
    for (n=0; n<8;n++) {
        send_event.packet.NR = 2;
        send_event.packet.NS = n;
        ax25_copy_packet(&send_event.packet, &dl.I_frames_sent[n]);
    }

    invoke_retransmission(&dl, 6);
    debug_print("## PASSED SELF TEST: test retransmission\n");

    in_test = FALSE;
    return TRUE;
}
