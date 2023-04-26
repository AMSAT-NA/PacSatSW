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

#include "ax25_util.h"
#include "str_util.h"

/* Forward functions */
void ax25_process_frame(char *from_callsign, char *to_callsign, int channel);
void ax25_next_state_from_primative(AX25_data_link_state_machine_t *dl_state_info, AX25_primative_t primative);
void ax25_next_state_from_packet(AX25_data_link_state_machine_t *dl_state_info, AX25_PACKET *decoded_packet);
void ax25_state_disc_prim(AX25_data_link_state_machine_t *dl_state_info, AX25_primative_t prim);
void ax25_state_disc_packet(AX25_data_link_state_machine_t *dl_state_info, AX25_PACKET *packet);
void ax25_state_wait_conn_prim(AX25_data_link_state_machine_t *dl_state_info, AX25_primative_t prim);
void ax25_state_wait_conn_packet(AX25_data_link_state_machine_t *dl_state_info, AX25_PACKET *packet);
void ax25_state_wait_release_prim(AX25_data_link_state_machine_t *dl_state_info, AX25_primative_t prim);
void ax25_state_wait_release_packet(AX25_data_link_state_machine_t *dl_state_info, AX25_PACKET *packet);
void ax25_state_connected_prim(AX25_data_link_state_machine_t *dl_state_info, AX25_primative_t prim);
void ax25_state_connected_packet(AX25_data_link_state_machine_t *dl_state_info, AX25_PACKET *packet);
void ax25_state_timer_rec_prim(AX25_data_link_state_machine_t *dl_state_info, AX25_primative_t prim);
void ax25_state_timer_rec_packet(AX25_data_link_state_machine_t *dl_state_info, AX25_PACKET *packet);

/* Local variables */
static uint8_t ax25_packet_buffer[AX25_PKT_BUFFER_LEN]; /* Static buffer used to store packet as it is processed and before copy to next queue */
static AX25_data_link_state_machine_t data_link_state_machine[NUM_OF_RX_CHANNELS];
static AX25_PACKET decoded_packet;


portTASK_FUNCTION_PROTO(Ax25Task, pvParameters)  {

    vTaskSetApplicationTaskTag((xTaskHandle) 0, (pdTASK_HOOK_CODE)Ax25TaskWD );
    ResetAllWatchdogs();
    printf("Initializing Ax25 Data Link Task\n");

    while(1) {
        // TODO - we will need to know the channel that this came in on
        BaseType_t xStatus = xQueueReceive( xRxPacketQueue, &ax25_packet_buffer, CENTISECONDS(1) );  // Wait to see if data available
        if( xStatus == pdPASS ) {
            /* Data was successfully received from the queue */
            char from_callsign[MAX_CALLSIGN_LEN];
            char to_callsign[MAX_CALLSIGN_LEN];

            decode_call(&ax25_packet_buffer[8], from_callsign);
            decode_call(&ax25_packet_buffer[1], to_callsign);

//            print_packet(AX25_data_link_state_machine_t *dl_state_info, "AX25", ax25_packet_buffer+1, ax25_packet_buffer[0]);
            // TODO - channel A is hard coded by passing 0
            ax25_process_frame(from_callsign, to_callsign, 0);

        }
        ReportToWatchdog(CurrentTaskWD);

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

//    if (uplink_shut) {
//        char shut[] = "PB Closed.";
//        int rc = tx_send_packet(AX25_data_link_state_machine_t *dl_state_info, BROADCAST_CALLSIGN, PBSHUT, PID_NO_PROTOCOL, (uint8_t *)shut, strlen(shut), DONT_BLOCK_IF_QUEUE_FULL);
//        //debug_print("SENDING: %s |%s|\n",PBSHUT, shut);
//        ReportToWatchdog(CurrentTaskWD);
//        return;
//    } else  {
//
//        char * CALL = PBLIST;
//        if (number_on_pb == MAX_PB_LENGTH) {
//            CALL = PBFULL;
//        }
//        pb_make_list_str(pb_status_buffer, sizeof(pb_status_buffer));
////        uint8_t buffer[] = "PB Empty.";
//        uint8_t len = strlen((char *)pb_status_buffer);
////        debug_print("SENDING: %s |%s|\n",CALL, pb_status_buffer);

    // TODO - this should show the connection status of the 4 receivers
            char open[] = "Open ABCD.";
            int rc = tx_send_packet(BBS_CALLSIGN, BBSTAT, PID_NO_PROTOCOL, (uint8_t *)open, strlen(open), DONT_BLOCK_IF_QUEUE_FULL);

//       int rc = tx_send_packet(AX25_data_link_state_machine_t *dl_state_info, BBS_CALLSIGN, BBSTAT, PID_NO_PROTOCOL, (uint8_t *)ax25_status_buffer, len, DONT_BLOCK_IF_QUEUE_FULL);
        ReportToWatchdog(CurrentTaskWD);
        return;
//    }
}


/**
 * ax25_process_frame()
 *
 * process a frame received from a ground station.
 * This is called from the main processing loop whenever a frame is received.
 */
void ax25_process_frame(char *from_callsign, char *to_callsign, int channel) {
    if (strcasecmp(to_callsign, BBS_CALLSIGN) == 0) {
        decode_packet(&ax25_packet_buffer[1], ax25_packet_buffer[0], &decoded_packet);

        if (data_link_state_machine[channel].state == DISCONNECTED) {
            strlcpy(data_link_state_machine[channel].callsign, from_callsign, MAX_CALLSIGN_LEN);
            data_link_state_machine[channel].channel = channel;
            ax25_next_state_from_packet(&data_link_state_machine[channel], &decoded_packet);
        } else {
            // this needs to be the same callsign, otherwise we are busy
            if (strcasecmp(data_link_state_machine[channel].callsign, from_callsign) == 0) {
                ax25_next_state_from_packet(&data_link_state_machine[channel], &decoded_packet);
            } else {
                // TODO - need to implement.  If command F<-P send DM.  Else ignore
                debug_print("AX25: BUSY!\n");
            }
        }
    } else if (strcasecmp(to_callsign, BROADCAST_CALLSIGN) == 0) {
        // this was sent to the Broadcast Callsign

        /* Add to the queue and wait for 10ms to see if space is available */
        BaseType_t xStatus = xQueueSendToBack( xPbPacketQueue, &ax25_packet_buffer, CENTISECONDS(1) );
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
 * DATA LINK STATE MACHINE
 */

void ax25_next_state_from_packet(AX25_data_link_state_machine_t *dl_state_info, AX25_PACKET *decoded_packet) {
    switch (dl_state_info->state) {
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

void ax25_next_state_from_primative(AX25_data_link_state_machine_t *dl_state_info, AX25_primative_t primative) {
    switch (dl_state_info->state) {
        case DISCONNECTED : {
            ax25_state_disc_prim(dl_state_info, primative);
            break;
        }
        case AWAITING_CONNECTION : {
            ax25_state_wait_conn_prim(dl_state_info, primative);
            break;
        }
        case AWAITING_RELEASE : {
            ax25_state_wait_release_prim(dl_state_info, primative);
            break;
        }
        case CONNECTED : {
            ax25_state_connected_prim(dl_state_info, primative);
            break;
        }
        case TIMER_RECOVERY : {
            ax25_state_timer_rec_prim(dl_state_info, primative);
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
 * Data Link Disconnected State
 */
void ax25_state_disc_prim(AX25_data_link_state_machine_t *dl_state_info, AX25_primative_t prim) {

}

void ax25_state_disc_packet(AX25_data_link_state_machine_t *dl_state_info, AX25_PACKET *packet) {
    debug_print("AX25: STATE DISC: ");
    switch (packet->frame_type) {
        case TYPE_U_UA : {
            debug_print("UA\n");
            break;
        }
        case TYPE_U_UI : {
            debug_print("UI\n");
            break;
        }
        case TYPE_U_DISC : {
            debug_print("UA\n");
            break;
        }
        case TYPE_U_SABM : {
            debug_print("SABM\n");

            break;
        }
        case TYPE_U_SABME : {
            debug_print("SABME\n");

            break;
        }
        default:
            break;
    }
}


/**
 * Data Link Connected State
 */
void ax25_state_wait_conn_prim(AX25_data_link_state_machine_t *dl_state_info, AX25_primative_t prim) {

}

void ax25_state_wait_conn_packet(AX25_data_link_state_machine_t *dl_state_info, AX25_PACKET *packet) {

}



void ax25_state_wait_release_prim(AX25_data_link_state_machine_t *dl_state_info, AX25_primative_t prim) {

}

void ax25_state_wait_release_packet(AX25_data_link_state_machine_t *dl_state_info, AX25_PACKET *packet) {

}



void ax25_state_connected_prim(AX25_data_link_state_machine_t *dl_state_info, AX25_primative_t prim) {

}

void ax25_state_connected_packet(AX25_data_link_state_machine_t *dl_state_info, AX25_PACKET *packet) {

}



void ax25_state_timer_rec_prim(AX25_data_link_state_machine_t *dl_state_info, AX25_primative_t prim) {

}

void ax25_state_timer_rec_packet(AX25_data_link_state_machine_t *dl_state_info, AX25_PACKET *packet) {

}
