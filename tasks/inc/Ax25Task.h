/*
 * Ax25Task.h
 *
 *  Created on: 19 Apr, 2022
 *      Author: Chris E. Thompson, G0KLA / VE2TCP
 */

#ifndef TASKS_INC_AX25TASK_H_
#define TASKS_INC_AX25TASK_H_

#include <pacsat.h>
#include "ax25_util.h"

#define MODULO 8;

typedef enum {
    DISCONNECTED,
    AWAITING_CONNECTION,
    AWAITING_RELEASE,
    CONNECTED,
    TIMER_RECOVERY,
    AWAITING_V2_2_CONNECTION
} AX25_data_link_state_t;

typedef enum {
    DL_NO_PRIMATIVE = 0,
// DL Primitives (Received from DL):
    DL_CONNECT_Confirm,
    DL_CONNECT_Indicate,
    DL_DISCONNECT_Confirm,
    DL_DISCONNECT_Indicate,
    DL_DATA_Indicate,
    DL_UNIT_DATA_Indicate,
    DL_ERROR_Indicate,
//DL Primitives (Sent to DL):
    DL_CONNECT_Request,
    DL_DISCONNECT_Request,
    DL_DATA_Request,
    DL_UNIT_DATA_Request,
    DL_FLOW_OFF_Request,
    DL_FLOW_ON_Request,
// Timer Events
    DL_TIMER_T1_Expire,
    DL_TIMER_T3_Expire
} AX25_primative_t;

typedef struct {
    rx_channel_t channel;
    AX25_primative_t primative;
    AX25_PACKET *packet; // This is NULL for simple primatives
} AX25_event_t;

typedef struct {
    AX25_data_link_state_t state;
    rx_channel_t channel; // Radio A, B, C, D
    char callsign[MAX_CALLSIGN_LEN];
    AX25_PACKET decoded_packet;
    AX25_PACKET response_packet;
} AX25_data_link_state_machine_t;

/*
 * Error Codes
 */
#define ERROR_A "F=1 received but P=1 not outstanding.";
#define ERROR_B "Unexpected DM with F=1 in states 3, 4 or 5.";
#define ERROR_C "Unexpected UA in states 3, 4 or 5.";
#define ERROR_D "UA received without F=1 when SABM or DISC was sent P=1.";
#define ERROR_E "DM received in states 3, 4 or 5.";
#define ERROR_F "Data link reset; i.e., SABM received in state 3, 4 or 5.";
#define ERROR_I "N2 timeouts: unacknowledged data.";
#define ERROR_J "N(r) sequence ERROR_.";
#define ERROR_L "Control field invalid or not implemented.";
#define ERROR_M "Information field was received in a U or S-type frame.";
#define ERROR_N "Length of frame incorrect for frame type.";
#define ERROR_O "I frame exceeded maximum allowed length.";
#define ERROR_P "N(s) out of the window.";
#define ERROR_Q "UI response received, or UI command with P=1 received.";
#define ERROR_R "UI frame exceeded maximum allowed length.";
#define ERROR_S "I response received.";
#define ERROR_T "N2 timeouts: no response to enquiry.";
#define ERROR_U "N2 timeouts: extended peer busy condition.";
#define ERROR_V "No DL machines available to establish connection.";

/*
 * Routine prototypes
 */
void Ax25Task(void *pvParameters);
void ax25_send_status();

#endif /* TASKS_INC_AX25TASK_H_ */
