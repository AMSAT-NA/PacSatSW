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

#define MODULO 8
#define K 4 /* The maximum number of frames we can have outstanding.  4 for MOD 8, 32 for MOD 128 */
#define IFRAME_QUEUE_LEN 10

typedef enum {
    DISCONNECTED,
    AWAITING_CONNECTION,
    AWAITING_RELEASE,
    CONNECTED,
    TIMER_RECOVERY,
    AWAITING_V2_2_CONNECTION
} AX25_data_link_state_t;

typedef enum {
    DL_NO_primitive = 0,
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
    DL_POP_IFRAME_Request,
    DL_UNIT_DATA_Request,
    DL_FLOW_OFF_Request,
    DL_FLOW_ON_Request,
// Timer Events
    DL_TIMER_T1_Expire,
    DL_TIMER_T3_Expire,
// Link Multiplexer Events (Radios/Channel coordination) received from DL
    LM_SEIZE_Request,
    LM_RELEASE_Request,
    LM_DATA_Request,
    LM_EXPEDITED_DATA_Request,
// LM Events sent to DL
    LM_SEIZE_Confirm,
    LM_DATA_Indicate
} AX25_primitive_t;

/*
 * Error Codes
 */
typedef enum {
    ERROR_A,
    ERROR_B,
    ERROR_C,
    ERROR_D,
    ERROR_E,
    ERROR_F,
    ERROR_G,
    ERROR_H,
    ERROR_I,
    ERROR_J,
    ERROR_K,
    ERROR_L,
    ERROR_M,
    ERROR_N,
    ERROR_O,
    ERROR_P,
    ERROR_Q,
    ERROR_R,
    ERROR_S,
    ERROR_T,
    ERROR_U,
    ERROR_V,
    NO_ERROR
} ax25_error_t;


typedef struct {
    uint8_t rx_channel;
    AX25_primitive_t primitive;
    AX25_PACKET packet; // This needs to be a copy to avoid the data being changed before it is processed
    ax25_error_t error_num;
} AX25_event_t;

typedef enum {
    version_2_0,
    version_2_2
} ax25_version_t;

typedef struct {
    AX25_data_link_state_t dl_state;
    uint8_t channel;
    char callsign[MAX_CALLSIGN_LEN];
    AX25_PACKET decoded_packet;
    AX25_PACKET response_packet;
    AX25_PACKET I_frames_sent[MODULO]; /* Circular buffer (queue) of information to be transmitted in I frames */
    uint8_t que_read_pos; // This is the head of the queue and where the next frame is read from
    uint8_t que_length; // how many items in the queue
    uint8_t VS;   /* Send state variable.  The next sequential number to be assigned to the next I frame */
    uint8_t VA;   /* Acknowledge state variable.  The last I frame we have an ACk for. */
    uint8_t VR;   /* Receive state variable.  The sequence number of the next expected received I frame.
                     Updated when an I frame received and their (NS) send sequence number equals VR */
    uint8_t RC;  /* Retry count.  When this equals AX25_RETRIES_N2 we disconnect. */
    int SRTInTicks; /* Smoothed round trip time for packets between the two stations */
    int T1VInTicks; /* Calculated time for T1 timer */
    int T1TimeWhenLastStoppedInTicks; /* Time on T1 when it was stopped */
    bool layer_3_initiated_the_request;  /* SABM was sent at request of Layer 3 (Uplink state machine) i.e. DL_CONNECT_Request */
    bool peer_receiver_busy; /* Remote station is busy and can not receive I frames */
    bool own_receiver_busy;  /* Layer 3 (the Uplink state machine) is busy and can not reveive I frames */
    bool reject_exception;   /* A REJ frame has been sent to the remote station */
    int srej_exception;     /* A selective reject has been sent to the remote station */
    bool achnowledge_pending; /* I frames received but not yet acknowledged to the remote station */
} AX25_data_link_state_machine_t;

/*
 * Routine prototypes
 */
void Ax25Task(void *pvParameters);
void ax25_send_status();
bool test_ax25_retransmission();

#endif /* TASKS_INC_AX25TASK_H_ */
