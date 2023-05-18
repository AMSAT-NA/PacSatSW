/*
 * ax25_util.h
 *
 *  Created on: Feb 20, 2023
 *      Author: g0kla
 */

#ifndef UTILITIES_INC_AX25_UTIL_H_
#define UTILITIES_INC_AX25_UTIL_H_

#include "config.h"

// Values for the command bit
#define AX25_COMMAND true
#define AX25_RESPONSE false

// U Frame Types
#define U_CONTROL_MASK       0b11101111
#define BITS_U_SABME         0b01101111
#define BITS_U_SABM          0b00101111
#define BITS_U_DISCONNECT    0b01000011
#define BITS_U_DISCONNECT_MODE 0b00001111
#define BITS_UA              0b01100011
#define BITS_U_FRAME_REJECT  0b10000111
#define BITS_UI              0b00000011
#define BITS_U_EXCH_ID       0b10101111
#define BITS_U_TEST          0b11100011

// S Frame BITSs - based on SS - supervisory BITS
#define S_CONTROL_MASK       0b00001111
#define BITS_S_RECEIVE_READY 0b0001
#define BITS_S_RECEIVE_NOT_READY 0b0101
#define BITS_S_REJECT 0b1001
#define BITS_S_SELECTIVE_REJECT 0b1101

typedef enum ax25_frame_type_e {

    TYPE_I = 0,   // Information
    TYPE_S_RR,    // Receive Ready - System Ready To Receive
    TYPE_S_RNR,   // Receive Not Ready - TNC Buffer Full
    TYPE_S_REJ,   // Reject Frame - Out of Sequence or Duplicate
    TYPE_S_SREJ,  // Selective Reject - Request single frame repeat
    TYPE_U_SABME, // Set Async Balanced Mode, Extended
    TYPE_U_SABM,  // Set Async Balanced Mode
    TYPE_U_DISC,  // Disconnect
    TYPE_U_DM,    // Disconnect Mode
    TYPE_U_UA,    // Unnumbered Acknowledge
    TYPE_U_FRMR,  // Frame Reject
    TYPE_U_UI,    // Unnumbered Information
    TYPE_U_XID,   // Exchange Identification
    TYPE_U_TEST  // Test

} ax25_frame_type_t;

typedef struct {
    ax25_frame_type_t frame_type;
    char to_callsign[MAX_CALLSIGN_LEN];
    char from_callsign[MAX_CALLSIGN_LEN];
    char via_callsign[MAX_CALLSIGN_LEN];
    int command;
    uint8_t control;
    uint8_t NR;
    uint8_t NS;
    int PF;
    uint8_t pid;
    uint8_t data[AX25_MAX_INFO_BYTES_LEN]; // If this is increased then data_len should be increased
    uint8_t data_len; // Set this to 0 if there are no data bytes
} AX25_PACKET;


int decode_call_and_command(uint8_t *c, char *call, int *final_call, int *command);
int decode_call(uint8_t *c, char *call);
int encode_call(char *name, uint8_t *buf, int final_call, int command);
uint8_t ax25_decode_packet(uint8_t *packet, int len, AX25_PACKET *decoded_packet);
void ax25_copy_packet(AX25_PACKET *packet, AX25_PACKET *to_packet);
int print_packet(char *label, uint8_t *packet, int len);
int print_decoded_packet(char *label, AX25_PACKET *decoded);

int test_ax25_util_print_packet();
int test_ax25_util_decode_packet();

#endif /* UTILITIES_INC_AX25_UTIL_H_ */
