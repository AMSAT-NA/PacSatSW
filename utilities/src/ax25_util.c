 /*
 * ax25_util.c
 *
 *  Created on: Feb 20, 2023
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



/*
 *  Convert a call from the shifted ascii form used in an
 *  AX.25 packet.
 *
 *  based on code in pb.c by OZ6BL et al, which was released with the GNU GPL
 *
 */
#include "ctype.h"

#include "pacsat.h"
#include "config.h"
#include "ax25_util.h"

/**
 * Decode a callsign from AX25 format.
 * The callsign is returned as a null terminated string in call
 * The encoded callsign is passed in buffer c
 * Final call is set to true if this is the last call, as per the AX25 spec.
 * The AX25 command/response bit is stored in command if it is set.  This is only
 * set for the destination callsign, if it is the last call in the list.
 */

int decode_call_and_command(uint8_t *c, char *call, int *final_call,
                            int *command)
{
    unsigned char *ep = c + 6;
    int ct = 0;

    while (ct < 6) {
        if (((*c >> 1) & 127) == ' ')
            break;

        *call = (*c >> 1) & 127;
        call++;
        ct++;
        c++;
    }

    if ((*ep & 0x1E) != 0) {
        *call = '-';
        call++;
        call += sprintf(call, "%d", (int)(((*ep) >> 1) & 0x0F));
    }

    *call = '\0';

    if (*ep & 1) {
        *final_call = 1;
    } else {
        *final_call = 0;
    }

    // Command/Response Bit is bit 7
    *command = (bool)((*ep >> 7) & 0b1);

    return true;
}

int decode_call(uint8_t *c, char *call)
{
    int f;
    int command;

    return decode_call_and_command(c, call, &f, &command);
}

/**
 * Convert a callsign to AX25 format.
 * The callsign is a null terminated string in name
 * The encoded callsign is written into buf, which must have space allocated by the
 * calling process.
 * Final call is set to true if this is the last call, as per the AX25 spec.
 * The AX25 command/response bit is stored in the destination (to) callsign
 */
int encode_call(char *name, uint8_t *buf, int final_call, int command)
{
    int ct   = 0;
    int ssid = 0;
    const char *p = name;
    char c;

    while (ct < 6) {
        c = toupper(*p);

        if (c == '-' || c == '\0')
            break;

        if (!isalnum(c)) {
            debug_print("ax25_utils: invalid symbol in callsign '%s'\n", name);
            return false;
        }

        buf[ct] = c << 1;

        p++;
        ct++;
    }

    while (ct < 6) {
        buf[ct] = ' ' << 1;
        ct++;
    }

    if (*p != '\0') {
        p++; // skip the dash
        /* We don't have the sscanf function so do some ascii magic here instead to calc ssid */
        ssid = *p - 48; // 48 is the ascii value for 1
        p++;
        if (*p != '\0') {
            ssid = ssid * 10 + *p - 48;
        }
//        debug_print("ssid: %d\n",ssid);
        if (ssid < 0 || ssid > 15) {
            debug_print("ax25_utils: SSID must follow '-' and be numeric in the range 0-15 - '%s'\n", name);
            return false;
         }
    }

    /* SSID goes in bits 1-4 */
    buf[6] = (ssid << 1) & 0x1E;
    command = (command & 0b1) << 7;
    buf[6] = buf[6] | command;
    if (final_call)
        buf[6] = buf[6] | 0x01;
    return true;
}

/**
 * Given a packet and its length, decode it.  The caller must
 * allocate the structure
 *
 * If the packet is too short or there is any other error, then
 * 0 is retuned.
 *
 */
uint8_t ax25_decode_packet(uint8_t *packet, int len,
                           AX25_PACKET *decoded_packet)
{
    if (len < 15)
        return 0;

    int final_call = false;
    int destBit = false;
    int sourceBit = false;
    int repeatedBit = false;
    int offset = 14;
    int i;

    decoded_packet->pid = 0; // initialize to zero and set if the packet has a pid
    decoded_packet->NR = 0; // initialize to zero and set if the packet has NR
    decoded_packet->NS = 0; // initialize to zero and set if the packet has NS
    decoded_packet->PF = 0; // initialize to zero and set if the packet has PF
    decoded_packet->via_callsign[0] = 0;

    decode_call_and_command(&packet[0], decoded_packet->to_callsign,
                            &final_call, &destBit);
    decode_call_and_command(&packet[7], decoded_packet->from_callsign,
                            &final_call, &sourceBit);
    // Decode the command bit
    if (destBit != sourceBit) {
        // then we are in version 2.x
        if (destBit)
            decoded_packet->command = 1;
        else
            decoded_packet->command = 0;
    } else {
        decoded_packet->command = destBit; // because both bits are the same
    }

    if (!final_call) {
        if (len < 22)
            return 0;
        decode_call_and_command(&packet[14], decoded_packet->via_callsign,
                                &final_call, &repeatedBit);
        if (!final_call) {
            debug_print("ERR: ax25_decode_packet() Invalid packet. Only 1 via supported\n");
            return FALSE;
        }
        decoded_packet->via_h = packet[20] & 0x80;
        offset = 21;
    }
    decoded_packet->control = packet[offset];
    if ((decoded_packet->control & 0b1) == 0) {
        // bit 0 = 0 then it is an I-frame
        if (len < offset + 2) {
            debug_print("ERR: ax25_decode_packet() Not enough bytes for an I-frame\n");
            return FALSE;
        }
        if ((len - offset - 2) > AX25_MAX_INFO_BYTES_LEN) {
            debug_print("ERR: ax25_decode_packet() Too many bytes for an I-frame.  Data would overflow.\n");
            // TODO - per the AX25 spec, this sort of error should cause re-establishment of the data link if in connected mode.
            // We should return a negative number with an AX25 ERROR code if this fails then the state machine can process the error
            return FALSE;
        }
        decoded_packet->frame_type = TYPE_I;
        decoded_packet->NR = (decoded_packet->control >> 5) & 0b111;
        decoded_packet->NS = (decoded_packet->control >> 1) & 0b111;
        decoded_packet->PF = (decoded_packet->control >> 4) & 0b1;
        decoded_packet->pid = packet[offset+1];
        decoded_packet->data_len = len-offset-2;
        for (i=0; i<(decoded_packet->data_len); i++) {
            decoded_packet->data[i] = packet[offset+2+i];
        }
    } else if ((decoded_packet->control & 0b11) == 0b11) {
        // bit 0 and 1 are both 1 then its a U frame
        int u_type = decoded_packet->control & U_CONTROL_MASK;

        decoded_packet->PF = (decoded_packet->control >> 4) & 0b1;
        switch (u_type) {
            case BITS_U_SABME:
                decoded_packet->frame_type = TYPE_U_SABME;
                break;

            case BITS_U_SABM:
                decoded_packet->frame_type = TYPE_U_SABM;
                break;

            case BITS_U_DISCONNECT:
                decoded_packet->frame_type = TYPE_U_DISC;
                break;

            case BITS_U_DISCONNECT_MODE:
                decoded_packet->frame_type = TYPE_U_DM;
                break;

            case BITS_UA:
                decoded_packet->frame_type = TYPE_U_UA;
                break;

            case BITS_U_FRAME_REJECT:
                decoded_packet->frame_type = TYPE_U_FRMR;
                break;

            case BITS_UI:
                decoded_packet->frame_type = TYPE_U_UI;
                decoded_packet->pid = packet[offset+1];
                decoded_packet->data_len = len-offset-2;
                for (i=0; i<(decoded_packet->data_len); i++) {
                    decoded_packet->data[i] = packet[offset+2+i];
                }
                break;

            case BITS_U_EXCH_ID:
                decoded_packet->frame_type = TYPE_U_XID;
                break;

            case BITS_U_TEST:
                decoded_packet->frame_type = TYPE_U_TEST;
                break;

            default:
                debug_print("ERR: ax25_decode_packet() Invalid U frame type %0x with control byte: %0x\n", u_type, decoded_packet->control);
                return FALSE;
        }

    } else if ((decoded_packet->control & 0b11) == 0b01) {
        // bit 0 = 1 and bit 1 = 0 then its an S frame
        int s_type = decoded_packet->control & S_CONTROL_MASK;
        decoded_packet->NR = (decoded_packet->control >> 5) & 0b111;
        decoded_packet->PF = (decoded_packet->control >> 4) & 0b1;
        switch (s_type) {
            case BITS_S_RECEIVE_READY:
                decoded_packet->frame_type = TYPE_S_RR;
                break;

            case BITS_S_RECEIVE_NOT_READY:
                decoded_packet->frame_type = TYPE_S_RNR;
                break;

            case BITS_S_REJECT:
                decoded_packet->frame_type = TYPE_S_REJ;
                break;

            case BITS_S_SELECTIVE_REJECT:
                decoded_packet->frame_type = TYPE_S_SREJ;
                break;

            default:
                debug_print("ERR: ax25_decode_packet() Invalid S frame type\n");
                return FALSE;
        }
    } else {
        debug_print("ERR: ax25_decode_packet() Frame type not supported\n");
        return FALSE;
    }
    return TRUE;
}

void ax25_copy_packet(AX25_PACKET *packet, AX25_PACKET *to_packet)
{
    int i;

    to_packet->frame_type = packet->frame_type;
    for (i=0; i< MAX_CALLSIGN_LEN; i++)
        to_packet->to_callsign[i] = packet->to_callsign[i];
    for (i=0; i< MAX_CALLSIGN_LEN; i++)
        to_packet->from_callsign[i] = packet->from_callsign[i];
    for (i=0; i< MAX_CALLSIGN_LEN; i++)
        to_packet->via_callsign[i] = packet->to_callsign[i];
    to_packet->command = packet->command;
    to_packet->control = packet->control;
    to_packet->NR = packet->NR;
    to_packet->NS = packet->NS;
    to_packet->PF = packet->PF;
    to_packet->pid = packet->pid;
    to_packet->data_len = packet->data_len;
    if (packet->data_len != 0) {
    for (i=0; i < packet->data_len; i++)
        to_packet->data[i] = packet->data[i];
    }
}

///**
// * Given a packet, encode it into the encoded_packet buffer which has max_len
// * Return the length of the encoded packet
// *
// * If there is an error then 0 or FALSE is returned.
// *
// */
//uint8_t ax25_encode_packet(AX25_PACKET *packet, uint8_t *encoded_packet, int max_len) {
//    if (max_len > 255) {
//        debug_print("ERR: ax25_encode_packet: Packet max length is 255\n");
//        return FALSE;
//
//
//    }
//    return FALSE;
//}

char *frame_type_strings[] = {"I","RR","RNR","REJ","SREJ", "SABME", "SABM",
                              "DISC", "DM", "UA","FRMR","UI", "XID", "TEST" };

int print_packet(char *label, uint8_t *packet, int len)
{
    AX25_PACKET decoded;

    ax25_decode_packet(packet, len, &decoded);
    print_decoded_packet(label, &decoded);
    return true;
}

int print_decoded_packet(char *label, AX25_PACKET *decoded)
{
    int loc;
    char *command;

    if (decoded->command)
        command = "Cmd";
    else
        command = "Res";
    debug_print("%s- %s: %s>%s",label, frame_type_strings[decoded->frame_type],
                decoded->from_callsign, decoded->to_callsign);
    if (decoded->via_callsign[0] != 0) {
        debug_print(",%s", decoded->via_callsign);
        if (decoded->via_h)
            debug_print(":h");
    }
    debug_print(" %s pid:%0x pf:%d ",command, decoded->pid, decoded->PF);
    if (decoded->frame_type == TYPE_I) {
        debug_print("nr:%d ns:%d ",decoded->NR, decoded->NS);
    } else if (decoded->frame_type == TYPE_S_RR ||
               decoded->frame_type == TYPE_S_RNR ||
               decoded->frame_type == TYPE_S_REJ ||
               decoded->frame_type == TYPE_S_SREJ) {
        debug_print("nr:%d ",decoded->NR);
    }
    debug_print("|len:%d| ",decoded->data_len);
    for (loc=0; loc<decoded->data_len; loc++) {
        debug_print("%x ",decoded->data[loc]);
    }
    debug_print("\n");
    return true;
}

/**
 * Test routines follow
 */

int test_ax25_util_print_packet()
{
    debug_print("   TEST: AX25 PRINT PACKET\n");
//    char *callsign = "G0KLA";
//    unsigned char buffer[7];
//    encode_call(callsign, buffer, 1, 0);
//    int i = 0;
//    for (i=0; i < 7; i++) {
//        debug_print("%x", buffer[i]);
//    }
    debug_print("\n");

    return 0;
}

int test_ax25_util_decode_packet()
{
    int rc = TRUE;
    AX25_PACKET packet;
    uint8_t by[] = { 0xac, 0x8a, 0x64, 0xa8, 0x86, 0xa0, 0x78, 0x8e, 0x60,
                     0x96, 0x98, 0x82, 0x40, 0xe1, 0x11, 0xe1 };

    printf("##### TEST AX25 UTIL DECODE\n");
    rc = ax25_decode_packet(&by[0], sizeof(by), &packet);
    print_decoded_packet("TEST:", &packet);

    if (packet.PF != 1) {
        printf("** Mismatched PF != 1\n");
        return FALSE;
    }
    if (packet.command != 0) {
        printf("** Mismatched command != 0\n");
        return FALSE;
    }
    if (packet.frame_type != TYPE_S_RR) {
        printf("** Mismatched type != RR\n");
        return FALSE;
    }

    if (rc == TRUE)
          printf("##### TEST AX25 UTIL DECODE: success\n");
      else
          printf("##### TEST AX25 UTIL DECODE: fail\n");

      return rc;
}
