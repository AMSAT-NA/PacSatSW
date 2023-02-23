/*
 * ax25_util.c
 *
 *  Created on: Feb 20, 2023
 *      Author: g0kla
 */



/*
 *  Convert a call from the shifted ascii form used in an
 *  AX.25 packet.
 *
 *  based on code in pb.c by OZ6BL et al, which was released with the GNU GPL
 *
 */

#include "pacsat.h"

int decode_call(uint8_t *c, char *call) {
    unsigned char *ep = c + 6;
    int ct = 0;

    while (ct < 6) {
        if (((*c >> 1) & 127) == ' ') break;

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

    if (*ep & 1) return 0;

    return 1;
}

/**
 * Convert a callsign to AX25 format
 * TODO - this was pasted in but has not been refactored and tested yet
 */
int encode_call(char *name, unsigned char *buf, int final_call, char command) {
    int ct   = 0;
    int ssid = 0;
    const char *p = name;
    char c;

    while (ct < 6) {
        c = toupper(*p);

        if (c == '-' || c == '\0')
            break;

        if (!isalnum(c)) {
//            debug_print("ax25_utils: invalid symbol in callsign '%s'\n", name);
            return 1;
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
        p++;

//        if (sscanf(p, "%d", &ssid) != 1 || ssid < 0 || ssid > 15) {
//            debug_print("ax25_utils: SSID must follow '-' and be numeric in the range 0-15 - '%s'\n", name);
//            return EXIT_FAILURE;
//        }
    }

    buf[6] = ((ssid + '0') << 1) & 0x1E;
    command = (command & 0b1) << 7;
    buf[6] = buf[6] | command;
    if (final_call)
        buf[6] = buf[6] | 0x01;
    return 0;
}

int test_ax25_util_decode_calls() {
    debug_print("   TEST: AX25 UTIL DECODE CALLS\n");
//    char callsign[7] = "G0KLA";
//    unsigned char buffer[7];
//    encode_call(callsign, buffer, 1, 0);
//    int i = 0;
//    for (i=0; i < 7; i++) {
//        debug_print("%x", buffer[i]);
//    }
    debug_print("\n");

    return 0;
}
