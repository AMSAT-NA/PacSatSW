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
#include "ctype.h"

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
 * Convert a callsign to AX25 format.
 * The callsign is a null terminated strin in name
 * The encoded callsign is written into buf, which must have space allocated by the
 * calling process.
 * Final call is set to true if this is the last call, as per the AX25 spec.
 * The AX25 command/response bit is stored in the destination (to) callsign
 */
int encode_call(char *name, uint8_t *buf, int final_call, uint8_t command) {
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
        debug_print("ssid: %d\n",ssid);
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

int test_ax25_util_decode_calls() {
    debug_print("   TEST: AX25 UTIL DECODE CALLS\n");
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
