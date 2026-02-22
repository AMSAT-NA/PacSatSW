/*
 * exp_interface.c
 *
 *  Created on: Feb 20, 2026
 *      Author: g0kla
 */

#include "pacsat.h"

void exp_can_handler(int canNum, int priority, int type, int msgid,
                  int dest, int src, uint8_t *data, unsigned int len)
{
    debug_print("EXP CAN Handler Received: canNum=%d priority=%d type=%d msgid=%d "
           "dest=%d src=%d len=%d\n",
           canNum, priority, type, msgid, dest, src, len);

    unsigned int i;
    for (i = 0; i < len; i++)
        debug_print(" %02x", data[i]);
    debug_print("\n");
}
