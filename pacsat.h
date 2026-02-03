/*
 * pacsat.h
 *
 *  Created on: Feb 26, 2019
 *      Author: burns
 *
 *      This file includes header files and contains overall definitions that are used throughout the rt-ihu code.
 *      Note that configuration information is in config.h, not here.
 *
 */

#ifndef PACSAT_H_
#define PACSAT_H_
#include <stdarg.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "printf.h" //Get the tiny printf stuff
#define TRUE true
#define FALSE false
#include "config.h"
#include "watchdogSupport.h"
#include "gpioDriver.h"

/* These are defined in  */
#define EXIT_SUCCESS 0
#define EXIT_FAILURE 1

/* GLOBAL VARIABLE DECLARATIONS - these are then defined in main.c or where they are first used
 * Declare the queues that are available to all tasks */
extern QueueHandle_t xRxPacketQueue; // This queue holds packets that have just been received and need to be processed by the Ax25Task / Data Link State Machine
extern QueueHandle_t xRxEventQueue; // This queue holds events that need to be processed by the Ax25Task / Data Link State Machine
extern QueueHandle_t xPbPacketQueue; // This holds packets that need to be processed by the Pacsat Broadcast task
extern QueueHandle_t xUplinkEventQueue; // This queue holds events/packets that need to be processed by the Uplink Task / State Machine
extern QueueHandle_t xTxPacketQueue; // This queue holds packets that are going to be transmitted
extern QueueHandle_t xIFrameQueue[NUM_RX_CHANNELS]; // Queue of frames to be transmitted


/* Let's skip stdio which has a lot of stuff and just define printf */
extern int printf(const char *fmt, ...);
extern int sprintf(char* str, const char *fmt, ...);

#define VOID ((void *)0)

/* Some temporary declarations to avoid errors till we remove the calls */
/*
 * For PacSat, these are just filler routines at first to allow it to compile.  We probably
 * don't need at least some of this
 */
bool CreateHeader(int x,void *y);
bool CreatePayload(int x, void *y);

/* Is the time in the current clock valid? */
extern bool time_valid;

extern bool monitorTxPackets, monitorRxPackets, monitorRSSI;


#endif /* PACSAT_H_ */
