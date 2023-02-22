/*
 * TncTask.c
 *
 *  Created on: Dec 1, 2022
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
 * =====================================================================
 *
 * The PB handles all broadcasts to the ground and processes all requests
 * for broadcasts.  The PB list holds the list of stations that have
 * requested broadcasts.

ALL ABOUT THE BROADCASTs
========================

The server maintains a queue with 10 entries; each entry is a hole list
request or broadcast start request. A particular station (as identified by
callsign, not including SSID) can have only one entry on the queue.

Entries are removed from the queue:
           after 10 minutes;
           after a hole list has been completely transmitted (for hole list);
           after a file has been completely transmitted (for start request);
           when a new request is received from a station already in the queue;
           if the file associated with the entry cannot be opened and read;

On a periodic basis we broadcast the PB Status with a UI packet from the BBS callsign
to one of the following callsigns: PBLIST, PBFULL, PBSHUT, PBSTAT

* PBLIST - This is used when the list is empty or when there are a list of callsins
  on the PB
* PBFULL - When there is no more room on the PB this callsign is used
* PBSHUT - If the broadcast protocol is not available then this callsign is used
* PBSTAT - TBD

A DIR or FILE request is sent from a ground station to the BROADCAST callsign of the
spacecraft with a PID 0xBD for a dir request  or 0xBB for a file reqest.

When a request is received we send OK <callsign> from the bbs callsign directly to the
callsign that sent the request with a PID of 0xF0 and the text OK <callsign>.  It looks
like it is terminated with a 0x0D linefeed, but that needs some more research to see if
it is required.

If there is an error in the request then we send NO -X where X is the error number.  It
is sent from the bbs callsign with a PID of 0xF0.

A DIR or FILE request is added to the PB assuming we have space on the PB list

If the File Number is not available for a file request then we send an error
packet: NO
 */

#include <PbTask.h>
#include "FreeRTOS.h"
#include "os_task.h"
#include "ax25_util.h"
#include "str_util.h"

static uint8_t pb_packet_buffer[AX25_PKT_BUFFER_LEN];
/* An entry on the PB list keeps track of the requester and where we are in the request process */
struct pb_entry {
    uint8_t pb_type; /* DIR or FILE request */
    char callsign[MAX_CALLSIGN_LEN];
    //DIR_NODE *node; /* Pointer to the node that we should broadcast next if this is a DIR request */
    uint32_t file_id; /* File id of the file we are broadcasting if this is a file request */
    uint32_t offset; /* The current offset in the file we are broadcasting or the PFH we are transmitting */
    uint8_t block_size; /* The maximum size of broadcasts. THIS IS CURRENTLY IGNORED but in theory is sent from the ground for file requests */
    void *hole_list; /* This is a DIR or FILE hole list */
    uint8_t hole_num; /* The number of holes from the request */
    uint8_t current_hole_num; /* The next hole number from the request that we should process when this one is done */
    uint32_t request_time; /* The time the request was received for timeout purposes */
};
typedef struct pb_entry PB_ENTRY;

/**
 * pb_list
 * This Directory and File broadcast list is a list of callsigns that will receive attention from
 * the PACSAT.  It stores the callsign and the request, which is for a file or a directory.
 * This is a static block of memory that exists throughout the duration of the program to
 * keep track of stations on the PB.
 */
static PB_ENTRY pb_list[MAX_PB_LENGTH];

static uint8_t number_on_pb = 0; /* This keeps track of how many stations are in the pb_list array */
static uint8_t current_station_on_pb = 0; /* This keeps track of which station we will send data to next */
static bool pb_shut = false;


/* Local Function prototypes */
void PbSendStatus();
void PbMakeListStr(char *buffer, int len);

bool PbTestStatus();

/**
 * The PB task monitors the PB Packet Queue and processes received packets.  It keeps track of stations
 * on the PB and sends them directory or file broadcast packets
 *
 */
portTASK_FUNCTION_PROTO(PbTask, pvParameters)  {

    ResetAllWatchdogs();
    debug_print("Initializing PB Task\n");

#ifdef DEBUG
    bool rc = PbTestStatus();     if (rc == FALSE) { debug_print("FAILED SELF TEST: PbTestStatus\n"); }

#endif

    /* Setup a timer to send the status periodically */
    xTimerHandle handle;
    volatile portBASE_TYPE timerStatus;
    int pvtID = 0;

    /* create a RTOS software timer - TODO period should be in MRAM and changeable from the ground after reset of task */
    handle = xTimerCreate( "PB", SECONDS(10), TRUE, &pvtID, PbSendStatus);
    /* start the timer */
    timerStatus = xTimerStart(handle, 0);
    if (timerStatus != pdPASS){
        debug_print("ERROR: Failed in init PB Status Timer\n");
// TODO =>        ReportError(RTOSfailure, FALSE, ReturnAddr, (int) PbTask); /* failed to create the RTOS timer */
    }

    while(1) {

        ReportToWatchdog(CurrentTaskWD);
        BaseType_t xStatus = xQueueReceive( xPbPacketQueue, &pb_packet_buffer, CENTISECONDS(10) );  // TODO - adjust block time vs watchdog
        if( xStatus == pdPASS ) {
            /* Data was successfully received from the queue */
            char from_callsign[MAX_CALLSIGN_LEN];
            char to_callsign[MAX_CALLSIGN_LEN];

            decode_call(&pb_packet_buffer[8], from_callsign);
            decode_call(&pb_packet_buffer[1], to_callsign);
            debug_print("PB: %s>%s:\n",from_callsign, to_callsign);
        }
        ReportToWatchdog(CurrentTaskWD);

    }
}

/**
 * PbSendStatus()
 *
 * This is called from an RTOS timer to send the status periodically
 * Puts a packet with the current status of the PB into the TxQueue
 *
 * Returns void to be compatible with timer callbacks
 *
 */
void PbSendStatus() {
    debug_print("PB Status being sent...\n");

    if (pb_shut) {
        char shut[] = "PB Closed.";
//////////        int rc = send_raw_packet(g_broadcast_callsign, PBSHUT, PID_NO_PROTOCOL, shut, sizeof(shut));
        return;
    } else  {
        char buffer[25];
        char * CALL = PBLIST;
        if (number_on_pb == MAX_PB_LENGTH) {
            CALL = PBFULL;
        }
        PbMakeListStr(buffer, sizeof(buffer));
        int len = strlen(buffer);
 //       char command[len]; // now put the list in a buffer of the right size
 //       strlcpy((char *)command, (char *)buffer,sizeof(command));
        debug_print("%s %s\n",CALL, buffer);
        //int rc = send_raw_packet(g_broadcast_callsign, CALL, PID_NO_PROTOCOL, command, sizeof(command));
        return;
    }
}

/**
 * pb_make_list_str()
 *
 * Build the status string that is periodically transmitted.
 * The *buffer to receive the string and its length len should be passed in.
 */
void PbMakeListStr(char *buffer, int len) {
    if (number_on_pb == 0)
        strlcpy(buffer, "PB Empty.", len);
    else
        strlcpy(buffer, "PB ", len);
    int i;
    for (i=0; i < number_on_pb; i++) {
            strlcat(buffer, pb_list[i].callsign, len);
        if (pb_list[i].pb_type == PB_DIR_REQUEST_TYPE)
            strlcat(buffer, "/D ", len);
        else
            strlcat(buffer, " ", len);
    }
}

bool PbTestStatus() {
    PbSendStatus();
    return true;
}
