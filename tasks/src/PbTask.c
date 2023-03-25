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

#include <strings.h> // TODO - do we really need this?  How much space does it use.

#include "pacsat.h"
#include "FreeRTOS.h"
#include "os_task.h"
#include "MET.h"
#include "inet.h"
#include "ax25_util.h"
#include "str_util.h"
#include "PbTask.h"
#include "TxTask.h"
#include "pacsat_dir.h"
#include "crc16.h"

static uint8_t data_bytes[AX25_MAX_DATA_LEN]; /* Static buffer used to store file bytes loaded from MRAM */
static uint8_t pb_packet_buffer[AX25_PKT_BUFFER_LEN]; /* Static buffer used to store packet as it is assembled and before copy to TX queue */
static char pb_status_buffer[135]; // 10 callsigns * 13 bytes + 4 + nul

/* An entry on the PB list keeps track of the requester and where we are in the request process */
struct pb_entry {
    uint8_t pb_type; /* DIR or FILE request */
    char callsign[MAX_CALLSIGN_LEN];
    DIR_NODE *node; /* The node that we should broadcast next if this is a DIR request.  Physically stored in the DIR linked list */
    uint32_t offset; /* The current offset in the file we are broadcasting or the PFH we are transmitting */
    uint8_t block_size; /* The maximum size of broadcasts. THIS IS CURRENTLY IGNORED but in theory is sent from the ground for file requests */
    uint8_t hole_list[MAX_PB_HOLES_LIST_BYTES]; /* This is a DIR or FILE hole list */
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
extern bool pb_shut;


/* Local Function prototypes */
int pb_send_ok(char *from_callsign);
int pb_send_err(char *from_callsign, int err);
void pb_send_status();
void pb_make_list_str(char *buffer, int len);
void pb_debug_print_list();
void pb_debug_print_list_item(int i);
int pb_add_request(char *from_callsign, int type, DIR_NODE * node, int file_id, int offset, void *holes, int num_of_holes);
int pb_remove_request(int pos);
void pb_process_frame(char *from_callsign, char *to_callsign, uint8_t *data, int len);
int get_num_of_dir_holes(int request_len);
int pb_handle_dir_request(char *from_callsign, unsigned char *data, int len);
void pb_debug_print_dir_holes(DIR_DATE_PAIR *holes, int num_of_holes);
void pb_debug_print_file_holes(FILE_DATE_PAIR *holes, int num_of_holes);
int pb_next_action();
int pb_make_dir_broadcast_packet(DIR_NODE *node, uint8_t *data_bytes, uint32_t *offset);

/**
 * The PB task monitors the PB Packet Queue and processes received packets.  It keeps track of stations
 * on the PB and sends them directory or file broadcast packets
 *
 */
portTASK_FUNCTION_PROTO(PbTask, pvParameters)  {

    ResetAllWatchdogs();
    debug_print("Initializing PB Task\n");

    /* Setup a timer to send the status periodically */
    xTimerHandle pbStatusTimerHandle;
    volatile portBASE_TYPE timerStatus;
    int pvtPbStatusTimerID = 0; // timer id

    /* create a RTOS software timer - TODO period should be in MRAM and changeable from the ground using xTimerChangePeriod() */
    pbStatusTimerHandle = xTimerCreate( "PB STATUS", SECONDS(10), TRUE, &pvtPbStatusTimerID, pb_send_status); // auto reload timer
    /* start the timer */
//    timerStatus = xTimerStart(pbStatusTimerHandle, 0); // Block time of zero as this can not block
    if (timerStatus != pdPASS) {
        debug_print("ERROR: Failed in init PB Status Timer\n");
// TODO =>        ReportError(RTOSfailure, FALSE, ReturnAddr, (int) PbTask); /* failed to create the RTOS timer */
        // TODO - it's possible this might fail.  Somehow we should recover from that.
    }

    while(1) {

        ReportToWatchdog(CurrentTaskWD);
        BaseType_t xStatus = xQueueReceive( xPbPacketQueue, &pb_packet_buffer, 0 );  // Don't block, we have a delay after this
        if( xStatus == pdPASS ) {
            /* Data was successfully received from the queue */
            char from_callsign[MAX_CALLSIGN_LEN];
            char to_callsign[MAX_CALLSIGN_LEN];

            decode_call(&pb_packet_buffer[8], from_callsign);
            decode_call(&pb_packet_buffer[1], to_callsign);
            debug_print("PB: %s>%s: Len: %d\n",from_callsign, to_callsign, pb_packet_buffer[0]);
            pb_process_frame(from_callsign, to_callsign, &pb_packet_buffer[0], pb_packet_buffer[0]);
            //pb_send_ok(from_callsign);
        }
        /* Yield some time so the OK or ERR is sent, or so that others may do some processing */
        vTaskDelay(CENTISECONDS(1));
        ReportToWatchdog(CurrentTaskWD);

        /* Now process the next station on the PB if there is one and take its action */
        if (number_on_pb != 0) {
            pb_next_action();
        }

    }
}

/**
 * pb_send_ok()
 *
 * Send a UI frame from the broadcast callsign to the station with PID BB and the
 * text OK <callsign>0x0Drequest_list
 */
int pb_send_ok(char *from_callsign) {
    int rc = true;
    char buffer[3 + MAX_CALLSIGN_LEN]; // OK + 10 char for callsign with SSID
    strlcpy(buffer,"OK ", sizeof(buffer));
    strlcat(buffer, from_callsign, sizeof(buffer));
    int len = 3 + strlen(from_callsign);
    buffer[len] = 0x0D; // this replaces the string termination
    rc = tx_send_packet(BROADCAST_CALLSIGN, from_callsign, PID_FILE, (uint8_t *)buffer, len, BLOCK_IF_QUEUE_FULL);

    return rc;
}

/**
 * pb_send_err()
 *
 * Send a UI frame to the station containing an error response.  The error values are defined in
 * the header file
 *
 * returns TRUE unless it is unable to send the data to the TX Radio Queue
 *
 */
int pb_send_err(char *from_callsign, int err) {
    int rc = TRUE;
    char err_str[2];
    snprintf(err_str, 3, "%d",err);
    char buffer[25];
    int len = 6 + strlen(err_str)+ strlen(from_callsign); // NO -XX + 10 char for callsign with SSID
    char CR = 0x0d;
    strlcpy(buffer,"NO -", sizeof(buffer));
    strlcat(buffer, err_str, sizeof(buffer));
    strlcat(buffer," ", sizeof(buffer));
    strlcat(buffer, from_callsign, sizeof(buffer));
    strncat(buffer,&CR,1); // very specifically add just one char to the end of the string for the CR
    rc = tx_send_packet(BROADCAST_CALLSIGN, from_callsign, PID_FILE, (uint8_t *)buffer, len, BLOCK_IF_QUEUE_FULL);

    return rc;
}


/**
 * pb_send_status()
 *
 * This is called from an RTOS timer to send the status periodically
 * Puts a packet with the current status of the PB into the TxQueue
 *
 * Returns void to be compatible with timer callbacks
 *
 * NOTE that pb_status_buffer is declared static because allocating a buffer of this
 * size causes a crash when this is called from a timer.
 *
 * We MUST NOT BLOCK because this can be called from a timer.  If the TX queue is full then we skip sending status
 *
 */
void pb_send_status() {
    ReportToWatchdog(CurrentTaskWD);
    //debug_print("PB Status being sent...\n");

    if (pb_shut) {
        char shut[] = "PB Closed.";
        int rc = tx_send_packet(BROADCAST_CALLSIGN, PBSHUT, PID_NO_PROTOCOL, (uint8_t *)shut, strlen(shut), DONT_BLOCK_IF_QUEUE_FULL);
        debug_print("SENDING: %s |%s|\n",PBSHUT, shut);
        ReportToWatchdog(CurrentTaskWD);
        return;
    } else  {

        char * CALL = PBLIST;
        if (number_on_pb == MAX_PB_LENGTH) {
            CALL = PBFULL;
        }
        pb_make_list_str(pb_status_buffer, sizeof(pb_status_buffer));
//        uint8_t buffer[] = "PB Empty.";
        uint8_t len = strlen((char *)pb_status_buffer);
//        debug_print("SENDING: %s |%s|\n",CALL, pb_status_buffer);

       int rc = tx_send_packet(BROADCAST_CALLSIGN, CALL, PID_NO_PROTOCOL, (uint8_t *)pb_status_buffer, len, DONT_BLOCK_IF_QUEUE_FULL);
        ReportToWatchdog(CurrentTaskWD);
        return;
    }
}


/**
 * pb_make_list_str()
 *
 * Build the status string that is periodically transmitted.
 * The *buffer to receive the string and its length len should be passed in.
 */
void pb_make_list_str(char *buffer, int len) {
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

void pb_debug_print_list() {
    char buffer[256];
    pb_make_list_str(buffer, sizeof(buffer));
    debug_print("%s\n",buffer);
    int i;
    for (i=0; i < number_on_pb; i++) {
        pb_debug_print_list_item(i);
    }
}

void pb_debug_print_list_item(int i) {
    debug_print("--%s Ty:%d ",pb_list[i].callsign,pb_list[i].pb_type);
    if (pb_list[i].node != NULL)
        debug_print("File: %04x ",pb_list[i].node->mram_file->file_id);
    else
        debug_print("File: NULL ");
    debug_print("Off:%d Holes:%d Cur:%d",pb_list[i].offset,pb_list[i].hole_num,pb_list[i].current_hole_num);
//    char buf[30];
//    time_t now = pb_list[i].request_time;
//    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", gmtime(&now));
    debug_print(" at:%d", pb_list[i].request_time);

    if (pb_list[i].pb_type == PB_DIR_REQUEST_TYPE)
        pb_debug_print_dir_holes((DIR_DATE_PAIR *)pb_list[i].hole_list, pb_list[i].hole_num);
    else
        pb_debug_print_file_holes((FILE_DATE_PAIR *)pb_list[i].hole_list, pb_list[i].hole_num);
}


/**
 * pb_add_request()
 *
 * Add a callsign and its request to the PB
 *
 * Make a copy of all the data because the original packet will be purged soon from the
 * circular buffer
 * Note that when we are adding an item the variable number_on_pb is pointing to the
 * empty slot where we want to insert data because the number is one greater than the
 * array index (which starts at 0)
 *
 * returns TRUE it it succeeds or FAIL if the PB is shut or full
 *
 */
int pb_add_request(char *from_callsign, int type, DIR_NODE * node, int file_id, int offset, void *holes, int num_of_holes) {
    debug_print("PB: Request from %s ", from_callsign);
    if (pb_shut) return FALSE;
    if (number_on_pb == MAX_PB_LENGTH) {
        return FALSE; // PB full
    }

    /* Each station can only be on the PB once, so reject if the callsign is already in the list */
    int i;
    for (i=0; i < number_on_pb; i++) {
        if ((strcmp(pb_list[i].callsign, from_callsign) == 0)) {
            debug_print(" .. already on PB. Ignored\n");
            return FALSE; // Station is already on the PB
        }
    }

    strlcpy(pb_list[number_on_pb].callsign, from_callsign, MAX_CALLSIGN_LEN);
    pb_list[number_on_pb].pb_type = type;
    pb_list[number_on_pb].offset = offset;
    //logicalTime_t time;
    //getTime(&time);
    uint32_t secs = getSeconds();
    debug_print(" at time: %i ",secs);
    pb_list[number_on_pb].request_time = secs;
    pb_list[number_on_pb].hole_num = num_of_holes;
    pb_list[number_on_pb].current_hole_num = 0;
    pb_list[number_on_pb].node = node;
//    if (node == NULL) {
//        pb_list[number_on_pb].node.file_id = file_id;
//        pb_list[number_on_pb].node.file_size = 0;
//        pb_list[number_on_pb].node.upload_time = 0;
//        pb_list[number_on_pb].node.address = 0;
//        pb_list[number_on_pb].node.body_offset = 0;
//    } else {
//        pb_list[number_on_pb].node.file_id = node->file_id;
//        pb_list[number_on_pb].node.file_size = node->file_size;
//        pb_list[number_on_pb].node.upload_time = node->upload_time;
//        pb_list[number_on_pb].node.address = node->address;
//        pb_list[number_on_pb].node.body_offset = node->body_offset;
//    }

    if (num_of_holes > 0) {
        if (type == PB_DIR_REQUEST_TYPE) {
            DIR_DATE_PAIR *dir_holes = (DIR_DATE_PAIR *)holes;
            DIR_DATE_PAIR *dir_hole_list = (DIR_DATE_PAIR *) pb_list[number_on_pb].hole_list;
            int i;
            for (i=0; i<num_of_holes; i++) {
                dir_hole_list[i].start = dir_holes[i].start;
                dir_hole_list[i].end = dir_holes[i].end;
            }
        } else {
            FILE_DATE_PAIR *file_holes = (FILE_DATE_PAIR *)holes;
            FILE_DATE_PAIR *file_hole_list = (FILE_DATE_PAIR *)pb_list[number_on_pb].hole_list;
            int i;
            for (i=0; i<num_of_holes; i++) {
                file_hole_list[i].offset = file_holes[i].offset;
                file_hole_list[i].length = file_holes[i].length;
            }
        }
    }
    number_on_pb++;
    debug_print(" .. Added\n");
    return TRUE;
}

/**
 * pb_remove_request()
 *
 * Remove the callsign at the designated position.  This is most likely the
 * head because we finished a request.
 *
 * note that the variable number_on_pb is one greater than the last array position
 * with data.
 *
 * Returns TRUE if it can be removed or FALSE if there was
 * no such item
 *
 */
int pb_remove_request(int pos) {
    if (number_on_pb == 0) return FALSE;
    if (pos >= number_on_pb) return FALSE;
    uint32_t secs = getSeconds();
    debug_print("PB: Removed %s at time %i\n",pb_list[pos].callsign, secs);
    if (pos != number_on_pb-1) {

        /* Remove the item and shuffle all the other items to the left */
        int i;
        for (i = pos + 1; i < number_on_pb; i++) {
            strlcpy(pb_list[i-1].callsign, pb_list[i].callsign, MAX_CALLSIGN_LEN);
            pb_list[i-1].pb_type = pb_list[i].pb_type;
            pb_list[i-1].offset = pb_list[i].offset;
            pb_list[i-1].request_time = pb_list[i].request_time;
            if (pb_list[i].hole_num > 0) {
                int j;
                for (j=0; j<MAX_PB_HOLES_LIST_BYTES; j++)  // TODO - we can shorten this based on number of holes and how long FILE or DIR hole is
                    pb_list[i-1].hole_list[j] = pb_list[i].hole_list[j];
            }
            pb_list[i-1].hole_num = pb_list[i].hole_num;
            pb_list[i-1].node = pb_list[i].node;
//            pb_list[i-1].node.file_id = pb_list[i].node.file_id;
//            pb_list[i-1].node.file_size = pb_list[i].node.file_size;
//            pb_list[i-1].node.upload_time = pb_list[i].node.upload_time;
//            pb_list[i-1].node.address = pb_list[i].node.address;
//            pb_list[i-1].node.body_offset = pb_list[i].node.body_offset;
            pb_list[i-1].current_hole_num = pb_list[i].current_hole_num;
        }
    }

    number_on_pb--;

    /* We have to update the station we will next send data to.
     * If a station earlier in the list was removed, then this decrements by one.
     * If a station later in the list was remove we do nothing.
     * If the current station was removed then we do nothing because we are already
     * pointing to the next station, unless we are at the end of the list */
    if (pos < current_station_on_pb) {
        current_station_on_pb--;
        if (current_station_on_pb <= 0)
            current_station_on_pb = 0;
    } else if (pos == current_station_on_pb) {
        if (current_station_on_pb >= number_on_pb)
            current_station_on_pb = 0;
    }
    return TRUE;
}

/**
 * pb_clear_list()
 *
 * Remove all entries on the pb list
 */
int pb_clear_list() {
    if (number_on_pb == 0) return TRUE;
    int i, rc;
    int num = number_on_pb;
    for (i=0; i < num; i++) {
        rc = pb_remove_request(number_on_pb - i - 1); // remove from end so we minimize copying of data
        if (rc != TRUE) return FALSE;
    }
    return TRUE;
}

/**
 * pb_process_frame()
 *
 * process a UI frame received from a ground station.  This may contain a Pacsat Broadcast request,
 * otherwise it can be ignored.
 * This is called from the main processing loop whenever a frame is received.
 *
 */
void pb_process_frame(char *from_callsign, char *to_callsign, uint8_t *data, int len) {
    if (strcasecmp(to_callsign, BBS_CALLSIGN) == 0) {
        // this was sent to the BBS Callsign and we can ignore it
        debug_print("BBS Request - Ignored\n");
    } else if (strcasecmp(to_callsign, BROADCAST_CALLSIGN) == 0) {
        // this was sent to the Broadcast Callsign

        struct t_broadcast_request_header *broadcast_request_header;
        broadcast_request_header = (struct t_broadcast_request_header *)data;
        debug_print("Broadcast Request: pid: %02x \n", broadcast_request_header->pid & 0xff);
        if ((broadcast_request_header->pid & 0xff) == PID_DIRECTORY) {
            pb_handle_dir_request(from_callsign, data, len);
        }
        if ((broadcast_request_header->pid & 0xff) == PID_FILE) {
            // File Request
            //pb_handle_file_request(from_callsign, data, len);
        }
    } else {
        debug_print("PB: Packet Ignored\n");
    }
}


int get_num_of_dir_holes(int request_len) {
    int num_of_holes = (request_len - sizeof(AX25_HEADER) - sizeof(DIR_REQ_HEADER)) / sizeof(DIR_DATE_PAIR);
    return num_of_holes;
}

DIR_DATE_PAIR * get_dir_holes_list(unsigned char *data) {
    DIR_DATE_PAIR *holes = (DIR_DATE_PAIR *)(data + sizeof(AX25_HEADER) + sizeof(DIR_REQ_HEADER) );
    return holes;
}

void pb_debug_print_dir_holes(DIR_DATE_PAIR *holes, int num_of_holes) {
    debug_print(" - %d holes: ",num_of_holes);
    int i;
    for (i=0; i< num_of_holes; i++) {
        debug_print("%x - %x, ",holes[i].start, holes[i].end);
//        char buf[30];
//        time_t now = holes[i].start;
//        strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", gmtime(&now));
//        debug_print("%s,", buf);
//        now = holes[i].end;
//        strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", gmtime(&now));
//        debug_print("%s ", buf);
    }
    debug_print("\n");
}

void pb_debug_print_file_holes(FILE_DATE_PAIR *holes, int num_of_holes) {
    debug_print(" - %d holes: ",num_of_holes);
    int i;
    for (i=0; i< num_of_holes; i++) {
        debug_print("%d,%d ", holes[i].offset, holes[i].length);
    }
    debug_print("\n");
}

void pb_debug_print_dir_req(unsigned char *data, int len) {
    DIR_REQ_HEADER *dir_header;
    dir_header = (DIR_REQ_HEADER *)(data + sizeof(AX25_HEADER));
    debug_print("DIR REQ: flags: %02x BLK_SIZE: %04x ", dir_header->flags & 0xff, dir_header->block_size &0xffff);
    if ((dir_header->flags & 0b11) == 0b00) {
        /* There is a holes list */
        int num_of_holes = get_num_of_dir_holes(len);
        if (num_of_holes == 0)
            debug_print("- missing hole list\n");
        else {
            DIR_DATE_PAIR *holes = get_dir_holes_list(data); //(DIR_DATE_PAIR *)(data + BROADCAST_REQUEST_HEADER_SIZE + DIR_REQUEST_HEADER_SIZE );
            pb_debug_print_dir_holes(holes, num_of_holes);
        }
    }
}

/**
 * pb_handle_dir_request()
 *
 * Process a dir request from a ground station
 *
 * Returns TRUE if the request could be processed, even if the
 * station was not added to the PB.  Only returns FALSE if there is
 * an unexpected error, such as the TX Radio Queue is unavailable.
 */
int pb_handle_dir_request(char *from_callsign, unsigned char *data, int len) {
    // Dir Request
    int rc=TRUE;
    DIR_REQ_HEADER *dir_header;
    dir_header = (DIR_REQ_HEADER *)(data + sizeof(AX25_HEADER));

    // TODO - we do not check bit 5, which must be 1 or the version bits, which must be 00.

    /* least sig 2 bits of flags are 00 if this is a fill request */
    if ((dir_header->flags & 0b11) == 0b00) {
#ifdef DEBUG
        pb_debug_print_dir_req(data, len);
#endif
        debug_print("DIR FILL REQUEST: flags: %02x BLK_SIZE: %04x\n", dir_header->flags & 0xff, dir_header->block_size &0xffff);

        /* Get the number of holes in this request and make sure it is in a valid range */
        int num_of_holes = get_num_of_dir_holes(len);
        if (num_of_holes < 1 || num_of_holes > AX25_MAX_DATA_LEN / sizeof(DIR_DATE_PAIR)) {
            /* This does not have a valid holes list */
            rc = pb_send_err(from_callsign, PB_ERR_FILE_INVALID_PACKET);
            if (rc != TRUE) {
                debug_print("\n Error : Could not send ERR Response to TNC \n");
                return FALSE;
            }
            return TRUE;
        }
        /* Add to the PB if we can*/
        DIR_DATE_PAIR * holes = get_dir_holes_list(data);
        if (pb_add_request(from_callsign, PB_DIR_REQUEST_TYPE, NULL, 0, 0, holes, num_of_holes) == TRUE) {
            // ACK the station
            rc = pb_send_ok(from_callsign);
            if (rc != TRUE) {
                debug_print("\n Error : Could not send OK Response to TNC \n");
                return FALSE;
            }
        } else {
            // the protocol says NO -1 means temporary problem. e.g. shut or you are already on the PB, and -2 means permanent
            rc = pb_send_err(from_callsign, PB_ERR_TEMPORARY);
            if (rc != TRUE) {
                debug_print("\n Error : Could not send ERR Response to TNC \n");
                return FALSE;
            }
        }
    } else {
        /* There are no other valid DIR Requests other than a fill */
        rc = pb_send_err(from_callsign, PB_ERR_FILE_INVALID_PACKET);
        if (rc != TRUE) {
            debug_print("\n Error : Could not send ERR Response to TNC \n");
            return FALSE;
        }
    }
    return rc;
}

/**
 * pb_next_action()
 *
 * When called take the next action for next station on the PB
 *
 * Returns TRUE, even if we can not process the request.  Only returns
 * FALSE if something goes badly wrong, such as we can not send data to
 * the TNC
 *
 */
int pb_next_action() {
    int rc = TRUE;

    //TODO - broadcast the number of bytes transmitted to BSTAT periodically so stations can calc efficiency

    //TODO - DIR maintenance.  Daily scan the dir for corrupt or expired files and delete them.  This could run slowly, 1 file scan per pb_action.  But what if we remove a file we are broadcasting?

    uint32_t now = getSeconds();
    if ((now - pb_list[current_station_on_pb].request_time) > PB_MAX_PERIOD_FOR_CLIENTS_IN_SECONDS) {
        /* This station has exceeded the time allowed on the PB */
        pb_remove_request(current_station_on_pb);
        /* If we removed a station then we don't want/need to increment the current station pointer */
        return TRUE;
    }

    if (uxQueueMessagesWaiting(xTxPacketQueue) > MAX_PKTS_IN_TX_PKT_QUEUE_FOR_TNC_TO_BE_BUSY) return TRUE; /* TNC is Busy */

    /**
     *  Process Request to broadcast directory
     */
    if (pb_list[current_station_on_pb].pb_type == PB_DIR_REQUEST_TYPE) {

        debug_print("Preparing DIR Broadcast for %s\n",pb_list[current_station_on_pb].callsign);
        if (pb_list[current_station_on_pb].hole_num < 1) {
            /* This is not a valid DIR Request.  There is no hole list.  We should not get here because this
             * should not have been added.  So just remove it. */
            debug_print("Invalid DIR request with no hole list from %s\n", pb_list[current_station_on_pb].callsign);
            pb_remove_request(current_station_on_pb);
            /* If we removed a station then we don't want/need to increment the current station pointer */
            return TRUE;
        }

        int current_hole_num = pb_list[current_station_on_pb].current_hole_num;
        DIR_DATE_PAIR *holes = (DIR_DATE_PAIR *)pb_list[current_station_on_pb].hole_list;
        DIR_NODE *node = dir_get_pfh_by_date(holes[current_hole_num], pb_list[current_station_on_pb].node);
        if (node == NULL) {
            /* We have finished the broadcasts for this hole, or there were no records for the hole, move to the next hole if there is one. */
            // TODO - need to test this logic.  If we list 10 holes with no data and then 1 hole with data, then do we miss our turn on the PB 10 times?
            pb_list[current_station_on_pb].current_hole_num++; /* Increment now.  If the data is bad and we can't make a frame, we want to move on to the next */
            if (pb_list[current_station_on_pb].current_hole_num == pb_list[current_station_on_pb].hole_num) {
                /* We have finished this hole list */
                debug_print("Added last hole for request from %s\n", pb_list[current_station_on_pb].callsign);
                pb_remove_request(current_station_on_pb);
                /* If we removed a station then we don't want/need to increment the current station pointer */
                return TRUE;
            }

            // TODO - What response if there were no PFHs at all for the request?  An error? Or do nothing
            // IS THIS NO -5???

        }
        else {
            /* We found a dir header */

            debug_print("DIR BD Offset %d: ", pb_list[current_station_on_pb].offset);

            /* Store the offset and pass it into the function that makes the broadcast packet.  The offset after
             * the broadcast is returned in this offset variable.  It equals the length of the PFH if the whole header
             * has been broadcast. */
            uint32_t offset = pb_list[current_station_on_pb].offset;
            int data_len = pb_make_dir_broadcast_packet(node, data_bytes, &offset);
            if (data_len == 0) {
                debug_print("** Could not create the test DIR Broadcast frame\n");
                /* To avoid a loop where we keep hitting this error, we remove the station from the PB */
                // TODO - this is a serious issue and we should log/report it in telemetry
                pb_remove_request(current_station_on_pb);
                return FALSE; }
            ReportToWatchdog(CurrentTaskWD);

            /* Send the fill and finish */
            int rc = tx_send_packet(BROADCAST_CALLSIGN, QST, PID_DIRECTORY, data_bytes, data_len, BLOCK_IF_QUEUE_FULL);
            ReportToWatchdog(CurrentTaskWD);

            if (rc != TRUE) {
                debug_print("Could not send broadcast packet to TNC \n");
                /* To avoid a loop where we keep hitting this error, we remove the station from the PB */
                // TODO - this is a serious issue and we should log/report it in telemetry
                pb_remove_request(current_station_on_pb);
                return FALSE;
            }

            /* check if we sent the whole PFH or if it is split into more than one broadcast */
            if (offset == node->mram_file->body_offset) {
                /* Then we have sent this whole PFH */
                pb_list[current_station_on_pb].node = node->next; /* Store where we are in this broadcast of DIR fills */
                pb_list[current_station_on_pb].offset = 0; /* Reset this ready to send the next one */

                if (node->next == NULL) {
                    /* There are no more records, we are at the end of the list, move to next hole if there is one */
                    pb_list[current_station_on_pb].current_hole_num++;
                    if (pb_list[current_station_on_pb].current_hole_num == pb_list[current_station_on_pb].hole_num) {
                        /* We have finished this hole list */
                        debug_print("Added last hole for request from %s\n", pb_list[current_station_on_pb].callsign);
                        pb_remove_request(current_station_on_pb);
                        /* If we removed a station then we don't want/need to increment the current station pointer */
                        return TRUE;
                    }
                }
            } else {
                pb_list[current_station_on_pb].offset = offset; /* Store the offset so we send the next part of the PFH next time */
            }


        }

    /**
     *  Process Request to broadcast a file or parts of a file
     */
    } else if (pb_list[current_station_on_pb].pb_type == PB_FILE_REQUEST_TYPE) {
#ifdef 0
        debug_print("Preparing FILE Broadcast for %s\n",pb_list[current_station_on_pb].callsign);

        char psf_filename[MAX_FILE_PATH_LEN];
        pfh_make_filename(pb_list[current_station_on_pb].node->pfh->fileId,get_dir_folder(), psf_filename, MAX_FILE_PATH_LEN);

        if (pb_list[current_station_on_pb].hole_num == 0) {
            /* Request to broadcast the whole file */
            /* SEND THE NEXT CHUNK OF THE FILE BASED ON THE OFFSET */
            int number_of_bytes_read = pb_broadcast_next_file_chunk(pb_list[current_station_on_pb].node->pfh, psf_filename,
                    pb_list[current_station_on_pb].offset, PB_FILE_DEFAULT_BLOCK_SIZE, pb_list[current_station_on_pb].node->pfh->fileSize);
            pb_list[current_station_on_pb].offset += number_of_bytes_read;
            if (number_of_bytes_read == 0) {
                pb_remove_request(current_station_on_pb);
                /* If we removed a station then we don't want/need to increment the current station pointer */
                return TRUE;
            }

            /* If we are done then remove this request */
            if (pb_list[current_station_on_pb].offset >= pb_list[current_station_on_pb].node->pfh->fileSize) {
                pb_remove_request(current_station_on_pb);
                /* If we removed a station then we don't want/need to increment the current station pointer */
                return TRUE;
            }
        } else { // there is a hole list

            /* Request to fill holes in the file */
            int current_hole_num = pb_list[current_station_on_pb].current_hole_num;
            debug_print("Preparing Fill %d of %d from FILE %04x for %s --",(current_hole_num+1), pb_list[current_station_on_pb].hole_num,
                    pb_list[current_station_on_pb].file_id, pb_list[current_station_on_pb].callsign);

            FILE_DATE_PAIR *holes = pb_list[current_station_on_pb].hole_list;

            if (pb_list[current_station_on_pb].offset == 0) {
                /* Then this is probablly a new hole, initialize to the start of it */
                pb_list[current_station_on_pb].offset = holes[current_hole_num].offset;
            }
            debug_print("  Chunk from %d length %d at offset %d\n",holes[current_hole_num].offset, holes[current_hole_num].length, pb_list[current_station_on_pb].offset);

            /* We are currently at byte pb_list[current_station_on_pb].offset for this request.  So this hole
             * still has the following remaining bytes */
            int remaining_length_of_hole = holes[current_hole_num].offset + holes[current_hole_num].length - pb_list[current_station_on_pb].offset;

            int number_of_bytes_read = pb_broadcast_next_file_chunk(pb_list[current_station_on_pb].node->pfh, psf_filename,
                    pb_list[current_station_on_pb].offset, remaining_length_of_hole, pb_list[current_station_on_pb].node->pfh->fileSize);
            pb_list[current_station_on_pb].offset += number_of_bytes_read;
            if (number_of_bytes_read == 0) {
                pb_remove_request(current_station_on_pb);
                /* If we removed a station then we don't want/need to increment the current station pointer */
                return TRUE;
            }
            if (pb_list[current_station_on_pb].offset >= holes[current_hole_num].offset + holes[current_hole_num].length
                    || pb_list[current_station_on_pb].offset >= pb_list[current_station_on_pb].node->pfh->fileSize) {
                /* We have finished this hole, or we are at the end of the file */
                pb_list[current_station_on_pb].current_hole_num++;
                if (pb_list[current_station_on_pb].current_hole_num == pb_list[current_station_on_pb].hole_num) {
                    /* We have finished the fole list */
                    pb_remove_request(current_station_on_pb);
                    /* If we removed a station then we don't want/need to increment the current station pointer */
                    return TRUE;
                } else {
                    /* Move the offset to the start of the next hole */
                    pb_list[current_station_on_pb].offset = holes[pb_list[current_station_on_pb].current_hole_num].offset;
                }
            }
        }
#endif
    }

    current_station_on_pb++;
    if (current_station_on_pb == number_on_pb)
        current_station_on_pb = 0;

    return rc;
}

/**
 * pb_make_dir_broadcast_packet()
 *
 * Generate the bytes needed for a dir broadcast based on a pacsat file header
 * Pass in the Pacsat file header, a pointer to the broadcast frame, the offset
 * if this is the second frame for a long header
 *

      flags          A bit field as follows:

           7  6  5  4  3  2  1  0
          /----------------------\
          |*  N  E  0  V  V  T  T|
          \----------------------/
      TT                  Two bit frame type identifier
                          00   PFH broadcast
                          01   reserved
                          10   reserved
                          11   reserved

      VV                  Two bit version identifier.  This version is 00.

      0                   Always 0 indicates a server generated frame.

      E              1    Last byte of frame is the last byte of the directory PFH.
                     0    Not the last frame.

      N              1    This is the newest file on the server.
                     0    This is not the newest file on the server.

      *                   Reserved, always 0.


      file_id    A number which identifies the file.  All directory broadcast
      frames which are part of the same file's PFH are tagged with this number.

      offset     This is  the offset from the start of the PFH for the first data
      byte in this frame.

      t_old     Number of seconds since 00:00:00 1/1/80. See below.

      t_new     Number of seconds since 00:00:00 1/1/80. See below.

           There  are no files other than the file  identified  by
           file_id with t_old <= UPLOAD_TIME <= t_new.

      The data portion of a directory broadcast frame will contain all or part of
      the PACSAT File header from the file identified by <file_id>. The <offset>
      field indicates where the data from the current frame belongs in the PFH.

      An <offset> of 0 and a <flags> field with the E bit set to 1 indicates that
      this directory broadcast frame contains the entire PFH for the identified
      file.

      RETURNS the length of the data packet created

 */
int pb_make_dir_broadcast_packet(DIR_NODE *node, uint8_t *data_bytes, uint32_t *offset) {
    int length = 0;
    PB_DIR_HEADER *dir_broadcast = (PB_DIR_HEADER *)data_bytes;
    uint8_t flag = 0;

    if (node->mram_file->body_offset < MAX_DIR_PFH_LENGTH) {
        flag |= 1UL << E_BIT; // Set the E bit, All of this header is contained in the broadcast frame
    }
    // get the endianness right
    dir_broadcast->offset = htotl(*offset);
    dir_broadcast->flags = flag;
    dir_broadcast->file_id = htotl(node->mram_file->file_id);

    /* The dates guarantee:
     "There   are  no  files  other  than  this  file   with
      t_old <= UPLOAD_TIME <= t_new"

      Practically speaking
      t_old is 1 second after the upload time of the prev file
      t_new is 1 second before the upload time of the next file
     */
    if (node->prev != NULL)
        dir_broadcast->t_old = htotl(node->prev->mram_file->upload_time + 1);
    else
        dir_broadcast->t_old = 0;
    if (node->next != NULL)
        dir_broadcast->t_new = htotl(node->next->mram_file->upload_time - 1);
    else {
        dir_broadcast->t_new = htotl(node->mram_file->upload_time); // no files past this one so use its own uptime for now
        flag |= 1UL << N_BIT; /* Set the N bit to say this is the newest file on the server */
    }

    int buffer_size = node->mram_file->body_offset - *offset;  /* This is how much we have left to read */
    if (buffer_size <= 0) return 0; /* This is a failure as we return length 0 */
    if (buffer_size >= MAX_DIR_PFH_LENGTH) {
        /* If we have an offset then we have already sent part of this, send the next part */
        // TODO - pass the offset into this function..
        buffer_size = MAX_DIR_PFH_LENGTH;
    }

    /* Read the data into the data_bytes buffer after the header bytes */
    int rc = dir_mram_read_file_chunk(node->mram_file,  data_bytes + sizeof(PB_DIR_HEADER), buffer_size, *offset) ;
    if (rc != TRUE) {
        return 0; // Error with the read
    }
    *offset = *offset + buffer_size;

    length = sizeof(PB_DIR_HEADER) + buffer_size +2;
    // TODO - no checksum yet
    int checksum = gen_crc(data_bytes, length-2);
    //debug_print("crc: %04x\n",checksum);

    /* Despite the Pacsat protocol being little endian, the CRC needs to be in network byte order, or big endian */
    unsigned char one = (unsigned char)(checksum & 0xff);
    unsigned char two = (unsigned char)((checksum >> 8) & 0xff);
    data_bytes[length-1] = one;
    data_bytes[length-2] = two;

//  if (check_crc(data_bytes, length+2) != 0) {
//      error_print("CRC does not match\n");
//      return 0;
//  }
//  debug_print("\n%02x %02x crc check: %04x\n",one, two, checksum);
//  for (int i=0; i< length; i++) {
//          printf("%02x ",data_bytes[i]);
//          if (i%8 == 0 && i!=0) printf("\n");
//  }

    return length; // return length of header + pfh + crc
}


/***********************************************************************************************
 *
 * TEST FUNCTIONS FOLLOW
 *
 ***********************************************************************************************/

bool pb_test_callsigns() {
    debug_print("## SELF TEST: pb_test_callsigns\n");
    bool rc = true;

    /* Test encode callsigns */
    char call1[] = "G0KLA";
    char call2[] = "PACSAT";
    char call3[] = "G0KLA-1";
    char call4[] = "G0KLA-2";
    char call5[] = "PACSAT-12";
    uint8_t encoded_call[7];
    int l = encode_call(call1, encoded_call, false, 0); if (l != true) return false;
    l = encode_call(call2, encoded_call, false, 0); if (l != true) return false;
    l = encode_call(call3, encoded_call, false, 0); if (l != true) return false;
    l = encode_call(call4, encoded_call, false, 0); if (l != true) return false;
    l = encode_call(call5, encoded_call, false, 0); if (l != true) return false;

    /* Test decode callsigns */
    uint8_t byteBuf[] = {27, 0xA0,0x84,0x98,0x92,0xA6,0xA8,0x00,0xA0,0x8C,0xA6,0x66,
                          0x40,0x40,0x17,0x03,0xF0,0x50,0x42,0x3A,0x20,0x45,0x6D,0x70,0x74,0x79,0x2E,0x0D};
 //   uint8_t byteBuf2[] =  {0x1a,0xa0,0x84,0x98,0x92,0xa6,0xa8,0x00,0xa0,0x82,0x86,0xa6,
 //                          0x82,0xa8,0x17,0x03,0xf0,0x50,0x42,0x20,0x45,0x6d,0x70,0x74,0x79,0x2e };
    char from_callsign[10];
    char to_callsign[10];

    decode_call(&byteBuf[1], to_callsign);
    decode_call(&byteBuf[8], from_callsign);
    debug_print("PB: %s>%s: lens %d %d\n",from_callsign, to_callsign, strlen(from_callsign), strlen(to_callsign));

    uint8_t out_to_callsign[7];
    uint8_t out_from_callsign[7];
    l = encode_call(to_callsign, out_to_callsign, false, 0);
    if (l != true) return false;
    int i;
    printf("%s: ",to_callsign);
    for (i=0; i<7;i++) {
        if (byteBuf[i+1] != out_to_callsign[i]) {
            printf("err->");
            rc = false;
        }
        printf("%x ",out_to_callsign[i]);
    }
    printf("\n");

    l = encode_call(from_callsign, out_from_callsign, true, 0);
    if (l != true) return false;
    printf("%s: ",from_callsign);
    for (i=0; i<7;i++) {
        if (byteBuf[i+8] != out_from_callsign[i]) {
            printf("err->");
            rc = false;
        }
        printf("%x ",out_from_callsign[i]);
    }

    char new_callsign[10];
    decode_call(&out_from_callsign[0], new_callsign);
    debug_print("Decodes to:%s: len %d \n",new_callsign, strlen(new_callsign));
    printf("\n");

    if (rc == FALSE) {
        debug_print("## FAILED SELF TEST: pb_test_callsigns\n");
    } else {
        debug_print("## PASSED SELF TEST: pb_test_callsigns\n");
    }
    return rc;
}

bool pb_test_ok() {
    debug_print("## SELF TEST: pb_test_ok\n");
    bool rc = true;
    int l = pb_send_ok("G0KLA");  if (l == false) rc = false;
    if (rc == FALSE) {
        debug_print("## FAILED SELF TEST: pb_test_ok\n");
    } else {
        debug_print("## PASSED SELF TEST: pb_test_ok\n");
    }
    return true;
}

bool pb_test_status() {
    debug_print("## SELF TEST: pb_test_status\n");
    bool rc = true;

//        uint8_t byteBuf[] = {27, 0xA0,0x84,0x98,0x92,0xA6,0xA8,0x00,0xA0,0x8C,0xA6,0x66,
//                             0x40,0x40,0x17,0x03,0xF0,0x50,0x42,0x3A,0x20,0x45,0x6D,0x70,0x74,0x79,0x2E,0x0D};
//        uint8_t byteBuf[] =  {0x1a,0xa0,0x84,0x98,0x92,0xa6,0xa8,0x00,0xa0,0x82,0x86,0xa6,
//                               0x82,0xa8,0x17,0x03,0xf0,0x50,0x42,0x20,0x45,0x6d,0x70,0x74,0x79,0x2e };

    pb_send_status();
    pb_debug_print_list();

    if (rc == FALSE) {
        debug_print("## FAILED SELF TEST: pb_test_status\n");
    } else {
        debug_print("## PASSED SELF TEST: pb_test_status\n");
    }
    return true;
}

int pb_test_list() {
    printf("##### TEST PB LIST\n");
    int rc = TRUE;

    char data[] = {0x25,0x9f,0x3d,0x63,0xff,0xff,0xff,0x7f};
    DIR_DATE_PAIR * holes = (DIR_DATE_PAIR *)&data;

    rc = pb_add_request("AC2CZ", PB_FILE_REQUEST_TYPE, NULL, 3, 0, NULL, 0);
    if (rc != TRUE) {printf("** Could not add callsign\n"); return FALSE; }
    rc = pb_add_request("VE2XYZ", PB_DIR_REQUEST_TYPE, NULL, 0, 0, NULL, 0);
    if (rc != TRUE) {printf("** Could not add callsign\n"); return FALSE; }
    pb_debug_print_list();
    if (strcmp(pb_list[0].callsign, "AC2CZ") != 0) {printf("** Mismatched callsign 0\n"); return FALSE;}
    if (strcmp(pb_list[1].callsign, "VE2XYZ") != 0) {printf("** Mismatched callsign 1\n"); return FALSE;}


    // Now remove the head
    debug_print("REMOVE HEAD\n");
    rc = pb_remove_request(0);
    if (rc != TRUE) {printf("** Could not remove request\n"); return FALSE; }
    pb_debug_print_list();
    if (strcmp(pb_list[0].callsign, "VE2XYZ") != 0) {printf("** Mismatched callsign 0 after head removed\n"); return FALSE;}

    debug_print("ADD two more Calls\n");
    rc = pb_add_request("G0KLA", PB_FILE_REQUEST_TYPE, NULL, 3, 0, NULL, 0);
    if (rc != TRUE) {printf("** Could not add callsign\n"); return FALSE; }
    rc = pb_add_request("WA1QQQ", PB_DIR_REQUEST_TYPE, NULL, 0, 0, NULL, 0);
    if (rc != TRUE) {printf("** Could not add callsign\n"); return FALSE; }
    pb_debug_print_list();

    rc = pb_remove_request(0);
    if (rc != TRUE) {printf("** Could not remove request 1\n"); return FALSE; }
    rc = pb_remove_request(0);
    if (rc != TRUE) {printf("** Could not remove request 2\n"); return FALSE; }
    rc = pb_remove_request(0);
    if (rc != TRUE) {printf("** Could not remove request 3\n"); return FALSE; }
    // Test Remove when empty, should do nothing
    rc = pb_remove_request(0);
    if (rc != FALSE) {printf("** Did not receive error message for remove request 4\n"); return FALSE; }
    rc = TRUE; /* Reset rc after the failure test above*/

    pb_debug_print_list();

    MRAM_FILE test_file;
    test_file.file_id = 3;
    test_file.body_offset = 56;

    DIR_NODE test_node;
    test_node.mram_file = &test_file;

    // Test PB Full
    debug_print("ADD Calls and test FULL\n");
    if( pb_add_request("AA1AAA-10", PB_DIR_REQUEST_TYPE, NULL, 0, 0, holes, 1) != TRUE) {debug_print("ERROR: Could not add call to PB list\n");return FALSE; }
    if( pb_add_request("BB1BBB-11", PB_DIR_REQUEST_TYPE, NULL, 3, 0, NULL, 0) != TRUE) {debug_print("ERROR: Could not add call to PB list\n");return FALSE; }
    if( pb_add_request("CC1CCC-13", PB_DIR_REQUEST_TYPE, NULL, 0, 0, NULL, 0) != TRUE) {debug_print("ERROR: Could not add call to PB list\n");return FALSE; }
    if( pb_add_request("DD1DDD-10", PB_DIR_REQUEST_TYPE, NULL, 0, 0, NULL, 0) != TRUE) {debug_print("ERROR: Could not add call to PB list\n");return FALSE; }
    if( pb_add_request("EE1EEE-11", PB_DIR_REQUEST_TYPE, NULL, 0, 0, NULL, 0) != TRUE) {debug_print("ERROR: Could not add call to PB list\n");return FALSE; }
    if( pb_add_request("FF1FFF-12", PB_DIR_REQUEST_TYPE, NULL, 3, 0, NULL, 0) != TRUE) {debug_print("ERROR: Could not add call to PB list\n");return FALSE; }
    if( pb_add_request("GG1GGG-13", PB_DIR_REQUEST_TYPE, &test_node, 3, 0, NULL, 0) != TRUE) {debug_print("ERROR: Could not add call to PB list\n");return FALSE; }
    if( pb_add_request("HH1HHH-10", PB_DIR_REQUEST_TYPE, NULL, 0, 0, NULL, 0) != TRUE) {debug_print("ERROR: Could not add call to PB list\n");return FALSE; }
    if( pb_add_request("II1III-11", PB_DIR_REQUEST_TYPE, NULL, 0, 0, NULL, 0) != TRUE) {debug_print("ERROR: Could not add call to PB list\n");return FALSE; }
    if( pb_add_request("JJ1JJJ-12", PB_DIR_REQUEST_TYPE, NULL, 0, 0, NULL, 0) != TRUE) {debug_print("ERROR: Could not add call to PB list\n");return FALSE; }
    if( pb_add_request("KK1KKK-13", PB_DIR_REQUEST_TYPE, NULL, 0, 0, NULL, 0) != FALSE) {debug_print("ERROR: Added call to FULL PB list\n");return FALSE; }

    if (strcmp(pb_list[0].callsign, "AA1AAA-10") != 0) {printf("** Mismatched callsign 0\n"); return FALSE;}
    if (strcmp(pb_list[1].callsign, "BB1BBB-11") != 0) {printf("** Mismatched callsign 1\n"); return FALSE;}
    if (strcmp(pb_list[2].callsign, "CC1CCC-13") != 0) {printf("** Mismatched callsign 2\n"); return FALSE;}
    if (strcmp(pb_list[3].callsign, "DD1DDD-10") != 0) {printf("** Mismatched callsign 3\n"); return FALSE;}
    if (strcmp(pb_list[4].callsign, "EE1EEE-11") != 0) {printf("** Mismatched callsign 4\n"); return FALSE;}
    if (strcmp(pb_list[5].callsign, "FF1FFF-12") != 0) {printf("** Mismatched callsign 5\n"); return FALSE;}
    if (strcmp(pb_list[6].callsign, "GG1GGG-13") != 0) {printf("** Mismatched callsign 6\n"); return FALSE;}
    if (strcmp(pb_list[7].callsign, "HH1HHH-10") != 0) {printf("** Mismatched callsign 7\n"); return FALSE;}
    if (strcmp(pb_list[8].callsign, "II1III-11") != 0) {printf("** Mismatched callsign 8\n"); return FALSE;}
    if (strcmp(pb_list[9].callsign, "JJ1JJJ-12") != 0) {printf("** Mismatched callsign 9\n"); return FALSE;}

    pb_debug_print_list();

//    debug_print("Process Current Call\n");
//    if (pb_next_action() != TRUE) { printf("** Could not take next PB action\n"); return FALSE; }
//    pb_debug_print_list();
//    debug_print("With current_station_on_pb = %d\n",current_station_on_pb);
//    if (strcmp(pb_list[current_station_on_pb].callsign, "B1B") != 0) {printf("** Mismatched callsign current call\n"); return FALSE;}

//  debug_print("Remove head\n");
//  // Remove 0 as though it was done
//  rc = pb_remove_request(0); // Head
//  if (rc != TRUE) {printf("** Could not remove request\n"); return FALSE; }
//  if (strcmp(pb_list[current_station_on_pb].callsign, "B1B") != 0) {printf("** Mismatched callsign current call after remove head\n"); return FALSE;}

    debug_print("Remove 5\n");
    // Remove 5 as though it timed out
    rc = pb_remove_request(5); // Now FIF
    if (rc != TRUE) {printf("** Could not remove request\n"); return FALSE; }
    if (strcmp(pb_list[current_station_on_pb].callsign, "AA1AAA-10") != 0) {printf("** Mismatched callsign current call after remove 5\n"); return FALSE;}
    if (strcmp(pb_list[5].callsign, "GG1GGG-13") != 0) {printf("** Mismatched callsign 5\n"); return FALSE;}
    /* Also confirm that the node copied over correctly */
    if (pb_list[5].node->mram_file->file_id != 3) {printf("** Mismatched file id for entry 5\n"); return FALSE;}
    if (pb_list[5].node->mram_file->body_offset != 56) {printf("** Mismatched body offset for entry 5\n"); return FALSE;}
    if (strcmp(pb_list[8].callsign, "JJ1JJJ-12") != 0) {printf("** Mismatched callsign 8\n"); return FALSE;}

    debug_print("Remove current station\n");
    // Remove the current station, which is also the head, should advance to next one
    rc = pb_remove_request(current_station_on_pb);
    if (rc != TRUE) {printf("** Could not remove request\n"); return FALSE; }
    if (strcmp(pb_list[current_station_on_pb].callsign, "BB1BBB-11") != 0) {printf("** Mismatched callsign current call after remove current station\n"); return FALSE;}

    pb_debug_print_list();
    debug_print("Remove 8 stations\n");
    rc = pb_remove_request(current_station_on_pb);
    if (rc != TRUE) {printf("** Could not remove request\n"); return FALSE; }
    rc = pb_remove_request(current_station_on_pb);
    if (rc != TRUE) {printf("** Could not remove request\n"); return FALSE; }
    rc = pb_remove_request(current_station_on_pb);
    if (rc != TRUE) {printf("** Could not remove request\n"); return FALSE; }
    rc = pb_remove_request(current_station_on_pb);
    if (rc != TRUE) {printf("** Could not remove request\n"); return FALSE; }
    rc = pb_remove_request(current_station_on_pb);
    if (rc != TRUE) {printf("** Could not remove request\n"); return FALSE; }
    rc = pb_remove_request(current_station_on_pb);
    if (rc != TRUE) {printf("** Could not remove request\n"); return FALSE; }
    rc = pb_remove_request(current_station_on_pb);
    if (rc != TRUE) {printf("** Could not remove request\n"); return FALSE; }
    rc = pb_remove_request(current_station_on_pb);
    if (rc != TRUE) {printf("** Could not remove request\n"); return FALSE; }
    pb_debug_print_list();

    if (rc == TRUE)
        printf("##### TEST PB LIST: success\n");
    else
        printf("##### TEST PB LIST: fail\n");
    return rc;
}

