/*
 * PbTask.h
 *
 *  Created on: Dec 1, 2022
 *      Author: Chris E. Thompson, G0KLA
 */

#ifndef TASKS_INC_PBTASK_H_
#define TASKS_INC_PBTASK_H_

#include <pacsat.h>

#define PID_FILE        0xBB
#define PID_DIRECTORY   0xBD
#define PID_NO_PROTOCOL 0xF0

#define MAX_PB_LENGTH 10 /* This is the maximum number of stations that can be on the PB at one time */
#define PB_DIR_REQUEST_TYPE 1
#define PB_FILE_REQUEST_TYPE 2

/* Error numbers sent in response to Broadcast requests */
#define PB_ERR_TEMPORARY 1
#define PB_ERR_FILE_NOT_AVAILABLE 2
#define PB_ERR_FILE_NOT_DOWNLOADABLE 3
#define PB_ERR_FILE_INVALID_PACKET 5

#define EXIT_LAST_CHUNK_SENT 2 /* This exit code is used when we have sent the last chunk of a file */

#define MAX_REQUEST_PACKETS 10 /* The maximum number of Dir Headers or File segments that will be sent in response to a request */
#define MAX_DIR_PFH_LENGTH 182 /* Shorten this for FX25? BD header is 9 longer than file header */
//#define MAX_BROADCAST_LENGTH 254 /* This was the limit on historical Pacsats. Can we make it longer? */
#define PB_FILE_DEFAULT_BLOCK_SIZE 191 /* 191 seems to be the MAX for FX25. This must be assuming 32 header bytes and 32 check bytes. AX25 Header is 17.  File Broadcast header is 9.*/
//#define PB_FILE_DEFAULT_BLOCK_SIZE 0xF4

#define PBLIST "PBLIST" // destination for PB Status when open
#define PBFULL "PBFULL" // destination for PB status when list is full
#define PBSHUT "PBSHUT" // destination for PB status when it is closed
#define QST "QST-1" // destination for broadcast dir and file frames
#define STATUS "STATUS" // destination for status frames
#define BSTAT "BSTAT" // destination for broadcast status frames
#define BBSTAT "BBSTAT" // destination for bbs status frames

#define L_BIT 0
#define E_BIT 5
#define N_BIT 6

#define PB_START_SENDING_FILE 0b00
#define PB_STOP_SENDING_FILE 0b01
#define PB_FILE_HOLE_LIST 0b10

/**
 *  The Server Sends frames with these headers
 */

struct t_file_header {
     uint8_t flags;
     uint32_t file_id;
     uint8_t file_type;
     unsigned int offset : 24;
} __attribute__ ((__packed__));
typedef struct t_file_header PB_FILE_HEADER;

struct t_dir_header { // sent by Pacsat
    uint8_t flags;
    uint32_t file_id;
    uint32_t offset;
    uint32_t t_old;
    uint32_t t_new;
} __attribute__ ((__packed__));
typedef struct t_dir_header PB_DIR_HEADER;


/**
 * The client sends frames with these headers, which we need to parse
 */

struct t_file_req_header {
    uint8_t flags;
    uint32_t file_id;
    uint16_t block_size;
} __attribute__ ((__packed__));
typedef struct t_file_req_header FILE_REQ_HEADER;

struct t_file_pair {
     unsigned int offset : 24;
     uint16_t length;
     } __attribute__ ((__packed__));
typedef struct t_file_pair FILE_DATE_PAIR;

struct t_dir_req_header { // sent by client
    uint8_t flags;
    uint16_t block_size;
} __attribute__ ((__packed__));
typedef struct t_dir_req_header DIR_REQ_HEADER;

struct t_dir_pair {
    uint32_t start;
    uint32_t end;
} __attribute__ ((__packed__));
typedef struct t_dir_pair DIR_DATE_PAIR;

struct t_broadcast_request_header {
    //uint8_t flag;
    uint8_t to_callsign[7];
    uint8_t from_callsign[7];
    uint8_t control_byte;
    uint8_t pid;
} __attribute__ ((__packed__));
typedef struct t_broadcast_request_header AX25_HEADER;


/*
 * Routine prototypes
 */
void PbTask(void *pvParameters);
int pb_clear_list();
bool pb_test_callsigns();
bool pb_test_ok();
bool pb_test_status();
int pb_test_list();

#endif /* TASKS_INC_PBTASK_H_ */
