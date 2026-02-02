/*
 * UplinkTask.h
 *
 *  Created on: May 3, 2023
 *      Author: g0kla
 */

#ifndef TASKS_INC_UPLINKTASK_H_
#define TASKS_INC_UPLINKTASK_H_

#include "config.h"

#define FTL0_PFH_BIT 2
#define FTL0_VERSION_BIT1 0
#define FTL0_VERSION_BIT2 1

typedef enum
{
    UL_UNINIT,      /* 0 */
    UL_CMD_WAIT,
    UL_CMD_OK,
    UL_DATA_RX,
    UL_ABORT
} uplink_state_t;

typedef enum
{
    DATA,               /* 0 */
    DATA_END,
    LOGIN_RESP,
    UPLOAD_CMD,
    UL_GO_RESP,
    UL_ERROR_RESP,          /* 5 */
    UL_ACK_RESP,
    UL_NAK_RESP         /* 7 */
} uplink_packet_type_t;

#define MAX_PACKET_ID 7

typedef enum
{
    ER_NONE,            /* 0 */
    ER_ILL_FORMED_CMD,
    ER_BAD_CONTINUE,
    ER_SERVER_FSYS,
    ER_NO_SUCH_FILE_NUMBER,
    ER_SELECTION_EMPTY_1,       /* 5 */
    ER_MANDATORY_FIELD_MISSING,
    ER_NO_PFH,
    ER_POORLY_FORMED_SEL,
    ER_ALREADY_LOCKED,
    ER_NO_SUCH_DESTINATION,     /* 10 */
    ER_SELECTION_EMPTY_2,
    ER_FILE_COMPLETE,
    ER_NO_ROOM,
    ER_BAD_HEADER,
    ER_HEADER_CHECK,        /* 15 */
    ER_BODY_CHECK           /* 16 */
} uplink_error_codes_t;

/* The server sends these packets */
typedef struct {
    uint32_t login_time;
    uint8_t login_flags;
} FTL0_LOGIN_DATA;

typedef struct {
    uint32_t server_file_no;
    uint32_t byte_offset;
} FTL0_UL_GO_DATA;

/* The client sends these packets */
typedef struct {
    uint32_t continue_file_no;
    uint32_t file_length;
} FTL0_UPLOAD_CMD;

typedef struct {
    uplink_state_t ul_state;
    uint8_t channel;             /* Radio A, B, C, D */
    char callsign[MAX_CALLSIGN_LEN];
    int file_id; /* File id of the file being uploaded */
    uint32_t request_time; /* The time the request was received for timeout purposes */
    uint32_t offset;
    uint32_t length;
} ftl0_state_machine_t;

#endif /* TASKS_INC_UPLINKTASK_H_ */

/*
 * Routine prototypes
 */
void UplinkTask(void *pvParameters);
bool ftl0_debug_list_upload_table();
int test_ftl0_upload_table();
int ftl0_get_space_reserved_by_upload_table();
int ftl0_get_num_of_files_in_upload_table();
void ftl0_maintenance();
