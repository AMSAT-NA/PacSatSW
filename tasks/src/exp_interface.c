/*
 * exp_interface.c
 *
 *  Created on: Feb 20, 2026
 *      Author: g0kla
 */

#include "pacsat.h"
#include "nonvolManagement.h"
#include "redposix.h"
#include "inet.h"
#include "str_util.h"
#include "exp_interface.h"
#include "TelemAndControlTask.h"

/* Forward declarations */
void exp_store_can(int type, int msgid, uint8_t *data, unsigned int len);

/* Local variables */


void exp_can_handler(int canNum, unsigned int type, unsigned int msgid,
		     unsigned int src, uint8_t *data, unsigned int len)
{
    debug_print("EXP CAN Handler Received: canNum=%d type=%d msgid=%d "
           "src=%d len=%d\n",
           canNum, type, msgid, src, len);

    unsigned int i;
    for (i = 0; i < len; i++)
        debug_print(" %02x", data[i]);
    debug_print("\n");

    switch (type) {
    case can_msg_type_status:
        break;
    case can_msg_type_telem:
        break;
    case can_msg_type_wod:
        break;
    default:
        exp_store_can(type, msgid, data, len);
        break;
    }
}

void intToLetterString(int num, char *result) {
    if (num >= 1 && num <= 4) {
        result[0] = 'a' + (num - 1);
        result[1] = '\0';
    } else {
        result[0] = 'a';
        result[1] = '\0'; // default string if out of range
    }
}
/**
 * Store one can packet into the EXP file.
 * The Message ID and length are stored as a 16 bit little endian byte pair.  This
 * matches the expectations at the ground station.  The actual data can be big or
 * little endian depending on the experiment and we pass it through unchanged.
 */
void exp_store_can(int type, int msgid, uint8_t *data, unsigned int len) {
    char exp_file_name_with_path[MAX_FILENAME_WITH_PATH_LEN];
    char file_id_str[4];
    unsigned int file_id = 1;
    if (type >= can_msg_type_eof1)
        file_id = type - can_msg_type_eof1 + 1;
    else
        file_id = type - can_msg_type_file1 + 1;
    if (file_id > 4 )
        file_id = 1;

    /* Make a new exp file name and start the file */
    char file_name[MAX_FILENAME_WITH_PATH_LEN];
    intToLetterString(file_id, file_id_str);
    strlcpy(file_name, EXP_PREFIX, sizeof(file_name));
    strlcat(file_name, file_id_str, sizeof(file_name));

    strlcpy(exp_file_name_with_path, EXP_FOLDER, sizeof(exp_file_name_with_path));
    strlcat(exp_file_name_with_path, file_name, sizeof(exp_file_name_with_path));
    strlcat(exp_file_name_with_path, ".tmp", sizeof(file_name));

    uint16_t combined = ((msgid & 0x0FFF) << 4) |
                            (len & 0x0F);
    uint8_t can_packet_id[2];
    can_packet_id[0] = (uint8_t)(combined & 0xFF);        // LSB
    can_packet_id[1] = (uint8_t)((combined >> 8) & 0xFF); // MSB


    /* Write bytes to the file */
    int32_t fp;
    int32_t numOfBytesWritten = -1;
    int32_t rc;
    int32_t exp_file_length;

    fp = red_open(exp_file_name_with_path, RED_O_CREAT | RED_O_APPEND | RED_O_WRONLY);
    if (fp == -1) {
        debug_print("Unable to open %s for writing: %s\n", exp_file_name_with_path, red_strerror(red_errno));
        return;
    } else {
        numOfBytesWritten = red_write(fp, &can_packet_id, 2);
        if (numOfBytesWritten != 2) {
            printf("Write returned: %d\n",numOfBytesWritten);
            if (numOfBytesWritten == -1) {
                printf("Unable to write id to %s: %s\n", exp_file_name_with_path, red_strerror(red_errno));
            }
            red_close(fp);
            return;
        }
        numOfBytesWritten = red_write(fp, data, len);
        if (numOfBytesWritten != len) {
            printf("Write returned: %d\n",numOfBytesWritten);
            if (numOfBytesWritten == -1) {
                printf("Unable to write data to %s: %s\n", exp_file_name_with_path, red_strerror(red_errno));
            }
            red_close(fp);
            return;
        }
        exp_file_length = red_lseek(fp, 0, RED_SEEK_END);
        rc = red_close(fp);
        if (rc != 0) {
            printf("Unable to close %s: %s\n", exp_file_name_with_path, red_strerror(red_errno));
            return;
        }
    }

    debug_print("Telem & Control: Stored EXP ID: %d size:%ld %s\n", combined,exp_file_length, exp_file_name_with_path);
    if ( (type >= can_msg_type_eof1) || (exp_file_length > ReadMRAMExpMaxFileSize()) )
        tac_roll_file(exp_file_name_with_path, EXP_FOLDER, file_name);
}
