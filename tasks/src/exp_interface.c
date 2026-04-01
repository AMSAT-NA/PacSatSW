/*
 * exp_interface.c
 *
 *  Created on: Feb 20, 2026
 *      Author: g0kla
 *
 * This file handles data from an experiment that is attached over a CAN Bus.
 *
 * Can packets will contain an ID with the format defined in CANTask.c
 * The ID contains a TYPE which will be interpretted as follows for data to/from the
 * experiments:
 *
 * Type       Purpose
 * -------    --------------------------------------------------------------------------
 * 0          Reserved
 * 1          Status packet,  message id indicates the action
 * 2          Telemetry to be broadcast, where msgid indicates the type of telemetry
 * 3          Telemetry to be stored in WOD, where msgid indicates the type of telemetry
 * 4          Opaque data to save in file 1
 * 5          Opaque data to save in file 2
 * 6          Opaque data to save in file 3
 * 7          Opaque data to save in file 4
 * 8          EOF End of file 1 - this is the last packet in this file
 * 9          EOF End of file 2 - this is the last packet in this file
 * 10        EOF End of file 3 - this is the last packet in this file
 * 11        EOF End of file 4 - this is the last packet in this file
 * 13-15     Reserved
 *
 * Message IDs for Type 1 status packets:
 * ID       Meaning                 Data
 * -------  ----------------------  -------------------------------------------------------
 * 0        Ignored
 * 1        Enter Safe Mode          If PACSAT is source: Data 0=reason, 0=commanded, 1=low power
 * 2        Enter File System Mode   Data 0=reason: 0=commanded; 1=power ok; 2= timeout of science
 * 3        Enter Science Mode       Data 0=reason: 0=commanded; 1=scheduled time; Data 1=timeout (in mins)
 *
 * For types 2 and 3 the following MSG Ids indicate the type of telemetry
 * ID       Meaning                 Data Units and Scale
 * -------  ----------------------  -------------------------------------------------------
 * 8        Solar Panel Temps       Data 0=+X, Data 1=-X, Data 2,3=+-Y Data 4,5=+-Z Unit: Degrees C, Scale: 0-255 = -20C to +107.5C
 * 9        Other temperatures      Up to 8 temperatures in data 0-7 Unit: Degrees C Scale 0-255 = -20C to +107.5C
 * 16       Solar Panel Volts       Data 0=+X, Data 1=-X, Data 2,3=+-Y Data 4,5=+-Z Unit: Volts Scale: 0-255 = 0-10V
 * 17       Other voltages          Up to 8 voltages in data 0-7 Unit: Volts Scale: 0-255 = 0-10V
 * 24       Binary Data 1           8 bits in data 0 Unit: True/False
 * 25       Binary Data 2           8 bits in data 0 Unit: True/False
 * 26       Binary Data 3           8 bits in data 0 Unit: True/False
 * 27       Binary Data 4           8 bits in data 0 Unit: True/False
 * 32       UTC Time                Data0=Year, Data1=Month (1-12), Data2=Day (1-31), Data3=Hour (0-23), Data 4=Minute (0-59), Data 5=Second (0-59)
 */

#include "pacsat.h"
#include "nonvolManagement.h"
#include "redposix.h"
#include "inet.h"
#include "str_util.h"
#include "exp_interface.h"
#include "TelemAndControlTask.h"

/* Forward declarations */
void exp_process_status_type(can_msg_type_t type, int msgid, uint8_t *data, unsigned int len);
void exp_process_telem_type(can_msg_type_t type, int msgid, uint8_t *data, unsigned int len);
void exp_process_wod_type(can_msg_type_t type, int msgid, uint8_t *data, unsigned int len);
void exp_store_can(can_msg_type_t type, int msgid, uint8_t *data, unsigned int len);

/* Local variables */
static Intertask_Message statusMsg; // Storage used to send messages to the Telemetry and Control task


void exp_can_handler(int canNum, unsigned int type, unsigned int msgid,
		     unsigned int src, uint8_t *data, unsigned int len)
{
//    debug_print("EXP CAN Handler Received: canNum=%d type=%d msgid=%d "
//           "src=%d len=%d\n",
//           canNum, type, msgid, src, len);
//
//    unsigned int i;
//    for (i = 0; i < len; i++)
//        debug_print(" %02x", data[i]);
//    debug_print("\n");

    switch (type) {
    case can_msg_type_status:
        exp_process_status_type((can_msg_type_t)type, msgid, data, len);
        break;
    case can_msg_type_telem:
        exp_process_telem_type((can_msg_type_t)type, msgid, data, len);
        break;
    case can_msg_type_wod:
        exp_process_wod_type((can_msg_type_t)type, msgid, data, len);
        break;
    default:
        exp_store_can((can_msg_type_t)type, msgid, data, len);
        break;
    }
}

/**
 * exp_process_status_type()
 *
 * Based on the message id, take appropriate action
 *
 */
void exp_process_status_type(can_msg_type_t type, int msgid, uint8_t *data, unsigned int len) {
    debug_print("EXP CAN Received Status Type: %d msgid=%d\n", type, msgid);
    switch (msgid) {
        case can_status_msg_id_enter_safe_mode:
            statusMsg.MsgType = TacEnterSafeMode;
            // TODO Log this happened - not an error
            NotifyInterTaskFromISR(ToTelemetryAndControl, &statusMsg);
            break;
        case can_status_msg_id_enter_fs_mode:
            statusMsg.MsgType = TacEnterFileSystemMode;
            // TODO Log this happened - not an error
            NotifyInterTaskFromISR(ToTelemetryAndControl, &statusMsg);
            break;
        case can_status_msg_id_enter_science_mode:
            statusMsg.MsgType = TacEnterScienceMode;
            // TODO Log this happened - not an error
            statusMsg.data[0] = data[1]; // The Timeout is in data1.
            NotifyInterTaskFromISR(ToTelemetryAndControl, &statusMsg);
            break;
        default:
            break;
        }
}

void exp_process_telem_type(can_msg_type_t type, int msgid, uint8_t *data, unsigned int len) {

}

void exp_process_wod_type(can_msg_type_t type, int msgid, uint8_t *data, unsigned int len) {

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
void exp_store_can(can_msg_type_t type, int msgid, uint8_t *data, unsigned int len) {
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
    if ( (type >= can_msg_type_eof1) || (exp_file_length > (ReadMRAMExpMaxFileSize4kBlocks() * 4096)) )
        tac_roll_file(exp_file_name_with_path, EXP_FOLDER, file_name);
}
