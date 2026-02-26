/*
 * CANTask.h
 *
 *  Created on: Dec 7, 2025
 *      Author: Corey Minyard  AE5KM
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
 */

#ifndef CANTASK_H
#define CANTASK_H

#define NUM_CAN_BUSSES 2

void CANInit(void);
void CANTask(void *pvParameters);

bool CANSend(int canNum, uint8_t priority, uint8_t type,
             uint16_t msgid, uint8_t dest,
             uint8_t *msg, unsigned int msglen);
void CANSetMyID(int canNum, int id);
int CANGetMyID(int canNum);

void CANEnableLoopback(int canNum, bool enable);

typedef void (*CANReceiveHandler)(int canNum,
                                  unsigned int type, unsigned int id,
                                  unsigned int src,
                                  uint8_t *msg, unsigned int msglen);

void CANRegisterReceiveHandler(int canNum, CANReceiveHandler rxhandler);

struct can_counts {
    uint32_t rx_msgs;
    uint32_t rx_long_msgs;
    uint32_t rx_msgs_lost;
    uint32_t rx_msgs_invalid_start_seq;
    uint32_t rx_msgs_unsupported_long_msg;
    uint32_t rx_msgs_sequence_mismatch;
    uint32_t rx_msgs_long_msg_too_long;
    uint32_t rx_msgs_new_source_in_long_msg;
    uint32_t rx_err_bit_stuff_count;
    uint32_t rx_err_form_count;
    uint32_t rx_err_crc_count;

    uint32_t tx_msgs;
    uint32_t tx_long_msgs;
    uint32_t tx_err_bit0_count;
    uint32_t tx_err_bit1_count;
    uint32_t tx_err_no_ack_count;
};

void CANGetCounts(int canNum, struct can_counts *counts);

#endif /* CANTASK_H */
