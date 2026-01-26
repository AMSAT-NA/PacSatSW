/*
 * CANTask.c
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

#include "pacsat.h"
#include "can.h"
#include "reg_can.h"
#include "errors.h"

#include "CANTask.h"

/*
 * This code uses message box 1 for transmission and message box 2+ for
 * receiving.  It changes the id on message box 1 to send to different
 * destinations.
 *
 * Message numbers (the message receiver box in the hardware, there are
 * 64 of them) are counted from 1 in the tech spec.  We use the same
 * convention, though it is unfortunate.
 *
 * The 29-bit id is used for the destination address, source address,
 * message priority, message type and message id.  These are defined
 * as follows:
 *
 *  28-24 - Priority
 *  23-16 - ID2
 *  15-12 - Type
 *  11-8  - Source
 *  7-4   - ID1
 *  3-0   - Destination
 *
 * Priority: These bits should be ignored by receiving software, and
 * filters should be set to “don’t care” for any of those
 * bits. Priority determines which packet will be sent if multiple
 * processors place packets on the bus at the same time. A 0 in this
 * field is the highest priority. A 31 (or 0x1F) is the lowest
 * priority.
 *
 * Source: Which processor sent the packet.
 *
 * Destination: The processor to receive the packet.
 *
 * ID1 and ID2: These values are more specific about the exact content
 * of the packet.  The ID may be from 0-255.  IDs from 1-15 may
 * be sent as multi-message sends up to 2048 bytes.
 *
 * If ID1 is 0, then ID2 contains the id for 255 different specific
 * types of sends, each with a data length of at most 8 bytes.
 *
 * If ID1 is non-0, then ID1 contains the id, but these sends can be
 * as long as 2048 bytes. This is accomplished by sending the message
 * 8 bytes at a time with the ID1 the same and ID2 used as a serial
 * number that is incremented for each subsequent message. This allows
 * the receiver to re-assemble the send even if it does not arrive in
 * order.
 *
 * Some of the large messages are fixed length and the values are set
 * in the id1_lengths array.  They may be marked there as
 * variable-length.
 *
 * For variable length send, the end of the send is marked with a
 * message length less than 8.  If the last length sent is 8, then a
 * zero-length message with is sent to mark the end of a message.
 * This means a variable length message may only be up to 2047 bytes
 * long, since the sequence would not be valid for the zero-length
 * message after 2048 bytes.
 *
 * Each CAN bus has a single buffer for receiving long messages, any
 * new long message that comes in will abort any current long message.
 */

/* How many time to we try to send a message before giving up. */
#define CAN_MAX_TX_ERRORS 8

bool trace_can;

/* We only use one message box for sending. */
#define CAN_TRANSMIT_BOX 1

/*
 * We use a number of message boxes for receiving, but they should
 * stay in order from the hardware, so that's ok.
 */
#define CAN_RECEIVE_FIRST_BOX 2
#define CAN_RECEIVE_NUM_BOXES 8

#define CAN_BOX_ENABLE 1
#define CAN_BOX_DISABLE 0
#define CAN_BOX_TRANSMIT 1
#define CAN_BOX_RECEIVE 0

/*
 * Maximum long message sizes.  This should not exceed 2047, though
 * that's pretty big.
 */
#define CAN1_MAX_RECV 512
#define CAN2_MAX_RECV 512

/*
 * Just the portion of the registers used for the interface.  There are
 * two of these interfaces, we use IF1 for transmit and IF2 for receive
 * so they don't conflict.
 */
typedef volatile struct canIfRegs {
#if ((__little_endian__ == 1) || (__LITTLE_ENDIAN__ == 1))
    uint8 NO;            /**< IF Command Register, Msg Number       */
    uint8 STAT;          /**< IF Command Register, Status           */
    uint8 CMD;           /**< IF Command Register, Command          */
    uint8 rsvd;          /**< IF Command Register, Reserved         */
#else
    uint8 rsvd;          /**< IF Command Register, Reserved         */
    uint8 CMD;           /**< IF Command Register, Command          */
    uint8 STAT;          /**< IF Command Register, Status           */
    uint8 NO;            /**< IF Command Register, Msg Number       */
#endif
    uint32 MSK;          /**< IF Mask Register                      */
    uint32 ARB;          /**< IF Arbitration Register               */
    uint32 MCTL;         /**< IF Message Control Register           */
    uint8 DATx[8U];      /**< IF Data A and B Registers             */
} canIfRegs;

#if ((__little_endian__ == 1) || (__LITTLE_ENDIAN__ == 1))
#else
static const uint8_t s_canByteOrder[8U] = {3U, 2U, 1U, 0U, 7U, 6U, 5U, 4U};
#endif

/*
 * Definitions for each CAN bus.  Note that the number here does not
 * correspond to the CAN bus number on the chip, CAN bus 2 is 0 here
 * and CAN bus 3 is 1.
 */
struct CANInfo {
    volatile bool TxWaiting;

    /* Controls access to the transmit function. */
    xSemaphoreHandle TxSemaphore;

    /* Used to signal that the transmit of a message is finished. */
    xSemaphoreHandle TxDoneSemaphore;

    /* Used to control access to IF2. */
    xSemaphoreHandle IFSemaphore;

    int myID; /* Source ID. */
    canBASE_t *regs;

    volatile bool in_bus_recovery;
    volatile int num_tx_errors;
    volatile int tx_error;

    struct can_counts counts;

    /* Receive lengths for the long messages, indexed by ID1. */
    const uint16_t *id1_lengths;

    CANReceiveHandler rxhandler;

    /* Used when receiving a message >8 bytes long. */
    uint8_t *long_rx_msg;
    uint16_t max_rx_msg_len;
    uint16_t long_rx_msg_len;
    bool long_rx_msg_var; /* Variable length. */
    uint8_t long_rx_src;
    uint8_t long_rx_type;
    uint8_t long_rx_id;
    int next_seq;
};

static uint8_t can_bus1_rx_data[CAN1_MAX_RECV];
static uint8_t can_bus2_rx_data[CAN2_MAX_RECV];

/*
 * For messages with id1 != 0, it's a multi-message send and this
 * marks how to handle it.  Values of 0 say to ignore the messages.
 * Values 1-2048 are the message length of a fixed-length message.  A
 * value of 4096-6143 says it's a variable-length message and the
 * maximum length is val-4096.
 */
static const uint16_t id1_lengths1[16] = {
    0,      0,      0,      0,
    0,      0,      0,      0,
    0,      0,      0,      0,
    0,      0,      0,   4096 + CAN1_MAX_RECV,
};
static const uint16_t id1_lengths2[16] = {
    0,      0,      0,      0,
    0,      0,      0,      0,
    0,      0,      0,      0,
    0,      0,      0,   4096 + CAN2_MAX_RECV,
};

static struct CANInfo can[NUM_CAN_BUSSES] = {
    { .long_rx_msg = can_bus1_rx_data, .max_rx_msg_len = CAN1_MAX_RECV,
      .id1_lengths = id1_lengths1 },
    { .long_rx_msg = can_bus2_rx_data, .max_rx_msg_len = CAN2_MAX_RECV,
      .id1_lengths = id1_lengths2 },
};


void CANInit(void)
{
    unsigned int i;

    /* CAN bus 1 is not used. */
    can[0].regs = canREG2;
    can[1].regs = canREG3;
    for (i = 0; i < NUM_CAN_BUSSES; i++) {
        vSemaphoreCreateBinary(can[i].TxSemaphore);
        vSemaphoreCreateBinary(can[i].TxDoneSemaphore);
        vSemaphoreCreateBinary(can[i].IFSemaphore);
        canEnableErrorNotification(can[i].regs);
        canEnableStatusChangeNotification(can[i].regs);
      }
    can[0].myID = CANA_ADDRESS;
    can[1].myID = CANB_ADDRESS;
    GPIOSetOn(CANAPower);
    GPIOSetOn(CANBPower);
}

void CANEnableLoopback(int canNum, bool enable)
{
    if (canNum >= NUM_CAN_BUSSES)
        return;

    if (enable)
        canEnableloopback(can[canNum].regs, Internal_Lbk);
    else
        canDisableloopback(can[canNum].regs);
}

/*
 * We use IF2, IF1 is reserved for the interrupt handler.
 */
static canIfRegs *CANGetIfRegs(int canNum)
{
    int offset = 0x120;

    if (xSemaphoreTake(can[canNum].IFSemaphore,
                       WATCHDOG_SHORT_WAIT_TIME) != pdTRUE) {
        /* If we can't get it within a few seconds...trouble */
        ReportToWatchdog(CurrentTaskWD);
        ReportError(CANInUse, false, TaskNumber, 0);
        return NULL;
    }
    return (struct canIfRegs *) (((uint8_t *) can[canNum].regs) + offset);
}

static void CANPutIFRegs(int canNum)
{
    xSemaphoreGive(can[canNum].IFSemaphore);
}

/*
 * Set the values of a message box.  This is used for transmit and
 * receive.
 *
 * canNum - The can bus number to use.
 * box - The message box to set.
 * enable - Set or clear the enable bit on the box.
 * transmit - Set up for transmit(1) or receive (0).
 * id - The 29-bit CAN id field.  For receive, the mask field will
 *      be and-ed with the incoming id and if it matches this it will
 *      be handled.  For transmit this is the id to send.
 * mask - The mask value to use for receive filtering.  Set to 0 for
 *      transmit.
 * msg - The message data to transmit.  Set to NULL for receive.
 * msglen - The number of bytes to send (up to 8).  Set to 8 for receive
 *      to receive the whole message.
 */
static bool CANSetOneBox(int canNum, int box, int enable, int transmit,
                         uint32_t id, uint32_t mask,
                         uint8_t *msg, unsigned int msglen)
{
    canIfRegs *regs;
    unsigned int i;

    if (canNum >= NUM_CAN_BUSSES)
        return false;
    if (box == 0 || box > 64)
        return false;
    if (msglen > 8)
        return false;

    enable = !!enable;
    transmit = !!transmit;

    regs = CANGetIfRegs(canNum);
    if (!regs)
        return false;

    /* Wait for the register to be free. */
    while ((regs->STAT & 0x80U) == 0x80U)
        ;

    regs->MSK = ((1 << 31) /* Extended ID. */
                 | (1 << 30) /* Use direction bit for filtering. */
                 | (mask & 0x1fffffffu));

    regs->ARB = ((enable << 31)
                 | (1 << 30) /* Extended ID. */
                 | (transmit << 29)
                 | (id & 0x1fffffffu));

    regs->MCTL = ((1 << 12) /* Use mask for filtering. */
                  | (transmit << 11) /* transmit interrupt enable. */
                  | (!transmit << 10) /* receive interrupt enable. */
                  | (transmit << 8) /* TxRqst */
                  | (transmit << 7) /* EoB */
                  | msglen);

    for (i = 0; transmit && i < msglen; i++) {
#if ((__little_endian__ == 1) || (__LITTLE_ENDIAN__ == 1))
        regs->DATx[i] = msg[i];
#else
        regs->DATx[s_canByteOrder[i]] = msg[i];
#endif
    }

    regs->CMD = ((1 << 7) /* Set data in message. */
                 | (1 << 6) /* Set mask. */
                 | (1 << 5) /* Set ARB. */
                 | (1 << 4) /* Set control. */
                 | (1 << 3) /* Clear interrupt pending. */
                 | (transmit << 2) /* Start transmit when committed. */
                 | ((transmit && msglen > 0) << 1)/* Transfer data bytes 0-3. */
                 | (transmit && msglen > 4)); /* Transfer data bytes 4-8. */

    /* Transfer the data. */
    regs->NO = box;

    CANPutIFRegs(canNum);

    return true;
}

/*
 * Extract message data for a message in the given box.  For receive
 * only.
 *
 * canNum - The can bus number to use.
 * box - The message box to get.
 * id - The 29-bit CAN id field that was received.
 * msg - The place to put the message, must be 8 bytes.
 * msglen - The actual message length (0-8).
 * msglost - If a message was lost since the last one, this will be true.
 */
static bool CANGetOneBox(int canNum, int box, uint32_t *id,
                         uint8_t *msg, unsigned int *msglen,
                         bool *msglost)
{
    canIfRegs *regs;
    unsigned int i;
    uint32 regIndex = (box - 1U) >> 5U;
    uint32 bitIndex = 1U << ((box - 1U) & 0x1FU);

    if (canNum >= NUM_CAN_BUSSES)
        return false;
    if (box == 0 || box > 64)
        return false;

    if ((can[canNum].regs->NWDATx[regIndex] & bitIndex) == 0)
        return false;

    regs = CANGetIfRegs(canNum);
    if (!regs)
        return false;

    while ((regs->STAT & 0x80U) == 0x80U)
        ;

    regs->CMD = ((0 << 7) /* Get data from message. */
                 | (1 << 5) /* Get ARB. */
                 | (1 << 4) /* Get MCTL. */
                 | (1 << 3) /* Clear interrupt pending. */
                 | (1 << 2) /* Clear new data present on Rx */
                 | (1 << 1) /* Transfer data bytes 0-3. */
                 | (1)); /* Transfer data bytes 4-8. */

    /* Transfer the data. */
    regs->NO = box;

    while ((regs->STAT & 0x80U) == 0x80U)
        ;

    *id = regs->ARB & 0x1fffffff;
    *msglen = regs->MCTL & 0xf;
    if (*msglen > 8)
        *msglen = 8;
    for (i = 0; i < *msglen; i++) {
#if ((__little_endian__ == 1) || (__LITTLE_ENDIAN__ == 1))
        msg[i] = regs->DATx[i];
#else
        msg[i] = regs->DATx[s_canByteOrder[i]];
#endif
    }
    *msglost = !!(regs->MCTL & 0x4000U);

    CANPutIFRegs(canNum);

    return true;
}

static bool CANSendOneMessage(int canNum, uint32_t id,
                              uint8_t *msg, unsigned int msglen)
{
    bool rv = true;
    struct CANInfo *ci = &can[canNum];

    if (trace_can) {
        unsigned int i;

        printf("Sending CAN message on CAN %d to id 0x%8.8x:\n",
               canNum, id);
        if (msglen > 0) {
            for (i = 0; i < msglen; i++)
                printf(" %2.2x", msg[i]);
            printf("\n");
        }
    }

    ReportToWatchdog(CurrentTaskWD);
    if (xSemaphoreTake(ci->TxDoneSemaphore,
                       WATCHDOG_SHORT_WAIT_TIME) != pdTRUE) {
        /* If we can't get it within a few seconds...trouble */
        ReportToWatchdog(CurrentTaskWD);
        ReportError(CANInUse, false, TaskNumber, 0);
        return false;
    }
    ReportToWatchdog(CurrentTaskWD);

    ci->num_tx_errors = 0;
    ci->tx_error = 0;
    ci->TxWaiting = true;

    CANSetOneBox(canNum, CAN_TRANSMIT_BOX,
                 CAN_BOX_ENABLE, CAN_BOX_TRANSMIT,
                 id, 0, msg, msglen);

    /* Wait for the message to be sent. */
    if (xSemaphoreTake(ci->TxDoneSemaphore,
                       WATCHDOG_SHORT_WAIT_TIME) != pdTRUE) {
        /* If we can't get it within a few seconds...trouble */
        ReportToWatchdog(CurrentTaskWD);
        ReportError(CANInUse, false, TaskNumber, 0);
        return false;
    }
    ReportToWatchdog(CurrentTaskWD);

    if (ci->tx_error)
        rv = false;
    else
        ci->counts.tx_msgs++;

    xSemaphoreGive(ci->TxDoneSemaphore);

    return rv;
}

/*
 * Send a CAN message on the given CAN bus.  This handles long messages
 * and splits them up into individual messages.  See the top of the
 * file for the meaning of the various parameters.
 */
bool CANSend(int canNum, int priority, int type, uint32_t id, int dest,
             uint8_t *msg, unsigned int msglen)
{
    uint32_t id1, id2;
    unsigned int i, left, dlc;
    bool rv = true, variable = false;
    struct CANInfo *ci = &can[canNum];

    if (canNum >= NUM_CAN_BUSSES)
        return false;
    if (priority >= 32)
        return false;
    if (type >= 16)
        return false;
    if (dest >= 16)
        return false;
    if (id >= 256)
        return false;
    if (msglen > 2048)
        return false;
    if (msglen > 8) {
        if (id >= 16 || id == 0)
            return false;
        if (ci->id1_lengths[id] == 0)
            return false;
        if (ci->id1_lengths[id] >= 4096) {
            if (msglen > ci->id1_lengths[id] - 4096)
                /* See the notes above on variable-length messages. */
                return false;
            variable = true;
        } else if (ci->id1_lengths[id] != msglen) {
            return false;
        }
        id1 = id;
        id2 = 0;
        ci->counts.tx_long_msgs++;
    } else if (msglen == 8 && id > 0 && id < 16) {
        /*
         * Special case, a variable-length 8-byte message must be sent
         * as two messages.
         */
        if (ci->id1_lengths[id] >= 4096)
            variable = true;
        id1 = id;
        id2 = 0;
        ci->counts.tx_long_msgs++;
    } else {
        id1 = 0;
        id2 = id;
    }

    if (xSemaphoreTake(ci->TxSemaphore,
                       WATCHDOG_SHORT_WAIT_TIME) != pdTRUE) {
        /* If we can't get it within a few seconds...trouble */
        ReportToWatchdog(CurrentTaskWD);
        ReportError(CANInUse, false, TaskNumber, 0);
        return false;
    }

    left = msglen;
    dlc = 0;
    for (i = 0; i < msglen; i += 8, left -= 8) {
        id = ((priority << 24)
              | (id2 << 16)
              | (type << 12)
              | (ci->myID << 8)
              | (id1 << 4)
              | dest);

        if (left > 8)
            dlc = 8;
        else
            dlc = left;

        if (!CANSendOneMessage(canNum, id, msg + i, dlc)) {
            rv = false;
            break;
        }
        id2++;
    }

    if (rv && (msglen == 0 || (variable && msglen >= 8 && dlc == 8))) {
       /*
         * If it was an empty message, or if it was a variable sized
         * multi-message send and the last message was 8 bytes, we
         * send an empty message to mark the end of the send.
         */
        id = ((priority << 24)
              | (id2 << 16)
              | (type << 12)
              | (ci->myID << 8)
              | (id1 << 4)
              | dest);

        if (!CANSendOneMessage(canNum, id, NULL, 0))
            rv = false;
    }

    xSemaphoreGive(ci->TxSemaphore);

    return rv;
}

/*
 * Used to set up information for a new long message.
 */
static bool CANSetupNewRxMsg(struct CANInfo *ci,
                             int type, int src, int msgid)
{
    if (ci->id1_lengths[msgid] == 0) {
        /* Not a supported long message. */
        ci->counts.rx_msgs_unsupported_long_msg++;
        return false;
    }

    if (ci->id1_lengths[msgid] >= 4096) {
        ci->max_rx_msg_len = ci->id1_lengths[msgid] - 4096;
        ci->long_rx_msg_var = true;
    } else {
        ci->max_rx_msg_len = ci->id1_lengths[msgid];
        ci->long_rx_msg_var = false;
    }
    ci->next_seq = 0;
    ci->long_rx_msg_len = 0;
    ci->long_rx_src = src;
    ci->long_rx_type = type;
    ci->long_rx_id = msgid;
    return true;
}

/*
 * Called when a message comes in, extract and handle it, piecing
 * together long messages as required and delivering them to a
 * user-defined function.
 */
static void CANHandleReceive(int canNum, int box)
{
    uint32_t id;
    uint8_t msg[8], *msgp;
    unsigned int msglen;
    bool msglost;
    int id1, id2, type, src, msgid;
    struct CANInfo *ci = &can[canNum];

    while (canIsRxMessageArrived(ci->regs, box)) {
        if (!CANGetOneBox(canNum, box, &id, msg, &msglen, &msglost))
            break;

        ci->counts.rx_msgs++;

        if (trace_can) {
            unsigned int i;

            printf("Got CAN message from CAN %d box %d id 0x%8.8x:\n",
                   canNum, box, id);
            if (msglen > 0) {
                for (i = 0; i < msglen; i++)
                    printf(" %2.2x", msg[i]);
                printf("\n");
            }
        }

        if (msglost) {
            if (trace_can)
                printf("CAN message lost\n");
            ci->counts.rx_msgs_lost++;
        }

        id2 = (id >> 16) & 0xff;
        id1 = (id >> 4) & 0xf;
        src = (id >> 8) & 0xf;
        type = (id >> 12) & 0xf;
        if (id1 == 0)
            msgid = id2;
        else
            msgid = id1;

        if (id1 == 0 || (id2 == 0 && msglen < 8)) {
            /* Single-buffer message. */
            if (ci->rxhandler)
                ci->rxhandler(canNum,
                              (id >> 24) & 0x1f, /* priority */
                              type,
                              msgid,
                              id & 0xf, /* dest */
                              src,
                              msg, msglen);
        } else {
            if (src != ci->long_rx_src || msgid != ci->long_rx_id ||
                    type != ci->long_rx_type) {
                /*
                 * Got a new source for a long message, abort the
                 * previous one.
                 */

                if (ci->next_seq != 0)
                    ci->counts.rx_msgs_new_source_in_long_msg++;

                if (id2 != 0) {
                    /* Ignore if not the first message. */
                    ci->counts.rx_msgs_invalid_start_seq++;
                    continue;
                }
                if (!CANSetupNewRxMsg(ci, type, src, msgid))
                    continue;
            } else if (id2 == 0) {
                /* Start of a new message. */
                if (!CANSetupNewRxMsg(ci, type, src, msgid))
                    continue;
            } else if (ci->next_seq != id2) {
                ci->next_seq = 0;
                ci->long_rx_msg_len = 0;
                ci->counts.rx_msgs_sequence_mismatch++;
            } else if (ci->long_rx_msg_len + msglen > ci->max_rx_msg_len) {
                ci->next_seq = 0;
                ci->long_rx_msg_len = 0;
                ci->counts.rx_msgs_long_msg_too_long++;
                continue;
            }
            /* Next message of a send. */
            ci->next_seq++;
            memcpy(ci->long_rx_msg + ci->long_rx_msg_len, msg, msglen);
            ci->long_rx_msg_len += msglen;

            if ((!ci->long_rx_msg_var &&
                 ci->long_rx_msg_len == ci->max_rx_msg_len)
                || (ci->long_rx_msg_var && msglen < 8)) {

                msgp = ci->long_rx_msg;
                msglen = ci->long_rx_msg_len;

                if (trace_can) {
                    unsigned int i;

                    printf("Long CAN message from CAN %d box %d id 0x%8.8x:",
                           canNum, box, id);
                    for (i = 0; i < msglen; i++) {
                        if (i % 16 == 0)
                            printf("\n");
                        printf(" %2.2x", msgp[i]);
                    }
                    printf("\n");
                }

                ci->counts.rx_long_msgs++;
                if (ci->rxhandler)
                    ci->rxhandler(canNum,
                                  (id >> 24) & 0x1f, /* priority */
                                  type,
                                  msgid,
                                  id & 0xf, /* dest */
                                  src,
                                  msgp, msglen);
                ci->next_seq = 0;
                ci->long_rx_msg_len = 0;
            }
        }
    }
}

/*
 * Register a handler to be called when a message comes in on a CAN
 * bus.
 */
void CANRegisterReceiveHandler(int canNum, CANReceiveHandler rxhandler)
{
    if (canNum >= NUM_CAN_BUSSES)
        return;
    can[canNum].rxhandler = rxhandler;
}

static void CANSetupRxBoxes(int canNum)
{
    unsigned int i;

    for (i = CAN_RECEIVE_FIRST_BOX;
         i < CAN_RECEIVE_FIRST_BOX + CAN_RECEIVE_NUM_BOXES;
         i++) {
        CANSetOneBox(canNum, i, CAN_BOX_ENABLE, CAN_BOX_RECEIVE,
                     can[canNum].myID, 0xf, NULL, 8);
    }
}

/*
 * Set the CAN bus id of the given can bus.  This is used for
 * transmission and reception.  See the top of the file for what these
 * mean.
 */
void CANSetMyID(int canNum, int id)
{
    static Intertask_Message msg;

    if (canNum >= NUM_CAN_BUSSES)
        return;
    if (id >= 16)
        return;

    /* Tell CANTask to update the id. */
    msg.MsgType = TacADCStartMsg;
    msg.argument = canNum;
    msg.argument2 = id;
    NotifyInterTaskFromISR(ToCANTask, &msg);
}

int CANGetMyID(int canNum)
{
    if (canNum >= NUM_CAN_BUSSES)
        return -1;

    return can[canNum].myID;
}

portTASK_FUNCTION_PROTO(CANTask, pvParameters)
{
    unsigned int i;

    vTaskSetApplicationTaskTag((xTaskHandle) 0, (pdTASK_HOOK_CODE)CANTaskWD);
    /* Enough for all message boxes and some for other purposes. */
    InitInterTask(ToCANTask, CAN_RECEIVE_NUM_BOXES * NUM_CAN_BUSSES + 5);
    ReportToWatchdog(CANTaskWD);

    for (i = 0; i < NUM_CAN_BUSSES; i++)
        CANSetupRxBoxes(i);

    for (;;) {
        Intertask_Message msg;
        int status;

        ReportToWatchdog(CurrentTaskWD);
        status = WaitInterTask(ToCANTask, CENTISECONDS(10), &msg);
        ReportToWatchdog(CurrentTaskWD);

        if (status != 1)
            /* No message. */
            continue;

        switch (msg.MsgType) {
        case CANRxDataMsg:
            CANHandleReceive(msg.argument, msg.argument2);
            break;

        case CANUpdateIDMsg:
            can[msg.argument].myID = msg.argument2;
            CANSetupRxBoxes(msg.argument);
            break;

        case CANErrorMsg:
            if (trace_can)
                printf("CAN %d Error %x errc %x\n",
                       msg.argument, msg.argument2,
                       can[msg.argument].regs->EERC);
            break;

        case CANStatusMsg:
            if (trace_can)
                printf("CAN %d Status %x\n", msg.argument, msg.argument2);
            break;
        }
    }
}

void CANGetCounts(int canNum, struct can_counts *counts)
{
    if (canNum >= NUM_CAN_BUSSES)
        return;

    *counts = can[canNum].counts;
}

/*
 * All errors, status, and message notifications come through here now
 * as the HALCoGen generated code is completely broken.
 *
 * We add our own changes to the generated code in can.c to just call
 * this function on any interrupt with the INT register setting that
 * has already been fetched.
 *
 * HALCoGen must be configured to Disable Automatic Retransmission, we
 * handle that here.  And it must not enable Auto Bus On.
 *
 * The way the chip handles auto-retry on its own also leaves
 * something to be desired.  There no way to control the number of
 * retries or anything like that.  So we do all the error recovery
 * here.
  */
void canStatusChangeNotification(canBASE_t *node, uint32 box)
{
    int canNum;
    static Intertask_Message msg;
    struct CANInfo *ci;
    BaseType_t higherPrioTaskWoken;

    if (node == canREG2)
        canNum = 0;
    else if (node == canREG3)
        canNum = 1;
    else
        return;

    ci = &can[canNum];

    if (box == 0x8000) {
        /*
         * There is an error set in the ES register.  That doesn't
         * necessarily mean there is anything wrong with the message
         * transmission, but we must handle the error.
         */
        uint32_t ES = node->ES;
        uint32_t LEC = ES & 0x7;

        if (trace_can && (ES & ((1 << 8) | (1 << 7) | (1 << 6)))) {
            /* Send error up to be printed. */
            msg.MsgType = CANErrorMsg;
            msg.argument = canNum;
            msg.argument2 = ES;
            NotifyInterTaskFromISR(ToCANTask, &msg);
        }

        if (ES & (1 << 8)) { /* Parity error */
            /* TODO - Have a counter for these. */
        }
        if (ES & (1 << 6)) { /* EWarn */
            /* TODO - In passive state, have a counter for these. */
        }

        if (ci->in_bus_recovery) {
            if (ES & (1 << 7))
                /* Still in bus recovery, don't do anything. */
                goto skip_lec_check;
            ci->in_bus_recovery = false;
        } else if (ES & (1 << 7)) { /* Bus off */
            /*
             * Clear the bus off by asserting Init, waiting for the ack
             * then deasserting Init.  Bus recovery will be started.
             */
            node->CTL |= 1;
            while ((node->CTL & 1) != 1)
                ;
            node->CTL &= ~1;
            ci->in_bus_recovery = true;
        }

        switch (LEC) {
        case 0: /* No error, a message should be pending. */
            box = node->INT;
            break;

        case 1: /* Stuff error, bit stuffing error on receive. */
            ci->counts.rx_err_bit_stuff_count++;
            break;

        case 2: /* Form error, invalid receive frame. */
            ci->counts.rx_err_form_count++;
            break;

        case 5: /* Bit0 Error, used for bus recovery. */
            ci->counts.tx_err_bit0_count++;
            goto handle_tx_err;
        case 4: /* Bit1 Error, when transmitting, another sender overrode. */
            ci->counts.tx_err_bit1_count++;
            goto handle_tx_err;
        case 3: /* ACK Error, transmitted frame didn't get an ACK. */
            ci->counts.tx_err_no_ack_count++;
        handle_tx_err:
            ci->num_tx_errors++;
            if (ci->num_tx_errors < CAN_MAX_TX_ERRORS) {
                /* Retry the write operation. */
                while ((node->IF1STAT & 0x80) == 0x80)
                    ;
                node->IF1CMD = ((1 << 7) /* Write data. */
                                | (1 << 3) /* Clear interrupt */
                                | (1 << 2)); /* Set the TxRqst/NewDat bit. */
                node->IF1NO = CAN_TRANSMIT_BOX;
                while ((node->IF1STAT & 0x80) == 0x80)
                    ;
            } else {
                /* Tell the user that it failed, if they are waiting. */
                if (can[canNum].TxWaiting) {
                    can[canNum].tx_error = LEC;
                    can[canNum].TxWaiting = false;
                    xSemaphoreGiveFromISR(ci->TxDoneSemaphore,
                                          &higherPrioTaskWoken);
                }
            }
            break;

        case 6: /* CRC Error, received message error. */
            ci->counts.rx_err_crc_count++;
            break;

        case 7: /* Nothing has happened since ES last read. */
            break;
        }

    skip_lec_check:
        if (trace_can && LEC > 0 && LEC < 7) {
            msg.MsgType = CANStatusMsg;
            msg.argument = canNum;
            msg.argument2 = LEC;
            NotifyInterTaskFromISR(ToCANTask, &msg);
        }
    }

    if (box > 0 && box <= 64) {
        /* Got a message. */

        /* Disable the interrupt. */
        while ((node->IF1STAT & 0x80) == 0x80)
            ;
        node->IF1CMD = 0x08;
        node->IF1NO = box;
        while ((node->IF1STAT & 0x80) == 0x80)
            ;

        if (box == CAN_TRANSMIT_BOX) {
            BaseType_t higherPrioTaskWoken;

            /* Transmitted message. */
            if (can[canNum].TxWaiting) {
                can[canNum].TxWaiting = false;
                xSemaphoreGiveFromISR(ci->TxDoneSemaphore,
                                      &higherPrioTaskWoken);
            }
        } else {
            static Intertask_Message msg;

            /* Received message */
            msg.MsgType = CANRxDataMsg;
            msg.argument = canNum;
            msg.argument2 = box;
            NotifyInterTaskFromISR(ToCANTask, &msg);
        }
    }
}
