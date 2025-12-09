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
 * FIXME - long messages need a buffer to receive them, and you can be
 * receiving up to 15 of them at a time, theoretically.  That's way
 * too much buffer space to leave lying around.  We will have to limit
 * the number of receive buffers somehow.
 */

bool trace_can;

#define CAN_TRANSMIT_BOX 1
#define CAN_RECEIVE_FIRST_BOX 2
#define CAN_RECEIVE_NUM_BOXES 8

#define CAN_BOX_ENABLE 1
#define CAN_BOX_DISABLE 0
#define CAN_BOX_TRANSMIT 1
#define CAN_BOX_RECEIVE 0

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

    volatile int tx_error;

    /* Receive lengths for the long messages, indexed by ID1. */
    uint16_t *id1_lengths;

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
static uint16_t id1_lengths1[16] = {
    0,      0,      0,      0,
    0,      0,      0,      0,
    0,      0,      0,      0,
    0,      0,      0,   4096 + CAN1_MAX_RECV,
};
static uint16_t id1_lengths2[16] = {
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
    can[0].myID = 15;
    can[1].myID = 14;
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
    if (xSemaphoreTake(can[canNum].TxDoneSemaphore,
                       WATCHDOG_SHORT_WAIT_TIME) != pdTRUE) {
        /* If we can't get it within a few seconds...trouble */
        ReportToWatchdog(CurrentTaskWD);
        ReportError(CANInUse, false, TaskNumber, 0);
        return false;
    }
    ReportToWatchdog(CurrentTaskWD);

    can[canNum].tx_error = 0;
    can[canNum].TxWaiting = true;

    CANSetOneBox(canNum, CAN_TRANSMIT_BOX,
                 CAN_BOX_ENABLE, CAN_BOX_TRANSMIT,
                 id, 0, msg, msglen);

    /* Wait for the message to be sent. */
    if (xSemaphoreTake(can[canNum].TxDoneSemaphore,
                       WATCHDOG_SHORT_WAIT_TIME) != pdTRUE) {
        /* If we can't get it within a few seconds...trouble */
        ReportToWatchdog(CurrentTaskWD);
        ReportError(CANInUse, false, TaskNumber, 0);
        return false;
    }
    ReportToWatchdog(CurrentTaskWD);

    if (can[canNum].tx_error)
        rv = false;

    xSemaphoreGive(can[canNum].TxDoneSemaphore);

    return rv;
}

/*
 * Send a CAN message on the given CAN bus.
 */
bool CANSend(int canNum, int priority, int type, uint32_t id, int dest,
             uint8_t *msg, unsigned int msglen)
{
    uint32_t id1, id2;
    unsigned int i, left, dlc;
    bool rv = true, variable = false;

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
        if (can[canNum].id1_lengths[id] == 0)
            return false;
        if (can[canNum].id1_lengths[id] >= 4096) {
            if (msglen > can[canNum].id1_lengths[id] - 4096)
                /* See the notes above on variable-length messages. */
                return false;
	    variable = true;
        } else if (can[canNum].id1_lengths[id] != msglen) {
            return false;
        }
        id1 = id;
        id2 = 0;
    } else if (msglen == 8 && id > 0 && id < 16) {
	/*
	 * Special case, a variable-length 8-byte message must be sent
	 * as two messages.
	 */
        if (can[canNum].id1_lengths[id] >= 4096)
	    variable = true;
        id1 = id;
        id2 = 0;
    } else {
        id1 = 0;
        id2 = id;
    }

    if (xSemaphoreTake(can[canNum].TxSemaphore,
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
              | (can[canNum].myID << 8)
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
              | (can[canNum].myID << 8)
              | (id1 << 4)
              | dest);

        if (!CANSendOneMessage(canNum, id, NULL, 0))
            rv = false;
    }

    xSemaphoreGive(can[canNum].TxSemaphore);

    return rv;
}

static bool CANSetupNewRxMsg(struct CANInfo *ci,
                             int type, int src, int msgid)
{
    if (ci->id1_lengths[msgid] == 0)
        /* Not a supported long message. */
        return false;

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

        if (msglost && trace_can)
            printf("CAN message lost\n");

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
                if (id2 != 0)
                    /* Ignore if not the first message. */
                    continue;
                if (!CANSetupNewRxMsg(ci, type, src, msgid))
                    continue;
            } else if (id2 == 0) {
                /* Start of a new message. */
                if (!CANSetupNewRxMsg(ci, type, src, msgid))
                    continue;
            } else if (ci->next_seq != id2 ||
                    ci->long_rx_msg_len + msglen > ci->max_rx_msg_len) {
                /* Sequence mismatch or message too long, just abort. */
                ci->next_seq = 0;
                ci->long_rx_msg_len = 0;
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
                printf("CAN %d Error %x\n", msg.argument, msg.argument2);
            break;

        case CANStatusMsg:
            if (trace_can)
                printf("CAN %d Status %x\n", msg.argument, msg.argument2);
            break;
        }
    }
}

/*
 * Handle an "error".  Note that you can get here in a non-error
 * situation if there are too many pending errors or the CAN bus is in
 * passive state.  That is generally ok, we handle it gracefully here.
 *
 * Note: This depends on some customization in the lower level
 * interrupt handler to pass all the ES bits.
 */
void canErrorNotification(canBASE_t *node, uint32 notification)
{
    int canNum;
    static Intertask_Message msg;
    BaseType_t higherPrioTaskWoken;

    if (!(notification & ((1 << 7) | (1 << 6))))
        /*
         * Only handle something on a bus off or too many error situation.
         * Thus ignore parity errors only.
         */
        return;

    if (node == canREG2)
        canNum = 0;
    else if (node == canREG3)
        canNum = 1;
    else
        return;

    if (notification & 0x7) {
        /*
         * There was an actual error.  Abort the TX operation.
         */
        while ((node->IF1STAT & 0x80) == 0x80)
            ;

        node->IF1MCTL = 0x00;
        node->IF1ARB = 0x00;
        node->IF1CMD = 0xb0;
        node->IF1NO  = CAN_TRANSMIT_BOX;

        while ((node->IF1STAT & 0x80) == 0x80)
            ;
    }

    if (notification & (1 << 6)) { /* Too many errors (EWarn) */
        /*
         * Clear the counters by asserting Init, waiting for the ack
         * then deasserting Init.
         *
         * FIXME - this doesn't work.  Query done on the TI forum.
         */
        node->CTL |= 1;
        while ((node->CTL & 1) != 1)
            ;
        node->CTL &= ~1;
    }

    /* Now tell the user, if they are waiting. */
    if (can[canNum].TxWaiting) {
        can[canNum].tx_error = notification & 0x7;
        can[canNum].TxWaiting = false;
        xSemaphoreGiveFromISR(can[canNum].TxDoneSemaphore,
                              &higherPrioTaskWoken);
    }

    if (trace_can) {
        /* Send status up to be printed. */
        msg.MsgType = CANErrorMsg;
        msg.argument = canNum;
        msg.argument2 = notification;
        NotifyInterTaskFromISR(ToCANTask, &msg);
    }
}

void canStatusChangeNotification(canBASE_t *node, uint32 notification)  
{
    int canNum;
    static Intertask_Message msg;

    /* This function just sends status up to be printed. */
    if (!trace_can)
        return;

    if (node == canREG2)
        canNum = 0;
    else if (node == canREG3)
        canNum = 1;
    else
        return;

    msg.MsgType = CANStatusMsg;
    msg.argument = canNum;
    msg.argument2 = notification;
    NotifyInterTaskFromISR(ToCANTask, &msg);
}

void canMessageNotification(canBASE_t *node, uint32 messageBox)  
{
    int canNum;

    if (node == canREG2)
        canNum = 0;
    else if (node == canREG3)
        canNum = 1;
    else
        return;

    if (messageBox == CAN_TRANSMIT_BOX) {
        BaseType_t higherPrioTaskWoken;

        /* Transmitted message. */
        if (can[canNum].TxWaiting) {
            can[canNum].TxWaiting = false;
            xSemaphoreGiveFromISR(can[canNum].TxDoneSemaphore,
                                  &higherPrioTaskWoken);
        }
    } else {
        static Intertask_Message msg;

        /* Received message */
        msg.MsgType = CANRxDataMsg;
        msg.argument = canNum;
        msg.argument2 = messageBox;
        NotifyInterTaskFromISR(ToCANTask, &msg);
    }
}
