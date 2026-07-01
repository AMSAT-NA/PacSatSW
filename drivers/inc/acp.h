
#ifndef ACP_DRIVER_H
#define ACP_DRIVER_H

#define ACP_MSG_SIZE 32

#define ACP_ERR_START_TIMEOUT -1 /* Antenna IRQ assertion timed out after CS. */
#define ACP_ERR_END_TIMEOUT -2 /* Antenna IRQ deassertion timed out at end. */
#define ACP_ERR_MUTEX_TIMEOUT -3 /* Mutex lock protecting ACP timed out. */

#define ACP_MSG_ID_INVALID 0xff

extern volatile bool acp_failed;

extern void (*acp_rx_msg_handler)(const unsigned char msg[ACP_MSG_SIZE]);

void acp_init(void);

int acp_send(const unsigned char *msg);

void acp_runner(void);

#endif /* ACP_DRIVER_H */
