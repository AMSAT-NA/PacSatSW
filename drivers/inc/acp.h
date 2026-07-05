
#ifndef ACP_DRIVER_H
#define ACP_DRIVER_H

#define ACP_MSG_SIZE 32

/* Error codes from the ACP message send. */
#define ACP_ERR_START_TIMEOUT -1 /* Antenna IRQ assertion timed out after CS. */
#define ACP_ERR_END_TIMEOUT -2 /* Antenna IRQ deassertion timed out at end. */
#define ACP_ERR_MUTEX_TIMEOUT -3 /* Mutex lock protecting ACP timed out. */

/* Value to send as a message ID if no message is sent. */
#define ACP_MSG_ID_INVALID 0xff

extern volatile bool acp_failed;

void acp_init(void);

int acp_send(const unsigned char *msg);

void acp_runner(void);

#define ACP_I2C_CMD 1
#define ACP_I2C_RSP 2
#define ACP_SET_GPIO 3
#define ACP_GET_GPIO 4
#define ACP_GPIO_VALUE 5
#define MAX_ACP_MSGID 6

extern void (*acp_handlers[MAX_ACP_MSGID])(unsigned char *msg);

/* Number of GPIOs on the ACP. */
#define NUM_ACP_GPIOS 5

/* Maximum data size of an ACP I2C message, both directions. */
#define MAX_I2C_ACP_MSG_SIZE (ACP_MSG_SIZE - 5)

/* I2C Status field in the ACP_I2C_RSP. */

#define I2C_ACP_STATUS_SUCCESS 0
/* Unknown error */
#define I2C_ACP_STATUS_ERROR 1
/* Unknown command code */
#define I2C_ACP_STATUS_UNDEFINEDCMD 2
#define I2C_ACP_STATUS_TIMEOUT 3
#define I2C_ACP_STATUS_CLOCK_TIMEOUT 4
#define I2C_ACP_STATUS_ADDR_NACK 5
#define I2C_ACP_STATUS_DATA_NACK 6
#define I2C_ACP_STATUS_ARB_LOST 7
#define I2C_ACP_STATUS_INCOMPLETE 8
#define I2C_ACP_STATUS_BUS_BUSY 9
#define I2C_ACP_STATUS_CANCEL 10
#define I2C_ACP_STATUS_INVALID_TRANS 11

#endif /* ACP_DRIVER_H */
