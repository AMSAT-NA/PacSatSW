#ifndef BSL_PROTOCOL_H
#define BSL_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

#define BSL_MAX_BUFFER_SIZE 1024
#define BSL_MAX_MSG_SIZE (BSL_MAX_BUFFER_SIZE + 8)

/*
 * Commands from the host to the target.
 */
#define BSL_CMD_CONNECTION 0X12
#define BSL_CMD_UNLOCK_BOOTLOADER 0X21
#define BSL_CMD_FLASH_RANGE_ERASE 0X23
#define BSL_CMD_MASS_ERASE 0X15
#define BSL_CMD_PROGRAM_DATA 0X20
#define BSL_CMD_PROGRAM_DATA_FAST 0X24
#define BSL_CMD_MEMORY_READ_BACK 0X29
#define BSL_CMD_FACTORY_RESET 0X30
#define BSL_CMD_GET_DEVICE_INFO 0X19
#define BSL_CMD_STANDALONE_VERIFICATION 0X26
#define BSL_CMD_START_APPLICATION 0X40
#define BSL_CMD_CHANGE_BAUD_RATE 0X52

/* First bytes of commands and responses. */
#define BSL_CMD_HEADER 0x80
#define BSL_RSP_HEADER 0x08

/*
 * Single byte responses to all packets.  Use for protocol errors, not
 * for operational responses.
 */
#define BSL_ACK 0x00
#define BSL_ERR_HEADER_INCORRECT 0x51
#define BSL_ERR_CHECKSUM_INCORRECT 0x52
#define BSL_ERR_PACKET_SIZE_ZERO 0x53
#define BSL_ERR_PACKET_SIZE_TOO_BIG 0x54
#define BSL_ERR_UNKNOWN_ERROR 0x55
#define BSL_ERR_UNKNOWN_BAUD_RATE 0x56

/*
 * Responses from the target to the host.
 */
#define BSL_RSP_MEMORY_READ_BACK 0X30
#define BSL_RSP_GET_DEVICE_INFO 0X31
#define BSL_RSP_STANDALONE_VERIFICATION 0X32
#define BSL_RSP_MESSAGE 0X3B
#define BSL_RSP_DETAILED_ERROR 0X3A

/*
 * Errors in message response.
 */
#define BSL_RSP_SUCCESS 0x00
#define BSL_RSP_ERR_BSL_LOCKED 0X01
#define BSL_RSP_ERR_BSL_PASSWORD 0X02
#define BSL_RSP_ERR_MULTIPLE_BSL_PASSWORD 0X03
#define BSL_RSP_ERR_UNKNOWN_COMMAND 0X04
#define BSL_RSP_ERR_INVALID_MEMORY_RANGE 0X05
#define BSL_RSP_ERR_INVALID_COMMAND 0X06
#define BSL_RSP_ERR_FACTORY_RESET_DISABLED 0X07
#define BSL_RSP_ERR_FACTORY_RESET_PASSWORD 0X08
#define BSL_RSP_ERR_READ_OUT 0X09
#define BSL_RSP_ERR_INVALID_ALIGNMENT 0X0A
#define BSL_RSP_ERR_INVALID_LENGTH 0X0B

/* Detailed error in detailed error response. */
#define BSL_ERR_DETAILED 0xf0

/* Baud rates for change baud rate command. */
#define BSL_BAUD_4800 1
#define BSL_BAUD_9600 2
#define BSL_BAUD_19200 3
#define BSL_BAUD_38400 4
#define BSL_BAUD_57600 5
#define BSL_BAUD_115200 6
#define BSL_BAUD_1000000 7
#define BSL_BAUD_2000000 8
#define BSL_BAUD_3000000 9

/* Offsets into device_info */
#define BSL_DEVINFO_COMMAND_INTERPRETER_VERSION 0
#define BSL_DEVINFO_BUILD_ID 2
#define BSL_DEVINFO_APPLICATION_VERSION 4
#define BSL_DEVINFO_ACTIVE_PLUG_IN_INTERFACE_VERSION 8
#define BSL_DEVINFO_MAX_BUFFER_SIZE 10
#define BSL_DEVINFO_BUFFER_START_ADDRESS 12
#define BSL_DEVINFO_BCR_CONFIGURATION_ID 16
#define BSL_DEVINFO_BSL_CONFIGURATION_ID 20

/*
 * Internal errors for things that go wrong in this software.
 */
#define BSL_INTERNAL_ERR 0x70
#define BSL_INTERNAL_INUSE 0x71
#define BSL_INTERNAL_TOO_BIG 0x72
#define BSL_INTERNAL_TIMED_OUT 0x73
#define BSL_INTERNAL_INVALID_RESPONSE 0x74
#define BSL_INTERNAL_RCV_TOO_BIG 0x75
#define BSL_INTERNAL_CRC_FAILURE 0x76

struct bsl_protocol {
    bool is_host;
    unsigned int rx_header_id; /* 0x80 on the target, 0x08 on the host. */
    unsigned int tx_header_id; /* 0x08 on the target, 0x80 on the host. */

    uint8_t rxbuffer[BSL_MAX_MSG_SIZE];
    unsigned int rxlen;
    unsigned int expected_len;
    volatile bool msg_ready;

    /* For the host waiting on a response from the target. */
    enum wait_state {
	BSL_NOT_WAITING = 0,
	BSL_WAITING_ACK,
	BSL_WAITING_RSP
    } wait_state;

    uint8_t txbuffer[BSL_MAX_MSG_SIZE];

    uint8_t device_info[24];

    /* For use by the user. */
    void *cb_data;

    /*
     * Send a message to the other end.  Returns BSL_ACK (0) for
     * success or BSL_INTERNAL_xxx on an error.
     */
    uint8_t (*send_msg)(struct bsl_protocol *p,
			uint8_t *data, unsigned int len);

    /*
     * Called when a full message is ready from the bsl_handle_xxx()
     * routines.  Called from that context, so may not be able to
     * schedule.  If this is called, bsl_xxx_check() should be called
     * in a context that can block. Optional, may be NULL.
     */
    void (*rx_msg_ready)(struct bsl_protocol *p);

    /*
     * For host only, got an ack.  Called from a low-level context.
     * Required on a host, set to NULL on a target.
     */
    void (*got_ack)(struct bsl_protocol *p, uint8_t ack);
};

uint32_t bsl_get_uint32(uint8_t *b);
void bsl_set_uint32(uint8_t *b, uint32_t v);
uint16_t bsl_get_uint16(uint8_t *b);
void bsl_set_uint16(uint8_t *b, uint16_t v);

/* Put data in p->txbuffer[4..len + 4] and call this to send.  Returns */
uint8_t bsl_send_buffer(struct bsl_protocol *p, uint8_t id, unsigned int len);

/*
 * Handle a byte from the serial port.  This only stores the data, and
 * can be called from an interrupt handler or other constrained context.
 * You must schedule bsl_check() to be called after adding data.
 */
void bsl_handle_byte(struct bsl_protocol *p, uint8_t byte);

/*
 * Handle an array of bytes from the serial port, just calls
 * bsl_handle_byte() on each byte.
 */
void bsl_handle_buffer(struct bsl_protocol *p, uint8_t *data, unsigned int len);

/* Set various values in the device_info array. */
void bsl_devinfo_set_command_interpreter_version(struct bsl_protocol *p,
						 unsigned int value);
void bsl_devinfo_set_build_id(struct bsl_protocol *p,
			      unsigned int value);
void bsl_devinfo_set_application_version(struct bsl_protocol *p,
					 unsigned int value);
void bsl_devinfo_set_active_plug_in_interface_version(struct bsl_protocol *p,
						      unsigned int value);
void bsl_devinfo_set_bsl_max_buffer_size(struct bsl_protocol *p,
					 unsigned int value);
void bsl_devinfo_set_bsl_buffer_start_address(struct bsl_protocol *p,
					      unsigned int value);
void bsl_devinfo_set_bcr_configuration_id(struct bsl_protocol *p,
					  unsigned int value);
void bsl_devinfo_set_bsl_configuration_id(struct bsl_protocol *p,
					  unsigned int value);

/* Get various values in the device_info array. */
unsigned int bsl_devinfo_get_command_interpreter_version(struct bsl_protocol *p);
unsigned int bsl_devinfo_get_build_id(struct bsl_protocol *p);
unsigned int bsl_devinfo_get_application_version(struct bsl_protocol *p);
unsigned int bsl_devinfo_get_active_plug_in_interface_version(struct bsl_protocol *p);
unsigned int bsl_devinfo_get_bsl_max_buffer_size(struct bsl_protocol *p);
unsigned int bsl_devinfo_get_bsl_buffer_start_address(struct bsl_protocol *p);
unsigned int bsl_devinfo_get_bcr_configuration_id(struct bsl_protocol *p);
unsigned int bsl_devinfo_get_bsl_configuration_id(struct bsl_protocol *p);

#endif /* BSL_PROTOCOL_H */
