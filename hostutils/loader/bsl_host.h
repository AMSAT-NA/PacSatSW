#ifndef BSL_HOST_H
#define BSL_HOST_H

#include "bsl_protocol.h"

struct bsl_host;

/*
 * Used for done callbacks to the various functions.  The result is
 * BCL_RSP_ERR_xxx or a BSL_ERR_xxx, or BSL_INTERNAL_xxx.
 */
typedef void (*bsl_done)(struct bsl_host *h, uint8_t result,
			 uint8_t *data, unsigned int len);

struct bsl_host {
    struct bsl_protocol p;

    bsl_done done;

    uint8_t current_cmd;

    /* Read from get device id. */
    unsigned int max_buffer_size;

    void *cb_data;

    bool busy; /* Currently processing a message. */

    /*
     * Is the message we are currently receiving expecting response
     * after the ack?
     */
    bool expect_rsp;

    /* Do a delay for the given number of milliseconds. */
    void (*delay)(struct bsl_host *h, unsigned int time_ms);
};

/*
 * Set up a bsl_host structure, including the protocol structure in it.
 */
void bsl_host_setup(struct bsl_host *h,
		    uint8_t (*send_msg)(struct bsl_protocol *p,
					uint8_t *data, unsigned int len),
		    void (*rx_msg_ready)(struct bsl_protocol *p),
		    void (*delay)(struct bsl_host *h, unsigned int time_ms),
		    void *cb_data);

/*
 * Reset the data handling for the host.
 */
void bsl_host_reset(struct bsl_host *h);

/*
 * Check to see if anything needs to be done, should be called after
 * the rx_msg_ready() callback is called, or if you don't do that,
 * after every bsl_handle_xxx().
 */
void bsl_host_check(struct bsl_host *h);

/*
 * All the functions below return BSL_ACK (0) if the operation was
 * successfully started, or BSL_INTERNAL_xxx if something happened.
 */

/* Connect to the target. */
uint8_t bsl_connect(struct bsl_host *h, bsl_done done);

/* Unlock the bootloader so we can write to the FLASH. */
uint8_t bsl_unlock(struct bsl_host *h, uint8_t password[32], bsl_done done);

/*
 * Erase all memory.
 */
uint8_t bsl_erase_all(struct bsl_host *h, bsl_done done);

/*
 * Erase a memory range.
 */
uint8_t bsl_erase_range(struct bsl_host *h,
			uint32_t start_addr, uint32_t end_addr,
			bsl_done done);

/*
 * Write the given data.
 */
uint8_t bsl_write_data(struct bsl_host *h,
		       uint32_t addr, uint8_t *data, uint32_t len,
		       bsl_done done);

/*
 * Write the given data.
 */
uint8_t bsl_read_data(struct bsl_host *h,
		      uint32_t addr, uint32_t len,
		      bsl_done done);

/*
 * Return the CRC32 for the given data range.
 */
uint8_t bsl_validate_data(struct bsl_host *h,
			  uint32_t addr, uint32_t len, uint32_t *crc,
			  bsl_done done);

/* Start the application. */
uint8_t bsl_start_app(struct bsl_host *h, bsl_done done);

/* Change the baud rate. */
uint8_t bsl_change_baud(struct bsl_host *h, uint8_t baud, bsl_done done);

#endif /* BSL_HOST_H */
