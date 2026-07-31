
#include <stddef.h>
#include <string.h>
#include "bsl_host.h"
#include "crc32.h"

static void
call_done(struct bsl_host *h, uint8_t ack, uint8_t *data, unsigned int len)
{
    struct bsl_protocol *p = &h->p;

    h->busy = false;
    p->msg_ready = false;
    h->done(h, ack, data, len);
}	  

void
bsl_host_check(struct bsl_host *h)
{
    struct bsl_protocol *p = &h->p;
    uint8_t rspid, *data, ack;
    unsigned int len;
    uint32_t calc_crc, msg_crc;

    if (!p->msg_ready)
	return;

    if (p->rxlen < p->expected_len) {
	call_done(h, BSL_INTERNAL_RCV_TOO_BIG, NULL, 0);
	return;
    }
    calc_crc = ~crc32(p->rxbuffer + 3, p->rxlen - 7);
    msg_crc = bsl_get_uint32(p->rxbuffer + p->rxlen - 4);
    if (calc_crc != msg_crc) {
	call_done(h, BSL_INTERNAL_CRC_FAILURE, NULL, 0);
	return;
    }

    len = p->rxlen - 8;
    data = p->rxbuffer + 4;

    rspid = p->rxbuffer[3];
    ack = BSL_ACK;

    switch (rspid){
    case BSL_RSP_MESSAGE:
	ack = p->rxbuffer[4];
	if (len < 1)
	    ack = BSL_INTERNAL_INVALID_RESPONSE;
	call_done(h, ack, NULL, 0);
	break;

    case BSL_RSP_DETAILED_ERROR:
	if (len < 3)
	    call_done(h, BSL_INTERNAL_INVALID_RESPONSE, NULL, 0);
	else
	    call_done(h, p->rxbuffer[4], p->rxbuffer + 5, 2);
	break;

    case BSL_RSP_MEMORY_READ_BACK:
	call_done(h, ack, p->rxbuffer + 4, len);
	break;

    case BSL_RSP_GET_DEVICE_INFO:
	if (len < 24) {
	    call_done(h, BSL_INTERNAL_INVALID_RESPONSE, NULL, 0);
	} else {
	    memcpy(p->device_info, data, 24);
	    h->max_buffer_size = bsl_devinfo_get_bsl_max_buffer_size(p);
	    call_done(h, ack, NULL, 0);
	}
	break;

    case BSL_RSP_STANDALONE_VERIFICATION:
	if (len < 4) {
	    call_done(h, BSL_INTERNAL_INVALID_RESPONSE, NULL, 0);
	} else {
	    call_done(h, ack, data, 4);
	}
	break;

    default:
	call_done(h, BSL_INTERNAL_INVALID_RESPONSE, NULL, 0);
	break;
    }    

 out:
}

static uint8_t
send_command(struct bsl_host *h, bsl_done done, uint8_t cmd, unsigned int len,
	     bool expect_rsp)
{
    struct bsl_protocol *p = &h->p;
    uint8_t rv;

    h->done = done;
    h->expect_rsp = expect_rsp;
    h->current_cmd = cmd;
    p->wait_state = BSL_WAITING_ACK;
    p->rxlen = 0;

    rv = bsl_send_buffer(p, cmd, len);
    if (rv) {
	p->wait_state = BSL_NOT_WAITING;
	return rv;
    }

    h->busy = true;

    return BSL_ACK;
}

static void
got_ack(struct bsl_protocol *p, uint8_t ack)
{
    struct bsl_host *h = p->cb_data;

    if (ack || !h->expect_rsp) {
	if (!ack && h->current_cmd == BSL_CMD_CONNECTION) {
	    /*
	     * Do a get device info after a connection completes.
	     * If you immediately send the get device info, some processors
	     * will fail.
	     */
	    h->delay(h, 200);
	    ack = send_command(h, h->done, BSL_CMD_GET_DEVICE_INFO, 0, true);
	    if (!ack)
		return;
	    /* Otherwise report the error. */
	}

	/* Got an error or no message expected. */
	h->busy = false;
	p->wait_state = BSL_NOT_WAITING;
	h->done(h, ack, NULL, 0);
	return;
    }

    p->wait_state = BSL_WAITING_RSP; /* Expecting a response. */
}

void
bsl_host_setup(struct bsl_host *h,
	       uint8_t (*send_msg)(struct bsl_protocol *p,
				   uint8_t *data, unsigned int len),
	       void (*rx_msg_ready)(struct bsl_protocol *p),
	       void (*delay)(struct bsl_host *h, unsigned int time_ms),
	       void *cb_data)
{
    memset(h, 0, sizeof(*h));

    h->p.is_host = true;
    h->p.rx_header_id = BSL_RSP_HEADER;
    h->p.tx_header_id = BSL_CMD_HEADER;
    h->p.cb_data = h;
    h->p.send_msg = send_msg;
    h->p.rx_msg_ready = rx_msg_ready;
    h->p.got_ack = got_ack;
    h->cb_data = cb_data;
    h->delay = delay;
}

void
bsl_host_reset(struct bsl_host *h)
{
    h->p.rxlen = 0;
    h->p.msg_ready = false;
    h->p.wait_state = BSL_NOT_WAITING;
    h->busy = false;
    h->expect_rsp = false;
}

uint8_t
bsl_connect(struct bsl_host *h, bsl_done done)
{
    if (h->busy)
	return BSL_INTERNAL_INUSE;

    return send_command(h, done, BSL_CMD_CONNECTION, 0, false);
}

uint8_t
bsl_unlock(struct bsl_host *h, uint8_t password[32], bsl_done done)
{
    struct bsl_protocol *p = &h->p;

    if (h->busy)
	return BSL_INTERNAL_INUSE;

    memcpy(p->txbuffer + 4, password, 32);
    return send_command(h, done, BSL_CMD_UNLOCK_BOOTLOADER, 32, true);
}

uint8_t
bsl_erase_all(struct bsl_host *h, bsl_done done)
{
    if (h->busy)
	return BSL_INTERNAL_INUSE;

    return send_command(h, done, BSL_CMD_MASS_ERASE, 0, true);
}

uint8_t
bsl_erase_range(struct bsl_host *h,
		uint32_t start_addr, uint32_t end_addr,
		bsl_done done)
{
    struct bsl_protocol *p = &h->p;

    if (h->busy)
	return BSL_INTERNAL_INUSE;

    if (end_addr <= start_addr)
	return BSL_INTERNAL_ERR;

    bsl_set_uint32(p->txbuffer + 4, start_addr);
    bsl_set_uint32(p->txbuffer + 8, end_addr);
    return send_command(h, done, BSL_CMD_FLASH_RANGE_ERASE, 8, true);
}

uint8_t
bsl_write_data(struct bsl_host *h,
	       uint32_t addr, uint8_t *data, uint32_t len,
	       bsl_done done)
{
    struct bsl_protocol *p = &h->p;

    if (h->busy)
	return BSL_INTERNAL_INUSE;

    if (len > BSL_MAX_BUFFER_SIZE || len > h->max_buffer_size)
	return BSL_INTERNAL_TOO_BIG;

    bsl_set_uint32(p->txbuffer + 4, addr);
    memcpy(p->txbuffer + 8, data, len);
    return send_command(h, done, BSL_CMD_PROGRAM_DATA, len + 4, true);
}

uint8_t
bsl_read_data(struct bsl_host *h,
	      uint32_t addr, uint32_t len,
	      bsl_done done)
{
    struct bsl_protocol *p = &h->p;

    if (h->busy)
	return BSL_INTERNAL_INUSE;

    bsl_set_uint32(p->txbuffer + 4, addr);
    bsl_set_uint32(p->txbuffer + 8, len);
    return send_command(h, done, BSL_CMD_MEMORY_READ_BACK, 8, true);
}

uint8_t
bsl_validate_data(struct bsl_host *h,
		  uint32_t addr, uint32_t len, uint32_t *crc,
		  bsl_done done)
{
    struct bsl_protocol *p = &h->p;

    if (h->busy)
	return BSL_INTERNAL_INUSE;

    bsl_set_uint32(p->txbuffer + 4, addr);
    bsl_set_uint32(p->txbuffer + 8, len);
    return send_command(h, done, BSL_CMD_STANDALONE_VERIFICATION, 8, true);
}

uint8_t
bsl_factory_reset(struct bsl_host *h, bsl_done done)
{
    if (h->busy)
	return BSL_INTERNAL_INUSE;

    return send_command(h, done, BSL_CMD_START_APPLICATION, 0, false);
}

uint8_t
bsl_start_app(struct bsl_host *h, bsl_done done)
{
    if (h->busy)
	return BSL_INTERNAL_INUSE;

    return send_command(h, done, BSL_CMD_START_APPLICATION, 0, false);
}

uint8_t
bsl_change_baud(struct bsl_host *h, uint8_t baud, bsl_done done)
{
    struct bsl_protocol *p = &h->p;

    if (h->busy)
	return BSL_INTERNAL_INUSE;

    p->txbuffer[4] = baud;
    return send_command(h, done, BSL_CMD_CHANGE_BAUD_RATE, 1, false);
}
