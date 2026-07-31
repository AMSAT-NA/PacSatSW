
#include "bsl_protocol.h"
#include "crc32.h"

uint32_t
bsl_get_uint32(uint8_t *b)
{
    return b[0] | (b[1] << 8) | (b[2] << 16) | (b[3] << 24);
}

void
bsl_set_uint32(uint8_t *b, uint32_t v)
{
    b[0] = v & 0xff;
    b[1] = (v >> 8) & 0xff;
    b[2] = (v >> 16) & 0xff;
    b[3] = (v >> 24) & 0xff;
}

uint16_t
bsl_get_uint16(uint8_t *b)
{
    return b[0] | (b[1] << 8);
}

void
bsl_set_uint16(uint8_t *b, uint16_t v)
{
    b[0] = v & 0xff;
    b[1] = (v >> 8) & 0xff;
}

uint8_t
bsl_send_buffer(struct bsl_protocol *p, uint8_t id, unsigned int len)
{
    uint32_t crc;

    p->txbuffer[0] = p->tx_header_id;
    bsl_set_uint16(p->txbuffer + 1, len + 1);
    p->txbuffer[3] = id;
    crc = ~crc32(p->txbuffer + 3, len + 1);
    bsl_set_uint32(p->txbuffer + len + 4, crc);
    return p->send_msg(p, p->txbuffer, len + 8);
}

void bsl_handle_byte(struct bsl_protocol *p, uint8_t byte)
{
    if (p->msg_ready)
	/* We can only handle one message at a time. */
	return;

    if (p->wait_state == BSL_WAITING_ACK) {
	p->got_ack(p, byte);
	return;
    }

    if (p->rxlen == 0 && byte != p->rx_header_id)
	return;

    if (p->rxlen < BSL_MAX_MSG_SIZE)
	p->rxbuffer[p->rxlen] = byte;
    p->rxlen++;

    if (p->rxlen > 3 && p->rxlen == p->expected_len) {
	p->msg_ready = true;
	p->wait_state = BSL_NOT_WAITING;
	if (p->rx_msg_ready)
	    p->rx_msg_ready(p);
    }
    if (p->rxlen == 3) {
	p->expected_len = bsl_get_uint16(p->rxbuffer + 1);
	p->expected_len += 7;
    }
}

void
bsl_handle_buffer(struct bsl_protocol *p, uint8_t *data, unsigned int len)
{
    unsigned int i;

    for (i = 0; i < len; i++)
	bsl_handle_byte(p, data[i]);
}

void
bsl_devinfo_set_command_interpreter_version(struct bsl_protocol *p,
					    unsigned int value)
{
    bsl_set_uint16(p->device_info + BSL_DEVINFO_COMMAND_INTERPRETER_VERSION,
		   value);
}

void
bsl_devinfo_set_build_id(struct bsl_protocol *p,
			 unsigned int value)
{
    bsl_set_uint16(p->device_info + BSL_DEVINFO_BUILD_ID,
		   value);
}

void
bsl_devinfo_set_application_version(struct bsl_protocol *p,
				    unsigned int value)
{
    bsl_set_uint32(p->device_info + BSL_DEVINFO_APPLICATION_VERSION,
		   value);
}

void
bsl_devinfo_set_active_plug_in_interface_version(struct bsl_protocol *p,
						 unsigned int value)
{
    bsl_set_uint16(p->device_info + BSL_DEVINFO_ACTIVE_PLUG_IN_INTERFACE_VERSION,
		   value);
}

void
bsl_devinfo_set_bsl_max_buffer_size(struct bsl_protocol *p,
				    unsigned int value)
{
    bsl_set_uint16(p->device_info + BSL_DEVINFO_MAX_BUFFER_SIZE,
		   value);
}

void
bsl_devinfo_set_bsl_buffer_start_address(struct bsl_protocol *p,
					 unsigned int value)
{
    bsl_set_uint32(p->device_info + BSL_DEVINFO_BUFFER_START_ADDRESS,
		   value);
}

void
bsl_devinfo_set_bcr_configuration_id(struct bsl_protocol *p,
				     unsigned int value)
{
    bsl_set_uint32(p->device_info + BSL_DEVINFO_BCR_CONFIGURATION_ID,
		   value);
}

void
bsl_devinfo_set_bsl_configuration_id(struct bsl_protocol *p,
				     unsigned int value)
{
    bsl_set_uint32(p->device_info + BSL_DEVINFO_BSL_CONFIGURATION_ID,
		   value);
}


/* Set various values in the device_info array. */
unsigned int
bsl_devinfo_get_command_interpreter_version(struct bsl_protocol *p)
{
    return bsl_get_uint16(p->device_info + BSL_DEVINFO_COMMAND_INTERPRETER_VERSION);
}

unsigned int
bsl_devinfo_get_build_id(struct bsl_protocol *p)
{
    return bsl_get_uint16(p->device_info + BSL_DEVINFO_BUILD_ID);
}

unsigned int
bsl_devinfo_get_application_version(struct bsl_protocol *p)
{
    return bsl_get_uint32(p->device_info + BSL_DEVINFO_APPLICATION_VERSION);
}

unsigned int
bsl_devinfo_get_active_plug_in_interface_version(struct bsl_protocol *p)
{
    return bsl_get_uint16(p->device_info + BSL_DEVINFO_ACTIVE_PLUG_IN_INTERFACE_VERSION);
}

unsigned int
bsl_devinfo_get_bsl_max_buffer_size(struct bsl_protocol *p)
{
    return bsl_get_uint16(p->device_info + BSL_DEVINFO_MAX_BUFFER_SIZE);
}

unsigned int
bsl_devinfo_get_bsl_buffer_start_address(struct bsl_protocol *p)
{
    return bsl_get_uint32(p->device_info + BSL_DEVINFO_BUFFER_START_ADDRESS);
}

unsigned int
bsl_devinfo_get_bcr_configuration_id(struct bsl_protocol *p)
{
    return bsl_get_uint32(p->device_info + BSL_DEVINFO_BCR_CONFIGURATION_ID);
}

unsigned int
bsl_devinfo_get_bsl_configuration_id(struct bsl_protocol *p)
{
    return bsl_get_uint32(p->device_info + BSL_DEVINFO_BSL_CONFIGURATION_ID);
}
