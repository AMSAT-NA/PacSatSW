/*
 * ao_rec_rx.h
 *
 *  Created on: Feb 28, 2017
 *      Author: fox
 */

#ifndef AO_REC_RX_H_
#define AO_REC_RX_H_

#define AO_FEC_DECODE_CRC_OK	0x80	/* stored in out[out_len-1] */


uint8_t
ao_fec_decode(const uint8_t *in, uint16_t len, uint8_t *out, uint8_t out_len, uint16_t (*callback)(void));


#endif /* AO_REC_RX_H_ */
