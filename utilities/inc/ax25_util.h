/*
 * ax25_util.h
 *
 *  Created on: Feb 20, 2023
 *      Author: g0kla
 */

#ifndef UTILITIES_INC_AX25_UTIL_H_
#define UTILITIES_INC_AX25_UTIL_H_

int decode_call(uint8_t *c, char *call);
int encode_call(char *name, uint8_t *buf, int final_call, uint8_t command);

int test_ax25_util_decode_calls();

#endif /* UTILITIES_INC_AX25_UTIL_H_ */
