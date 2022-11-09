/*
 * inet.h
 *
 *  Created on: Jul 18, 2019
 *      Author: bfisher
 */

#ifndef AMSAT_UTILITIES_INC_INET_H_
#define AMSAT_UTILITIES_INC_INET_H_

uint32_t htonl(uint32_t in);
uint32_t ntohl(uint32_t in);
uint16_t htons(uint16_t in);
uint16_t ntohs(uint16_t in);
uint16_t hton12(uint16_t in);
uint16_t ntoh12(uint16_t in);


uint32_t htotl(uint32_t in);
uint32_t ttohl(uint32_t in);
uint16_t htots(uint16_t in);
uint16_t ttohs(uint16_t in);
uint32_t ttoh24(uint32_t in);


#endif /* AMSAT_UTILITIES_INC_INET_H_ */
