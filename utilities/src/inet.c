/*
 * inet.c
 *
 *  Created on: Jul 18, 2019
 *      Author: bfisher
 */

/*
 * Network order routine which might be standard somewhere, but not with Atollic C
 */
#include <pacsat.h>
#include <stdint.h>
#include <stdio.h>

// Host to network (i.e. big endian)
uint32_t htonl(uint32_t in){
	return in;
}
uint32_t ntohl(uint32_t in){
	return in;
}
uint16_t htons(uint16_t in){
	return in;
}
uint16_t ntohs(uint16_t in){
	return in;
}

// Host to telemetry (i.e. to small endian)

uint32_t htotl(uint32_t in){
    uint32_t network,host;
    uint8_t *pNet,*pHost;
    int i,j;
    host = in;
    pNet = (uint8_t *)&network;
    pHost = (uint8_t *)&host;
    for(i=0,j=3;i<4;i++,j--){
        pNet[i] = pHost[j];
    }
    return network;
}
uint32_t ttohl(uint32_t in){
    uint32_t network,host;
    uint8_t *pNet,*pHost;
    int i,j;
    network = in;
    pNet = (uint8_t *)&network;
    pHost = (uint8_t *)&host;
    for(i=0,j=3;i<4;i++,j--){
        pHost[i] = pNet[j];
    }
    return host;
}
uint32_t ttoh24(uint32_t in){
    /*
     * This is horrendous for a little endian guy.  We had a 24-bit quantity
     * but it was passed into the routine as 32 bits.  Thus a 0 byte was added
     * AT THE LOWEST ADDRESS (i.e. most significant since this is big endian).  We
     * need to ignore that byte.  So we need to move bytes 1,2 and 3 to bytes 3,2,1
     * in the host version.
     */
    uint32_t telem,host=0;
    uint8_t *pTelem,*pHost;
    int i,j;
    telem = in;
    pTelem = (uint8_t *)&telem;
    pHost = (uint8_t *)&host;
    for(i=3,j=1;j<4;i--,j++){
        //printf("host[%d] get net[%d] which is %x\n",i,j,pTelem[j]);
        pHost[i] = pTelem[j];
    }
    return host;
}


uint16_t htots(uint16_t in){
    uint16_t network,host;
    uint8_t *pNet,*pHost;
    int i,j;
    host = in;
    pNet = (uint8_t *)&network;
    pHost = (uint8_t *)&host;
    for(i=0,j=1;i<2;i++,j--){
        pNet[i] = pHost[j];
    }
    return network;
}
uint16_t ttohs(uint16_t in){
    uint16_t network,host;
    uint8_t *pNet,*pHost;
    int i,j;
    network = in;
    pNet = (uint8_t *)&network;
    pHost = (uint8_t *)&host;
    for(i=0,j=1;i<2;i++,j--){
        pHost[i] = pNet[j];
    }
    return host;
}


