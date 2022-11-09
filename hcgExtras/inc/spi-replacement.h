/*
 * spi-replacement.h
 *
 *  Created on: Mar 14, 2019
 *      Author: burns
 */

#ifndef HCGEXTRAS_INC_SPI_REPLACEMENT_H_
#define HCGEXTRAS_INC_SPI_REPLACEMENT_H_
#include "spi.h"
void spiGetDataByte(spiBASE_t *spi, const spiDAT1_t *dataconfig_t, uint32 blocksize, uint8 * destbuff);
void spiSendDataByte(spiBASE_t *spi, const spiDAT1_t *dataconfig_t, uint32 blocksize, uint8 * srcbuff);
void spiSendAndGetDataByte(spiBASE_t *spi, const spiDAT1_t *dataconfig_t, uint32 blocksize, uint8 * srcbuff, uint8 * destbuff);





#endif /* HCGEXTRAS_INC_SPI_REPLACEMENT_H_ */
