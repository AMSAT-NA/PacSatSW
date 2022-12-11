/*
 * spi.h
 *
 *  Created on: Feb 19, 2012
 *      Author: Burns
 */

#ifndef SPI_DRIVER_H_
#define SPI_DRIVER_H_

/* Useful typedefs */
#include <pacsat.h>
#include "stdint.h"
typedef enum {MRAM0Dev = 0, MRAM1Dev = 1, DCTDev = 2, InvalidSPI} SPIDevice;
#define NUMBER_OF_BUSES 3
typedef union {
    uint8_t byte[4];
    uint32_t word;
}ByteToWord;

/* External Functions */
void SPIInit(SPIDevice device);

bool SPISendCommand(SPIDevice device, uint32_t command,uint8_t comLength, void *sndBuffer,
		uint16_t sndLength,  void *rcvBuffer, uint16_t rcvLength);

/*
 * SPIBidirectional sends and receives the same number of bytes simultaneously.  This is mostly useful for
 * the AX5043 where the first transmitted byte is the command and the first received byte is the status.
 * In any case, the txBuffer and the rxBuffer can be the same (but of course the command will be over-written
 * by the status.
 */
bool SPIBidirectional(SPIDevice device, void *txBuffer, void *rxBuffer, uint16_t length);


#define SPI_NONE 0 /* Use for length if there is no command, send data or receive data */

#endif /* SPI_H_ */
