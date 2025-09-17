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
/* If AX5043 devices added to this enum then updates need to be made to ax5043_access.h */
#ifdef LAUNCHPAD_HARDWARE
typedef enum {MRAM0Dev = 0, MRAM1Dev = 1, MRAM2Dev=2, MRAM3Dev=3, AX5043Dev0 = 4,AX5043Dev1, InvalidSPI} SPIDevice;
#else
typedef enum {MRAM0Dev = 0, MRAM1Dev = 1, MRAM2Dev=2, MRAM3Dev=3, Rx1AX5043Dev = 4,
	      Rx2AX5043Dev,Rx3AX5043Dev,Rx4AX5043Dev,TxAX5043Dev, InvalidSPI} SPIDevice;
#endif
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
