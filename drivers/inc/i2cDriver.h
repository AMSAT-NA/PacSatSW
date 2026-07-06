/*
 * spi.h
 *
 *  Created on: Feb 19, 2012
 *      Author: Burns
 */

#ifndef I2C_DRIVER_H_
#define I2C_DRIVER_H_

/* Useful typedefs */
#include <pacsat.h>
#include "stdint.h"
#include "I2cAddresses.h"

typedef enum {
    I2C1=0, /* I2C bus hosted on the TMS570. */
#ifdef AFSK_HARDWARE3
    /* Next three are I2C busses on the ACP. */
    I2C2,
    I2C3,
    I2C4,
#endif
    NUM_I2C_BUSSES
} I2cBusNum;

/* External Functions */
void i2c_init(void);

bool I2cSendCommand(I2cBusNum device, uint32_t address, void *sndBuffer,
		    uint16_t sndLength, void *rcvBuffer, uint16_t rcvLength);

#endif /* SPI_H_ */
