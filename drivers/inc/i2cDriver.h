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
    I2C1=0,
    I2C2
}I2cBusNum;

#define I2c1Port I2C1
#define I2c2Port I2C2

/* External Functions */
void I2cInit(I2cBusNum thisBusNumber);
void I2cResetBus(uint32_t busNum,bool isError);

bool I2cSendCommand(I2cBusNum device, uint32_t address, void *sndBuffer,
		uint16_t sndLength,  void *rcvBuffer, uint16_t rcvLength);

#define I2C_NONE 0 /* Use for length if there is no send data or receive data */

#endif /* SPI_H_ */
