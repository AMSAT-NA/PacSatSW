/*
 * ADS7828.h
 *
 *  Created on: Oct 23, 2013
 *      Author: Mike McCann KB2GHZ
 */

#ifndef I2CPOLL_H_
#define I2CPOLL_H_

#include "I2cAddresses.h"
#include "i2cDriver.h"

void I2CDevicePoll();
bool TxTempIsOk(void);
bool CpuTempIsOk(void);
bool RTCIsOk(void);

#endif /* ADS7828_H_ */
