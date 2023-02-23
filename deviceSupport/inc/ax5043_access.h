/*
 * ax5043.h
 *
 *  Created on: Mar 11, 2019
 *      Author: burns
 */

#ifndef DRIVERS_INC_AX5043_ACCESS_H_
#define DRIVERS_INC_AX5043_ACCESS_H_
#include <stdbool.h>
#include "spiDriver.h"

// TODO - These are duplicated in ax5043.h - where should they go!!
bool IsRxing(SPIDevice device);
void ax5043StartRx(SPIDevice device);
void ax5043StopRx(SPIDevice device);
void ax5043StartTx(SPIDevice device);
void ax5043StopTx(SPIDevice device);
void ax5043PowerOn(SPIDevice device);
void ax5043PowerOff(SPIDevice device);
uint8_t ax5043_off(SPIDevice device);
uint8_t ax5043_off_xtal(SPIDevice device);
void ax5043WriteReg(SPIDevice device, unsigned int reg, unsigned int val);
void ax5043WriteRegMulti(SPIDevice device, unsigned int firstReg, uint8_t *writeVal,uint8_t length);
void ax5043ReadRegMulti(SPIDevice device, unsigned int firstReg, uint8_t *readVal,uint8_t length);
unsigned int ax5043ReadLongreg(SPIDevice device, unsigned int reg,int bytes);
unsigned int ax5043ReadReg(SPIDevice device, unsigned int reg);
bool ax5043SetClockout(SPIDevice device);

#endif /* DRIVERS_INC_AX5043_ACCESS_H_ */
