/*
 * ax5043.h
 *
 *  Created on: Mar 11, 2019
 *      Author: burns
 *
 * This file contains the headers for AX5043 routines that are common across modes and frequencies
 *
 */

#ifndef DRIVERS_INC_AX5043_ACCESS_H_
#define DRIVERS_INC_AX5043_ACCESS_H_
#include <stdbool.h>
#include "spiDriver.h"


typedef uint8_t AX5043Device;

void ax5043StartRx(AX5043Device device,
                   uint32_t freq, enum radio_modulation mod);
void ax5043StopRx(AX5043Device device);
void ax5043StartTx(AX5043Device device,
                   uint32_t freq, enum radio_modulation mod);
void ax5043StopTx(AX5043Device device);

void ax5043PowerOn(AX5043Device device);
void ax5043PowerOff(AX5043Device device);
uint8_t ax5043_off(AX5043Device device);
bool ax5043_rxing(AX5043Device device);

bool ax5043RxWorking(AX5043Device device);
void ax5043Test(AX5043Device device);
void ax5043Dump(AX5043Device device);

unsigned int ax5043ReadReg(AX5043Device device, unsigned int reg);
void ax5043WriteReg(AX5043Device device, unsigned int reg, unsigned int val);

#endif /* DRIVERS_INC_AX5043_ACCESS_H_ */
