/*
 * radio.h
 *
 *  Created on: Mar 11, 2019
 *      Author: burns
 *
 * This file contains the headers for generic radio routines.
 *
 */

#ifndef DRIVERS_INC_AX5043_ACCESS_H_
#define DRIVERS_INC_AX5043_ACCESS_H_
#include <stdbool.h>
#include "spiDriver.h"


void ax5043StartRx(rfchan chan,
                   uint32_t freq, enum radio_modulation mod);
void ax5043StopRx(rfchan chan);
void ax5043StartTx(rfchan chan,
                   uint32_t freq, enum radio_modulation mod);
void ax5043StopTx(rfchan chan);

void ax5043PowerOn(rfchan chan);
void ax5043PowerOff(rfchan chan);
uint8_t ax5043_off(rfchan chan);
bool ax5043_rxing(rfchan chan);

bool ax5043RxWorking(rfchan chan);
void ax5043Test(rfchan chan);
void ax5043Dump(rfchan chan);

unsigned int ax5043ReadReg(rfchan chan, unsigned int reg);
void ax5043WriteReg(rfchan chan, unsigned int reg, unsigned int val);

#endif /* DRIVERS_INC_AX5043_ACCESS_H_ */
