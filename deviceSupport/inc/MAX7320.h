/*
 * RFSwitch.h
 *
 *  Created on: Sep 17, 2019
 *      Author: bfisher
 */

#ifndef DEVICESUPPORT_INC_RFSWITCH_H_
#define DEVICESUPPORT_INC_RFSWITCH_H_

#include <pacsat.h>
#include "I2cAddresses.h"

bool ReadRFSwitch(uint8_t *retval);
bool WriteRFSwitch(uint8_t sendVal);
bool RFSwitchIsOk(void);


#define RF_SWITCH_LBAND (1<<4)
#define RF_SWITCH_CBAND (1<<3)
#define RF_SWITCH_SBAND (1<<2)
#define RF_SWITCH_DEFAULT RF_SWITCH_LBAND


#endif /* DEVICESUPPORT_INC_RFSWITCH_H_ */
