/*
 * ADS7828.h
 *
 *  Created on: Oct 23, 2013
 *      Author: Mike McCann KB2GHZ
 */

#ifndef ADS7828_H_
#define ADS7828_H_

#include "I2cAddresses.h"
#include "i2cDriver.h"
/* ADS7828 channels */
#define CH0 0x00
#define CH1 0x40
#define CH2 0x10
#define CH3 0x50
#define CH4 0x20
#define CH5 0x60
#define CH6 0x30
#define CH7 0x70

/* ADC input modes */
#define SINGLE_ENDED  0x80
#define DIFFERENTIAL  0x00

/* ADS7828 Power-Down and reference selection */
#define PD00 0x00   /* power down between A/D conversions */
#define PD01 0x04   /* internal reference OFF and A/D converter ON */
#define PD10 0x08   /* internal reference ON and A/D converter OFF */
#define PD11 0x0C   /* internal reference ON and A/D converter ON  */

#define INTERNAL_REFERENCE_CONVERTER_ON  PD11
#define ADC_OFF_INTERNAL_REF_ON PD10

void I2CDevicePoll();
void getADCchannels(int num, I2cBusNum port, uint8_t I2Caddress, uint16_t *storage);
bool RTTempIsOk(void);
bool RTCIsOk(void);

#endif /* ADS7828_H_ */
