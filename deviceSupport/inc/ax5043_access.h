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

#define NUM_AX5043_SPI_DEVICES 5 // Increase this if Devices added to enum in spiDriver.h (and relevant pin config complete)
typedef enum {AX5043Dev0 = 0,AX5043Dev1,AX5043Dev2,AX5043Dev3,AX5043Dev4,InvalidAX5043Device} AX5043Device;
#ifdef LAUNCHPAD_HARDWARE
#define TX_DEVICE AX5043Dev1
#else
#define TX_DEVICE AX5043Dev4
#endif
#define RX1_DEVICE AX5043Dev0
#define RX2_DEVICE AX5043Dev1
#define RX3_DEVICE AX5043Dev2
#define RX4_DEVICE AX5043Dev3

bool IsRxing(AX5043Device device);
void ax5043StartRx(AX5043Device device, bool antenna_differential);
void ax5043StopRx(AX5043Device device);
void ax5043StartTx(AX5043Device device, bool antenna_differential);
void ax5043StopTx(AX5043Device device);
void ax5043PowerOn(AX5043Device device);
void ax5043PowerOff(AX5043Device device);
uint8_t ax5043_off(AX5043Device device);
uint8_t ax5043_off_xtal(AX5043Device device);
void ax5043WriteReg(AX5043Device device, unsigned int reg, unsigned int val);
void ax5043WriteRegMulti(AX5043Device device, unsigned int firstReg, uint8_t *writeVal,uint8_t length);
void ax5043ReadRegMulti(AX5043Device device, unsigned int firstReg, uint8_t *readVal,uint8_t length);
unsigned int ax5043ReadLongreg(AX5043Device device, unsigned int reg,int bytes);
unsigned int ax5043ReadReg(AX5043Device device, unsigned int reg);
bool ax5043SetClockout(AX5043Device device);

#endif /* DRIVERS_INC_AX5043_ACCESS_H_ */
