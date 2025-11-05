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


#define RX1_DEVICE AX5043Dev0
#ifdef LAUNCHPAD_HARDWARE
#define NUM_AX5043_SPI_DEVICES 2
#define TX_DEVICE AX5043Dev1
#else
#define NUM_AX5043_SPI_DEVICES 5
#define RX2_DEVICE AX5043Dev1
#define RX3_DEVICE AX5043Dev2
#define RX4_DEVICE AX5043Dev3
#define TX_DEVICE AX5043Dev4
#endif

typedef enum {
    AX5043Dev0 = 0,
    AX5043Dev1,
#if NUM_AX5043_SPI_DEVICES > 2
    AX5043Dev2,
    AX5043Dev3,
    AX5043Dev4,
#endif
    InvalidAX5043Device
} AX5043Device;


void ax5043StartRx(AX5043Device device, bool antenna_differential);
void ax5043StopRx(AX5043Device device);
void ax5043StartTx(AX5043Device device, bool antenna_differential);
void ax5043StopTx(AX5043Device device);

void ax5043PowerOn(AX5043Device device);
void ax5043PowerOff(AX5043Device device);
uint8_t ax5043_off(AX5043Device device);

bool ax5043RxWorking(AX5043Device device);
void ax5043Test(AX5043Device device);
void ax5043Dump(AX5043Device device);

unsigned int ax5043ReadReg(AX5043Device device, unsigned int reg);
void ax5043WriteReg(AX5043Device device, unsigned int reg, unsigned int val);

#endif /* DRIVERS_INC_AX5043_ACCESS_H_ */
