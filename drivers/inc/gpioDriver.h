
/*
 * gpio.h
 *
 *  Created on: Sep 16, 2012
 *      Author: fox
 */
#ifndef GPIO_H_
#define GPIO_H_
#include <pacsat.h>
#include "gio.h"
#include "config.h"
#include "interTaskNotify.h"


typedef enum gu {None=-1,
	 LED1,LED2,LED3,DCTInterrupt,CommandStrobe,CommandBits,
	 NumberOfGPIOs
} Gpio_Use;

#define GPIO_UNUSED false
#define GPIO_OFF false
#define GPIO_ON true

// These are for the WhoAmI gpio
#define GPIO_SECONDARY true
#define GPIO_PRIMARY false

#define NO_MESSAGE (IntertaskMessageType)-1
#define NO_TASK (DestinationTask)-1
bool GPIOEzInit(Gpio_Use whichGpio);
bool GPIOInit( Gpio_Use whichGpio,DestinationTask task, IntertaskMessageType, Gpio_Use auxGPIO);
bool GPIOIsOn(Gpio_Use whichGpio);
void GPIOSetOn(Gpio_Use whichGpio);
void GPIOSetOff(Gpio_Use whichGpio);
void GPIOToggle(Gpio_Use whichGpio);
uint16_t GPIORead(Gpio_Use whichGpio);
void GPIOSetPinDirection(gioPORT_t *regPtr,int pinNum,bool IsOut);
void GPIOSetTristate(Gpio_Use gpioNum);
void GPIOSetPushPull(Gpio_Use gpioNum);

#endif /* GPIO_H_ */

