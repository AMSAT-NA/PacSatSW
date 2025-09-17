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

#ifdef LAUNCHPAD_HARDWARE
typedef enum gu {
    None=-1,
    LED1, LED2, AX5043Interrupt0, AX5043Interrupt1,
    AX5043_Sel0, AX5043_Sel1,
    MRAM0_Sel, MRAM1_Sel, MRAM2_Sel, MRAM3_Sel,
    NumberOfGPIOs
} Gpio_Use;

#elif defined(BLINKY_HARDWARE)
typedef enum gu {
    None=-1,
    LED1, LED2, LED3,
    AX5043_Rx1_Interrupt, AX5043_Rx2_Interrupt, AX5043_Rx3_Interrupt, AX5043_Rx4_Interrupt,
    AX5043_Tx_Interrupt,
    AX5043_Rx1_Sel, AX5043_Rx2_Sel, AX5043_Rx3_Sel, AX5043_Rx4_Sel,
    AX5043_Tx_Sel,
    MRAM0_Sel, MRAM1_Sel, MRAM2_Sel, MRAM3_Sel,
    SSPAPower,AX5043Power,
    NumberOfGPIOs
} Gpio_Use;

#else
typedef enum gu {
    None=-1,
    LED1, LED2, LED3,
    AX5043_Rx1_Interrupt, AX5043_Rx2_Interrupt, AX5043_Rx3_Interrupt, AX5043_Rx4_Interrupt,
    AX5043_Tx_Interrupt,
    AX5043_Rx1_Sel, AX5043_Rx2_Sel, AX5043_Rx3_Sel, AX5043_Rx4_Sel,
    AX5043_Tx_Sel,
    MRAM0_Sel, MRAM1_Sel, MRAM2_Sel, MRAM3_Sel,
    SSPAPower,AX5043Power,
    NumberOfGPIOs
} Gpio_Use;
#endif

#define GPIO_UNUSED false
#define GPIO_OFF false
#define GPIO_ON true

// These are for the WhoAmI gpio
#define GPIO_SECONDARY true
#define GPIO_PRIMARY false

#define NO_MESSAGE (IntertaskMessageType)-1
#define NO_TASK (DestinationTask)-1
bool GPIOEzInit(Gpio_Use whichGpio);
bool GPIOInit(Gpio_Use whichGpio, DestinationTask task,
	      IntertaskMessageType, Gpio_Use auxGPIO);
bool GPIOIsOn(Gpio_Use whichGpio);
void GPIOSetOn(Gpio_Use whichGpio);
void GPIOSetOff(Gpio_Use whichGpio);
void GPIOToggle(Gpio_Use whichGpio);
uint16_t GPIORead(Gpio_Use whichGpio);
void GPIOSetPinDirection(Gpio_Use whichGpio ,bool IsOut);
void GPIOSetTristate(Gpio_Use gpioNum);
void GPIOSetPushPull(Gpio_Use gpioNum);

#endif /* GPIO_H_ */

