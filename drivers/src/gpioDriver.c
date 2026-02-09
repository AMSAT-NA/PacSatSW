/*
 * gpio.c
 *
 *  Created on: Sep 16, 2012
 *      Author: Burns Fisher
 *
 *  This is the glue (interface) layer code to use a general purpose I/O device for simple
 *  single-bit input and output.  Copied liberally from examples that are part of
 *
    FreeRTOS V7.0.2 - Copyright (C) 2011 Real Time Engineers Ltd.
 */

/*HalCoGen headers*/
#include <pacsat.h>
#include "gio.h"
#include "spi.h"
#include "het.h"
#include "i2c.h"
#include "sci.h"
#include "reg_system.h"

/* Golf headers*/
#include "interTaskNotify.h"
#include "hardwareConfig.h"
#include "gpioDriver.h"


/* FreeRTOS Headers */
#include "FreeRTOS.h"
#include "os_semphr.h"

/*
 * Adding a GPIO...
 *
 * To add a GPIO, first add the name to the Gpio_Use enum(s) in the
 * gpioDriver.h file.  There are different ones for different
 * hardware, so add it to the right ones.
 *
 * Next add a GPIOInfo structure under the proper hardware ifdefs
 * further down in the file.  This structure must be const.
 *
 * Then add your new GPIOInfo struct to GPIOInfoStructures[] for the given
 * hardware and to to GPIONames[].
 */

/*
 * Base functions for a GPIO type, like the PORT ones (gio, het, spi),
 * CAN, eclk, dummy, etc.
 */
typedef struct GPIOFuncs {
    /* Set the particular pin to the given raw value. */
    void (*setBit)(const GPIOHandler *h, uint16_t pinNum, uint16_t val);

    /* Get the raw value of the given pin. */
    uint16_t (*getBit)(const GPIOHandler *h, uint16_t pinNum);

    /* Toggle the raw value of the given pin. */
    void (*toggleBit)(const GPIOHandler *h, uint16_t pinNum);

    /* Set the direction of the pin, out for true, in for false. */
    void (*setDirectionOut)(const GPIOHandler *h, uint16_t pinNum, bool v);

    /* Set whether the given pin is open drain. */
    void (*setOpenDrain)(const GPIOHandler *h, uint16_t pinNum, bool v);
} GPIOFuncs;

/*
 * Structure to represent a GPIO type.  You must use const when you
 * define this.
 *
 * To create a new GPIO type, you must create a GPIOFuncs structure
 * (assuming you can't use an existing one).  The data value can be
 * anything you want.  For port types it holds the direction and
 * gioPORT info, for instance.  There are plenty of examples below.
 */
struct _GPIOHandler {
    void *data;
    const GPIOFuncs *funcs;

    /*
     * Can the GPIO in this type be used as an interrupt?  This should
     * only be set for gioA and gioB types.
     */
    bool CanInterrupt;
};

/*
 * Dynamic info about a port.  Right now it just holds the I/O
 * direction.
 */
typedef struct PortGPIOInfo {
    gioPORT_t *port;
    uint32_t direction;
} PortGPIOInfo;

/*
 * Functions for handling standard PORT types, gio, het, and spi GPIOs.
 */
static void PORT_setBit(const GPIOHandler *h, uint16_t pinNum, uint16_t val)
{
    PortGPIOInfo *info = h->data;
    gioPORT_t *port = info->port;

    gioSetBit(port, pinNum, val);
}

static uint16_t PORT_getBit(const GPIOHandler *h, uint16_t pinNum)
{
    PortGPIOInfo *info = h->data;
    gioPORT_t *port = info->port;
    uint16_t val;

    val = gioGetBit(port, pinNum);
    return val;
}

static void PORT_toggleBit(const GPIOHandler *h, uint16_t pinNum)
{
    PortGPIOInfo *info = h->data;
    gioPORT_t *port = info->port;

    return gioToggleBit(port, pinNum);
}

static void PORT_setDirectionOut(const GPIOHandler *h, uint16_t pinNum, bool v)
{
    PortGPIOInfo *info = h->data;
    gioPORT_t *port = info->port;

    if (v)
        info->direction |= 1 << pinNum;
    else
        info->direction &= ~(1 << pinNum);
    port->DIR = info->direction;
    port->PULDIS = info->direction;
}

static void PORT_setOpenDrain(const GPIOHandler *h, uint16_t pinNum, bool v)
{
    PortGPIOInfo *info = h->data;
    gioPORT_t *port = info->port;

    if (v)
        port->PDR |= 1 << pinNum;
    else
        port->PDR &= ~(1 << pinNum);
}

static const GPIOFuncs PortGPIOFuncs = {
    .setBit = PORT_setBit,
    .getBit = PORT_getBit,
    .toggleBit = PORT_toggleBit,
    .setDirectionOut = PORT_setDirectionOut,
    .setOpenDrain = PORT_setOpenDrain,
};

static struct PortGPIOInfo gioPortAGPIOInfo = {
    .port = gioPORTA
};

const GPIOHandler gioPortAGPIO = {
    .data = &gioPortAGPIOInfo,
    .funcs = &PortGPIOFuncs,
    .CanInterrupt = true
};

static struct PortGPIOInfo gioPortBGPIOInfo = {
    .port = gioPORTB
};

const GPIOHandler gioPortBGPIO = {
    .data = &gioPortBGPIOInfo,
    .funcs = &PortGPIOFuncs,
    .CanInterrupt = true
};

/*
 * The interrupt handler uses these to map from the incoming interrupts
 * back to the GPIOs.
 */
#define MAX_gioPORTA_PINS 8
static Gpio_Use gioPortA_Interrupts[MAX_gioPORTA_PINS] = {
    No_GPIO, No_GPIO, No_GPIO, No_GPIO, No_GPIO, No_GPIO, No_GPIO, No_GPIO
};
#define MAX_gioPORTB_PINS 4
static Gpio_Use gioPortB_Interrupts[MAX_gioPORTB_PINS] = {
    No_GPIO, No_GPIO, No_GPIO, No_GPIO
};

static struct PortGPIOInfo hetPort1GPIOInfo = {
    .port = hetPORT1
};

const GPIOHandler hetPort1GPIO = {
    .data = &hetPort1GPIOInfo,
    .funcs = &PortGPIOFuncs
};

static struct PortGPIOInfo spiPort1GPIOInfo = {
    .port = spiPORT1
};

const GPIOHandler spiPort1GPIO = {
    .data = &spiPort1GPIOInfo,
    .funcs = &PortGPIOFuncs
};

static struct PortGPIOInfo spiPort3GPIOInfo = {
    .port = spiPORT3
};

const GPIOHandler spiPort3GPIO = {
    .data = &spiPort3GPIOInfo,
    .funcs = &PortGPIOFuncs
};

static struct PortGPIOInfo spiPort5GPIOInfo = {
    .port = spiPORT5
};

const GPIOHandler spiPort5GPIO = {
    .data = &spiPort5GPIOInfo,
    .funcs = &PortGPIOFuncs
};


/*
 * Functions for handling standard CAN gpio types.
 */
typedef struct CANGPIOInfo {
    canBASE_t *can;
} CANGPIOInfo;

static void CANGPIO_setBit(const GPIOHandler *h, uint16_t pinNum, uint16_t val)
{
    CANGPIOInfo *info = h->data;
    canBASE_t *can = info->can;

    if (pinNum == CAN_GPIO_RX) {
        if (val)
            can->RIOC |= 2;
        else
            can->RIOC &= ~2;
    } else if (pinNum == CAN_GPIO_TX) {
        if (val)
            can->TIOC |= 2;
        else
            can->TIOC &= ~2;
    }
}

static uint16_t CANGPIO_getBit(const GPIOHandler *h, uint16_t pinNum)
{
    CANGPIOInfo *info = h->data;
    canBASE_t *can = info->can;
    uint16_t rv = 0;

    if (pinNum == CAN_GPIO_RX) {
        rv = can->RIOC;
    } else if (pinNum == CAN_GPIO_TX) {
        rv = can->TIOC;
    }
    return rv & 1;
}

static void CANGPIO_toggleBit(const GPIOHandler *h, uint16_t pinNum)
{
    CANGPIOInfo *info = h->data;
    canBASE_t *can = info->can;

    if (pinNum == CAN_GPIO_RX) {
        can->RIOC ^= 2;
    } else if (pinNum == CAN_GPIO_TX) {
        can->TIOC ^= 2;
    }
}

static void CANGPIO_setDirectionOut(const GPIOHandler *h, uint16_t pinNum,
                                    bool val)
{
    CANGPIOInfo *info = h->data;
    canBASE_t *can = info->can;

    if (pinNum == CAN_GPIO_RX) {
        if (val)
            can->RIOC |= 4;
        else
            can->RIOC &= ~4;
    } else if (pinNum == CAN_GPIO_TX) {
        if (val)
            can->TIOC |= 4;
        else
            can->TIOC &= ~4;
    }
}

static void CANGPIO_setOpenDrain(const GPIOHandler *h, uint16_t pinNum, bool val)
{
    CANGPIOInfo *info = h->data;
    canBASE_t *can = info->can;

    if (pinNum == CAN_GPIO_RX) {
        if (val)
            can->RIOC |= (1 << 16);
        else
            can->RIOC &= ~(1 << 16);
    } else if (pinNum == CAN_GPIO_TX) {
        if (val)
            can->TIOC |= (1 << 16);
        else
            can->TIOC &= ~(1 << 16);
    }
}

static const GPIOFuncs CANGPIOFuncs = {
    .setBit = CANGPIO_setBit,
    .getBit = CANGPIO_getBit,
    .toggleBit = CANGPIO_toggleBit,
    .setDirectionOut = CANGPIO_setDirectionOut,
    .setOpenDrain = CANGPIO_setOpenDrain,
};

static struct CANGPIOInfo can1GPIOInfo = {
    .can = canREG1
};

const GPIOHandler can1GPIO = {
    .data = &can1GPIOInfo,
    .funcs = &CANGPIOFuncs
};

/*
 * Functions for handling standard ECLK gpio types.
 */
static void ECLKGPIO_setBit(const GPIOHandler *h, uint16_t pinNum, uint16_t val)
{
    systemREG1->SYSPC4 = val;
}

static uint16_t ECLKGPIO_getBit(const GPIOHandler *h, uint16_t pinNum)
{
    return systemREG1->SYSPC3;
;
}

static void ECLKGPIO_toggleBit(const GPIOHandler *h, uint16_t pinNum)
{
    systemREG1->SYSPC4 = !systemREG1->SYSPC4;
}

static void ECLKGPIO_setDirectionOut(const GPIOHandler *h, uint16_t pinNum,
                                     bool val)
{
    systemREG1->SYSPC1 = 0; // Set to GPIO mode.
    systemREG1->SYSPC2 = val;
}

static void ECLKGPIO_setOpenDrain(const GPIOHandler *h, uint16_t pinNum,
                                  bool val)
{
    systemREG1->SYSPC7 = val;
}

static const GPIOFuncs ECLKGPIOFuncs = {
    .setBit = ECLKGPIO_setBit,
    .getBit = ECLKGPIO_getBit,
    .toggleBit = ECLKGPIO_toggleBit,
    .setDirectionOut = ECLKGPIO_setDirectionOut,
    .setOpenDrain = ECLKGPIO_setOpenDrain,
};

const GPIOHandler eclkGPIO = {
    .data = NULL,
    .funcs = &ECLKGPIOFuncs
};

/*
 * Functions for handling standard DUMMY gpio types.  These remove ugly
 * ifdefs for setting things.
 */
static void DUMMYGPIO_setBit(const GPIOHandler *h, uint16_t pinNum, uint16_t val)
{
}

static uint16_t DUMMYGPIO_getBit(const GPIOHandler *h, uint16_t pinNum)
{
    return 0;
}

static void DUMMYGPIO_toggleBit(const GPIOHandler *h, uint16_t pinNum)
{
}

static void DUMMYGPIO_setDirectionOut(const GPIOHandler *h, uint16_t pinNum,
                                    bool val)
{
}

static void DUMMYGPIO_setOpenDrain(const GPIOHandler *h, uint16_t pinNum,
                                   bool val)
{
}

static const GPIOFuncs DUMMYGPIOFuncs = {
    .setBit = DUMMYGPIO_setBit,
    .getBit = DUMMYGPIO_getBit,
    .toggleBit = DUMMYGPIO_toggleBit,
    .setDirectionOut = DUMMYGPIO_setDirectionOut,
    .setOpenDrain = DUMMYGPIO_setOpenDrain,
};

const GPIOHandler dummyGPIO = {
    .data = NULL,
    .funcs = &DUMMYGPIOFuncs
};

/*
 * This structure defines each GPIO that is in use.
 */
typedef struct _GPIOInfo {
    /*
     * This describes the particular GPIO type.  The current set of
     * available ones is in gpioDriver.h, search for GPIOHandler.  See
     * comments on the GPIOHandler structure for more details.
     */
    const GPIOHandler *info;

    /*
     * The particular pin number for the GPIO.  Each GPIO type has one
     * or more pins, this tells you which one.  For GPIO types with one
     * pin (eclk for instance) you don't have to set this.
     */
    uint16_t PinNum;

    /*
     * The initial state of an output.  It takes negative logic into
     * account.  This only has to be set for output GPIOs.
     */
    bool InitialStateOn;

    /*
     * If true, the GPIO is an output.  If false it is an input.
     */
    bool DirectionIsOut;

    /*
     * If true, and the device is set up for interrupts, an interrupt
     * will be generated on rising and falling edges of the GPIO.
     * Otherwise it will be the edge set by HalGoGen.
     */
    bool InterruptBothEdges;

    /*
     * If true, set the output to open collector.  If not, it's a push
     * pull output.  Only valid for outputs.
     */
    bool OpenDrain;

    /*
     * If true, set 1 as the "off" value and 0 as the "on" value.  If false,
     * do the opposite.
     *
     * This affects GPIOIsOn, GPIOSetOn and GPIOSetOff.  If does not
     * affect GPIORead or GPIOSet.
     */
    bool NegativeLogic;
} GPIOInfo;

/*
 * Each structure below defines one of the GPIOs in use.
 */

static const GPIOInfo LED1Info = {
    .info                 = &GPIOLed1Port,
    .PinNum               = GPIOLed1Pin,
    .InitialStateOn       = GPIO_OFF,
    .DirectionIsOut       = GPIO_OUT,
#ifndef LAUNCHPAD_HARDWARE
    .NegativeLogic        = true,
#endif
};

static const GPIOInfo LED2Info = {
    .info                 = &GPIOLed2Port,
    .PinNum               = GPIOLed2Pin,
    .InitialStateOn       = GPIO_OFF,
    .DirectionIsOut       = GPIO_OUT,
#ifndef LAUNCHPAD_HARDWARE
    .NegativeLogic        = true,
#endif
};

static const GPIOInfo MRAM0_Selector = {
    .info                 = &SPI_MRAM0_Select_Port,
    .PinNum               = SPI_MRAM0_Select_Pin,
    .InitialStateOn       = GPIO_ON,
    .DirectionIsOut       = GPIO_OUT,
    .NegativeLogic        = true,
};

static const GPIOInfo MRAM1_Selector = {
    .info                 = &SPI_MRAM1_Select_Port,
    .PinNum               = SPI_MRAM1_Select_Pin,
    .InitialStateOn       = GPIO_ON,
    .DirectionIsOut       = GPIO_OUT,
    .NegativeLogic        = true,
};

static const GPIOInfo MRAM2_Selector = {
    .info                 = &SPI_MRAM2_Select_Port,
    .PinNum               = SPI_MRAM2_Select_Pin,
    .InitialStateOn       = GPIO_ON,
    .DirectionIsOut       = GPIO_OUT,
    .NegativeLogic        = true,
};

static const GPIOInfo MRAM3_Selector = {
    .info                 = &SPI_MRAM3_Select_Port,
    .PinNum               = SPI_MRAM3_Select_Pin,
    .InitialStateOn       = GPIO_ON,
    .DirectionIsOut       = GPIO_OUT,
    .NegativeLogic        = true,
};

static const GPIOInfo AX5043_Rx1_InterruptInfo = {
    .info                 = &GPIO_Rx1AX5043InterruptPort,
    .PinNum               = GPIO_Rx1AX5043InterruptPin,
    .DirectionIsOut       = GPIO_IN,
    .NegativeLogic        = true,
};

static const GPIOInfo AX5043_Tx_InterruptInfo = {
    .info                 = &GPIO_TxAX5043InterruptPort,
    .PinNum               = GPIO_TxAX5043InterruptPin,
    .DirectionIsOut       = GPIO_IN,
    .NegativeLogic        = true,
};

static const GPIOInfo AX5043_Rx1_Selector = {
    .info                 = &SPI_Rx1AX5043_Select_Port,
    .PinNum               = SPI_Rx1AX5043_Select_Pin,
    .InitialStateOn       = GPIO_OFF,
    .DirectionIsOut       = GPIO_OUT,
    .NegativeLogic        = true,
};

static const GPIOInfo AX5043_Tx_Selector = {
    .info                 = &SPI_TxAX5043_Select_Port,
    .PinNum               = SPI_TxAX5043_Select_Pin,
    .InitialStateOn       = GPIO_OFF,
    .DirectionIsOut       = GPIO_OUT,
    .NegativeLogic        = true,
};

#ifdef LAUNCHPAD_HARDWARE

static const GPIOInfo LED3Info = {
    .info                 = &dummyGPIO,
    .InitialStateOn       = GPIO_OFF,
    .DirectionIsOut       = GPIO_OUT,
};

static const GPIOInfo SSPAPowerInfo = {
    .info                 = &dummyGPIO,
    .InitialStateOn       = GPIO_OFF,
    .DirectionIsOut       = GPIO_OUT,
};

static const GPIOInfo Ax5043PowerInfo = {
    .info                 = &dummyGPIO,
    .InitialStateOn       = GPIO_OFF,
    .DirectionIsOut       = GPIO_OUT,
};

static const GPIOInfo WatchdogInfo = {
    .info                 = &dummyGPIO,
    .InitialStateOn       = GPIO_OFF,
    .DirectionIsOut       = GPIO_OUT,
};

/*
 * Use this array to index to the correct GPIOInfoStructure based on the GPIO
 * enum index.
 */

static const GPIOInfo *GPIOInfoStructures[NumberOfGPIOs] = {
    &LED1Info, &LED2Info, &LED3Info,
    &AX5043_Rx1_InterruptInfo, &AX5043_Tx_InterruptInfo,
    &AX5043_Rx1_Selector, &AX5043_Tx_Selector,
    &MRAM0_Selector, &MRAM1_Selector, &MRAM2_Selector, &MRAM3_Selector,
    &SSPAPowerInfo, &Ax5043PowerInfo, &WatchdogInfo
};

#ifdef DEBUG
static const char *GPIONames[NumberOfGPIOs] = {
    "LED1", "LED2", "LED3", "AX5043_Rx1_Interrupt", "AX5043_Tx_Interrupt",
    "AX5043_Rx1_Sel", "AX5043_Tx_Sel",
    "MRAM0_Sel", "MRAM1_Sel", "MRAM2_Sel", "MRAM3_Sel",
    "SSPAPower", "AX5043Power", "Watchdog",
};
#endif

#else

static const GPIOInfo LED3Info = {
    .info                 = &GPIOLed3Port,
    .PinNum               = GPIOLed3Pin,
    .InitialStateOn       = GPIO_OFF,
    .DirectionIsOut       = GPIO_OUT,
    .NegativeLogic        = true,
};

static const GPIOInfo AX5043_Rx2_InterruptInfo = {
    .info                 = &GPIO_Rx2AX5043InterruptPort,
    .PinNum               = GPIO_Rx2AX5043InterruptPin,
    .DirectionIsOut       = GPIO_IN,
    .NegativeLogic        = true,
};

static const GPIOInfo AX5043_Rx3_InterruptInfo = {
    .info                 = &GPIO_Rx3AX5043InterruptPort,
    .PinNum               = GPIO_Rx3AX5043InterruptPin,
    .DirectionIsOut       = GPIO_IN,
    .NegativeLogic        = true,
};

static const GPIOInfo AX5043_Rx4_InterruptInfo = {
    .info                 = &GPIO_Rx4AX5043InterruptPort,
    .PinNum               = GPIO_Rx4AX5043InterruptPin,
    .DirectionIsOut       = GPIO_IN,
    .NegativeLogic        = true,
};

static const GPIOInfo AX5043_Rx2_Selector = {
    .info                 = &SPI_Rx2AX5043_Select_Port,
    .PinNum               = SPI_Rx2AX5043_Select_Pin,
    .InitialStateOn       = GPIO_OFF,
    .DirectionIsOut       = GPIO_OUT,
    .NegativeLogic        = true,
};

static const GPIOInfo AX5043_Rx3_Selector = {
    .info                 = &SPI_Rx3AX5043_Select_Port,
    .PinNum               = SPI_Rx3AX5043_Select_Pin,
    .InitialStateOn       = GPIO_OFF,
    .DirectionIsOut       = GPIO_OUT,
    .NegativeLogic        = true,
};

static const GPIOInfo AX5043_Rx4_Selector = {
    .info                 = &SPI_Rx4AX5043_Select_Port,
    .PinNum               = SPI_Rx4AX5043_Select_Pin,
    .InitialStateOn       = GPIO_OFF,
    .DirectionIsOut       = GPIO_OUT,
    .NegativeLogic        = true,
};

static const GPIOInfo SSPAPowerInfo = {
    .info                 = &GPIOsspaPowerPort,
    .PinNum               = GPIOsspaPowerPin,
    .InitialStateOn       = GPIO_OFF,
    .DirectionIsOut       = GPIO_OUT,
    .NegativeLogic        = true,
};

static const GPIOInfo Ax5043PowerInfo = {
    .info                 = &GPIOax5043PowerPort,
    .PinNum               = GPIOax5043PowerPin,
    .InitialStateOn       = GPIO_OFF,
    .DirectionIsOut       = GPIO_OUT,
#ifdef BLINKY_HARDWARE
    .NegativeLogic        = true,
#endif
};

static const GPIOInfo WatchdogInfo = {
    .info                 = &spiPort1GPIO,
    .PinNum               = SPI_PIN_CS1,
    .InitialStateOn       = GPIO_OFF,
    .DirectionIsOut       = GPIO_OUT,
#ifdef BLINKY_HARDWARE
    .NegativeLogic        = true,
#endif
};

#if defined(BLINKY_HARDWARE)
static const GPIOInfo *GPIOInfoStructures[NumberOfGPIOs] =
{
    &LED1Info, &LED2Info, &LED3Info,
    &AX5043_Rx1_InterruptInfo, &AX5043_Rx2_InterruptInfo, &AX5043_Rx3_InterruptInfo,
    &AX5043_Rx4_InterruptInfo, &AX5043_Tx_InterruptInfo,
    &AX5043_Rx1_Selector, &AX5043_Rx2_Selector, &AX5043_Rx3_Selector,
    &AX5043_Rx4_Selector, &AX5043_Tx_Selector,
    &MRAM0_Selector, &MRAM1_Selector, &MRAM2_Selector, &MRAM3_Selector,
    &SSPAPowerInfo, &Ax5043PowerInfo, &WatchdogInfo,
};

#ifdef DEBUG
static const char *GPIONames[NumberOfGPIOs] = {
    "LED1", "LED2", "LED3",
    "AX5043_Rx1_Interrupt", "AX5043_Rx2_Interrupt", "AX5043_Rx3_Interrupt", "AX5043_Rx4_Interrupt",
    "AX5043_Tx_Interrupt",
    "AX5043_Rx1_Sel", "AX5043_Rx2_Sel", "AX5043_Rx3_Sel", "AX5043_Rx4_Sel",
    "AX5043_Tx_Sel",
    "MRAM0_Sel", "MRAM1_Sel", "MRAM2_Sel", "MRAM3_Sel",
    "SSPAPower", "AX5043Power", "Watchdog"
};
#endif

#else

static const GPIOInfo OtherFaultInfo = {
    .info                 = &gioPortBGPIO,
    .PinNum               = 3,
    .DirectionIsOut       = GPIO_IN,
};

static const GPIOInfo OtherPowerOffStateInfo = {
    .info                 = &hetPort1GPIO,
    .PinNum               = 11,
    .DirectionIsOut       = GPIO_IN,
};

static const GPIOInfo OtherPresenceInfo = {
    .info                 = &gioPortAGPIO,
    .PinNum               = 2,
    .DirectionIsOut       = GPIO_IN,
    .NegativeLogic        = true,
};

static const GPIOInfo OtherActiveInfo = {
    .info                 = &gioPortAGPIO,
    .PinNum               = 6,
    .DirectionIsOut       = GPIO_IN,
};

static const GPIOInfo OtherPowerOffInfo = {
    .info                 = &hetPort1GPIO,
    .PinNum               = 10,
    .InitialStateOn       = GPIO_OFF,
    .DirectionIsOut       = GPIO_OUT,
};

static const GPIOInfo ActiveInfo = {
    .info                 = &gioPortBGPIO,
    .PinNum               = 1,
    .InitialStateOn       = GPIO_OFF,
    .DirectionIsOut       = GPIO_OUT,
};

static const GPIOInfo UmbilicalAttachedInfo = {
    .info                 = &eclkGPIO,
    .InitialStateOn       = GPIO_OFF,
    .DirectionIsOut       = GPIO_OUT,
};

static const GPIOInfo LNAPowerInfo = {
    .info                 = &hetPort1GPIO,
    .PinNum               = 5,
    .InitialStateOn       = GPIO_OFF,
    .DirectionIsOut       = GPIO_OUT,
};

static const GPIOInfo MeasurePowerInfo = {
    .info                 = &hetPort1GPIO,
    .PinNum               = 12,
    .InitialStateOn       = GPIO_OFF,
    .DirectionIsOut       = GPIO_OUT,
};

static const GPIOInfo AX5043_Rx1_PowerInfo = {
    .info                 = &hetPort1GPIO,
    .PinNum               = 4,
    .InitialStateOn       = GPIO_OFF,
    .DirectionIsOut       = GPIO_OUT,
    .NegativeLogic        = true,
};

static const GPIOInfo AX5043_Rx2_PowerInfo = {
    .info                 = &hetPort1GPIO,
    .PinNum               = 9,
    .InitialStateOn       = GPIO_OFF,
    .DirectionIsOut       = GPIO_OUT,
    .NegativeLogic        = true,
};

static const GPIOInfo AX5043_Rx3_PowerInfo = {
    .info                 = &hetPort1GPIO,
    .PinNum               = 7,
    .InitialStateOn       = GPIO_OFF,
    .DirectionIsOut       = GPIO_OUT,
    .NegativeLogic        = true,
};

static const GPIOInfo AX5043_Rx4_PowerInfo = {
    .info                 = &hetPort1GPIO,
    .PinNum               = 3,
    .InitialStateOn       = GPIO_OFF,
    .DirectionIsOut       = GPIO_OUT,
    .NegativeLogic        = true,
};

static const GPIOInfo AX5043_Tx_PowerInfo = {
    .info                 = &can1GPIO,
    .PinNum               = CAN_GPIO_TX,
    .InitialStateOn       = GPIO_OFF,
    .DirectionIsOut       = GPIO_OUT,
    .NegativeLogic        = true,
};

static const GPIOInfo CANAPowerInfo = {
    .info                 = &hetPort1GPIO,
    .PinNum               = 15,
    .InitialStateOn       = GPIO_OFF,
    .DirectionIsOut       = GPIO_OUT,
    .NegativeLogic        = true,
};

static const GPIOInfo CANBPowerInfo = {
    .info                 = &spiPort1GPIO,
    .PinNum               = SPI_PIN_CS0,
    .InitialStateOn       = GPIO_OFF,
    .DirectionIsOut       = GPIO_OUT,
    .NegativeLogic        = true,
};


static const GPIOInfo *GPIOInfoStructures[NumberOfGPIOs] =
{
    &LED1Info, &LED2Info, &LED3Info,
    &AX5043_Rx1_InterruptInfo, &AX5043_Rx2_InterruptInfo, &AX5043_Rx3_InterruptInfo,
    &AX5043_Rx4_InterruptInfo, &AX5043_Tx_InterruptInfo,
    &AX5043_Rx1_Selector, &AX5043_Rx2_Selector, &AX5043_Rx3_Selector,
    &AX5043_Rx4_Selector, &AX5043_Tx_Selector,
    &MRAM0_Selector, &MRAM1_Selector, &MRAM2_Selector, &MRAM3_Selector,
    &SSPAPowerInfo, &Ax5043PowerInfo, &WatchdogInfo,

    &OtherFaultInfo, &OtherPowerOffStateInfo, &OtherPresenceInfo, &OtherActiveInfo,
    &OtherPowerOffInfo, &ActiveInfo,

    &UmbilicalAttachedInfo,
    &LNAPowerInfo, &MeasurePowerInfo,

    &AX5043_Rx1_PowerInfo, &AX5043_Rx2_PowerInfo, &AX5043_Rx3_PowerInfo,
    &AX5043_Rx4_PowerInfo, &AX5043_Tx_PowerInfo,

    &CANAPowerInfo, &CANBPowerInfo, 
};

#ifdef DEBUG
static const char *GPIONames[NumberOfGPIOs] = {
    "LED1", "LED2", "LED3",
    "AX5043_Rx1_Interrupt", "AX5043_Rx2_Interrupt", "AX5043_Rx3_Interrupt", "AX5043_Rx4_Interrupt",
    "AX5043_Tx_Interrupt",
    "AX5043_Rx1_Sel", "AX5043_Rx2_Sel", "AX5043_Rx3_Sel", "AX5043_Rx4_Sel",
    "AX5043_Tx_Sel",
    "MRAM0_Sel", "MRAM1_Sel", "MRAM2_Sel", "MRAM3_Sel",
    "SSPAPower", "AX5043Power", "Watchdog",

    "OtherFault", "OtherPowerOffState", "OtherPresense", "OtherActive",
    "OtherPowerOff", "ImActive",

    "UmbilicalAttached",
    "LNAPower", "MeasurePower",

    "AX5043_Rx1_Power", "AX5043_Rx2_Power", "AX5043_Rx3_Power", "AX5043_Rx4_Power",
    "AX5043_Tx_Power",

    "CANAPower", "CANBPower",
};
#endif

#endif
#endif

bool gpio_usable[NumberOfGPIOs];
static const struct gpio_irq_info *gpio_irq_info[NumberOfGPIOs];

#ifdef DEBUG
const char *GPIOToName(Gpio_Use whichGpio)
{
    if (whichGpio < (Gpio_Use) 0)
        return "None";
    if (whichGpio >= NumberOfGPIOs)
        return "?";
    return GPIONames[(int)whichGpio];
}
#endif

bool GPIOEzInit(Gpio_Use whichGpio)
{
    // Save some typing, reading, and some code space for most init calls
    return GPIOInit(whichGpio, NULL);
}

bool GPIOInit(Gpio_Use whichGpio, const struct gpio_irq_info *irqinfo)
{
    const GPIOInfo *thisGPIO;

#ifdef UNDEFINE_BEFORE_FLIGHT
    if (whichGpio >= NumberOfGPIOs)
        return false;
#endif

    thisGPIO = GPIOInfoStructures[whichGpio];

#ifdef UNDEFINE_BEFORE_FLIGHT
    if (irqinfo) {
        // If they are asking for an interrupt, make sure it's supported.
        if (!thisGPIO->info->CanInterrupt || thisGPIO->DirectionIsOut) {
            return false; // No can do.
        }
    }
#endif

    if (thisGPIO->DirectionIsOut) {

        /*
         * Here if this is an "out" GPIO, we set the initial state,
         * including open collector-ness
         */

        uint16_t pinNum = thisGPIO->PinNum;

        gpio_usable[whichGpio] = true;

        /*
         * Set open collector first to avoid driving the output high
         * if it shouldn't be driven.
         */
        thisGPIO->info->funcs->setOpenDrain(thisGPIO->info, pinNum, 
                                            thisGPIO->OpenDrain);

        if (thisGPIO->InitialStateOn)
            GPIOSetOn(whichGpio);
        else
            GPIOSetOff(whichGpio);

        /*
         * Delay setting the direction until here to avoid glitching
         * the GPIO.  Setting these before the output value is set
         * could cause whatever happened to be there to go out for a
         * little.
         */
        thisGPIO->info->funcs->setDirectionOut(thisGPIO->info,
                                               thisGPIO->PinNum, true);
    } else {

        /*
         * Here the direction is in so check for and set up interrupt
         * possibilities
         */

        if (irqinfo) {
            struct PortGPIOInfo *info = thisGPIO->info->data;
            gioPORT_t *port = info->port;

            /*
             * Here we have a "real" GPIO port and we are a asking for
             * a message to be sent when it changes.  That means we
             * set up an interrupt.
             */
            gpio_irq_info[whichGpio] = irqinfo;

            /*
             * This should really be handled through the abstraction,
             * but GIOA and GIOB are the only things that can receive
             * interrupts, so it's easier to just leave this for now.
             */
             
            /*
             * Next we have to figure out whether this is an interrupt
             * on both edges or not.  HalCoGen can set up for which
             * edge direction if it is only one, but if we want an
             * interrupt on both directions, we have to set the INTDET
             * register, which HalCoGen can't currently do.  INTDET
             * had 32 bits, 8 for each of GPIOA, GPIOB, GPIOC and
             * GPIOD (our chip does not have C and D).  So shift a bit
             * to match the pin number of the GPIO and then over by 8
             * if it is GPIOB.
             */

            if (port == gioPORTA) {
                if (thisGPIO->PinNum >= MAX_gioPORTA_PINS)
                    return false;
                gioPortA_Interrupts[thisGPIO->PinNum] = whichGpio;
            } else {
                if (thisGPIO->PinNum >= MAX_gioPORTB_PINS)
                    return false;
                gioPortB_Interrupts[thisGPIO->PinNum] = whichGpio;
            }

            if (thisGPIO->InterruptBothEdges) {
                uint32_t mask = 1 << thisGPIO->PinNum;

                if (port == gioPORTB) {
                    mask <<= 8;
                }
                /*
                 * Now finally set the bit in INTDET to say both directions.
                 */
                gioREG->INTDET |= mask;
            }
            gioEnableNotification(port, thisGPIO->PinNum);
        }

        gpio_usable[whichGpio] = true;
    }

    return true;
}

void GPIOSet(Gpio_Use whichGpio, bool v)
{
    const GPIOInfo *thisGPIO = GPIOInfoStructures[whichGpio];

    if (!gpio_usable[whichGpio])
        return;

#ifdef DEBUG_BUILD
    if (thisGPIO->Mode != GPIO_Mode_OUT) {
        ReportError(IllegalGPIOOutput, TRUE, PortNumber, whichGpio);
    }
#endif

    thisGPIO->info->funcs->setBit(thisGPIO->info, thisGPIO->PinNum, v);
}

void GPIOSetOn(Gpio_Use whichGpio)
{
    const GPIOInfo *thisGPIO = GPIOInfoStructures[whichGpio];

    GPIOSet(whichGpio, !thisGPIO->NegativeLogic);
}

void GPIOSetOff(Gpio_Use whichGpio)
{
    const GPIOInfo *thisGPIO = GPIOInfoStructures[whichGpio];

    GPIOSet(whichGpio, thisGPIO->NegativeLogic);
}

void GPIOToggle(Gpio_Use whichGpio)
{
    const GPIOInfo *thisGPIO = GPIOInfoStructures[whichGpio];

    if (!gpio_usable[whichGpio])
        return;

#ifdef DEBUG_BUILD
    if (thisGPIO->Mode != GPIO_Mode_IN) {
        ReportError(IllegalGPIOOutput, TRUE, PortNumber, whichGpio);
    }
#endif

    thisGPIO->info->funcs->toggleBit(thisGPIO->info, thisGPIO->PinNum);
}

bool GPIOIsOn(Gpio_Use whichGpio)
{
    const GPIOInfo *thisGPIO = GPIOInfoStructures[whichGpio];
    uint16_t val;

    val = GPIORead(whichGpio);
    val = val == !thisGPIO->NegativeLogic;
    return val;
}

uint16_t GPIORead(Gpio_Use whichGpio)
{
    const GPIOInfo *thisGPIO = GPIOInfoStructures[whichGpio];
    uint16_t val;

    if (!gpio_usable[whichGpio])
        return 0;

#ifdef DEBUG_BUILD
    if (thisGPIO->Mode != GPIO_Mode_IN) {
        ReportError(IllegalGPIOOutput, TRUE, PortNumber, whichGpio);
    }
#endif

    val = thisGPIO->info->funcs->getBit(thisGPIO->info, thisGPIO->PinNum);
    return val;
}

static void GPIOIntRoutine(Gpio_Use whichGPIO)
{
    /*
     * Interrupt handler.  Send the message requested in the init.
     * Note that reading a GPIO can be done from an interrupt routine.
     * Care must be taken not to mess with GPIORead to prevent this.
     */
    if (gpio_irq_info[whichGPIO])
        gpio_irq_info[whichGPIO]->handler(gpio_irq_info[whichGPIO]->handler_data);
    //EndInterruptRoutine();
}

void gioNotification(gioPORT_t *port, uint32 bit)
{
    Gpio_Use thisGPIO;

    if (port == gioPORTA) {
        if (bit >= MAX_gioPORTA_PINS)
            return;
        thisGPIO = gioPortA_Interrupts[bit];
        if (thisGPIO == No_GPIO)
            return;
    } else if (port == gioPORTB) {
        if (bit >= MAX_gioPORTB_PINS)
            return;
        thisGPIO = gioPortB_Interrupts[bit];
        if (thisGPIO == No_GPIO)
            return;
    } else {
        return;
    }

    GPIOIntRoutine(thisGPIO);
}
