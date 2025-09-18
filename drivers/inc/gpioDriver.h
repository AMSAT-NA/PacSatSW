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


/*
 * IMPORTANT: If you add something to Gpio_Use, make sure to add it to
 * GPIONames and GPIOInfoStructures in gpioDriver.c.  For instructions
 * on adding GPIOs, see the comments near the top of gpioDriver.c.
 */
#ifdef LAUNCHPAD_HARDWARE
typedef enum gu {
    No_GPIO=-1,
    LED1, LED2, LED3, AX5043_Rx1_Interrupt, AX5043_Tx_Interrupt,
    AX5043_Sel0, AX5043_Sel1,
    MRAM0_Sel, MRAM1_Sel, MRAM2_Sel, MRAM3_Sel,

    // Dummy GPIOs
    SSPAPower, AX5043Power, Watchdog
    NumberOfGPIOs
} Gpio_Use;

#elif defined(BLINKY_HARDWARE)
typedef enum gu {
    No_GPIO=-1,
    LED1, LED2, LED3,
    AX5043_Rx1_Interrupt, AX5043_Rx2_Interrupt, AX5043_Rx3_Interrupt, AX5043_Rx4_Interrupt,
    AX5043_Tx_Interrupt,
    AX5043_Rx1_Sel, AX5043_Rx2_Sel, AX5043_Rx3_Sel, AX5043_Rx4_Sel,
    AX5043_Tx_Sel,
    MRAM0_Sel, MRAM1_Sel, MRAM2_Sel, MRAM3_Sel,
    SSPAPower, AX5043Power, Watchdog,
    NumberOfGPIOs
} Gpio_Use;

#else
typedef enum gu {
    No_GPIO=-1,
    LED1, LED2, LED3,
    AX5043_Rx1_Interrupt, AX5043_Rx2_Interrupt, AX5043_Rx3_Interrupt, AX5043_Rx4_Interrupt,
    AX5043_Tx_Interrupt,
    AX5043_Rx1_Sel, AX5043_Rx2_Sel, AX5043_Rx3_Sel, AX5043_Rx4_Sel,
    AX5043_Tx_Sel,
    MRAM0_Sel, MRAM1_Sel, MRAM2_Sel, MRAM3_Sel,
    SSPAPower, AX5043Power, Watchdog,

    OtherFault, OtherPowerOffState, OtherPresense, OtherActive,
    OtherPowerOff, ImActive,

    UmbilicalAttached,
    LNAPower, MeasurePower,

    AX5043_Rx1_Power, AX5043_Rx2_Power, AX5043_Rx3_Power, AX5043_Rx4_Power,
    AX5043_Tx_Power,

    CANAPower, CANBPower,

    NumberOfGPIOs
} Gpio_Use;
#endif

typedef struct _GPIOHandler GPIOHandler;

/*
 * Standard GPIOs.  These are the only ones that support interrupts.
 */
extern const GPIOHandler gioPortAGPIO;
extern const GPIOHandler gioPortBGPIO;

/*
 * Using HET pins for GPIO.  Use the HET number as the pin number.
 */
extern const GPIOHandler hetPort1GPIO;

/*
 * Using SPI pins as GPIOs.  Use the spiPinSelect values in spi.h
 * for the pin numbers.
 */
extern const GPIOHandler spiPort1GPIO;
extern const GPIOHandler spiPort3GPIO;
extern const GPIOHandler spiPort5GPIO;

/*
 * A GPIO that doesn't do anything and always reads 0.  This is
 * useful for avoiding ugly ifdefs in code, for instance on power
 * enables when the particular board doesn't have one.
 */
extern const GPIOHandler dummyGPIO;

/* CAN bus GPIO pins and structure. */
#define CAN_GPIO_RX 0
#define CAN_GPIO_TX 1
extern const GPIOHandler can1GPIO;

/* GPIO for the ECLK pin, there is only one. */
extern const GPIOHandler eclkGPIO;

#define GPIO_OFF false
#define GPIO_ON true

#define NO_MESSAGE ((IntertaskMessageType)-1)
#define NO_TASK ((DestinationTask)-1)

/*
 * Initialize a GPIO.  All GPIOs used by the system must be initialized
 * before they can be used.
 *
 * If task is not NO_TASK, then this GPIO must support interrupts
 * (gioA or giob) and when an interrupt comes in on the GPIO, the
 * given msg is sent to the task.
 */
bool GPIOInit(Gpio_Use whichGpio, DestinationTask task,
	      IntertaskMessageType msg);

/* Simplified GPIO init with no tasks or messages. */
bool GPIOEzInit(Gpio_Use whichGpio);

/*
 * Read the raw value of the GPIO, no logic translation is done.
 */
uint16_t GPIORead(Gpio_Use whichGpio);

/*
 * Returns true if the GPIO is on and false if it is not.  This takes
 * negative logic into account, if the GPIO is set as negative logic,
 * then if the raw value is 0, it is on.
 */
bool GPIOIsOn(Gpio_Use whichGpio);

/*
 * Set the raw value of the GPIO, no logic translation is done.
 */
void GPIOSet(Gpio_Use whichGpio, bool v);

/*
 * Set the GPIO to on related to if the GPIO is negative or positive
 * logic.  If it is negative logic, 0 is on and 1 if off.
 */
void GPIOSetOn(Gpio_Use whichGpio);

/*
 * Set the GPIO to off related to if the GPIO is negative or positive
 * logic.
 */
void GPIOSetOff(Gpio_Use whichGpio);

/* Toggle the output value of the GPIO. */
void GPIOToggle(Gpio_Use whichGpio);

/* Convert a GPIO enum to a string.  Only available if DEBUG is set. */
const char *GPIOToName(Gpio_Use whichGpio);

#endif /* GPIO_H_ */
