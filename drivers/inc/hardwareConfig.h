/*
 * hardwareConfig.h
 *
 *  Created on: Apr 26, 2019
 *      Author: bfisher
 */
#ifndef DRIVERS_INC_HARDWARECONFIG_H_
#define DRIVERS_INC_HARDWARECONFIG_H_

#define EXP_WITH_HET_I2C

#include "config.h"

#define GPIO_IN 0
#define GPIO_OUT 1

#ifdef LAUNCHPAD_HARDWARE

//GPIO Definitions

//Note: For the moment, we will assume that HalCoGen has the pullup/open collector/direction settings
// initialized correctly.  We can read them here, and all the bits for a port have to be set at once.
// We could work around that, but it seems not important at the moment.

/*
 * Debug LEDs
 */

//Yellow LED
#define GPIOLed1Port gioPortBGPIO
#define GPIOLed1Pin 1

//Green LED
#define GPIOLed2Port gioPortBGPIO
#define GPIOLed2Pin 2

#define GPIO_Rx1AX5043InterruptPort gioPortBGPIO
#define GPIO_Rx1AX5043InterruptPin  0
#define GPIO_TxAX5043InterruptPort gioPortBGPIO
#define GPIO_TxAX5043InterruptPin  3


// Serial port definitions

#define COM1_Port 0 /* 0 = HET1, 1=HET2 */
#define COM1_HET_Port hetPort1GPIO
#define COM1_HET_Reg hetREG1
#define COM1_HET_Rx_Pin 5
#define COM1_HET_Tx_Pin 4

#define COM2_Port sciREG
#define COM3_Port scilinREG

// SPI Definitions

#define SPI_MRAM_Reg spiREG1
#define SPI_MRAM0_Select_Port spiPort1GPIO /*Gpio port for chip select*/
#define SPI_MRAM0_Select_Pin SPI_PIN_CS0
#define SPI_MRAM1_Select_Port spiPort1GPIO /*Gpio port for chip select*/
#define SPI_MRAM1_Select_Pin SPI_PIN_CS1
#define SPI_MRAM2_Select_Port spiPort1GPIO /*Gpio port for chip select*/
#define SPI_MRAM2_Select_Pin SPI_PIN_CS2
#define SPI_MRAM3_Select_Port spiPort1GPIO /*Gpio port for chip select*/
#define SPI_MRAM3_Select_Pin SPI_PIN_CS5
#define SPI_MRAM_Data_Format SPI_FMT_0


#define SPI_AX5043_Reg spiREG3
#define SPI_Rx1AX5043_Select_Port spiPort3GPIO /*Gpio port for chip select*/
#define SPI_Rx1AX5043_Select_Pin SPI_PIN_CS1
#define SPI_TxAX5043_Select_Port spiPort3GPIO /*Gpio port for chip select*/
#define SPI_TxAX5043_Select_Pin SPI_PIN_CS4
#define SPI_AX5043_Data_Format SPI_FMT_0

#elif defined(BLINKY_HARDWARE)

//GPIO Definitions

//Note: For the moment, we will assume that HalCoGen has the pullup/open collector/direction settings
// initialized correctly.  We can read them here, and all the bits for a port have to be set at once.
// We could work around that, but it seems not important at the moment.

/*
 * Power for SSPA and AX5043
 */
#define GPIOsspaPowerPort spiPort5GPIO
#define GPIOsspaPowerPin 11 /*This is SOMI[0]*/
#define GPIOax5043PowerPort spiPort5GPIO
#define GPIOax5043PowerPin 10 /*This is SIMO[0]*/


/*
 * Debug LEDs
 */

//Red LED
#define GPIOLed1Port hetPort1GPIO
#define GPIOLed1Pin 0

//Yellow LED
#define GPIOLed2Port hetPort1GPIO
#define GPIOLed2Pin 1

//Green LED
#define GPIOLed3Port hetPort1GPIO
#define GPIOLed3Pin 2

// Serial port definitions

#define COM3_Port 0 //Does not exist on PacSat
#define COM2_Port scilinREG
#define COM1_Port sciREG


// SPI Definitions

#define SPI_MRAM_Reg spiREG1
#define SPI_MRAM0_Select_Port spiPort1GPIO
#define SPI_MRAM1_Select_Port spiPort3GPIO
#define SPI_MRAM2_Select_Port spiPort1GPIO
#define SPI_MRAM3_Select_Port spiPort5GPIO
#define SPI_MRAM0_Select_Pin 0
#define SPI_MRAM1_Select_Pin 0
#define SPI_MRAM2_Select_Pin 2
#define SPI_MRAM3_Select_Pin 0
#define SPI_MRAM_Data_Format SPI_FMT_0

// AX5043 SPI pins

#define SPI_AX5043_Reg spiREG3
#define SPI_Rx1AX5043_Select_Port hetPort1GPIO /*Gpio port for chip select*/
#define SPI_Rx1AX5043_Select_Pin 3
#define SPI_Rx2AX5043_Select_Port hetPort1GPIO
#define SPI_Rx2AX5043_Select_Pin 7
#define SPI_Rx3AX5043_Select_Port hetPort1GPIO
#define SPI_Rx3AX5043_Select_Pin 9
#define SPI_Rx4AX5043_Select_Port hetPort1GPIO
#define SPI_Rx4AX5043_Select_Pin 4
#define SPI_TxAX5043_Select_Port hetPort1GPIO
#define SPI_TxAX5043_Select_Pin 12
#define SPI_AX5043_Data_Format SPI_FMT_0

//AX5043 IRQ Pins

#define GPIO_Rx1AX5043InterruptPort gioPortAGPIO
#define GPIO_Rx1AX5043InterruptPin  1
#define GPIO_Rx2AX5043InterruptPort gioPortBGPIO
#define GPIO_Rx2AX5043InterruptPin  0
#define GPIO_Rx3AX5043InterruptPort gioPortAGPIO
#define GPIO_Rx3AX5043InterruptPin  7
#define GPIO_Rx4AX5043InterruptPort gioPortAGPIO
#define GPIO_Rx4AX5043InterruptPin  5
#define GPIO_TxAX5043InterruptPort gioPortBGPIO
#define GPIO_TxAX5043InterruptPin  2

#else /* AFSK hardware */

//GPIO Definitions

//Note: For the moment, we will assume that HalCoGen has the pullup/open collector/direction settings
// initialized correctly.  We can read them here, and all the bits for a port have to be set at once.
// We could work around that, but it seems not important at the moment.

/*
 * Power for SSPA and AX5043
 */
#define GPIOsspaPowerPort hetPort1GPIO
#define GPIOsspaPowerPin 14 /*This is N2HET1[14]*/
#define GPIOax5043PowerPort hetPort1GPIO
#define GPIOax5043PowerPin 20 /*This is N2HET1[20]*/


/*
 * Debug LEDs
 */

//Red LED
#define GPIOLed1Port hetPort1GPIO
#define GPIOLed1Pin 0

//Yellow LED
#define GPIOLed2Port hetPort1GPIO
#define GPIOLed2Pin 1

//Green LED
#define GPIOLed3Port hetPort1GPIO
#define GPIOLed3Pin 2


// Serial port definitions

#define COM3_Port 0 //Does not exist on PacSat
#define COM2_Port scilinREG
#define COM1_Port sciREG


// SPI Definitions

#define SPI_MRAM_Reg spiREG3
#define SPI_MRAM0_Select_Port spiPort3GPIO
#define SPI_MRAM1_Select_Port spiPort3GPIO
#define SPI_MRAM2_Select_Port spiPort1GPIO
#define SPI_MRAM3_Select_Port spiPort3GPIO
#define SPI_MRAM0_Select_Pin SPI_PIN_CS0
#define SPI_MRAM1_Select_Pin SPI_PIN_ENA
#define SPI_MRAM2_Select_Pin SPI_PIN_CS2
#define SPI_MRAM3_Select_Pin SPI_PIN_CS1
#define SPI_MRAM_Data_Format SPI_FMT_0

// AX5043 SPI pins

#define SPI_AX5043_Reg spiREG1
#define SPI_Rx1AX5043_Select_Port can1GPIO /*Gpio port for chip select*/
#define SPI_Rx1AX5043_Select_Pin CAN_GPIO_RX
#define SPI_Rx2AX5043_Select_Port hetPort1GPIO
#define SPI_Rx2AX5043_Select_Pin 24
#define SPI_Rx3AX5043_Select_Port hetPort1GPIO
#define SPI_Rx3AX5043_Select_Pin 26
#define SPI_Rx4AX5043_Select_Port hetPort1GPIO
#define SPI_Rx4AX5043_Select_Pin 4
#define SPI_TxAX5043_Select_Port spiPort5GPIO
#define SPI_TxAX5043_Select_Pin SPI_PIN_ENA
#define SPI_AX5043_Data_Format SPI_FMT_0

//AX5043 IRQ Pins

#define GPIO_Rx1AX5043InterruptPort gioPortAGPIO
#define GPIO_Rx1AX5043InterruptPin  1
#define GPIO_Rx2AX5043InterruptPort gioPortBGPIO
#define GPIO_Rx2AX5043InterruptPin  0
#define GPIO_Rx3AX5043InterruptPort gioPortAGPIO
#define GPIO_Rx3AX5043InterruptPin  7
#define GPIO_Rx4AX5043InterruptPort gioPortAGPIO
#define GPIO_Rx4AX5043InterruptPin  5
#define GPIO_TxAX5043InterruptPort gioPortBGPIO
#define GPIO_TxAX5043InterruptPin  2

#endif /* Launch Pad Hardware vs Blinky vs AFSK */

#endif /* DRIVERS_INC_HARDWARECONFIG_H_ */
