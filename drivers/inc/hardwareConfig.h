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

//GPIO Definitions

//Note: For the moment, we will assume that HalCoGen has the pullup/open collector/direction settings
// initialized correctly.  We can read them here, and all the bits for a port have to be set at once.
// We could work around that, but it seems not important at the moment.

#define GPIO_IN 0
#define GPIO_OUT 1

#define GPIO_UNUSED_IN_PORT hetPORT1
#define GPIO_UNUSED_IN_PIN 16
#define GPIO_UNUSED_OUT_PORT hetPORT1
#define GPIO_UNUSED_OUT_PIN 18

/*
 * Power for SSPA and AX5043
 */
#define GPIOsspaPowerPort spiPORT5
#define GPIOsspaPowerPin 11 /*This is SOMI[0]*/
#define GPIOax5043PowerPort spiPORT5
#define GPIOax5043PowerPin 10 /*This is SIMO[0]*/


/*
 * Debug LEDs
 */

//Red LED
#define GPIOLed1Port hetPORT1
#define GPIOLed1Pin 0

//Yellow LED
#define GPIOLed2Port hetPORT1
#define GPIOLed2Pin 1

//Green LED
#define GPIOLed3Port hetPORT1
#define GPIOLed3Pin 2


#define GPIOCommandStrobePort gioPORTA
#define GPIOCommandStrobePin 0
#define GPIOCommandBit0Port hetPORT1
#define GPIOCommandBit0Pin 15





// Serial port definitions

#define COM3_Port 0 //Does not exist on PacSat
#define COM2_Port scilinREG
#define COM1_Port sciREG


// SPI Definitions

#define SPI_MRAM_Reg spiREG1
#define SPI_MRAM02_Select_Port spiPORT1 /*Gpio port for chip select on MRAM 0,2*/
#define SPI_MRAM1_Select_Port spiPORT3
#define SPI_MRAM3_Select_Port spiPORT5
#define SPI_MRAM0_Select_Pin 0
#define SPI_MRAM1_Select_Pin 0
#define SPI_MRAM2_Select_Pin 2
#define SPI_MRAM3_Select_Pin 0
#define SPI_MRAM_Data_Format SPI_FMT_0

// AX5043 SPI pins

#define SPI_DCT_Reg spiREG3
#define SPI_DCT_Select_Port hetPORT1 /*Gpio port for chip select*/
#define SPI_Rx1DCT_Select_Pin 3
#define SPI_Rx2DCT_Select_Pin 7
#define SPI_Rx3DCT_Select_Pin 9
#define SPI_Rx4DCT_Select_Pin 4
#define SPI_TxDCT_Select_Pin 12
#define SPI_DCT_Data_Format SPI_FMT_0

//AX5043 IRQ Pins

#define GPIO_Rx1DCTInterruptPort gioPORTA
#define GPIO_Rx1DCTInterruptPin  1
#define GPIO_Rx2DCTInterruptPort gioPORTB
#define GPIO_Rx2DCTInterruptPin  0
#define GPIO_Rx3DCTInterruptPort gioPORTA
#define GPIO_Rx3DCTInterruptPin  7
#define GPIO_Rx4DCTInterruptPort gioPORTA
#define GPIO_Rx4DCTInterruptPin  5
#define GPIO_TxDCTInterruptPort gioPORTB
#define GPIO_TxDCTInterruptPin  2


#endif /* DRIVERS_INC_HARDWARECONFIG_H_ */
