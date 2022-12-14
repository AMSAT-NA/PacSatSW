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
 * Debug LEDs
 */

//Yellow LED
#define GPIOLed1Port gioPORTB
#define GPIOLed1Pin 1

//Green LED
#define GPIOLed2Port gioPORTB
#define GPIOLed2Pin 2

#define GPIOCommandStrobePort gioPORTA
#define GPIOCommandStrobePin 0
#define GPIOCommandBit0Port hetPORT1
#define GPIOCommandBit0Pin 15
#define GPIOCommandBit3Port hetPORT1
#define GPIOCommandBit3Pin 16

#define GPIO_DCTInterruptPort gioPORTB
#define GPIO_DCTInterruptPin  0



// Serial port definitions

#define COM1_Port 0 /* 0 = HET1, 1=HET2 */
#define COM1_HET_Port hetPORT1
#define COM1_HET_Reg hetREG1
#define COM1_HET_Rx_Pin 5
#define COM1_HET_Tx_Pin 4

#define COM2_Port sciREG
#define COM3_Port scilinREG

// SPI Definitions

#define SPI_MRAM_Reg spiREG1
#define SPI_MRAM_Select_Port spiPORT1 /*Gpio port for chip select*/
#define SPI_MRAM0_Select_Pin 0
#define SPI_MRAM1_Select_Pin 1
#define SPI_MRAM_Data_Format SPI_FMT_0

#define SPI_DCT_Reg spiREG3
#define SPI_DCT_Select_Port spiPORT3 /*Gpio port for chip select*/
#define SPI_DCT_Select_Pin 1
#define SPI_DCT_Data_Format SPI_FMT_0



#define I2c2_HET_Port hetPORT2 /* For GPIO-like ops*/
#define I2c2_HET_Reg hetREG2
#define I2c2_HET_SCL_Pin 16
#define I2c2_HET_SDA_Pin 14


#endif /* DRIVERS_INC_HARDWARECONFIG_H_ */
