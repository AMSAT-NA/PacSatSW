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
#define GPIOLed1Port hetPORT1
#define GPIOLed1Pin 2

//Green LED
#define GPIOLed2Port hetPORT1
#define GPIOLed2Pin 1

//Red LED
#define GPIOLed3Port hetPORT1
#define GPIOLed3Pin 0

//Power Alert Inhibit
#define GPIOAlertPort hetPORT1
#define GPIOAlertPin 22

/*
 * 4 IHU Coordination lines (between RT-IHU and LIHU) and the disable line
 */
#define GPIOL0Port gioPORTA /*L0 is bus pin 17*/
#define GPIOL0Pin 6

#ifdef RTIHU_BOARD_V10
#define GPIOL1Port gioPORTA /*A2 is actually Exp1, but on old board we will test IHU Coordination*/
#define GPIOL1Pin 2         /*L1 is Bus pin 13*/
#define GPIOWhoAmIPort gioPORTB
#define GPIOWhoAmIPin 3
#else
#define GPIOL1Port gioPORTB
#define GPIOL1Pin 3
#define GPIOWhoAmIPort hetPORT1
#define GPIOWhoAmIPin 11
#endif

#define GPIOIHaveBusPort hetPORT1
#define GPIOIHaveBusPin 3

#define GPIOR0Port gioPORTA /* Bus pin 15 */
#define GPIOR0Pin 5
#define GPIOR1Port gioPORTA /* Bus pin 19 */
#define GPIOR1Pin 7
#ifdef RTIHU_BOARD_V10
#define GPIOCommandStrobePort 0
#define GPIOCommandStrobePin 0
#define GPIOCommandBit0Port 0
#define GPIOCommandBit0Pin 15
#define GPIOCommandBit3Port 0
#define GPIOCommandBit3Pin 16
#else
#define GPIOCommandStrobePort gioPORTA
#define GPIOCommandStrobePin 0
#define GPIOCommandBit0Port hetPORT1
#define GPIOCommandBit0Pin 15
#define GPIOCommandBit3Port hetPORT1
#define GPIOCommandBit3Pin 16

#endif

#ifdef RTIHU_BOARD_V10
#define GPIO_PBEnablePort GPIO_UNUSED_OUT_PORT
#define GPIO_PBEnablePin GPIO_UNUSED_OUT_PIN

#define GPIO_AttachedPort GPIO_UNUSED_IN_PORT
#define GPIO_AttachedPin GPIO_UNUSED_IN_PIN

#define GPIO_RfCtrlPort GPIO_UNUSED_OUT_PORT
#define GPIO_RfCtrlPin GPIO_UNUSED_OUT_PIN

#else
#define GPIO_PBEnablePort hetPORT1
#define GPIO_PBEnablePin 28

#define GPIO_AttachedPort hetPORT1
#define GPIO_AttachedPin 8

#define GPIO_RfCtrlPort hetPORT1
#define GPIO_RfCtrlPin 30

#define GPIO_OneWirePort hetPORT1
#define GPIO_OneWirePin 14

#define GPIO_SDRPwrPort hetPORT1
#define GPIO_SDRPwrPin 26

#define GPIO_ChargeAttPort hetPORT1
#define GPIO_ChargeAttPin 24

#define GPIO_RFPwrCtlPort spiPORT3
#define GPIO_RFPwrCtlPin  5



#endif

/*
 * This section is stuff for the Digital Command Transceiver
 */

//Power On
#ifdef RTIHU_BOARD_V10
#define GPIO_PAPowerPort  gioPORTA
#define GPIO_PAPowerPin  0
#define GPIO_DCTPowerPort gioPORTA
#define GPIO_DCTPowerPin 1
#else
#define GPIO_PAPowerPort  hetPORT1
#define GPIO_PAPowerPin   10
#define GPIO_DCTPowerPort hetPORT1
#define GPIO_DCTPowerPin 12
#endif

//Over current flag

#define GPIO_PAFlagPort gioPORTB
#define GPIO_PAFlagPin 0

#define GPIO_DCTFlagPort gioPORTB
#define GPIO_DCTFlagPin 1

#define GPIO_DCTInterruptPort gioPORTB
#define GPIO_DCTInterruptPin  2

/*
 * Watchdog (V1.1 and later)
 */

#define GPIO_WatchdogPort spiPORT1
#define GPIO_WatchdogPin  1
/*
 * SDR 10GHz Tx
 */
#define GPIO_10GHzSspaPort spiPORT1
#define GPIO_10GHzSspaPin 2

/*
 * Experiment switches
 */
#define GPIO_Exp1EnablePort gioPORTA
#define GPIO_Exp1EnablePin 2
#define GPIO_Exp2EnablePort gioPORTA
#define GPIO_Exp2EnablePin 5
#define GPIO_Exp4EnablePort gioPORTA
#define GPIO_Exp4EnablePin 7


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


// I2c definitions

#define GPIOI2c1ResetPin 18
#define GPIOI2c1ResetPort hetPORT1
#define GPIOI2c2ResetPin 20
#define GPIOI2c2ResetPort hetPORT1

#define I2c2_HET_Port hetPORT2 /* For GPIO-like ops*/
#define I2c2_HET_Reg hetREG2
#define I2c2_HET_SCL_Pin 16
#define I2c2_HET_SDA_Pin 14


#endif /* DRIVERS_INC_HARDWARECONFIG_H_ */
