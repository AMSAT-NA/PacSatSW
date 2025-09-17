/*
 * ax5043_access.c
 *
 *  Created on: Jun 15, 2019
 *      Author: bfisher
 *
 * This file contains the headers for AX5043 routines that are common across modes and frequencies.
 * For example, it contains the routines to read and write registers and the routines to
 * start and stop the radio.  It does not apply any configurations specific to modes or
 * frequency bands.  For PACSAT that is all contained in ax5043_ax25.c
 *
 */


// Generic golf stuff--also include most CLIB things that are required

#include <pacsat.h>
#include "sys_common.h"
#include "spi.h"
#include "spi-replacement.h"
//#include "gio.h"
//#include "sys_core.h"
#include "gpioDriver.h"
//FreeRTOS headers
#include "FreeRTOS.h"
#include "os_task.h"

//Pacsat headers
#include "spiDriver.h"
#include "ax5043.h"
#include "ax5043_access.h"
#include "ax5043-ax25.h"
#include "nonvol.h"
#include "nonvolManagement.h"


/* Local Variables */
static bool PowerOn[NUM_AX5043_SPI_DEVICES+4];
static bool Rxing[NUM_AX5043_SPI_DEVICES+4];
static bool Txing[NUM_AX5043_SPI_DEVICES+4];

/* This lookup table returns the SPIDevice id for a given AX5043Device id */
#ifdef LAUNCHPAD_HARDWARE
static SPIDevice ax5043_spi_devices[] = {AX5043Dev0,AX5043Dev1};
#else
static SPIDevice ax5043_spi_devices[] = {Rx1AX5043Dev,Rx2AX5043Dev,Rx3AX5043Dev,Rx4AX5043Dev,TxAX5043Dev};
#endif

bool IsRxing(AX5043Device device){
    return Rxing[device];
}
void ax5043WriteRegMulti(AX5043Device device, unsigned int firstReg, uint8_t *val,uint8_t length){
    uint8_t srcbuf[4];
    uint32_t *command = (uint32_t *)srcbuf;
    srcbuf[0] = 0x00f0 | ((firstReg & 0xf00) >> 8);
    srcbuf[1] = (firstReg & 0xff);

    SPIDevice spidevice = ax5043_spi_devices[device];
    SPISendCommand(spidevice,*command,2,val,(uint16_t)length,0,0);

}
void ax5043ReadRegMulti(AX5043Device device, unsigned int firstReg, uint8_t *val,uint8_t length){
    uint8_t srcbuf[4];
    uint32_t *command = (uint32_t *)srcbuf;
    srcbuf[0] = 0x0070 | ((firstReg & 0xf00) >> 8);
    srcbuf[1] = (firstReg & 0xff);

    SPIDevice spidevice = ax5043_spi_devices[device];
    SPISendCommand(spidevice,*command,2,0,0,val,(uint16_t)length);

}

void ax5043WriteReg(AX5043Device device, unsigned int reg, unsigned int val)  {
  //spiDAT1_t dc = {.WDEL = false, .CS_HOLD = true, .DFSEL = SPI_FMT_0, .CSNR = 1 };
  uint8_t srcbuf[3];


  srcbuf[0] = 0x00f0 | ((reg & 0xf00) >> 8);
  srcbuf[1] = (reg & 0xff);
  srcbuf[2] = val & 0xff;


//  gioSetBit(spiPORT3,1,0);  //Set CS1 1 low
//  spi_write(1,3,srcbuf);
//  gioSetBit(spiPORT3,1,1);  //Set CS1 1 high
  SPIDevice spidevice = ax5043_spi_devices[device];
  SPISendCommand(spidevice,0,0,srcbuf,3,0,0);

#if 0
  if ((reg != AX5043_FIFODATA) && (reg != AX5043_FIFOSTAT)) {
      if ((reg < (sizeof(axregs)/sizeof(axregs[0]))) && (axregs[reg] != NULL)) {
          printf("ax5043WriteReg: %s: %02.2x\n", axregs[reg], val);
      } else {
          printf("ax5043WriteReg: %x: %02.2x\n", reg, val);
      }
  }
#endif

}

unsigned int ax5043ReadLongreg(AX5043Device device, unsigned int reg,int bytes)
{
  uint8_t srcbuf[2];
  uint8_t dstbuf[4]={0,0,0,0};
  unsigned int retval=0,i;
  if(bytes>4)bytes=4;

  srcbuf[0] = 0x0070 | ((reg & 0xf00) >> 8);
  srcbuf[1] = (reg & 0xff);

  SPIDevice spidevice = ax5043_spi_devices[device];
  SPISendCommand(spidevice,0,0,srcbuf,2,dstbuf,bytes);
  for(i=0;i<bytes;i++){
      retval <<= 8;
      retval |= dstbuf[i];
  }
  return retval;
}
unsigned int ax5043ReadReg(AX5043Device device, unsigned int reg){
    return ax5043ReadLongreg(device,reg,1);
}

/*
 * Set the sysclock output ratio with the input cryatal.
 */
#define DIVIDEby2 0x5
#define DIVIDEby1 0x4
bool ax5043SetClockout(AX5043Device device){
    unsigned int retVal;
    retVal = ax5043ReadReg(device, AX5043_PINFUNCSYSCLK);
    ax5043WriteReg(device, AX5043_PINFUNCSYSCLK, DIVIDEby1); // Change the clock divider so it outputs 16MHz
    return (retVal != DIVIDEby1); // Return true if we actually changed something
}

/*
 * Common area to start and stop transmit and receive and to power off for safe mode
 */

void ax5043PowerOn(AX5043Device device){
    // Later boards are active high
    //GPIOSetOn(AX5043Power);
    PowerOn[device]=true;
    vTaskDelay(CENTISECONDS(1)); // Don't try to mess with it for a bit
}
void ax5043PowerOff(AX5043Device device){
    //GPIOSetOff(PAPower);  // Make sure the PA is off if we are turning off the 5043.
    //GPIOSetOff(AX5043Power);
    PowerOn[device] = Rxing[device] = Txing[device] = false;
}

uint8_t ax5043_off_xtal(AX5043Device device) {
    ax5043WriteReg(device, AX5043_PWRMODE, AX5043_PWRSTATE_XTAL_ON);
    ax5043WriteReg(device, AX5043_LPOSCCONFIG, 0x00); // LPOSC off
    return AXRADIO_ERR_NOERROR;
}

uint8_t ax5043_off(AX5043Device device) {
    uint8_t retVal;

    retVal = ax5043_off_xtal(device);
    if (retVal != AXRADIO_ERR_NOERROR) {
        return retVal;
    }

    ax5043WriteReg(device, AX5043_PWRMODE, AX5043_PWRSTATE_POWERDOWN);

    return AXRADIO_ERR_NOERROR;
}

/**
 * TODO - this now starts the AX25 RX.  It should be setup to start multiple receivers
 * We also need another function to start the command receiver.
 *
 */
void ax5043StartRx(AX5043Device device, bool antenna_differential){
    //printf("StartRx: Power=%d,Txing=%d,Rxing=%d\n",PowerOn,Txing,Rxing);
    ax5043StopTx(device);
    if(!PowerOn[device]){
        ax5043PowerOn(device);
        PowerOn[device] = true;
    }
    if(!Rxing[device]){
//        start_ax25_rx(device, RATE_1200);
        bool rate = ReadMRAMBoolState(StateAx25Rate9600);
        start_ax25_rx(device, rate, antenna_differential);
        Rxing[device]=true; Txing[device]=false;
    }
}
void ax5043StopRx(AX5043Device device){
    //printf("StopRx: Power=%d,Txing=%d,Rxing=%d\n",PowerOn,Txing,Rxing);
    if(Rxing[device] & PowerOn[device]){
        ax5043_off(device);  // Do not turn off power, just stop receiving
        Txing[device]=Rxing[device]=false;
    }
}

void ax5043StartTx(AX5043Device device, bool antenna_differential){
    //printf("StartTx: Power=%d,Txing=%d,Rxing=%d\n",PowerOn,Txing,Rxing);
    if(Rxing[device]){
        ax5043StopRx(device);
    }
    if(!PowerOn[device]){
        ax5043PowerOn(device);
        PowerOn[device] = true;
    }

    bool rate = ReadMRAMBoolState(StateAx25Rate9600);
    start_ax25_tx(device, rate, antenna_differential);
    Txing[device] = true; Rxing[device] = false;
}
void ax5043StopTx(AX5043Device device){
    //printf("StopTx: Power=%d,Txing=%d,Rxing=%d\n",PowerOn,Txing,Rxing);
    if(Txing[device] && PowerOn[device]){
        ax5043_off(device);
        Txing[device]=Rxing[device]=false;
    }
    ax5043PowerOff(device);
}

