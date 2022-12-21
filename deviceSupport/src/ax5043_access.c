/*
 * ax5043_access.c
 *
 *  Created on: Jun 15, 2019
 *      Author: bfisher
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

//Golf headers
#include "spiDriver.h"
#include "ax5043.h"
#include "ax5043_access.h"
#include "ax5043-2M-AFSK-externs.h"
#include "nonvol.h"


#if 0
#include "temp-ax5043-xmit-single-file/ax5043-debug-regs.h" //DEBUG RBG
#endif
static bool PowerOn=false,Rxing=false,Txing=false;
bool IsRxing(void){
    return Rxing;
}
void ax5043WriteRegMulti(unsigned int firstReg, uint8_t *val,uint8_t length){
    uint8_t srcbuf[4];
    uint32_t *command = (uint32_t *)srcbuf;
    srcbuf[0] = 0x00f0 | ((firstReg & 0xf00) >> 8);
    srcbuf[1] = (firstReg & 0xff);

    SPISendCommand(DCTDev0,*command,2,val,(uint16_t)length,0,0);

}
void ax5043ReadRegMulti(unsigned int firstReg, uint8_t *val,uint8_t length){
    uint8_t srcbuf[4];
    uint32_t *command = (uint32_t *)srcbuf;
    srcbuf[0] = 0x0070 | ((firstReg & 0xf00) >> 8);
    srcbuf[1] = (firstReg & 0xff);

    SPISendCommand(DCTDev0,*command,2,0,0,val,(uint16_t)length);

}

void ax5043WriteReg(unsigned int reg, unsigned int val)  {
  //spiDAT1_t dc = {.WDEL = false, .CS_HOLD = true, .DFSEL = SPI_FMT_0, .CSNR = 1 };
  uint8_t srcbuf[3];


  srcbuf[0] = 0x00f0 | ((reg & 0xf00) >> 8);
  srcbuf[1] = (reg & 0xff);
  srcbuf[2] = val & 0xff;


//  gioSetBit(spiPORT3,1,0);  //Set CS1 1 low
//  spi_write(1,3,srcbuf);
//  gioSetBit(spiPORT3,1,1);  //Set CS1 1 high
  SPISendCommand(DCTDev0,0,0,srcbuf,3,0,0);

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

unsigned int ax5043ReadLongreg(unsigned int reg,int bytes)
{
  uint8_t srcbuf[2];
  uint8_t dstbuf[4]={0,0,0,0};
  unsigned int retval=0,i;
  if(bytes>4)bytes=4;

  srcbuf[0] = 0x0070 | ((reg & 0xf00) >> 8);
  srcbuf[1] = (reg & 0xff);

  SPISendCommand(DCTDev0,0,0,srcbuf,2,dstbuf,bytes);
  for(i=0;i<bytes;i++){
      retval <<= 8;
      retval |= dstbuf[i];
  }
  return retval;
}
unsigned int ax5043ReadReg(unsigned int reg){
    return ax5043ReadLongreg(reg,1);
}

/*
 * Set the sysclock output ratio with the input cryatal.
 */
#define DIVIDEby2 0x5
#define DIVIDEby1 0x4
bool ax5043SetClockout(void){
    unsigned int retVal;
    retVal = ax5043ReadReg(AX5043_PINFUNCSYSCLK);
    ax5043WriteReg(AX5043_PINFUNCSYSCLK, DIVIDEby1); // Change the clock divider so it outputs 16MHz
    return (retVal != DIVIDEby1); // Return true if we actually changed something
}

/*
 * Common area to start and stop transmit and receive and to power off for safe mode
 */

void ax5043PowerOn(void){
    // Later boards are active high
    //GPIOSetOn(DCTPower);
    PowerOn=true;
    vTaskDelay(CENTISECONDS(1)); // Don't try to mess with it for a bit
}
void ax5043PowerOff(void){
    //GPIOSetOff(PAPower);  // Make sure the PA is off if we are turning off the 5043.
    //GPIOSetOff(DCTPower);
    PowerOn = Rxing = Txing = false;
}
void ax5043StartRx(void){
    //printf("StartRx: Power=%d,Txing=%d,Rxing=%d\n",PowerOn,Txing,Rxing);
    ax5043StopTx();
    if(!PowerOn){
        ax5043PowerOn();
        PowerOn = true;
    }
    if(!Rxing){
        start_rx();
        Rxing=true; Txing=false;
    }
}
void ax5043StopRx(void){
    //printf("StopRx: Power=%d,Txing=%d,Rxing=%d\n",PowerOn,Txing,Rxing);
    if(Rxing & PowerOn){
        ax5043_off();  // Do not turn off power, just stop receiving
        Txing=Rxing=false;
    }
}

void ax5043StartTx(void){
    //printf("StartTx: Power=%d,Txing=%d,Rxing=%d\n",PowerOn,Txing,Rxing);
    if(Rxing){
        ax5043StopRx();
    }
    if(!PowerOn){
        ax5043PowerOn();
        PowerOn = true;
    }
    start_tx();
    Txing = true; Rxing = false;
}
void ax5043StopTx(void){
    //printf("StopTx: Power=%d,Txing=%d,Rxing=%d\n",PowerOn,Txing,Rxing);
    if(Txing && PowerOn){
        ax5043_off();
        Txing=Rxing=false;
    }
    ax5043PowerOff();
}

