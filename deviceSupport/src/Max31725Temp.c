/*
 * Max31725Temp.c
 *
 *  Created on: Aug 20, 2019
 *      Author: bfisher
 */
#ifdef BLINKY_HARDWARE
#include <pacsat.h>
#include "I2cAddresses.h"
#include "Max31725Temp.h"
#include "i2cDriver.h"
static uint8_t buf[2],send;
const uint32 tempAddr[]={PACBOARD_TX_TEMP_ADDRESS,PACBOARD_CPU_TEMP_ADDRESS};
bool InitTemp31725(TempSensor sensor){
    buf[0] = MAX31725_REG_CONFIG;
    buf[1] = MAX31725_CONFIG_TIMEOUT;
    return I2cSendCommand(PACBOARD_TEMP_PORT,tempAddr[sensor],buf,2,0,0);
    };

bool GetConfig31725(TempSensor sensor,uint8_t *cfg){
    send = MAX31725_REG_CONFIG;
    return I2cSendCommand(PACBOARD_TEMP_PORT,tempAddr[sensor],&send,1,cfg,1);
}
bool GetTemp31725(TempSensor sensor,int16_t *temp){
    bool retval=true;
    int16_t localTemp;
    send = MAX31725_REG_TEMP;
    retval = I2cSendCommand(PACBOARD_TEMP_PORT, tempAddr[sensor], &send,1,&localTemp,2);
    *temp = localTemp >> 4; //Only 12 bits.
    return retval;
}
bool Get8BitTemp31725(TempSensor sensor,uint8_t *temp8){
    int16_t temp;
    uint8_t utemp;
    bool fract,stat = GetTemp31725(sensor,&temp);
    if(!stat)temp = 0xFFFF;
    /*
     *   Here is where we turn the 12 bit temp into
     *   8 bits.  It arrives as 12 bits of 2's complement
     *   8.4 fixed point for a nominal range of -128-+127C
     *   We turn it into 7.1 with an offset so that a mantissa of
     *   0-127 means -20 - +107C and the 1 bit fraction is (0 or .5)
     */
    temp += 7; // Round up to the next 1 bit fraction (or 1/2)
    fract = ((temp & 0x8) != 0);  //Get the most significant fraction bit
    // Hopefully the compiler does the below in one step.
    temp = temp >> 4; // Now we have 2's complement degrees C (-128 - +127)
    utemp = (uint8_t)(temp +20); // Offset by 20 so that 0 = -20C, 20 = 0C, 40=20C, 127 = 107C
    utemp = utemp << 1; // Leave a fraction bit at the bottom.  Now 0xFe = 107C
    *temp8 = utemp | (fract ? 1:0);
    return stat;
}
#endif
