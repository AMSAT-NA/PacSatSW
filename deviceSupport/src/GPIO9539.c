/*
 * GPIO9539.c
 *
 *  Created on: Aug 20, 2019
 *      Author: bfisher
 *
 *  This is the device support routine for a PCA9539, a GPIO expander module
 *  on I2c
 */
#include <pacsat.h>
#include "GPIO9539.h"
#include "i2cDriver.h"
#if 0
bool ReadPCA9539Config(uint8_t config[2]){
    uint8_t command = PCA9539_REG_CONFIG0;
    return(I2cSendCommand(I2C1,PCA9539_ADDR,&command,1,config,2));
}
bool WritePCA9539Config(uint8_t config[2]){
    uint8_t command[3];
    command[0]= PCA9539_REG_CONFIG0;
    command[1] = config[0];
    command[2] = config[1];
    return(I2cSendCommand(I2C1,PCA9539_ADDR,command,3,0,0));
}
bool WritePCA9539Bits(int bank, uint8_t *gpioBits){
    bool retVal;
    uint8_t bits[2];
    bits[0] = (bank==0)?PCA9539_REG_OUTPUT0:PCA9539_REG_OUTPUT1;
    bits[1] = *gpioBits;
    retVal = I2cSendCommand(I2C1, PCA9539_ADDR, bits,2,0,0);
    return retVal;
}
#endif
bool ReadPCA9539Bits(int bank, uint8_t *gpioBits){
    uint8_t command;
    command = (bank==0)?PCA9539_REG_INPUT0:PCA9539_REG_INPUT1;
    return I2cSendCommand(I2C1, PCA9539_ADDR, &command,1,gpioBits,1);
}
