/*
 * HETInterrupt.c
 *
 *  Created on: Dec 21, 2019
 *      Author: bfisher
 *
 *  The HET notification for both I2c and UART comes here.  I could hack into HALCoGen's HET routines, but
 *  I don't think I will.
 *
 */
#include "het.h"
#include "serialDriver.h"
#include "uartEmulator.h"
#include "i2cEmulator.h"
void hetNotification(hetBASE_t *het, uint32 offset){
    if (het == hetREG1)serialHETInterrupt(offset);
    else i2cHETInterrupt(offset);
}


