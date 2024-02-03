/*
 * ADS7828.c
 *
 *  Created on: Oct 14, 2013
 *      Author: Mike McCann KB2GHZ
 */
#include <pacsat.h>
#include "stdint.h"
#include "i2cDriver.h"
#include "errors.h"
#include "ADS7828.h"
#include "FreeRTOS.h"
#include "os_task.h"
#include "Max31725Temp.h"
#include "Max31331Rtc.h"


uint16_t uint16_swap(uint16_t);


/* ADS7828 ADC conversion command table
 * single-ended input conversion mode
 * using the internal voltage reference.
 * This table is used with the MPPT and the ICR
 */

uint8_t ADS7828Cmds[] =     {		SINGLE_ENDED | CH0 | INTERNAL_REFERENCE_CONVERTER_ON,
									SINGLE_ENDED | CH1 | INTERNAL_REFERENCE_CONVERTER_ON,
									SINGLE_ENDED | CH2 | INTERNAL_REFERENCE_CONVERTER_ON,
									SINGLE_ENDED | CH3 | INTERNAL_REFERENCE_CONVERTER_ON,
									SINGLE_ENDED | CH4 | INTERNAL_REFERENCE_CONVERTER_ON,
									SINGLE_ENDED | CH5 | INTERNAL_REFERENCE_CONVERTER_ON,
									SINGLE_ENDED | CH6 | INTERNAL_REFERENCE_CONVERTER_ON,
									SINGLE_ENDED | CH7 | INTERNAL_REFERENCE_CONVERTER_ON
};

/* This function reads the specified number of ADS7828 ADC channels
 * and stores the results in memory
 */
void getADCchannels(int num, I2cBusNum port, uint8_t I2Caddress, uint16_t *storage) {
	int i;
	uint8_t powerOffCmd;
	bool success=true;

	for (i = 0; i < num; i++) {
	    bool thisOneOk;
        //*storage = 0xff00;
        thisOneOk = I2cSendCommand(port,I2Caddress, &ADS7828Cmds[i],1,storage,2);
        if(!thisOneOk){
            success = false;
            *storage = 0;
        }
		storage++;
	}

	/* power off the ADS7828 if there was a failure */

	if(!success){
	    powerOffCmd = ADC_OFF_INTERNAL_REF_ON;
	    I2cSendCommand(port,I2Caddress, &powerOffCmd,0,NULL,0);
	}
}

