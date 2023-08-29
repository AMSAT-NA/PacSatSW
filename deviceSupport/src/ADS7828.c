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


uint16_t uint16_swap(uint16_t);

bool RTCStat=false,RTTempStat=false;

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

/* this function tests for the presence of I2c devices.  It ADS7828 ADCs in the PSU and Battery cards */
void I2CDevicePoll()  {
    uint8_t cfg[1];

    //uint8_t dummy=5,sendVal = (SINGLE_ENDED | CH1 | INTERNAL_REFERENCE_CONVERTER_ON); // Turn on the reference and converter
    // Within this module we poll the 7828s.  Then we call other device support routines for the rest
	//ICRStat = I2cSendCommand(ICR_ADC_I2C_PORT,ICR_ADC_I2C_ADDRESS,&sendVal,1,&dummy,1);
    //CSSStat = I2cSendCommand(CSS_ADC_I2C_PORT,CSS_ADC_I2C_ADDRESS,&sendVal,1,&dummy,1);
    //SolarStat = I2cSendCommand(SOLAR_ADC_I2C_PORT,SOLAR_ADC_I2C_ADDRESS,&sendVal,1,&dummy,1);
    RTTempStat = InitTemp31725();
    RTCStat = GetStatus31331(cfg);
}
bool RTCIsOk(void){
    return RTCStat;
}
bool RTTempIsOk(void){
    return RTTempStat;
}
