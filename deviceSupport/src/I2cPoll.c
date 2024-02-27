/*
 * I2cPoll.c
 *
 *  Created on: Feb 3, 2024
 *      Author: Burns Fisher WB1FJ
 */
#include <pacsat.h>
#include "stdint.h"
#include "i2cDriver.h"
#include "errors.h"
#include "I2cPoll.h"
#include "FreeRTOS.h"
#include "os_task.h"
#include "Max31725Temp.h"
#include "Max31331Rtc.h"

static bool RTCStat=false,TxTempStat=false,CpuTempStat=false;


/* this function tests for the presence of I2c devices.  It ADS7828 ADCs in the PSU and Battery cards */
void I2CDevicePoll()  {
    uint8_t cfg[1];

    //uint8_t dummy=5,sendVal = (SINGLE_ENDED | CH1 | INTERNAL_REFERENCE_CONVERTER_ON); // Turn on the reference and converter
    // Within this module we poll the 7828s.  Then we call other device support routines for the rest
	//ICRStat = I2cSendCommand(ICR_ADC_I2C_PORT,ICR_ADC_I2C_ADDRESS,&sendVal,1,&dummy,1);
    //CSSStat = I2cSendCommand(CSS_ADC_I2C_PORT,CSS_ADC_I2C_ADDRESS,&sendVal,1,&dummy,1);
    //SolarStat = I2cSendCommand(SOLAR_ADC_I2C_PORT,SOLAR_ADC_I2C_ADDRESS,&sendVal,1,&dummy,1);
    TxTempStat = InitTemp31725(TxTemp);
    CpuTempStat = InitTemp31725(CpuTemp);
    RTCStat = GetStatus31331(cfg);
}
bool RTCIsOk(void){
    return RTCStat;
}
bool TxTempIsOk(void){
    return TxTempStat;
}
bool CpuTempIsOk(void){
    return CpuTempStat;
}
