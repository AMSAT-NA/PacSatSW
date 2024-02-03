/*
 * Max31725Temp.h
 *
 *  Created on: Aug 20, 2019
 *      Author: bfisher
 */


#ifndef DEVICESUPPORT_INC_MAX31725TEMP_H_
#define DEVICESUPPORT_INC_MAX31725TEMP_H_

/*
 * Addresses for Golf
 */
#include "I2cAddresses.h"
typedef enum _Temp {
    TxTemp=0,
    CpuTemp=1
}TempSensor;
#define MAX31725_ADDR RTIHU_TMP_ADDRESS
#define MAX31725_PORT RTIHU_TMP_PORT

#define MAX31725_REG_TEMP 0
#define MAX31725_REG_CONFIG 1
#define MAX31725_CONFIG_TIMEOUT 0x40

bool InitTemp31725(TempSensor);
bool GetTemp31725(TempSensor,int16_t *);
bool GetConfig31725(TempSensor,uint8_t *cfg);
bool Get8BitTemp31725(TempSensor,uint8_t *temp8);


#endif /* DEVICESUPPORT_INC_MAX31725TEMP_H_ */
