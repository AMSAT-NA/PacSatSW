/*
 * MPU9250-Gyro.h
 *
 *  Created on: Feb 20, 2018
 *      Author: fox
 */

#ifndef MPU9250_GYRO_H_
#define MPU9250_GYRO_H_

#include "spi.h"

/* Definitions for the MPU9250 registers */

#define G9250_REG_I2CDIS 0xF /* Disable I2C*/
#define G9250_READ 0x80
#define G9250_WRITE 0x0
#define G9250_CMD_I2CDIS 0b00011011

#endif /* MPU9250_GYRO_H_ */

/* Routines */

bool GyroInit(void);
bool GyroWriteOneRegister(uint8_t regAddress,uint8_t value);
bool GyroReadRegisters(uint8_t regAddress, uint8_t *values, uint16_t length);
uint16_t GyroGetTemp(void);
bool GyroReadSpin(uint16_t *xyz);
bool GyroReadAccel(uint16_t *xyz);
bool GyroReadMag(uint16_t *xyz);
bool GyroReadI2CStatus(uint8_t *status);
uint8_t MagWhami(void);
