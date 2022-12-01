/*
 * flash.h
 *
 *  Created on: Nov 12, 2012
 *      Author: fox
 */

#ifndef NONVOL_H_
#define NONVOL_H_

/*
 * The NV routines can be used to write to any non-volatile memory
 * area that needs some sort of special access.  For example, the current
 * version can write to either STM32L flash or to an external F-RAM/M-RAM.
 */

#include <pacsat.h>
#include "stdint.h"
typedef enum {LocalEEPROMData,ExternalMRAMData} NVType;

bool writeNV(void const * const data, uint32_t dataLength, NVType memoryType, uint32_t address);
bool readNV(void *data, uint32_t dataLength, NVType memoryType,  uint32_t address);
int getSizeNV(NVType type);
int initNV(NVType type);
uint8_t ReadMRAMStatus(void);
void WriteMRAMStatus(uint8_t);
bool MRAMWriteEnable(void);

#endif /* NONVOL_H_ */
