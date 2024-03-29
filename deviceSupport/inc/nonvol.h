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
#include "spiDriver.h"
#include "stdint.h"

typedef enum {
    /* Storage for configureation info. */
    NVConfigData = 0,

    /* Area for the filesystem. */
    NVFileSystem,
    NVEntireMRAM
} NVType;

bool writeNV(void const * const data, uint32_t dataLength, NVType memoryType, uint32_t address);
bool readNV(void *data, uint32_t dataLength, NVType memoryType,  uint32_t address);

/*
 * Return the size of the non-volatile region.  This will initialize
 * the non-volatile region, too, so no need to do any init calls.
 */
int getSizeNV(NVType type);

#endif /* NONVOL_H_ */
