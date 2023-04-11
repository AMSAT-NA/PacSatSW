/*
 * flash.c

 *
 *  Created on: Nov 12, 2012
 *      Author: Burns Fisher, AMSAT-NA
 *
 *  This is a set of generic services for accessing non-volatile
 *  RAM.  It can be used to read or write either memory that is
 *  directly accessible via a local address, or memory that is
 *  accessed via an external bus like SPI.
 */
#include <stddef.h>
#include "mram.h"
#include "nonvol.h"
#include "spiDriver.h"
#include "errors.h"
#include "CANSupport.h"

bool writeNV(void const * const data, uint32_t dataLength, NVType type,
	     uint32_t nvAddress)
{
    switch (type) {
    case NVStatisticsArea:
	return writeMRAM(0, data, dataLength, nvAddress);

    case NVFileSystem:
	return writeMRAM(1, data, dataLength, nvAddress);

    default:
	return false;
    }
}

bool readNV(void *data, uint32_t dataLength, NVType type, uint32_t nvAddress)
{
    switch (type) {
    case NVStatisticsArea:
	return readMRAM(0, data, dataLength, nvAddress);

    case NVFileSystem:
	return readMRAM(1, data, dataLength, nvAddress);

    default:
	return false;
    }
}

int getSizeNV(NVType type)
{
    switch (type) {
    case NVStatisticsArea:
	return getSizeMRAM(0);

    case NVFileSystem:
	return getSizeMRAM(1);

    default:
	return 0;
    }
}
