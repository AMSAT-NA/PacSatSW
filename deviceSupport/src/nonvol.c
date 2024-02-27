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

bool writeNV(void const * const data, uint32_t dataLength, NVType type,
	     uint32_t nvAddress)
{
    switch (type) {
    case NVConfigData:
	return writeMRAM(0, data, dataLength, nvAddress);

    case NVFileSystem:
	return writeMRAM(1, data, dataLength, nvAddress);

    case NVEntireMRAM:
        return writeMRAM(2, data, dataLength, nvAddress);
    default:
	return false;
    }
}

bool readNV(void *data, uint32_t dataLength, NVType type, uint32_t nvAddress)
{
    switch (type) {
    case NVConfigData:
	return readMRAM(0, data, dataLength, nvAddress);

    case NVFileSystem:
	return readMRAM(1, data, dataLength, nvAddress);

    case NVEntireMRAM:
    return readMRAM(2, data, dataLength, nvAddress);

    default:
	return false;
    }
}

int getSizeNV(NVType type)
{
    switch (type) {
    case NVConfigData:
	return getMRAMPartitionSize(0);

    case NVFileSystem:
        return getMRAMPartitionSize(1);
    case NVEntireMRAM:
        return getMRAMSize(0); // todo: Get entire bank size, not just one.
    default:
	return 0;
    }
}
