/*
 * flash.c

 *
 *  Created on: Nov 12, 2012
 *      Author: Burns Fisher, AMSAT-NA
 *
 *      This is a set of generic services for accessing non-volatile RAM.  It can be used
 *      to read or write either memory that is directly accessible via a local address, or
 *      memory that is accessed via an external bus like SPI.
 *
 * 		The NVR is logically organized into named banks with 0-based addresses.  At this
 * 		point we have "LocalEEPROMData" (data on the STM32L card) and "ExternalMRAMData",
 * 		(data on an external MRAM/FRAM accessed via SPI bus).
 *
 * 		Note that currently the local memory is only accessed 1 32-bit word at a time.  This
 * 		should be changed so that all the memory works the same.
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
    if (type == LocalEEPROMData) {
        // This is here just in case there is a way to do this on RT-IHU
        return false;
    } else if (type == ExternalMRAMData) {
	return writeMRAM(data, dataLength, nvAddress);
    }
    return false;
}

bool readNV(void *data, uint32_t dataLength, NVType type, uint32_t nvAddress)
{
    if (type == LocalEEPROMData){
        return false;
    } else if (type == ExternalMRAMData){
	return readMRAM(data, dataLength, nvAddress);
    }
    return false;
}

int getSizeNV(NVType type)
{
    if (type != ExternalMRAMData)
	return 0;
    return getSizeMRAM();
}
