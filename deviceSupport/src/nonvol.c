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
        /*
         * This code knows about the commands for and has been tested with
         * an external RAMTRON F-RAM and an Eversource MRAM.
         */

        ByteToWord writeCommand, framAddress;
        SPIDevice mramDev;

        framAddress.word = nvAddress;
        mramDev = GetMRAMAndAddress(&framAddress.word);
        if (mramDev == InvalidSPI) {
            return false;
        }
        MRAMWriteEnable(mramDev);
        writeCommand.byte[0] = FRAM_OP_WRITE;
        // The MRAM address is big endian, but so is the processor.
        writeCommand.byte[1] = framAddress.byte[1];
        writeCommand.byte[2] = framAddress.byte[2];
        writeCommand.byte[3] = framAddress.byte[3];
        //printf("Write %x to addr %x in MRAM %d, requested addr=%x\n",
        //       *(uint32_t *)data,framAddress.word,(int)mramDev,nvAddress);

	/* Now write */
        SPISendCommand(mramDev, writeCommand.word, ADDRESS_BYTES+1,
                       (uint8_t *)data, dataLength, NULL, 0);

        return true;
    }
    return false;
}

bool readNV(void *data, uint32_t dataLength, NVType type, uint32_t nvAddress)
{
    int retry;
    SPIDevice mramDev;

    if (type == LocalEEPROMData){
        return false;
    } else if (type == ExternalMRAMData){
        ByteToWord framAddress, ourAddress;

        /*
         * See comments above regarding writing the MRAM
         */

        ourAddress.word = nvAddress;
        mramDev = GetMRAMAndAddress(&ourAddress.word);
        if (mramDev == InvalidSPI) {
            return false;
        }
        framAddress.byte[1] = ourAddress.byte[1];  // Address is big-endian.
        framAddress.byte[2] = ourAddress.byte[2];  // Address is big-endian.
        framAddress.byte[3] = ourAddress.byte[3];  // Address is big-endian.
        framAddress.byte[0] = FRAM_OP_READ;

        retry = SPI_MRAM_RETRIES;
        while (retry-- > 0){
            /* Retry a few times before we give up */
            if (SPISendCommand(mramDev, framAddress.word, ADDRESS_BYTES+1, 0, 0,
			       (uint8_t *) data, dataLength)){
                //printf("Read %x from addr %x in MRAM %d, requested addr=%x\n",*(uint32_t *)data,ourAddress.word,(int)mramDev,nvAddress);
                return true;
            }
            //          IHUSoftErrorData.SPIRetries++;
        }
        ReportError(SPIMramTimeout, TRUE, ReturnAddr,
		    (int)__builtin_return_address(0));
    }
    return false;
}

int getSizeNV(NVType type)
{
    if (type != ExternalMRAMData)
	return 0;
    return getSizeMRAM();
}
