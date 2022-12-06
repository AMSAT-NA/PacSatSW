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
#ifdef FRAM64K
#define ADDRESS_BYTES 2
#else
#define ADDRESS_BYTES 3
#endif
static SPIDevice GetMRAMAndAddress(int *addr){
    int MRAMSize=512*1024;  // todo:  Calculate this from mram size
    int MRAMNumber=0,numberOfMRAMs=2;
    while(*addr > MRAMSize){
        *addr-=MRAMSize;
        MRAMNumber++;
        if(MRAMNumber >= numberOfMRAMs){
            return InvalidSPI;
        }
    }
    return (SPIDevice)(MRAM0Dev+MRAMNumber); //Assume MRAM values are in order

}
bool writeNV(void const * const data, uint32_t dataLength,NVType type, uint32_t nvAddress){
    if (type == LocalEEPROMData){
        // This is here just in case there is a way to do this on RT-IHU
        return false;
    } else if (type == ExternalMRAMData){
        /*
         * This code knows about the commands for and has been tested with
         * an external RAMTRON F-RAM and an Eversource MRAM.
         */

        ByteToWord writeCommand,framAddress;
        SPIDevice mramDev;
        framAddress.word = nvAddress;
        mramDev = GetMRAMAndAddress(&framAddress.word)
        MRAMWriteEnable(mramDev);
        writeCommand.byte[0] = FRAM_OP_WRITE;
        // The MRAM address is big endian, but so is the processor.
        writeCommand.byte[1] = framAddress.byte[1];
        writeCommand.byte[2] = framAddress.byte[2];
        writeCommand.byte[3] = framAddress.byte[3];

        SPISendCommand(mramDev, writeCommand.word,ADDRESS_BYTES+1, /* Now write    */
                       (uint8_t *)data,dataLength,  NULL,0);

        return TRUE;
    }
    return FALSE;
}

bool readNV(void *data, uint32_t dataLength, NVType type, uint32_t nvAddress){
    int retry;
    if (type == LocalEEPROMData){
        return false;
    } else if (type == ExternalMRAMData){
        ByteToWord framAddress,ourAddress;

        /*
         * See comments above regarding writing the MRAM
         */

        ourAddress.word = nvAddress;
        framAddress.byte[1] = ourAddress.byte[1];  // Address is big-endian.
        framAddress.byte[2] = ourAddress.byte[2];  // Address is big-endian.
        framAddress.byte[3] = ourAddress.byte[3];  // Address is big-endian.
        framAddress.byte[0] = FRAM_OP_READ;

        retry=SPI_MRAM_RETRIES;
        while(retry-- > 0){
            /* Retry a few times before we give up */
            if(SPISendCommand(MRAMDev, framAddress.word,ADDRESS_BYTES+1,0,0,
                              (uint8_t *)data, dataLength)){
                return TRUE;
            }
            //          IHUSoftErrorData.SPIRetries++;
        }
        ReportError(SPIMramTimeout,TRUE,ReturnAddr,(int)__builtin_return_address(0));
    }
    return FALSE;
}
uint8_t ReadMRAMStatus(void){
    ByteToWord command;
    uint8_t data;
    command.byte[0]=FRAM_OP_RDSR;
    SPISendCommand(MRAMDev,command.word,1,0,0,&data,1);
    return data;
}
bool MRAMWriteEnable(SPIDevice mramNum){
    bool stat;
    ByteToWord command;
    command.byte[0] = FRAM_OP_WREN;
    stat = SPISendCommand(mramNum,command.word,1, NULL,0,NULL,0);        /* Write enable */
    return stat;
}
void WriteMRAMStatus(SPIDevice mramNum,uint8_t status){
    ByteToWord command;
    command.byte[0] = FRAM_OP_WREN;
    SPISendCommand(mramNum,command.word,1,0,0,0,0);
    command.byte[0] = FRAM_OP_WRSR;
    command.byte[1] = status;
    SPISendCommand(mramNum,command.word,2,0,0,0,0);


}
int getSizeNV(NVType type){
    /*
     * This routine 'knows' about the FRAM and the on-board FLASH data sizes
     * although it knows about the FRAM via a define in fram.h.  For the flash,
     * it just returns the (known) size.  For FRAM, it checks to see that there
     * is a non-0 status return before returning the known FRAM size.
     */
    static int framSize=0;
    if (type == ExternalMRAMData){
        int temp1M,temp4M;
        /* For the IHU board, it could be using either 1M or 4M */
        /* We find out by writing a number to each max address and
         * seeing which one comes back.  For the smaller part, the 4M address
         * wraps and ends up at the same address as the 1M address */
        const int end1M=FRAM_1M_ADDRESS_MAX,end4M=FRAM_4M_ADDRESS_MAX;
        // First read what was in these two locations originally
        readNV((uint32_t *) &temp1M, sizeof(temp1M), ExternalMRAMData, (int)FRAM_1M_ADDRESS_MAX-3);
        readNV((uint32_t *) &temp4M, sizeof(temp4M), ExternalMRAMData, (int)FRAM_4M_ADDRESS_MAX-3);
        // Now write the MRAM size into each maximum address
        writeNV((uint32_t *) &end4M, sizeof(end4M), ExternalMRAMData, (int)FRAM_4M_ADDRESS_MAX-3);
        writeNV((uint32_t *) &end1M, sizeof(end1M), ExternalMRAMData, (int)FRAM_1M_ADDRESS_MAX-3);
        //Whichever one we get back when we read the 4Meg address wins
        readNV((uint32_t *) &framSize, sizeof(framSize), ExternalMRAMData, (int)FRAM_4M_ADDRESS_MAX-3);
        //Now restore the original data
        writeNV((uint32_t *) &temp1M, sizeof(temp1M), ExternalMRAMData, (int)FRAM_1M_ADDRESS_MAX-3);
        writeNV((uint32_t *) &temp4M, sizeof(temp4M), ExternalMRAMData, (int)FRAM_4M_ADDRESS_MAX-3);
        if(framSize == 0xffffffff)framSize=0;// If we got back all ones, the SPI got nothing.  It's not there.

        return framSize;
    }
    return 0;
}
int initNV(NVType type){
    // Initialize status register to 0 so there are no memory banks write protected
    WriteMRAMStatus(0);
    return FRAM_1M_ADDRESS_MAX;
}


