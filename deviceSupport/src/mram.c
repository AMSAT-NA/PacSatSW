/*
 * mram.c
 *
 *  Created on: Dec 11, 2022
 *      Author: bfisher
 *
 *      Extract MRAM specific stuff from general NV code in nonvol.c
 *
 */
#include <stddef.h>
#include "mram.h"
#include "nonvol.h"
#include "spiDriver.h"
#include "errors.h"
#include "CANSupport.h"
#include "MET.h"

static const SPIDevice MRAM_Devices[PACSAT_MAX_MRAMS] =
    {MRAM0Dev, MRAM1Dev, MRAM2Dev, MRAM3Dev};
static uint32_t MRAMSize[PACSAT_MAX_MRAMS];
static int numberOfMRAMs = 0;
static int totalMRAMSize = 0;

static int mramPartitionSize[MAX_MRAM_PARTITIONS] = { 0, 0 };
static int mramPartitionOffset[MAX_MRAM_PARTITIONS] = { 0, 0 };

static int addressToMRAMNum(uint32_t *addr)
{
    int MRAMNumber = 0;

    while (*addr >= MRAMSize[MRAMNumber]) {
        *addr -= MRAMSize[MRAMNumber];
        MRAMNumber++;
        if (MRAMNumber > numberOfMRAMs){
            return -1;
        }
    }
    return MRAMNumber;
}

bool MRAMSleep(int mramNum)
{
    ByteToWord command;

    if (mramNum >= PACSAT_MAX_MRAMS)
        return false;
    command.byte[0] = FRAM_OP_SLEEP;
    return SPISendCommand(MRAM_Devices[mramNum], command.word, 1,
                          NULL, 0, NULL, 0);
}

bool MRAMWake(int mramNum)
{
    ByteToWord command;

    if (mramNum >= PACSAT_MAX_MRAMS)
        return false;
    command.byte[0] = MRAM_OP_WAKE;
    return SPISendCommand(MRAM_Devices[mramNum], command.word, 1,
                          NULL, 0, NULL, 0);
}

uint8_t readMRAMStatus(int mramNum)
{
    /*
     * MRAM documentation says that if RDSR is executed immediately after READ, the results
     * will be wrong.  One solution they say is to read it twice.
     */
    ByteToWord command;
    uint8_t data;

    if (mramNum >= PACSAT_MAX_MRAMS)
        return false;
    command.byte[0] = FRAM_OP_RDSR;
    SPISendCommand(MRAM_Devices[mramNum], command.word, 1, 0, 0, &data, 1);
    SPISendCommand(MRAM_Devices[mramNum], command.word, 1, 0, 0, &data, 1);
    return data;
}

bool writeEnableMRAM(int mramNum)
{
    ByteToWord command;

    if (mramNum >= PACSAT_MAX_MRAMS)
        return false;

    command.byte[0] = FRAM_OP_WREN;
    /* Write enable */
    return SPISendCommand(MRAM_Devices[mramNum], command.word,
                          1, NULL, 0, NULL, 0);
}

void writeMRAMStatus(int mramNum, uint8_t status)
{
    ByteToWord command;

    if (mramNum >= PACSAT_MAX_MRAMS)
        return;

    command.byte[0] = FRAM_OP_WREN;
    SPISendCommand(MRAM_Devices[mramNum], command.word, 1, 0, 0, 0, 0);
    command.byte[0] = FRAM_OP_WRSR;
    command.byte[1] = status;
    SPISendCommand(MRAM_Devices[mramNum], command.word, 2, 0, 0, 0, 0);
}

int getSizeMRAM(int partition)
{
    if (partition < 0 || partition >= MAX_MRAM_PARTITIONS)
        return 0;
    if (!numberOfMRAMs) {
        if (initMRAM() == 0)
            return 0;
    }
    return mramPartitionSize[partition];
}

static int getPartitionOffset(int partition)
{
    if (numberOfMRAMs == 0)
        return -1;
    if (partition < 0 || partition > MAX_MRAM_PARTITIONS)
        return -1;

    return mramPartitionOffset[partition];
}

bool writeMRAM(int partition,
               void const * const data, uint32_t length, uint32_t address)
{
    ByteToWord writeCommand, framAddress;
    int base, mramNum;
    SPIDevice mramDev;

    base = getPartitionOffset(partition);
    if (base < 0) {
        ReportError(MRAMwrite, false, PortNumber, (int) partition);
        return false;
    }

    if (address + length > mramPartitionSize[partition]) {
        ReportError(MRAMwrite, false, PortNumber, (int) address);
        return false;
    }

    /*
     * This code knows about the commands for and has been tested with
     * an external RAMTRON F-RAM and an Eversource MRAM.
     */

    framAddress.word = address + base;
    mramNum = addressToMRAMNum(&framAddress.word);
    if (mramNum < 0) {
        ReportError(MRAMwrite, false, TaskNumber, (int) address);
        return false;
    }
    writeEnableMRAM(mramNum);
    mramDev = MRAM_Devices[mramNum];

    writeCommand.byte[0] = FRAM_OP_WRITE;
    // The MRAM address is big endian, but so is the processor.
    writeCommand.byte[1] = framAddress.byte[1];
    writeCommand.byte[2] = framAddress.byte[2];
    writeCommand.byte[3] = framAddress.byte[3];
    //printf("Write %x to addr %x in MRAM %d, requested addr=%x\n",
    //       *(uint32_t *)data,framAddress.word,(int)mramDev,nvAddress);

    /* Now write */
    SPISendCommand(mramDev, writeCommand.word, ADDRESS_BYTES+1,
                   (uint8_t *) data, length, NULL, 0);

    return true;
}

bool readMRAM(int partition,
              void *data, uint32_t length, uint32_t address)
{
    ByteToWord framAddress, ourAddress;
    SPIDevice mramDev;
    int base, mramNum, retry;

    /*
     * See comments above regarding writing the MRAM
     */

    base = getPartitionOffset(partition);
    if (base < 0) {
        ReportError(MRAMread, false, PortNumber, (int) partition);
        return false;
    }

    if (address + length > mramPartitionSize[partition]) {
        ReportError(MRAMread, false, PortNumber, (int) address);
        return false;
    }

    ourAddress.word = address + base;
    mramNum = addressToMRAMNum(&ourAddress.word);
    if (mramNum < 0) {
        ReportError(MRAMread, false, TaskNumber, (int) address);
        return false;
    }
    mramDev = MRAM_Devices[mramNum];

    framAddress.byte[1] = ourAddress.byte[1];  // Address is big-endian.
    framAddress.byte[2] = ourAddress.byte[2];  // Address is big-endian.
    framAddress.byte[3] = ourAddress.byte[3];  // Address is big-endian.
    framAddress.byte[0] = FRAM_OP_READ;

    retry = SPI_MRAM_RETRIES;
    while (retry-- > 0){
        /* Retry a few times before we give up */
        if (SPISendCommand(mramDev, framAddress.word, ADDRESS_BYTES+1, 0, 0,
                           (uint8_t *) data, length)){
            //printf("Read %x from addr %x in MRAM %d, requested addr=%x\n",*(uint32_t *)data,ourAddress.word,(int)mramDev,nvAddress);
            return true;
        }
        //          IHUSoftErrorData.SPIRetries++;
    }
    ReportError(SPIMramTimeout, TRUE, ReturnAddr,
                (int)__builtin_return_address(0));
    return false;
}

static void writeMRAMWord(SPIDevice dev, uint32_t addr, uint32_t val)
{
    /*
     * This just makes the MRAM size routine easier to read
     */
    ByteToWord mramAddr;
    uint32_t value = val;

    mramAddr.word = addr;
    mramAddr.byte[0] = FRAM_OP_WRITE;
    //printf("Write %x to addr %x\n",value,addr);
    SPISendCommand(dev, mramAddr.word, ADDRESS_BYTES+1, &value, 4, 0, 0);
}

static uint32_t readMRAMWord(SPIDevice dev, uint32_t addr)
{
    ByteToWord mramAddr;
    uint32_t value;

    mramAddr.word = addr;
    mramAddr.byte[0] = FRAM_OP_READ;
    SPISendCommand(dev, mramAddr.word, ADDRESS_BYTES+1, 0, 0, &value, 4);
    //printf("Read %x from addr %x\n",value,addr);
    return value;
}

int getMRAMSize(int mramNum)
{
    /*
     * This routine 'knows' about the FRAM and the on-board FLASH data sizes
     * although it knows about the FRAM via a define in fram.h.  For the flash,
     * it just returns the (known) size.  For FRAM, it checks to see that there
     * is a non-0 status return before returning the known FRAM size.
     */
    int i, sizeMultiple = 64*1024; //Assume smallest is 64K increasing in multiples of 64K.
    uint32_t saveVal0, saveValTest;
    uint32_t addr0Val = 0x1f2f3f97, testAddrVal = 0x994499ab;
    SPIDevice dev;

    if (mramNum >= PACSAT_MAX_MRAMS)
        return 0;

    dev = MRAM_Devices[mramNum];

    writeEnableMRAM(mramNum);

    /*
     * Setup for test by reading and saving address 0; then write a random value into
     * address 0 so we can see when it changes
     */
    addr0Val += xTaskGetTickCount(); //Get a sort of random value
    saveVal0 = readMRAMWord(dev, 0);
    writeMRAMWord(dev, 0, addr0Val);
    if (readMRAMWord(dev, 0) != addr0Val) {
        return 0;  //Chip does not work or exist
    }

    /*
     * Now write a different number every "sizeMultiple" bytes and see if it wraps.
     */

    for (i=1; i<32; i++) {
        uint32_t readBack;
        uint32_t testAddr = i * sizeMultiple;
        saveValTest = readMRAMWord(dev, testAddr);
        writeMRAMWord(dev, testAddr, testAddrVal);
        readBack = readMRAMWord(dev, 0);
        if (readBack != addr0Val) {
            //printf("Address 0 has changed to %x writing to addr %x\n",readBack,testAddr);
            break;
        } else {
            //printf("Address 0 is unchanged writing to addr %x\n",testAddr);
            writeMRAMWord(dev, testAddr, saveValTest); //Restore the test address value
        }
    }
    writeMRAMWord(dev, 0, saveVal0);  //Restore original address 0
    return i * sizeMultiple;

}

int initMRAM()
{
    // Initialize status register to 0 so there are no memory banks write protected
    int i, size;

    /* Already initialized. */
    if (numberOfMRAMs)
        return totalMRAMSize;

    for (i=0; i<PACSAT_MAX_MRAMS; i++) {
        size += MRAMSize[i] = getMRAMSize(MRAM_Devices[i]);
        if (MRAMSize[i] != 0)
            numberOfMRAMs++;
    }
    if (size > MRAM_PARTITION_0_SIZE) {
        totalMRAMSize = size;
        mramPartitionSize[0] = MRAM_PARTITION_0_SIZE;
        mramPartitionSize[1] = size - MRAM_PARTITION_0_SIZE;
        mramPartitionOffset[1] = MRAM_PARTITION_0_SIZE;
        printf("MRAMs initialized with %d mrams, %d total bytes\n",
               numberOfMRAMs, totalMRAMSize);
    } else {
        printf("MRAM init failure, not enough space\n");
    }
    return totalMRAMSize;
}

/* Test code below here. */

bool testMRAM(int size)
{
    // Size is bytes words
    int addr,i,startTime;
    bool ok=true;
    int valBase = xTaskGetTickCount();
    uint32_t write[32],read[32];
    printf("Testing with read/write size =");
    switch (size){
    case 4:
    case 8:
    case 16:
    case 32:
    case 64:
    case 128:
        printf(" %d bytes",size);
        break;
    default:
        size=4;
        printf(" %d bytes (defaulted)",size);
    }
    printf(", random value base is 0x%x\n",valBase);
    startTime = getSeconds();
    for(i=0;i<(size/4);i++){
        write[i] = valBase+(i*4); // Put in the byte address of the word plus valbase.
    }
    for(addr=0;ok;addr+=size){ // Each loop is 1KByte
        ok=writeNV(&write,size,NVFileSystem,addr);
        if(!ok)break;
        if(addr % (1024*64) == 0){
            printf("%dKb written\n",addr/1024);
        }
        for(i=0;i<size/4;i++){
            write[i] += size; // Put in the address of the next set of words plus valbase.
        }

    }
    ok=true;
    for(i=0;i<(size/4);i++){
        write[i] = valBase+(i*4); // Put in the byte address of the word plus valbase.
    }
    for(addr=0;ok;addr+=size){ // Each loop is 1KByte
        ok=readNV(&read,size,NVFileSystem,addr);
        if(!ok)break;
        for(i=0;i<(size/4);i++){
            if(read[i] != write[i]){
                printf("At address %x, read %x, not %x\n",addr+i*4,read[i],write[i]);
            }
            write[i] += size; // Put in the address of the next set of words plus valbase.
        }
        if(addr % (1024*64) == 0){
            printf("%dKb read\n",addr/1024);
        }

    }
    printf("Time is %d seconds\n",getSeconds()-startTime);
    return ok;
}
