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
#include "MET.h"
#include "MRAMmap.h"
#include "nonvolManagement.h"

/*
 * MRAMAddressBytes tells us how many bytes are required to send an address.  Only very small (under 64KB)
 * MRAMs need only 2 bytes.  We will never fly with those, so we only do the weird set of checks if
 * UNDEFINE_BEFORE_FLIGHT is defined.  Otherwise, MRAMAddressBytes will be a constant, so the compiler should
 * remove any code for 2-byte addresses.
 */
#ifdef UNDEFINE_BEFORE_FLIGHT
static uint32_t MRAMAddressBytes=3;
#else
#define MRAMAddressBytes 3
#endif
static const SPIDevice MRAM_Devices[PACSAT_MAX_MRAMS] =
    {MRAM0Dev, MRAM1Dev, MRAM2Dev, MRAM3Dev};
static uint32_t MRAMSize[PACSAT_MAX_MRAMS];
static int numberOfMRAMs = 0;
static int totalMRAMSize = 0;

static int mramPartitionSize[MAX_MRAM_PARTITIONS] = { 0, 0, 0 };
static int mramPartitionOffset[MAX_MRAM_PARTITIONS] = { 0, 0, 0 };

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

int getMRAMPartitionSize(int partition)
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
    if (base < 0)
        return false;
    address += base;

    /*
     * This code knows about the commands for and has been tested with
     * an external RAMTRON F-RAM and an Eversource MRAM.
     */

    framAddress.word = address;
    mramNum = addressToMRAMNum(&framAddress.word);
    if (mramNum < 0) {
        return false;
    }
    writeEnableMRAM(mramNum);
    mramDev = MRAM_Devices[mramNum];

    writeCommand.byte[0] = FRAM_OP_WRITE;
    // The MRAM address is big endian, but so is the processor.
    // Note that MRAMAddressBytes will be a #define constant for the flight model, and thus the 2-bytes
    // addresses code will not be compiled.
    if (MRAMAddressBytes == 2){
        writeCommand.byte[1] = framAddress.byte[2]; //We would not have this problem with little endian
        writeCommand.byte[2] = framAddress.byte[3];
    } else {
        writeCommand.byte[1] = framAddress.byte[1];
        writeCommand.byte[2] = framAddress.byte[2];
        writeCommand.byte[3] = framAddress.byte[3];
    }
    //printf("Write %x to addr %x in MRAM %d, requested addr=%x\n",
    //       *(uint32_t *)data,framAddress.word,(int)mramDev,nvAddress);

    /* Now write */
    SPISendCommand(mramDev, writeCommand.word, MRAMAddressBytes+1,
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
    if (base < 0)
        return false;
    address += base;

    ourAddress.word = address;
    mramNum = addressToMRAMNum(&ourAddress.word);
    if (mramNum < 0) {
        return false;
    }
    mramDev = MRAM_Devices[mramNum];
    if(MRAMAddressBytes == 2){
        framAddress.byte[1] = ourAddress.byte[2];  // Address is big-endian.
        framAddress.byte[2] = ourAddress.byte[3];  // Address is big-endian.
    } else {
        framAddress.byte[1] = ourAddress.byte[1];  // Address is big-endian.
        framAddress.byte[2] = ourAddress.byte[2];  // Address is big-endian.
        framAddress.byte[3] = ourAddress.byte[3];  // Address is big-endian.
    }
    framAddress.byte[0] = FRAM_OP_READ;

    retry = SPI_MRAM_RETRIES;
    while (retry-- > 0){
        /* Retry a few times before we give up */
        if (SPISendCommand(mramDev, framAddress.word, MRAMAddressBytes+1, 0, 0,
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

static void writeMRAMWord(SPIDevice dev, uint32_t inAddr, uint32_t val)
{
    /*
     * This just makes the MRAM size routine easier to read
     */
    ByteToWord mramAddr;
    ByteToWord addr;
    uint32_t value = val;
    addr.word = inAddr;
    if( MRAMAddressBytes == 2){
        mramAddr.byte[1] = addr.byte[2];
        mramAddr.byte[2] = addr.byte[3];
    }else{
        mramAddr.byte[1] = addr.byte[1];
        mramAddr.byte[2] = addr.byte[2];
        mramAddr.byte[3] = addr.byte[3];
    }
    mramAddr.byte[0] = FRAM_OP_WRITE;
    //printf("Write %x to addr %x\n",value,addr);
    SPISendCommand(dev, mramAddr.word, MRAMAddressBytes+1, &value, 4, 0, 0);
}

static uint32_t readMRAMWord(SPIDevice dev, uint32_t inAddr)
{
    ByteToWord mramAddr,addr;
    uint32_t value;
    addr.word=inAddr;
    if(MRAMAddressBytes == 2){
        mramAddr.byte[1] = addr.byte[2];
        mramAddr.byte[2] = addr.byte[3];
    } else {
        mramAddr.byte[1] = addr.byte[1];
        mramAddr.byte[2] = addr.byte[2];
        mramAddr.byte[3] = addr.byte[3];
    }
    mramAddr.byte[0] = FRAM_OP_READ;
    SPISendCommand(dev, mramAddr.word, MRAMAddressBytes+1, 0, 0, &value, 4);
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
    int i, sizeMultiple = 16*1024; //Assume smallest is 16K increasing in multiples of 16K.
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
    saveValTest = readMRAMWord(dev,0);
    if (saveValTest != addr0Val) {
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
#ifdef UNDEFINE_BEFORE_FLIGHT
static int findMRAMAddressSize(){
    /*
     * This routine calculates by writing values in weird places (see comments) whether we have a 2
     * byte address scheme or 3.  FOr the flight model, it will be 3 so we will skip this code.
     * Here we assume that if there are any MRAMs, they are all use the same size address and that
     * one of them is MRAM0.
     */
    ByteToWord mramAddr;
    uint8_t size=0;
    uint32_t value[2];
    mramAddr.word=0; //Set all 4 bytes to 0
    mramAddr.byte[0] = FRAM_OP_WRITE;  // Set first byte to READ
    mramAddr.byte[3] = 2;
    value[0] = 0x12345678;
    value[1] = 0xfedc;
    SPISendCommand(MRAM0Dev, mramAddr.word, 4,
                   value, 4,
                   0,0);
    /*
     * If this is a two byte address, then the 3rd address byte (which has the value 2) will be taken as data and
     * written into address 0.  So the first 4 bytes in the MRAM will be 02123456.  For a 3 byte address, the 2
     * will be taken as an address, so nothing is writen into the first two bytes.  Thus the start of the MRAM will
     * be xxxx1234
     */
    mramAddr.word = 0;
    value[0]=value[1]=0;
    mramAddr.byte[0] = FRAM_OP_READ;
    SPISendCommand(MRAM0Dev, mramAddr.word, 4, NULL,0, &value[0], (uint16_t) 8);
/////////////////////////////////////////////
    printf("Value = %x\n",value);
    if((value[0] & 0xffff)== 0x1234){
        // As we see above, only a 3-byte address will have this number in the least significant 2 bytes.
        // (Remember it is big endian)
        size = 3;
        writeMRAMStatus(0,MRAM_STATUS_ADDR_3); // Remember the size in unused bits in the status register
    }
    else {
        size = 2;
        writeMRAMStatus(0,MRAM_STATUS_ADDR_2); // Remember the size in unused bits in the sr.
    }
    return size;
}
#endif
int getMRAMAddressSize(){
#ifdef UNDEFINE_BEFORE_FLIGHT
    uint8_t stat = readMRAMStatus(0);
    // If it has been initialized, the address size is in the status register
    // Otherwise, we have to figure out the address size and put it there.
    stat = stat & MRAM_STATUS_ADDR_MASK;
    if(stat == MRAM_STATUS_ADDR_2){
        return 2;
    } else if (stat == MRAM_STATUS_ADDR_3){
        return 3;
    } else {
        return findMRAMAddressSize();
    }
#else
    return MRAMAddressBytes; // This will be a constant if UNDEFINE_BEFORE_FLIGHT is undefined
#endif
}
int initMRAM()
{
    // Initialize status register to 0 so there are no memory banks write protected
    int i, size;

    /* Already initialized. */
    if (numberOfMRAMs)
    return totalMRAMSize;
    writeEnableMRAM(0);
#ifdef UNDEFINE_BEFORE_FLIGHT
    MRAMAddressBytes = getMRAMAddressSize();
#endif
    size=0;
    for (i=0; i<PACSAT_MAX_MRAMS; i++) {
        size += MRAMSize[i] = getMRAMSize(MRAM_Devices[i]);
        if (MRAMSize[i] != 0)
            numberOfMRAMs++;
    }
    if (size > MRAM_PARTITION_0_SIZE) {
        totalMRAMSize = size;
        mramPartitionSize[0] = sizeof(MRAMmap_t);
        mramPartitionSize[1] = size - MRAM_PARTITION_0_SIZE;
        mramPartitionSize[2] = totalMRAMSize;
        mramPartitionOffset[1] = MRAM_PARTITION_0_SIZE;
    }
    if(!CheckMRAMVersionNumber()){
        printf("\n\n\n\n*****MRAM layout has changed.  You must issue clear mram******\n\n\n\n");
    }
    return totalMRAMSize;
}

/* Test code below here. */

bool testMRAM(int size)
{
    // Size is bytes words
    int addr,i,startTime;
    bool ok=true,testOk=true;
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
        if(addr % (1024*16) == 0){
            printf("Write starting at %dKB\n",addr/1024);
        }
        ok=writeNV(&write,size,NVEntireMRAM,addr);
        if(!ok)break;
        ok&=readNV(&read,size,NVEntireMRAM,0);
        if(read[0] != valBase){
            printf("Addr 0 got overwritten\n");
            testOk=false;
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
        if(addr % (1024*16) == 0){
            printf("Read starting at %dKB\n",addr/1024);
        }
       ok=readNV(&read,size,NVEntireMRAM,addr);
        if(!ok)break;
        for(i=0;i<(size/4);i++){
            if(read[i] != write[i]){
                printf("At address %x, read %x, not %x\n",addr+i*4,read[i],write[i]);
                testOk=false;
            }
            write[i] += size; // Put in the address of the next set of words plus valbase.
        }
    }
    if(testOk){
        printf("Test is ok!  ");
    } else {
        printf("Test failed!  ");
    }
    printf("Time is %d seconds\n",getSeconds()-startTime);
    return ok;
}
