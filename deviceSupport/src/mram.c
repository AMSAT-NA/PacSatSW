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

const SPIDevice MRAM_Devices[PACSAT_MAX_MRAMS] =
    {MRAM0Dev, MRAM1Dev, MRAM2Dev, MRAM3Dev};
static uint32_t MRAMSize[PACSAT_MAX_MRAMS];
static int numberOfMRAMs = 0;
static int totalMRAMSize = 0;

SPIDevice GetMRAMAndAddress(uint32_t *addr)
{
    int MRAMNumber = 0;

    while (*addr >= MRAMSize[MRAMNumber]) {
        *addr-=MRAMSize[MRAMNumber];
        MRAMNumber++;
        if (MRAMNumber > numberOfMRAMs){
            return InvalidSPI;
        }
    }
    return MRAM_Devices[MRAMNumber]; //Assume MRAM values are in order
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

uint8_t ReadMRAMStatus(SPIDevice mram)
{
    /*
     * MRAM documentation says that if RDSR is executed immediately after READ, the results
     * will be wrong.  One solution they say is to read it twice.
     */
    ByteToWord command;
    uint8_t data;

    command.byte[0] = FRAM_OP_RDSR;
    SPISendCommand(mram, command.word, 1, 0, 0, &data, 1);
    SPISendCommand(mram, command.word, 1, 0, 0, &data, 1);
    return data;
}

bool MRAMWriteEnable(SPIDevice mramNum)
{
    bool stat;
    ByteToWord command;

    command.byte[0] = FRAM_OP_WREN;
    /* Write enable */
    stat = SPISendCommand(mramNum, command.word, 1, NULL, 0, NULL, 0);
    return stat;
}

void WriteMRAMStatus(SPIDevice mramNum, uint8_t status)
{
    ByteToWord command;

    command.byte[0] = FRAM_OP_WREN;
    SPISendCommand(mramNum,command.word,1,0,0,0,0);
    command.byte[0] = FRAM_OP_WRSR;
    command.byte[1] = status;
    SPISendCommand(mramNum,command.word,2,0,0,0,0);
}

int getSizeMRAM(void)
{
    if (numberOfMRAMs)
        return totalMRAMSize;
    return initMRAM();
}

void writeMRAMWord(SPIDevice dev, uint32_t addr, uint32_t val)
{
    /*
     * This just makes the MRAM size routine easier to read
     */
    ByteToWord mramAddr;
    uint32_t value = val;

    mramAddr.word = addr;
    mramAddr.byte[0] = FRAM_OP_WRITE;
    MRAMWriteEnable(dev);
    //printf("Write %x to addr %x\n",value,addr);
    SPISendCommand(dev, mramAddr.word, ADDRESS_BYTES+1, &value, 4, 0, 0);
}

uint32_t readMRAMWord(SPIDevice dev, uint32_t addr)
{
    ByteToWord mramAddr;
    uint32_t value;

    mramAddr.word = addr;
    mramAddr.byte[0] = FRAM_OP_READ;
    SPISendCommand(dev, mramAddr.word, ADDRESS_BYTES+1, 0, 0, &value, 4);
    //printf("Read %x from addr %x\n",value,addr);
    return value;
}

int getMRAMSize(SPIDevice dev)
{
    /*
     * This routine 'knows' about the FRAM and the on-board FLASH data sizes
     * although it knows about the FRAM via a define in fram.h.  For the flash,
     * it just returns the (known) size.  For FRAM, it checks to see that there
     * is a non-0 status return before returning the known FRAM size.
     */
    int i, sizeMultiple = 64*1024; //Assume smallest is 64K increasing in multiples of 64K.
    uint32_t saveVal0, saveValTest;
    uint32_t addr0Val=0x1f2f3f97, testAddrVal = 0x994499ab;

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

    for(i=1;i<32;i++){
        uint32_t readBack;
        uint32_t testAddr = i * sizeMultiple;
        saveValTest = readMRAMWord(dev, testAddr);
        writeMRAMWord(dev, testAddr, testAddrVal);
        readBack = readMRAMWord(dev, 0);
        if(readBack != addr0Val){
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
    int i;

    for (i=0; i<PACSAT_MAX_MRAMS; i++) {
        totalMRAMSize += MRAMSize[i] = getMRAMSize(MRAM_Devices[i]);
        if (MRAMSize[i] != 0)
            numberOfMRAMs++;
    }
    return totalMRAMSize;
}

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
        ok=writeNV(&write,size,ExternalMRAMData,addr);
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
        ok=readNV(&read,size,ExternalMRAMData,addr);
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
