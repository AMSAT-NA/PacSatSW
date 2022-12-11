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

static uint32_t MRAMSize[NUMBER_OF_MRAMS];
static SPIDevice mramDev[NUMBER_OF_MRAMS]={MRAM0Dev,MRAM1Dev};

SPIDevice GetMRAMAndAddress(uint32_t *addr){
    int MRAMNumber=0;
    while(*addr >= MRAMSize[MRAMNumber]){
        *addr-=MRAMSize[MRAMNumber];
        MRAMNumber++;
        if(MRAMNumber > NUMBER_OF_MRAMS){
            return InvalidSPI;
        }
    }
    return mramDev[MRAMNumber]; //Assume MRAM values are in order

}

bool MRAMSleep(int mramNum){
    bool stat;
    ByteToWord command;
    command.byte[0] = FRAM_OP_SLEEP;
    stat = SPISendCommand(mramDev[mramNum],command.word,1, NULL,0,NULL,0);        /* Write enable */
    return stat;
}

bool MRAMWake(int mramNum){
    bool stat;
    ByteToWord command;
    command.byte[0] = MRAM_OP_WAKE;
    stat = SPISendCommand(mramDev[mramNum],command.word,1, NULL,0,NULL,0);        /* Write enable */
    return stat;
}


uint8_t ReadMRAMStatus(SPIDevice mram){
    /*
     * MRAM documentation says that if RDSR is executed immediately after READ, the results
     * will be wrong.  One solution they say is to read it twice.
     */
    ByteToWord command;
    uint8_t data;
    command.byte[0]=FRAM_OP_RDSR;
    SPISendCommand(mram,command.word,1,0,0,&data,1);
    SPISendCommand(mram,command.word,1,0,0,&data,1);
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
int getSizeNV(NVType x){
    /*
     * For this processor, the only NV memory is MRAM
     */
    if(x!=ExternalMRAMData)return 0;
    return initMRAM();
}
void writeMRAMWord(SPIDevice dev,uint32_t addr,uint32_t val){
    /*
     * This just makes the MRAM size routine easier to read
     */
    ByteToWord mramAddr;
    uint32_t value = val;
    mramAddr.word = addr;
    mramAddr.byte[0] = FRAM_OP_WRITE;
    MRAMWriteEnable(dev);
    //printf("Write %x to addr %x\n",value,addr);
    SPISendCommand(dev,mramAddr.word,ADDRESS_BYTES+1,&value,4,0,0);
}
uint32_t readMRAMWord(SPIDevice dev,uint32_t addr){
    ByteToWord mramAddr;
    uint32_t value;
    mramAddr.word = addr;
    mramAddr.byte[0] = FRAM_OP_READ;
    SPISendCommand(dev,mramAddr.word,ADDRESS_BYTES+1,0,0,&value,4);
    //printf("Read %x from addr %x\n",value,addr);
    return value;
}
int getMRAMSize(SPIDevice dev){
    /*
     * This routine 'knows' about the FRAM and the on-board FLASH data sizes
     * although it knows about the FRAM via a define in fram.h.  For the flash,
     * it just returns the (known) size.  For FRAM, it checks to see that there
     * is a non-0 status return before returning the known FRAM size.
     */
    int i,sizeMultiple=64*1024; //Assume smallest is 64K increasing in multiples of 64K.
    uint32_t saveVal0,saveValTest,addr0Val=0x1f2f3f97,testAddrVal = 0x994499ab;

    /*
     * Setup for test by reading and saving address 0; then write a random value into
     * address 0 so we can see when it changes
     */

    saveVal0 = readMRAMWord(dev,0);
    writeMRAMWord(dev,0,addr0Val);

    /*
     * Now write a different number every "sizeMultiple" bytes and see if it wraps.
     */

    for(i=1;i<32;i++){
        uint32_t readBack;
        uint32_t testAddr = i*sizeMultiple;
        saveValTest = readMRAMWord(dev,testAddr);
        writeMRAMWord(dev,testAddr,testAddrVal);
        readBack = readMRAMWord(dev,0);
        if(readBack != addr0Val){
            //printf("Address 0 has changed to %x writing to addr %x\n",readBack,testAddr);
            break;
        } else {
            //printf("Address 0 is unchanged writing to addr %x\n",testAddr);
            writeMRAMWord(dev,testAddr,saveValTest); //Restore the test address value
        }
    }
    writeMRAMWord(dev,0,saveVal0);  //Restore original address 0
    return i*sizeMultiple;

}


int initMRAM(){
    // Initialize status register to 0 so there are no memory banks write protected
    WriteMRAMStatus(MRAM0Dev,0);
    WriteMRAMStatus(MRAM1Dev,0);
    MRAMSize[0] = getMRAMSize(MRAM0Dev);
    MRAMSize[1] = getMRAMSize(MRAM1Dev);
    return MRAMSize[0]+MRAMSize[1];
}



