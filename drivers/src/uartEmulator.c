#if HET
#include <HET_EMU_SCI.h>
#include <pacsat.h>
#include "het.h"
#include <stdio.h>
#include <uartEmulator.h>
#include "hardwareConfig.h"
#include "gpioDriver.h"
/*
 * These routines are the APIs for the N2HET UART emulator.
 *
 * Notice that these routines are heavily tied to the N2HET code in drivers/src/HET_EMU_SCI.het.  Specifically,
 * they generally write into a data or control value for an N2HET instructions, which is then used as an
 * immediate value within the N2HET instruction.  So the instruction number (for example '2' in
 * hetRAM1->Instruction[2].Data refers to the data field of the third (0 based counting) instruction in the
 * N2HET RAM.
 *
 */
void HetUARTEnableNotification(hetBASE_t *regs){
    regs->INTENAS = (1 << pHET_SendOverINT_0); //This is the instruction that generates a Tx interrupt
    regs->INTENAS = (1 << pHET_DoneRec_0);     //This is the instruction that generates an Rx interrupt
}
void HetUARTSetBaudrate(hetRAMBASE_t *hetram, unsigned int baud){
    /*
     * Write the number of LR (Loop resolution periods) per bit into instruction 0
     */
#ifdef RTIHU_BOARD_V10
    const int LoopResolution=1600; // Het LR in nanoseconds
#else
    const int LoopResolution=1600;
#endif
    unsigned int loopsPerSecond = (1000000000/LoopResolution);
    unsigned int loopsPerBit = ((loopsPerSecond/baud)&0xfff)-1; /* -1 is fencepost*/
    unsigned int oldControl =     hetRAM1->Instruction[0].Control;
    unsigned int newControl = (oldControl & 0xfe000000) | loopsPerBit;
    unsigned int rxOldCtl = hetRAM1->Instruction[17].Control;
    unsigned int rxNewCtl = (rxOldCtl & 0xfe000000) | loopsPerBit;
    hetram->Instruction[0].Control = newControl;
    hetram->Instruction[17].Control = rxNewCtl;
}
bool HetUARTIsActive(hetRAMBASE_t *hetram){
    return hetram->Instruction[2].Data != 0;
}
/** @fn void HetUART1PutChar(unsigned char Data)
 *   @brief Sends a character of characters over HET emulated SCI
 *
 *   @param[in]  Data		The character to send
 */
void HetUARTPutChar(hetRAMBASE_t *hetram, char Data)
{
    unsigned int Tmp = Data;

    Tmp <<= 1;                                    // Shift in start bit (0)
    Tmp |= 0x00000200;                            // Add stop bit (1)
    while(hetRAM1->Instruction[2].Data != 0);
    hetRAM1->Instruction[4].Data = Tmp << 7;  // Load TX buffer
    hetRAM1->Instruction[2].Data =  10 << 7;  // Load bit count
}

/** @fn void HetUART1PutText(unsigned char *text)
 *   @brief Sends a string of characters over HET emulated SCI
 *
 *   @param[in]  text		The string of characters to send
 */
void HetUART1PutText(char *text)
{
    while(*text != 0)
    {
        HetUARTPutChar(hetRAM1, *text++);
    }
}

/** @fn unsigned HetUART1Printf(const char *_format, ...)
 *   @brief sends data to terminal (HET emulated SCI)
 *
 *   @param[in]  _format - string with format argument
 *   @return      length of sent string
 *
 *   Sends formated string to terminal on HET emulated SCI
 */

unsigned HetUART1Printf(const char *_format, ...)
{
    char str[128];
    int length = -1, k = 0;

    va_list argList;
    va_start( argList, _format );

    length = vsnprintf(str, sizeof(str), _format, argList);

    va_end( argList );

    if (length > 0)
    {
        for(k=0; k<length; k++)
        {
            HetUARTPutChar(hetRAM1, str[k]);
        }
    }

    return (unsigned)length;
}


/** @fn char HetUART1GetChar()
 *   @brief		Gets a character from the HET Emulated SCI Receive Buffer if one is available
 *
 *	@return		The character in the receive buffer, if one is available. If not, 0.
 */
char HetUART1GetChar(bool noWait)
{
    unsigned int HetFlag;
    HetFlag = hetREG1->FLG;
    if(noWait || (HetFlag & (1<<23)))
    {
        hetREG1->FLG = (1<<23); // clear this bit
        HetFlag = ((char)(hetRAM1->Instruction[25].Data));
        return HetFlag;
    }
    else
        return 0;
}

void HetUARTInit(void){
    // Set all the right registers for the HET emulated UART (but not ready for interrupts yet)
    hetREG1->INTENAC = 0xFFFFFFFF; // No interrupts enabled in HET yet
    hetREG1->FLG = 0xFFFFFFFF;
    GPIOSetPinDirection(hetPORT1, COM1_HET_Tx_Pin, true); // Set the Tx pin direction to out
    GPIOSetPinDirection(hetPORT1, COM1_HET_Rx_Pin, false); // And the Rx pin direction to in
    hetREG1->GCR = 0x01030001;  // Enable the UART emulator
}
#endif
