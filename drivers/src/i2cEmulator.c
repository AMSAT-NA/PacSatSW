#include <i2cEmulator.h>
#include <pacsat.h>
#include "het.h"
#include "HET_EMU_I2C.h"
#include <stdio.h>
#include "hardwareConfig.h"
#include "gpioDriver.h"

/*
 * This module is adapted from TI's HET I2C emulator code by WB1FJ
 * Dec 26, 2019.  It is used to communicate with the HET microcode.
 * The actual driver that deals with interrupts, semaphores, and stuff
 * is in i2cDriver.c because it is mostly very similar to the i2c hardware
 * driver.  Note that the comments are a bit, uh, cryptic.  WB1FJ does
 * not fully understand how the microcode works either.
 */

void HetI2CInit(void){
    /*
     * This routine is sort of a hybrid between the AMSAT and TI stuff.  It sets the interrupts
     * and other HET registers per TI, but also sets the pin direction as we do with
     * other pins in AMSAT Golf software.  I've also added stopping the HET, waiting for it
     * to stop, clearing the first data word so it will continue waiting, setting the pins
     * both high and then restarting it.
     */

    // Set all the right registers for the HET emulated I2c
    I2c2_HET_Reg->GCR = 0x01030000; // Stop the HET program
    while(I2c2_HET_Reg->ADDR != 0){}; // Wait till it stops
    I2c2_HET_Reg->INTENAC = 0xFFFFFFFF; // No interrupts enabled in HET yet
    I2c2_HET_Reg->FLG = 0xFFFFFFFF; // Clear all interrupt flags
    GPIOSetPinDirection(I2c2_HET_Port, I2c2_HET_SCL_Pin, true); // Set both pin directions to out
    GPIOSetPinDirection(I2c2_HET_Port, I2c2_HET_SDA_Pin, true); // ...which really means both in and out
    gioSetBit(I2c2_HET_Port,I2c2_HET_SCL_Pin,1); // Make sure they are both high
    gioSetBit(I2c2_HET_Port,I2c2_HET_SDA_Pin,1);
    hetRAM2->Instruction[0].Data = 0;  // Make sure the Tx buffer is empty
    I2c2_HET_Reg->GCR = 0x01030001;  // Enable the N2HET running the I2c emulator
    I2c2_HET_Reg->INTENAS = 0xFFFFFFFF; // Ok, enable interrupts
}
void HetI2CStop(void){
    /*
     * This routine is sort of a hybrid between the AMSAT and TI stuff.  It sets the interrupts
     * and other HET registers per TI, but also sets the pin direction as we do with
     * other pins in AMSAT Golf software.  I've also added stopping the HET, waiting for it
     * to stop, clearing the first data word so it will continue waiting, setting the pins
     * both high and then restarting it.
     */

    // Set all the right registers for the HET emulated I2c
    I2c2_HET_Reg->GCR = 0x01030000; // Stop the HET program
    while(I2c2_HET_Reg->ADDR != 0){}; // Wait till it stops
    I2c2_HET_Reg->INTENAC = 0xFFFFFFFF; // No interrupts enabled in HET yet
    I2c2_HET_Reg->FLG = 0xFFFFFFFF; // Clear all interrupt flags
    I2c2_HET_Port->PDR |= 1 << I2c2_HET_SCL_Pin;  //Tri-state the pin if the output register is high
    I2c2_HET_Port->DSET = 1 << I2c2_HET_SCL_Pin;  //Set the output register high
    I2c2_HET_Port->PDR |= 1 << I2c2_HET_SDA_Pin;  //Tri-state the pin if the output register is high
    I2c2_HET_Port->DSET = 1 << I2c2_HET_SDA_Pin;  //Set the output register high

}

/*
 * All I2c transactions start by calling this routine to send an address.  You also set up
 * what it will do AFTER sending the address--read or write (RW), how many bytes (NumOfBytes) whether
 * it will do a transmit interrupt after sending the address, and whether it will send a stop
 * bit after the whole address.  There is no need to bother interrupting if the next thing that
 * will happen after this address is a receive, so you might as well wait for the Rx interrupt.
 */

void HetI2CPutAddr(char Addr, char RW, char NumOfBytes, char IntEna, char StopBit)
{
  unsigned int Tmp = Addr;

  Tmp <<= 23;                                    // Shift in start bit (0)
  Tmp |= (NumOfBytes<<7);
  Tmp |= (IntEna & 1) <<18;
  if(StopBit) Tmp |= 0x8000;
  if(RW) //read
	  Tmp |= 0x806B0000;                            // Add start bit, stopdata ready, bit (1)
  else //write
	  Tmp |= 0x802B0000;                            // Add start bit, stopdata ready, bit (1)
  while(hetRAM2->Instruction[0].Data != 0);
  hetRAM2->Instruction[0].Data = Tmp;  // Load TX buffer
}

/*
 * This routine gets called from the interrupt routine (it might work from the mainline since it loops)
 * to send a character of data.  In this case, you can specify whether there should be an interrupt
 * after the character is sent.
 */

void HetI2CPutData(char Data, char IntEna)
{
  unsigned int Tmp = Data;

  Tmp <<= 22;                                    // Shift in start bit (0)
  Tmp |= (IntEna & 1) <<18;
  Tmp |= 0x802A0000;                            // Add start bit, stopdata ready, bit (1)
  while(hetRAM2->Instruction[0].Data != 0);
  hetRAM2->Instruction[0].Data = Tmp;  // Load TX buffer
}

/*
 * This routine GETS a byte after it has been read into the HET buffer. Again,
 * this might work without interrupts, but we call it when we get a receive
 * interrupt.  All bytes received interrupt, so we call this after the Rx interrupt.
 */

bool HetI2CGetChar(char *retData)
{
        /*
         * For reasons unknown, the first bytes of 2 when reading the ADS7828 frequently (always?) generates
         * an interrupt but does not have 0x80 in the specified location.  And essentially always the
         * data is actually there without a loop.  So we put in a small loop just in case, but grab
         * the data anyway if the value never comes up with the 0x80 bit set.
         */
        uint32_t data=0;
	    int loop=2;
	    while(((data & 0x80) == 0) && (loop-- >= 0)){ // Wait till there is data there.
	        data=(hetRAM2->Instruction[pHET_CleanRecDat_1].Data);
	    }
	    *retData = (char)((data>>8) & 0xff);
		return true;
}
