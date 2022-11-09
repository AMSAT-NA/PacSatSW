/*
 * OneWire.c
 *
 *  Created on: Nov 15, 2020
 *      Author: bfisher
 */
#include <pacsat.h>
#include "rti.h"
#include "gpioDriver.h"

#define COUNTER_SPEED_USEC 10
#define defineDelay(name,usec) static int name=usec*COUNTER_SPEED_USEC;

defineDelay(A,6)
defineDelay(B,64)
defineDelay(C,60)
defineDelay(D,10)
defineDelay(E,9)
defineDelay(F,55)
defineDelay(G,0)
defineDelay(H,480)
defineDelay(I,70)
defineDelay(J,410)
#define PORTADDRESS OneWire
bool OWInit(){
    return GPIOInit(PORTADDRESS,NO_TASK,NO_MESSAGE,None);

}
static void outp(Gpio_Use gpioName,int value){
    if(value==0)GPIOSetOff(gpioName); // This is really pull down
    else GPIOSetOn(gpioName); // Since it is open collector, this is really 'release'
}
static int inp(Gpio_Use gpioName){
    return GPIOIsOn(gpioName)?1:0;
}

static tickDelay(int value){
    unsigned int curVal = rtiREG1->CNT[0].FRCx;
    unsigned int goalVal = curVal+value;
    if(goalVal < curVal){
        while(goalVal < curVal)
        {
                curVal = rtiREG1->CNT[0].FRCx;
        }; // Wait for counter to wrap
    }
    while(curVal < goalVal )
    {
        curVal = rtiREG1->CNT[0].FRCx;
    }; // Now wait for counter to catch up to goal
}


//---------------------------------------------------------------------------
// Generate a 1-Wire reset, return 1 if no presence detect was found,
// return 0 otherwise.
// (NOTE: Does not handle alarm presence from DS2404/DS1994)
//
int OWTouchReset(void)
{
    int result;
    tickDelay(G);
    outp(PORTADDRESS,0x00); // Drives DQ low
    tickDelay(H);
    outp(PORTADDRESS,0x01); // Releases the bus
    tickDelay(I);
    result = inp(PORTADDRESS) ^ 0x01; // Sample for presence pulse from slave
    tickDelay(J); // Complete the reset sequence recovery
    return result; // Return sample presence pulse result
}
//---------------------------------------------------------------------------
// Send a 1-Wire write bit. Provide 10us recovery time.
//
void OWWriteBit(int bit)
{
    if (bit)
    {
        // Write '1' bit
        outp(PORTADDRESS,0x00); // Drives DQ low
        tickDelay(A);
        outp(PORTADDRESS,0x01); // Releases the bus
        tickDelay(B); // Complete the time slot and 10us recovery
    }
    else
    {
        // Write '0' bit
        outp(PORTADDRESS,0x00); // Drives DQ low
        tickDelay(C);
        outp(PORTADDRESS,0x01); // Releases the bus
        tickDelay(D);
    }
}
//---------------------------------------------------------------------------
// Read a bit from the 1-Wire bus and return it. Provide 10us recovery time.
//
int OWReadBit(void)
{
    int result;
    outp(PORTADDRESS,0x00); // Drives DQ low
    tickDelay(A);
    outp(PORTADDRESS,0x01); // Releases the bus
    tickDelay(E);
    result = inp(PORTADDRESS) & 0x01; // Sample the bit value from the slave
    tickDelay(F); // Complete the time slot and 10us recovery
    return result;
}
//This is all for bit-wise manipulation of the 1-Wire bus. The above routines can be built upon to create
//byte-wise manipulator functions as seen in Example 3.
//Example 3. Derived 1-Wire Functions
//---------------------------------------------------------------------------
// Write 1-Wire data byte
//
void OWWriteByte(int data)
{
    int loop;
    // Loop to write each bit in the byte, LS-bit first
    for (loop = 0; loop < 8; loop++)
    {
        OWWriteBit(data & 0x01);
        // shift the data byte for the next bit
        data >>= 1;
    }
}
//---------------------------------------------------------------------------
// Read 1-Wire data byte and return it
//
int OWReadByte(void)
{
    int loop, result=0;
    for (loop = 0; loop < 8; loop++)
    {
        // shift the result to get it ready for the next bit
        result >>= 1;
        // if result is one, then set MS bit
        if (OWReadBit())
            result |= 0x80;
    }
    return result;
}
//---------------------------------------------------------------------------
// Write a 1-Wire data byte and return the sampled result.
//
int OWTouchByte(int data)
{
    int loop, result=0;
    for (loop = 0; loop < 8; loop++)
    {
        // shift the result to get it ready for the next bit
        result >>= 1;
        // If sending a '1' then read a bit else write a '0'
        if (data & 0x01)
        {
            if (OWReadBit())
                result |= 0x80;
        }
        else
            OWWriteBit(0);
        // shift the data byte for the next bit
        data >>= 1;
    }
    return result;
}
//---------------------------------------------------------------------------
// Write a block 1-Wire data bytes and return the sampled result in the same
// buffer.
//
void OWBlock(unsigned char *data, int data_len)
{
    int loop;
    for (loop = 0; loop < data_len; loop++)
    {
        data[loop] = OWTouchByte(data[loop]);
    }
}
