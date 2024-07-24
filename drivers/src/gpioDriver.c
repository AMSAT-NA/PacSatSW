/*
 * gpio.c
 *
 *  Created on: Sep 16, 2012
 *      Author: Burns Fisher
 *
 *  This is the glue (interface) layer code to use a general purpose I/O device for simple
 *  single-bit input and output.  Copied liberally from examples that are part of
 *
    FreeRTOS V7.0.2 - Copyright (C) 2011 Real Time Engineers Ltd.
 */

/*HalCoGen headers*/
#include <pacsat.h>
#include "gio.h"
#include "spi.h"
#include "het.h"
#include "i2c.h"
#include "sci.h"

/* Golf headers*/
#include "interTaskNotify.h"
#include "hardwareConfig.h"
#include "gpioDriver.h"


/* FreeRTOS Headers */
#include "FreeRTOS.h"
#include "os_semphr.h"

/* Forward Routine Declarations */
void GPIOIntRoutine(Gpio_Use whichGPIO);


/*
 * This structure defines each GPIO that is in use.  A couple fields will need to be
 * added if there is more than one GPIO that generates interrupts (EXTIPortSource is
 * not "EXTI_None")
 */

typedef struct _GPIOInfo {
    gioPORT_t *GPIOPort;
    uint16_t PinNum;
    uint16_t NumBits;
    bool initialstateOn;
    bool DirectionIsOut;
    bool CanInterrupt;
    bool InterruptBothEdges;
    bool OpenCollector;
    bool initialStateTristate;

} GPIOInfo;
#define EXTI_None 0xff

static bool GPIOUsable[NumberOfGPIOs];

/*
 * Each structure below defines one of the GPIOs in use.
 */

#ifdef LAUNCHPAD_HARDWARE

static const GPIOInfo LED1Info = {
                                  GPIOLed1Port//GPIOLed3Port,
                                  ,GPIOLed1Pin
                                  ,1
                                  ,GPIO_OFF
                                  ,GPIO_OUT
                                  ,false,false // No interrupts
                                  ,false,false  // Open collector, not tristate

};
static const GPIOInfo LED2Info = {
                                  GPIOLed2Port
                                  ,GPIOLed2Pin
                                  ,1
                                  ,GPIO_OFF
                                  ,GPIO_OUT
                                  ,false,false // No interrupts
                                  ,false,false  // Open collector, not tristate

};


static const GPIOInfo CommandStrobeInfo = {
                                           GPIOCommandStrobePort
                                           ,GPIOCommandStrobePin
                                           ,1
                                           ,GPIO_UNUSED // Default off
                                           ,GPIO_IN
                                           ,true,false // Interrupts on one edge only
                                           ,false,false // Not Open collector nor tristate
};
static const GPIOInfo CommandBitsInfo = {
                                         GPIOCommandBit0Port
                                         ,GPIOCommandBit0Pin
                                         ,2 //Read 2 bits at a time
                                         ,GPIO_UNUSED // Default off
                                         ,GPIO_IN
                                         ,false,false // No interrupts
                                         ,false,false // Not Open collector nor tristate
};

static const GPIOInfo AX5043InterruptInfo = {
                                             GPIO_DCTInterruptPort
                                             ,GPIO_DCTInterruptPin
                                             ,1
                                             ,GPIO_UNUSED // Default off
                                             ,GPIO_IN
                                             ,true,false //Interrupts one one edge only
                                             ,false,false // Not Open collector nor tristate
};



/*
 * Use this array to index to the correct GPIOInfoStructure based on the GPIO
 * enum index.
 */

static const GPIOInfo *GPIOInfoStructures[NumberOfGPIOs] =
{
 &LED1Info,&LED2Info,&AX5043InterruptInfo,&CommandStrobeInfo,&CommandBitsInfo
};
#else

static const GPIOInfo LED1Info = {
                                  GPIOLed1Port//GPIOLed3Port,
                                  ,GPIOLed1Pin
                                  ,1
                                  ,GPIO_OFF
                                  ,GPIO_OUT
                                  ,false,false // No interrupts
                                  ,false,false  // Not open collector, not tristate

};
static const GPIOInfo LED2Info = {
                                  GPIOLed2Port
                                  ,GPIOLed2Pin
                                  ,1
                                  ,GPIO_OFF
                                  ,GPIO_OUT
                                  ,false,false // No interrupts
                                  ,false,false  // Not open collector, not tristate

};
static const GPIOInfo LED3Info = {
                                  GPIOLed3Port
                                  ,GPIOLed3Pin
                                  ,1
                                  ,GPIO_OFF
                                  ,GPIO_OUT
                                  ,false,false // No interrupts
                                  ,false,false  // Not open collector, not tristate

};


static const GPIOInfo CommandStrobeInfo = {
                                           GPIOCommandStrobePort
                                           ,GPIOCommandStrobePin
                                           ,1
                                           ,GPIO_UNUSED // Default off
                                           ,GPIO_IN
                                           ,true,false // Interrupts on one edge only
                                           ,false,false // Not Open collector nor tristate
};
static const GPIOInfo CommandBitsInfo = {
                                         GPIOCommandBit0Port
                                         ,GPIOCommandBit0Pin
                                         ,2 //Read 2 bits at a time
                                         ,GPIO_UNUSED // Default off
                                         ,GPIO_IN
                                         ,false,false // No interrupts
                                         ,false,false // Not Open collector nor tristate
};

static const GPIOInfo AX0InterruptInfo = {
                                             GPIO_Rx1DCTInterruptPort
                                             ,GPIO_Rx1DCTInterruptPin
                                             ,1
                                             ,GPIO_UNUSED // Default off
                                             ,GPIO_IN
                                             ,true,false //Interrupts one one edge only
                                             ,false,false // Not Open collector nor tristate
};
static const GPIOInfo AX1InterruptInfo = {
                                             GPIO_Rx2DCTInterruptPort
                                             ,GPIO_Rx2DCTInterruptPin
                                             ,1
                                             ,GPIO_UNUSED // Default off
                                             ,GPIO_IN
                                             ,true,false //Interrupts one one edge only
                                             ,false,false // Not Open collector nor tristate
};
static const GPIOInfo AX2InterruptInfo = {
                                             GPIO_Rx3DCTInterruptPort
                                             ,GPIO_Rx3DCTInterruptPin
                                             ,1
                                             ,GPIO_UNUSED // Default off
                                             ,GPIO_IN
                                             ,true,false //Interrupts one one edge only
                                             ,false,false // Not Open collector nor tristate
};
static const GPIOInfo AX3InterruptInfo = {
                                             GPIO_Rx4DCTInterruptPort
                                             ,GPIO_Rx4DCTInterruptPin
                                             ,1
                                             ,GPIO_UNUSED // Default off
                                             ,GPIO_IN
                                             ,true,false //Interrupts one one edge only
                                             ,false,false // Not Open collector nor tristate
};
static const GPIOInfo AX4InterruptInfo = {
                                             GPIO_TxDCTInterruptPort
                                             ,GPIO_TxDCTInterruptPin
                                             ,1
                                             ,GPIO_UNUSED // Default off
                                             ,GPIO_IN
                                             ,true,false //Interrupts one one edge only
                                             ,false,false // Not Open collector nor tristate
};
static const GPIOInfo SSPAPowerInfo = {
                                  GPIOsspaPowerPort
                                  ,GPIOsspaPowerPin
                                  ,1
                                  ,GPIO_ON
                                  ,GPIO_OUT
                                  ,false,false // No interrupts
                                  ,false,false  // Not open collector, not tristate

};
static const GPIOInfo Ax5043PowerInfo = {
                                  GPIOax5043PowerPort
                                  ,GPIOax5043PowerPin
                                  ,1
                                  ,GPIO_OFF
                                  ,GPIO_OUT
                                  ,false,false // No interrupts
                                  ,false,false  // Not open collector, not tristate

};



/*
 * Use this array to index to the correct GPIOInfoStructure based on the GPIO
 * enum index.
 */

static const GPIOInfo *GPIOInfoStructures[NumberOfGPIOs] =
{
 &LED1Info,&LED2Info,&LED3Info,&AX0InterruptInfo,&AX1InterruptInfo,&AX2InterruptInfo,&AX3InterruptInfo,
 &AX4InterruptInfo,&CommandStrobeInfo,&CommandBitsInfo,&SSPAPowerInfo,&Ax5043PowerInfo
};

#endif

static int GPIOInterruptLastIndex = 0;
static Gpio_Use GPIOInterruptList[NumberOfGPIOs]={None}; /*List of GPIOs that support interrupts*/
static IntertaskMessageType GPIOMessage[NumberOfGPIOs];
static DestinationTask GPIOMessageDestination[NumberOfGPIOs];
static Gpio_Use GPIOAuxGPIO1[NumberOfGPIOs];

bool GPIOEzInit(Gpio_Use whichGpio){
    // Save some typing, reading, and some code space for most init calls
    return GPIOInit(whichGpio,NO_TASK,NO_MESSAGE,None);
}
bool GPIOInit( Gpio_Use whichGpio,DestinationTask task, IntertaskMessageType msg,
               Gpio_Use auxDataGPIO1)
{
    /*
     * Note:  GPIOInitialize returns a void * (think of it as an opaque value)
     * which is in fact the semaphore that is being used for this GPIO.  It might
     * be useful later to help implement the equivalent of an OR.  The second argument
     * can be specified for input GPIOs with interrupts associated.  In this void HetSetPinDirection(hetBASE_t *regPtr,int pinNum,bool IsOut);
     * case,
     * when the current GPIO interrupt fires, "alsoRead" will also be read, and the
     * value returned by GPIOWait.
     */

    const GPIOInfo *thisGPIO = GPIOInfoStructures[whichGpio];
    int portIndex,i;

    // This whole mess below is to allow us to set the direction correctly.  The direction bits are write-only and
    // must be written for the entire port.  So we keep track of what directions the port has, and re-write each
    // time one of them is specified.

    if(thisGPIO->GPIOPort ==0)return false; //Todo: Make sure we have all the GPIOs set up (now missing I2c reset)
    portIndex = (thisGPIO->GPIOPort==gioPORTA)?0:(thisGPIO->GPIOPort==gioPORTB)?1:
           (thisGPIO->GPIOPort==hetPORT1)?2:(thisGPIO->GPIOPort==spiPORT1)?3:
           (thisGPIO->GPIOPort==spiPORT5)?4:5;
#ifdef UNDEFINE_BEFORE_FLIGHT
    if(portIndex == 5)
        return false; ///////////Need to add new ports above
    if(task!= NO_TASK){
        // If they are asking for an interrupt and either
        if(
                (portIndex > 1) || //...it is not a GIO port or...
                (thisGPIO->DirectionIsOut) //...it is an output port or...
                //We could check to see whether it is in the GIO interrupt list
        ) return false; // No can do.
    }
#endif
    /*
     * If there is more than one pin read together, init all of them.  Note that they MUST be
     * adjacent in the same port, and the 'thisGPIO' structure must have the data for the first one.
     * All data except the pin number must be the same for both, and they must be input only and only
     * the first one can interrupt
     */
    for(i=0;i<thisGPIO->NumBits;i++){
        int pin = thisGPIO->PinNum + i;
        GPIOSetPinDirection(thisGPIO->GPIOPort,pin,thisGPIO->DirectionIsOut);
    }
    if(thisGPIO->DirectionIsOut){

        /*
         * Here if this is an "out" GPIO, we set the initial state, including open collector-ness
         */


        uint16_t pinNum = thisGPIO->PinNum;
        gioPORT_t *thisPort = thisGPIO->GPIOPort;
            if(thisGPIO->OpenCollector || thisGPIO->initialStateTristate){
                thisPort->PDR |= 1 << pinNum;  //Tri-state the pin if the output register is high
            }
            if(thisGPIO->initialstateOn || thisGPIO->initialStateTristate){
                thisPort->DSET = (1<<pinNum);
            } else {
                thisPort->DCLR = (1<<pinNum);
            }
    } else {

        /* Here the direction is in so check for and set up interrupt possibilities */

        if(thisGPIO->CanInterrupt && (task != NO_TASK)){
            /*
             * Here we have a "real" GPIO port and we are a asking for a message to be sent when it changes.
             * That means we set up an interrupt.
             */
            GPIOMessage[whichGpio] = msg;
            GPIOMessageDestination[whichGpio] = task;
            GPIOAuxGPIO1[whichGpio] = auxDataGPIO1;
            /*
             * Set up a quick search list of GPIOs that can take interrupts for the interrupt
             * handler to use
             */
            GPIOInterruptList[GPIOInterruptLastIndex++] = whichGpio;
            GPIOInterruptList[GPIOInterruptLastIndex] = None;

            /*
             * Next we have to figure out whether this is an interrupt on both edges or not.  HalCoGen
             * can set up for which edge direction if it is only one, but if we want an interrupt on
             * both directions, we have to set the INTDET register, which HalCoGen can't currently do.
             * INTDET had 32 bits, 8 for each of GPIOA, GPIOB, GPIOC and GPIOD (our chip does not have
             * C and D).  So shift a bit to match the pin number of the GPIO and then over by 8 if it is
             * GPIOB.
             */

            if(thisGPIO->InterruptBothEdges){
                uint32_t mask = 1 << thisGPIO->PinNum;
                if(thisGPIO->GPIOPort == gioPORTB){
                    mask <<= 8;
                }
                /*
                 * Now finally set the bit in INTDET to say both directions.
                 */
                gioREG->INTDET |= mask;

            }
            gioEnableNotification(thisGPIO->GPIOPort, thisGPIO->PinNum);
        }
    }
    GPIOUsable[whichGpio] = true;
    return true;
}

void GPIOSetOn(Gpio_Use whichGpio)
{
    const GPIOInfo *thisGPIO = GPIOInfoStructures[whichGpio];
#ifdef DEBUG_BUILD
    if(thisGPIO->Mode != GPIO_Mode_OUT)ReportError(IllegalGPIOOutput,TRUE,PortNumber,whichGpio);
#endif
    if(GPIOUsable[whichGpio]){
        gioSetBit(thisGPIO->GPIOPort, thisGPIO->PinNum,1);
    }
 }
void GPIOSetOff(Gpio_Use whichGpio)
{
    const GPIOInfo *thisGPIO = GPIOInfoStructures[whichGpio];
#ifdef DEBUG_BUILD
    if(thisGPIO->Mode != GPIO_Mode_OUT)ReportError(IllegalGPIOOutput,TRUE,PortNumber,whichGpio);
#endif
    if(GPIOUsable[whichGpio]){
        gioSetBit(thisGPIO->GPIOPort, thisGPIO->PinNum,0);
    }
}

void GPIOToggle(Gpio_Use whichGpio)
{
    const GPIOInfo *thisGPIO = GPIOInfoStructures[whichGpio];
    if(GPIOUsable[whichGpio]){
        gioToggleBit(thisGPIO->GPIOPort,thisGPIO->PinNum);
    }
}
/*
 *
 */
bool GPIOIsOn(Gpio_Use whichGpio)
{
    const GPIOInfo *thisGPIO = GPIOInfoStructures[whichGpio];
#ifdef DEBUG_BUILD
    if(thisGPIO->Mode != GPIO_Mode_OUT)ReportError(IllegalGPIOOutput,TRUE,PortNumber,whichGpio);
#endif
    return gioGetBit(thisGPIO->GPIOPort,thisGPIO->PinNum) == 1;
}

uint16_t GPIORead(Gpio_Use whichGpio)
{
    //	extern bool IgnoreUmbilical;
    uint32_t readData;
    const GPIOInfo *thisGPIO = GPIOInfoStructures[whichGpio];

#ifdef DEBUG_BUILD
    if(thisGPIO->Mode != GPIO_Mode_IN)ReportError(IllegalGPIOInput,TRUE,PortNumber,whichGpio);
#endif

    readData = gioGetPort(thisGPIO->GPIOPort);
    readData = readData >> thisGPIO->PinNum;
    return readData & ~(0xffffffff << thisGPIO->NumBits);
}

void gioNotification(gioPORT_t *port, uint32 bit)
{
    int i;
    for(i=0;GPIOInterruptList[i]!=None;i++){
        //todo:  We could speed this up a bit by having separate lists for each port, or at least ordering them
        // and we could also arrange a separate table of interrupting GPIOs.
        Gpio_Use thisGPIO = GPIOInterruptList[i];
        if((port == GPIOInfoStructures[thisGPIO]->GPIOPort) && (bit == GPIOInfoStructures[thisGPIO]->PinNum)){
            GPIOIntRoutine(thisGPIO);
            break;
        }

    }
}

void GPIOIntRoutine(Gpio_Use whichGPIO)
{
    /*
     * Interrupt handler.  Clear the pending bit, and then, if requested read 1 or 2 extra
     * GPIOs.  Finally send the message requested in the init.  Note that reading a GPIO can
     * be done from an interrupt routine.  Care must be taken not to mess with GPIORead to prevent
     * this.
     */
    Intertask_Message message;
    message.MsgType = GPIOMessage[whichGPIO];
    if(GPIOAuxGPIO1[whichGPIO] != None){
        message.argument = GPIORead(GPIOAuxGPIO1[whichGPIO]);
    }
    NotifyInterTaskFromISR(GPIOMessageDestination[whichGPIO],&message);

    //EndInterruptRoutine();
}

///////////////////////////////////////////
//This is used for both HETUART and GPIO///
///////////////////////////////////////////

void GPIOSetPinDirection(gioPORT_t *regPtr,int pinNum,bool IsOut){
    static uint32_t portDirection[]={0,0,0,0,0,0,0};
    int portIndex;
    portIndex = regPtr==hetPORT1?0:regPtr==hetPORT2?1:regPtr==gioPORTA?2:regPtr==spiPORT1?3:
            regPtr==spiPORT3?4:regPtr==spiPORT5?5:6;
    portDirection[portIndex] |=
            ((IsOut) ? (1<<pinNum):0);
    regPtr->DIR = portDirection[portIndex];     // Set any pins that we know should be output to output
    regPtr->PULDIS |= portDirection[portIndex]; // All the output pins should have pull disabled
}

void GPIOSetTristate(Gpio_Use gpioNum){
    const GPIOInfo *thisGPIO = GPIOInfoStructures[gpioNum];
    gioPORT_t *thisPort = thisGPIO->GPIOPort;
    int pinNum = thisGPIO->PinNum;
    if(thisGPIO->DirectionIsOut){
        thisPort->PDR |= 1 << pinNum;  //Tri-state the pin if the output register is high
        thisPort->DSET = 1 << pinNum;  //Set the output register high
        GPIOUsable[gpioNum] = false;
    }
    return;
}
void GPIOSetPushPull(Gpio_Use gpioNum){
    const GPIOInfo *thisGPIO = GPIOInfoStructures[gpioNum];
    gioPORT_t *thisPort = thisGPIO->GPIOPort;
    int pinNum = thisGPIO->PinNum;
    uint32_t mask = 0xFFFFFFFF;
    if(thisGPIO->initialstateOn){
        thisPort->DSET = 1 << pinNum;
    } else {
        thisPort->DCLR = 1 << pinNum;  //Set the output register low
    }
    if(thisGPIO->OpenCollector){
        thisPort->PDR |= 1 << pinNum;  // Enable Open drain (same as tristate except it is marked usable)
    } else {
        mask = 0xFFFFFFFF ^ (1<<pinNum); // Set the bit we care about to 0; all the rest to 1
        thisPort->PDR &= mask;  //Switch off open drain on the pin
    }
    GPIOUsable[gpioNum] = true;
    return;
}
