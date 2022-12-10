/*
 * main.c
 *
 *  Created on: Feb 26, 2019
 *      Author: burns
 *  Updated with WD Code Oct 10, 2019 R.Gopstein
 */


#define MAIN_FUNCTION
/* Common headers */

#include <i2cEmulator.h>
#include <pacsat.h>
#include <stdarg.h>
#include <uartEmulator.h>
#include "sys_common.h"
#include "system.h"
#include "sys_core.h"
#include "gio.h"
#include "pinmux.h"
#include "spi.h"
#include "het.h"
#include "i2c.h"
#include "sci.h"
#include "can.h"
#include "rti.h"
#include "adc.h"
#include "FreeRTOS.h"
#include "os_task.h"

//Flight software headers
#include "TMS570Hardware.h"
#include "MET.h"
#include "ax5043_access.h"
#include "ax5043.h"
#include "serialDriver.h"
#include "TMS570Hardware.h"
#include "spiDriver.h"
#include "i2cDriver.h"
#include "gpioDriver.h"
#include "ConsoleTask.h"
#include "CommandTask.h"
#include "TelemetryRadio.h"
#include "RTISetup.h"
#include "nonvol.h"
#include "canID.h"
#include "CANSupport.h"
#include "Max31725Temp.h"
#include "GPIO9539.h"
#include "errors.h"
#include "nonvolManagement.h"
#include "consoleRoutines.h"


#define FLIPBITS(x) (               \
        ((x & 0x80) >> 7) | ((x & 0x01) << 7) |    \
        ((x & 0x40) >> 5) | ((x & 0x02) << 5) |    \
        ((x & 0x20) >> 3) | ((x & 0x04) << 3) |    \
        ((x & 0x10) >> 1) | ((x & 0x08) << 1))

//Forward routine
void ConsoleTask(void *pvParameters);

//Global Variables (most should not be done this way)
bool CANPrintTelemetry,CANPrintCoord,CANPrintCommands,CANPrintAny,CANPrintCount,CANPrintErrors,CANPrintEttus;

/*
 * Diagnostic payload pieces for each processor
 */

rt1Errors_t localErrorCollection;
int currentStandbyDiagIndex=0,currentLegacyDiagIndex=0;

//Module variables
static volatile bool i2cSCDWait=true,i2cRxWait=false;
static volatile uint32_t interruptBits = 0;

bool JustReleasedFromBooster; //Extern definition
bool AllTasksStarted = false,CoordinationMessageReceived = false,SimDoppler=false;
resetMemory_t tempPrintReset;
void startup(void)
{
    /*
     * Start of by doing a bunch of HalCoGen routine initializations.  In most cases, this still will require
     * Golf Driver inits after the OS starts, but in many cases, the IO will work before the OS starts.
     */
#ifndef UNDEFINE_BEFORE_FLIGHT
#error Be sure to enable self test in processor
#endif
    InitErrors();
    gioInit();
    muxInit();
    hetInit();
    sciInit();
    sciDisableNotification(sciREG,SCI_TX_INT | SCI_RX_INT); // No interrupts before we start the OS
    sciSetBaudrate(sciREG, COM2_BAUD);
    i2cInit();
    spiInit();
    adcInit();
    /*
     * A few things need to be started and initialized via HAL routines before we get the OS
     * and drivers running.  Here, SPI chip select gios require their direction to be set and
     * raised to high (not selected)
     */
    gioSetDirection(gioPORTB,6);
    gioSetBit(gioPORTB,1,1);
    gioSetBit(gioPORTB,2,1);
    gioSetDirection(spiPORT1,7); // Make chip select pins be output
    gioSetDirection(spiPORT3,7); //
    gioSetDirection(spiPORT5,7); //
    gioSetBit(spiPORT3,0,1);
    gioSetBit(spiPORT3,1,1);  //Set chip selects high just in case
    gioSetBit(spiPORT3,2,1);
    /*
     * RTI is used by FreeRTOS as its clock and also by the watchdog as its counter.
     * FreeRTOS uses counter 0, compare 0 for its interrupt.  Let's start the counter
     * here.  We'll start the compare interrupt when FreeRTOS wants it (in os_port.c)
     */
    rtiInit();
    rtiStartCounter(rtiCOUNTER_BLOCK0);
    /*
     * The two High-end Timers are used for the console (COM1) and for a secondary I2c (I2C2)
     */
    HetUARTInit();
#ifndef RTIHU_ALL_I2C1
    //HetI2CInit();
    HetI2CStop();
#endif

    /*
     * Many of the devices that are working now only via the HalCoGen routines still use interrupt...
     * they just don't use any of the OS features till later.
     */

    _enable_interrupt_();

#ifdef RTIHU_BOARD_V10
    /*
     * For the V10 board, we need to turn on the 5043 right away.  That's because V10 uses the 5043's clock
     * output and we need to set that clock speed to match what we want on the MCU.
     *
     * On top of everything else, the pins are different between V10 and later
     */
    gioSetBit(gioPORTA, 1, 1);
#endif

    HetUARTSetBaudrate(hetRAM1,COM1_BAUD);

    /* Serial port and LEDs */
    hetREG1->DIR=0x00000017; //We want them to start out as output, I suppose.
    hetREG1->DOUT=0x00000017;

//Probably not used for pacsat
    MyLocalCanID = RTIHU_Primary;
    PartnerLocalCanID = RTIHU_Secondary;

    xTaskCreate(ConsoleTask, "Console", CONSOLE_STACK_SIZE,
                NULL,CONSOLE_PRIORITY, NULL);

    StartWatchdogTimer();  //This doesn't actually run until the scheduler starts.  Run it all the time to reset the HW WD
#ifdef WATCHDOG_ENABLE    /* start up the watchdog - relies on CheckAndResetWatchdogs running faster than timeout */
    InitWatchdog();  // Starts the HW watchdog.  CheckAndResetWatchdogs needs to start up before the WD times out (300 mSec)

#endif

    /*
      * Here is the end of the first part of initialization.  Now we get the OS
      * running.  We will be creating a single task right now, and we will then
      * start the OS.  When that task runs, it will create the other required tasks, use the clock
      * to wait in orbit after first release from the LV, and open antennas as required.
      *
      * This scheme allows us to be sure that the tasks have pre-conditions met (like having
      * other tasks running or otherwise requiring the OS) for their initialization.
      */




    /* Start the tasks and timer running. */
    vTaskStartScheduler();
    // We won't be returning here.  Continue in Console Task below
    while (1); // Just in case, this will hit the watchdog

}
void ConsoleTask(void *pvParameters){
    bool haveWaited,umbilicalAttached; //todo: Fix charger when we get that line in V1.2
    //GPIOInit(WatchdogFeed,NO_TASK,NO_MESSAGE,None);
    ResetAllWatchdogs(); // This is started before MET and other tasks so just reporting in does not help

    /*
     * Now we have an OS going, so we call the Golf init routines, which use OS structures like
     * semaphores and queues.
     */
    SerialInitPort(COM1,COM1_BAUD, 10,10);//Max of 38400 for the moment
    SerialInitPort(COM2,COM2_BAUD,10,10);
    SPIInit(DCTDev);
    SPIInit(MRAM0Dev);
    SPIInit(MRAM1Dev);
    initNV(ExternalMRAMData); // Make sure MRAM is initialized
    initMET();
    I2cInit(I2C1);
    I2cInit(I2C2);
    GPIOEzInit(LED1);
    GPIOEzInit(LED2);


    /*
     * The rest of the code in the Console task that is in this module is just testing devices.
     * This is likely to disappear after we are comfortable with the stability of everything.
     */

    ResetAllWatchdogs(); // We waited a bit; better make sure the WDs are happy
    printID();

    /*
     * Time to wait for the post-release timer if we have to
     */
    haveWaited = ReadMRAMBoolState(StateInOrbit);
    umbilicalAttached = true;
    //todo:  Include this when the line is ready chargerAttached=GPIOIsOn(UmbilicalAttached);

    if(umbilicalAttached){
        // We are on the ground, so don't force the wait.  Just
        // set the have waited (AKA in orbit) flag and go on.
        if(!haveWaited){
            printf("Umbilical connected but InOrbit flag false.  Setting the flag, not waiting or deploying\n");
        }
        haveWaited = true;
        WriteMRAMBoolState(StateInOrbit,true); // The umbilical was connected so cancel the wait
    }
#if 0 //def UNDEFINE_BEFORE_FLIGHT
    if(!CheckMRAMVersionNumber()){
        printf("MRAM format has changed.  Skipping possible waits and antenna deploy\n");
        haveWaited = true;
    }
#endif
    //////////////////////////////////////// Deployables //////////////////////////////////////////////////

    /*
     * Time to release the antennas.
     */
    xTaskCreate(CANTask, "Can", CAN_STACK_SIZE,
                NULL,CAN_PRIORITY, NULL);

    /*
     * We create the coordination task first.  It will get us switched into either "InControl" or "AliveNotControlling".
     * When we are in one or the other, we init the appropriate GPIOs.
     */
   xTaskCreate(CommandTask, "Command", COMMAND_STACK_SIZE,
                NULL,COMMAND_PRIORITY, NULL);
    xTaskCreate(TelemetryRadioTask,"Radio",RADIO_STACK_SIZE,
                NULL,RADIO_PRIORITY,NULL);

    //AlertFlashingWait(CENTISECONDS(50),CENTISECONDS(10),CENTISECONDS(3));
    AllTasksStarted = true;
    StartStableCount();
    // Now head off to do the real work of the console task
    RealConsoleTask();
}
void vApplicationIdleHook(){
    ReportToWatchdog(IdleWD);
}
