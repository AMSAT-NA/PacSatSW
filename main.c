/*
 * main.c
 *
 *  Created on: Feb 26, 2019
 *      Author: burns
 *  Updated with WD Code Oct 10, 2019 R.Gopstein
 */


#define MAIN_FUNCTION
/* Common headers */

#include <pacsat.h>
#include <stdarg.h>
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
#include "mram.h"
#include "ADS7828.h"

//Flight software headers
#include "TMS570Hardware.h"
#include "MET.h"
#include "ax5043_access.h"
#include "ax5043.h"
#include "serialDriver.h"
#include "TMS570Hardware.h"
#include "spiDriver.h"
#include "i2cDriver.h"
#include "I2cPoll.h"
#include "gpioDriver.h"
#include "Max31331Rtc.h"
#include "ConsoleTask.h"
#include "CommandTask.h"
#include "TxTask.h"
#include "RxTask.h"
#include "Ax25Task.h"
#include "PbTask.h"
#include "UplinkTask.h"
#include "TelemAndControlTask.h"
#include "pacsat_dir.h"
#include "hardwareConfig.h"
//#include "TelemetryRadio.h"
#include "RTISetup.h"
#include "nonvol.h"
#include "canID.h"
#include "Max31725Temp.h"
#include "GPIO9539.h"
#include "errors.h"
#include "nonvolManagement.h"
#include "consoleRoutines.h"
#include "redposix.h"
#include "reg_sci.h"

/* Only needed for test routines */
#include "ax25_util.h"


#define FLIPBITS(x) (               \
        ((x & 0x80) >> 7) | ((x & 0x01) << 7) |    \
        ((x & 0x40) >> 5) | ((x & 0x02) << 5) |    \
        ((x & 0x20) >> 3) | ((x & 0x04) << 3) |    \
        ((x & 0x10) >> 1) | ((x & 0x08) << 1))

//Forward routine
void ConsoleTask(void *pvParameters);

//Global Variables Definitions (these are declared in pacsat.h or another header file where they are required)
QueueHandle_t xRxPacketQueue; /* RTOS Queue for packets received by the RX (AX25 Data Link) */
QueueHandle_t xRxEventQueue; /* RTOS Queue for events received by the AX25 Data Link */
QueueHandle_t xPbPacketQueue; /* RTOS Queue for packets received and sent to the PB task */
QueueHandle_t xUplinkEventQueue; /* RTOS Queue for events received and sent to the UPLINK task */
QueueHandle_t xTxPacketQueue; /* RTOS Queue for packets sent to the TX */
QueueHandle_t xIFrameQueue[NUM_OF_RX_CHANNELS]; /* RTOS Queues for Data IFrames sent from Uplink to AX25 Data Link */
bool rate_9600; /* The rate for the AX25 link.  Loaded from MRAM.  */
bool CANPrintTelemetry,CANPrintCoord,CANPrintCommands,CANPrintAny,CANPrintCount,CANPrintErrors,CANPrintEttus,
monitorPackets;



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
    sciInit();
    sciDisableNotification(sciREG,SCI_TX_INT | SCI_RX_INT); // No interrupts before we start the OS
    sciSetBaudrate(sciREG, COM2_BAUD);

    //GPIO Easy Inits can be done before the OS is started
#ifdef LAUNCHPAD_HARDWARE

#else
    GPIOEzInit(LED1);
    GPIOEzInit(LED2);
    GPIOEzInit(LED3);
    GPIOSetOff(LED1);
    GPIOSetOff(LED2);
    GPIOSetOff(LED3);
    GPIOEzInit(SSPAPower);
    GPIOEzInit(AX5043Power);

    GPIOToggle(LED1);
#endif
    sciSend(sciREG,38,"Starting a test on the SCI register\r\n");

    i2cInit();
    spiInit();
    adcInit();
    /*
     * A few things need to be started and initialized via HAL routines before we get the OS
     * and drivers running.  Here, SPI chip select gios require their direction to be set and
     * raised to high (not selected)
     */
    //MRAM
#ifdef LAUNCHPAD_HARDWARE
    gioSetDirection(gioPORTB,6);
    gioSetBit(gioPORTB,1,1);
    gioSetBit(gioPORTB,2,1);
    gioSetDirection(spiPORT1,7); // Make chip select pins be output
    gioSetDirection(spiPORT3,7); //
    gioSetDirection(spiPORT5,7); //
    gioSetBit(spiPORT3,0,1);

    gioSetBit(spiPORT3,1,1);  //Set chip selects high just in case
    gioSetBit(spiPORT3,2,1);

#else
    gioSetDirection(spiPORT1,1U<<0 || 1U<<2); // Make chip select pins be output
    gioSetDirection(spiPORT3,1); // Make chip select pins be output
    gioSetDirection(spiPORT5,1); //
    gioSetBit(spiPORT1,0,1);
    gioSetBit(spiPORT1,2,1);  //Set chip selects high just in case
    gioSetBit(spiPORT3,0,1);
    gioSetBit(spiPORT5,0,1);  //Set chip selects high just in case
    //
    // Need to do the 5043 bits too.  Especially doing these here since we don't use
    // HET2 and thus have no initHET routine from HCG.  Why does "set direction" what the entire
    // register, while SetBit takes the bit number?
    //
    gioSetDirection(SPI_DCT_Select_Port,
                    (1U<<SPI_Rx1DCT_Select_Pin)  |
                    (1U<<SPI_Rx2DCT_Select_Pin)  |
                    (1U<<SPI_Rx3DCT_Select_Pin)  |
                    (1U<<SPI_Rx4DCT_Select_Pin)  |
                    (1U<<SPI_TxDCT_Select_Pin));

    gioSetBit(SPI_DCT_Select_Port,SPI_Rx1DCT_Select_Pin,1); // Make chip select pins be high
    gioSetBit(SPI_DCT_Select_Port,SPI_Rx2DCT_Select_Pin,1); // Make chip select pins be high
    gioSetBit(SPI_DCT_Select_Port,SPI_Rx3DCT_Select_Pin,1); // Make chip select pins be high
    gioSetBit(SPI_DCT_Select_Port,SPI_Rx4DCT_Select_Pin,1); // Make chip select pins be high
    gioSetBit(SPI_DCT_Select_Port,SPI_TxDCT_Select_Pin,1); // Make chip select pins be high
#endif
    /*
     * RTI is used by FreeRTOS as its clock and also by the watchdog as its counter.
     * FreeRTOS uses counter 0, compare 0 for its interrupt.  Let's start the counter
     * here.  We'll start the compare interrupt when FreeRTOS wants it (in os_port.c)
     */
    rtiInit();
    rtiStartCounter(rtiCOUNTER_BLOCK0);

    /*
     * Many of the devices that are working now only via the HalCoGen routines still use interrupt...
     * they just don't use any of the OS features till later.
     */

    _enable_interrupt_();
#ifdef LAUNCHPAD_HARDWARE
#ifdef HET
    HetUARTSetBaudrate(hetRAM1,COM1_BAUD);
#endif
    /* Serial port and LEDs */
    hetREG1->DIR=0x00000017; //We want them to start out as output, I suppose.
    hetREG1->DOUT=0x00000017;
#endif

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
    debug_print("Starting console task\n");
    /*
     * Now we have an OS going, so we call the init routines, which use OS structures like
     * semaphores and queues.
     */

    SerialInitPort(COM1,COM1_BAUD, 10,10);//Max of 38400 for the moment
    SerialInitPort(COM2,COM2_BAUD,10,10);

    // Initialize the SPI driver for our SPI devices

#ifdef LAUNCHPAD_HARDWARE
    //    SPIInit(TxDCTDev); // This is the transmitter on UHF
    //    SPIInit(Rx1DCTDev); // This is the receiver on VHF on the Pacsat Booster Board

    SPIInit(DCTDev0); // This is the receiver on VHF on the Pacsat Booster Board
    SPIInit(DCTDev1); // This is the transmitter on UHF
#else
    SPIInit(Rx1DCTDev); // This is the receiver on VHF on the Pacsat Booster Board
    SPIInit(Rx2DCTDev); // This is the receiver on VHF on the Pacsat Booster Board
    SPIInit(Rx3DCTDev); // This is the receiver on VHF on the Pacsat Booster Board
    SPIInit(Rx4DCTDev); // This is the receiver on VHF on the Pacsat Booster Board
    SPIInit(TxDCTDev); // This is the transmitter on UHF
#endif
    SPIInit(MRAM0Dev);
    SPIInit(MRAM1Dev);
    SPIInit(MRAM2Dev);
    SPIInit(MRAM3Dev);
    if (initMRAM(false) == -1) {
        debug_print("MRAM NOT SETUP - Full re-init\n");
        initMRAM(true); //Init this thing from scratch (address size, data size, partitions etc)
        IHUInitSaved(); //Init stuff that we won't want to change on reboot
        SetupMRAM();    //Init stuff that do change (epoch number etc)
#ifdef DEBUG
        WriteMRAMBoolState(StateInOrbit,true); // Don't get confused by in orbit state!
#endif
    }
    initMET();
    I2cInit(I2C1);

#ifdef LAUNCHPAD_HARDWARE
    I2cInit(I2C2);
    GPIOEzInit(LED1);
    GPIOEzInit(LED2);
    GPIOInit(DCTInterrupt,ToRxTask,Rx0DCTInterruptMsg,None);
    //    GPIOInit(Rx0DCTInterrupt,ToRxTask,Rx0DCTInterruptMsg,None);
    //    GPIOInit(TxDCTInterrupt,ToTxTask,TxDCTInterruptMsg,None);
#else
    GPIOInit(Rx0DCTInterrupt,ToRxTask,Rx0DCTInterruptMsg,None);
    GPIOInit(Rx1DCTInterrupt,ToRxTask,Rx1DCTInterruptMsg,None);
    GPIOInit(Rx2DCTInterrupt,ToRxTask,Rx2DCTInterruptMsg,None);
    GPIOInit(Rx3DCTInterrupt,ToRxTask,Rx3DCTInterruptMsg,None);
    GPIOInit(TxDCTInterrupt,ToTxTask,TxDCTInterruptMsg,None);
#endif
    /* Poll the I2C devices to see which are working.
     * This also calls the init routine for the temperature device */
    I2CDevicePoll();

    /*
     * The rest of the code in the Console task that is in this module is just testing devices.
     * This is likely to disappear after we are comfortable with the stability of everything.
     */

    ResetAllWatchdogs(); // We waited a bit; better make sure the WDs are happy
    printID();

    if (red_init() == -1) {
        printf("Unable to initialize filesystem: %s\n",
               red_strerror(red_errno));
    } else {
        if (red_mount("/") == -1) {
            if (red_errno == RED_EIO) {
                printf("Filesystem mount failed due to corruption, reformatting\n");
                if (red_format("/") == -1) {
                    printf("Unable to format filesystem: %s\n",
                           red_strerror(red_errno));
                } if (red_mount("/") == -1) {
                    printf("Mount after format failed, filesystem broken: %s\n",
                           red_strerror(red_errno));
                }
            } else {
                printf("Unable to mount filesystem: %s\n",
                       red_strerror(red_errno));
            }
        }
    }

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


#if 1
    /* Load the directory from MRAM and perform some integrity checks */
    int32_t ret = dir_check_folders();
    if (ret == -1) {
        debug_print("ERROR: Could not create the needed folders in MRAM\n");
        // TODO - bad or fatal - need to handle or log this error
    }
    int rc = dir_load();
    if (rc != TRUE) {
        debug_print("ERROR: Could not load the directory from MRAM\n");
        // TODO - bad or fatal - need to handle or log this error
    }

    xTaskCreate(CommandTask, "Command", COMMAND_STACK_SIZE,
                NULL,COMMAND_PRIORITY, NULL);
    xTaskCreate(RxTask,"RxTask",RX_STACK_SIZE, NULL, RX_PRIORITY,NULL);
    xTaskCreate(Ax25Task,"Ax25Task",AX25_STACK_SIZE, NULL, AX25_PRIORITY,NULL);
    xTaskCreate(PbTask,"PbTask",PB_STACK_SIZE, NULL, PB_PRIORITY,NULL);
    xTaskCreate(UplinkTask,"UplinkTask",UPLINK_STACK_SIZE, NULL, UPLINK_PRIORITY,NULL);
    xTaskCreate(TxTask,"TxTask",RADIO_STACK_SIZE, NULL,TX_PRIORITY,NULL);
    xTaskCreate(TelemAndControlTask,"Telem and Control Task",TELEMETRY_STACK_SIZE, NULL,TELEMETRY_PRIORITY,NULL);
#endif
    debug_print("Free heap size after tasks launched: %d\n",xPortGetFreeHeapSize());

    //AlertFlashingWait(CENTISECONDS(50),CENTISECONDS(10),CENTISECONDS(3));
    AllTasksStarted = true;
    StartStableCount();

    bool rtc = InitRtc31331();
    if (rtc == FALSE) {
        debug_print("*** NO 31331 RTC: SET THE UNIX TIME BEFORE UPLOADING ANY TEST FILES ***\n");
    } else {
        debug_print("31331 RTC detected\n");
        uint32_t utime = 0;
        rtc = GetRtcTime31331(&utime);
        setUnixTime(utime);
    }

    // Now head off to do the real work of the console task
    RealConsoleTask();
}
void vApplicationIdleHook(){
    ReportToWatchdog(IdleWD);
}
#ifdef DEBUG
void vApplicationMallocFailedHook(void){
    printf("Malloc Failed.  Heap is too small?  Current heap available is 0x%x\n",
           xPortGetFreeHeapSize());
    while(1){
        taskYIELD();
    }
}
//vApplicationStackOverflowHook( ( TaskHandle_t ) pxCurrentTCB, pxCurrentTCB->pcTaskName );
vApplicationStackOverflowHook(pxCurrentTCB,taskName ){
    printf("Stack overflow in task %s\n",taskName);
    while(1){
        taskYIELD();
    }

}

#endif
/* This is used to null out debug_print and debug trace statements when DEBUG is off */
void NullPrint(char * format,...){
};
