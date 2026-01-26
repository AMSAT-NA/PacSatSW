/*
 * consoleRoutines.c
 *
 *  Created on: Sep 21, 2020
 *      Author: bfisher
 */
#include <pacsat.h>
#include "stdarg.h"
#include "stdlib.h"
#include "rti.h"
#include "consoleRoutines.h" // My own
#include "downlink.h"
#include "serialDriver.h"
#include "nonvolManagement.h"
#include "nonvol.h"
#include "ADS7828.h"
#include "errors.h"
#include "LinearInterp.h"
#include "inet.h"
#include "adc.h"
#include "I2cPoll.h"


extern rt1Errors_t localErrorCollection;

extern char *ErrMsg[], *LIHUErrMsg[],*TaskNames[];

static char *nextNum;

static const char *getTaskName(int task)
{
    return TaskNames[task];
}

void print8BitTemp(uint8_t temp8)
{
    int16_t temp = temp8;
    printf("%d (%3d.%1d degrees C)", temp8,
           (temp >> 1) - 20, (temp & 1) * 5);
}

void print8BitVolts(uint8_t volt8,uint16_t maxV)
{
    // Specify the voltage that 255 is equivalent too as max*100.
    // That is, for a max of 13.3V, specify 1330.
    int16_t volt = volt8, interpVal;

    interpVal = LinearInterpolate(volt, 255, maxV);
    printf("%3d (%2d.%1d volts) ", volt8,
           (interpVal / 100), ((interpVal % 100) + 5) / 10);
}

uint16_t parseNumber(char *afterCommand)
{
    return (uint16_t) strtol(afterCommand, &nextNum, 0);
}

uint16_t parseNextNumber(void)
{
    char *string;

    string = nextNum;
    return (uint16_t) strtol(string, &nextNum, 0);
}

uint32_t parseNumber32(char *afterCommand)
{
    return (uint32_t) strtoul(afterCommand, &nextNum, 0);
}

uint32_t parseNextNumber32(void)
{
    char *string;

    string = nextNum;
    return (uint32_t) strtoul(string, &nextNum, 0);
}

void skip_command_spaces(char **str)
{
    char *s = *str;

    while (*s && *s == ' ')
        s++;
    *str = s;
}

void receiveLine(COM_NUM ioCom, char *commandString, char prompt, bool echo)
{
    /*
     * Wait until something arrives in the queue - this routine will
     * block indefinitely provided INCLUDE_vTaskSuspend is set to 1 in
     * FreeRTOSConfig.h.
     */
    static char prevCommandString[COM_STRING_SIZE], prevSize; /* For up-arrow */
    char receivedChar;
    int charNum = 0;
    // Back overwrite space, back
    const char deleteString[4] = { '\b', ' ', '\b', 0 };
    int escSeq=0;

    if (!CheckMRAMVersionNumber()) {
        printf("\n ***MRAM format has changed\n"
               "***Command 'preflight init' or 'init mram' or 'init new proc' required!\n");
    }

    if (echo)
        printf("");
    SerialPutChar(ioCom, prompt, 0);
    while (TRUE) {
        bool gotChar;
        /*
         * Read a character and process it
         */
        gotChar = SerialGetChar(ioCom, &receivedChar, SHORT_WAIT_TIME);
        dwdReset();
        if (gotChar) {
            receivedChar &= 0x7f; // 7-bit ASCII
            // Is it a delete character?
            if ((receivedChar == '\x7f') | (receivedChar == '\b')) {
                escSeq=0;
                if (charNum > 0) { // If there is a character, delete it...
                    commandString[--charNum] = 0; //...in the buffer and...
                    if (echo)
                        //...on the screen
                        SerialPutString(ioCom, deleteString, 0);
                }
            } else if (escSeq==0 && receivedChar == 27) { /* Escape */
                escSeq=1;
            } else if ((escSeq==1) && receivedChar == '[') {
                escSeq=2;
            }
            /* Here we got an up arrow */
            else if ((escSeq == 2) && receivedChar == 'A') {
                escSeq = 0;
                strncpy(commandString, prevCommandString, COM_STRING_SIZE);
                charNum = prevSize + 1;
                commandString[charNum] = 0;
                printf("\r%c%s", prompt, commandString);

            } else if (receivedChar != '\n') {
                /*Ignore line feed.  Only watch for CR*/
                /* Echo the received character and record it in the buffer */
                escSeq = 0;
                if (echo)
                    // COM0, value, block time is unused
                    SerialPutChar(ioCom, receivedChar, 0);
                /* Force it to be lower case, plus it won't change digits */
                commandString[charNum++] = receivedChar | 0x20;
            }
            /*
             * All the code above echos type characters and allows one
             * to use delete to edit characters.  We don't really do
             * anything else until we see a carriage return (\r), and
             * then we execute the if clause below, which is pretty
             * much the rest of the routine.
             */

            // The user hit 'enter' or carriage return
            if (receivedChar == '\r') {
                int dest = 0, src = 0;

                if (echo)
                    SerialPutChar(ioCom, '\n', 0); // Also echo a line feed
                /*
                 * Ok, we have the end of the command line.  Decode it
                 * and do the command.
                 */
                commandString[--charNum] = 0; // Replace the CR with a null terminator
                /* Now squeeze out extra spaces */
                while (commandString[src] != 0) {
                    commandString[dest] = commandString[src++];
                    /*
                     * If we have multiple spaces, only take the last one
                     */
                    if ((commandString[dest] != ' ')
                            || (commandString[src] != ' '))
                        dest++;
                }
                if (commandString[0]!= 0){
                    strncpy(prevCommandString, commandString, COM_STRING_SIZE);
                    prevSize = --dest;
                }
                return;
            }

        }
    }
}

void PreflightInitNow(CanIDNode cameFrom)
{
    int size;

    size=SetupMRAM(); //This should be first.  It might overwrite part of MRAM.
    WriteMRAMBoolState(StateInOrbit, false);
    //todo: The rest of the stuff has not been implemented yet
    printf("WOD Frequency,size set to Default, and WOD Indices initialized\n");

    printf("WOD Storage and All variables initialized and ready for flight\n");

    printf("MRAM size found to be %d bytes\n", size);
    printf("Size of MRAM data structure is %d\n", sizeof(MRAMmap_t));
    if(size < sizeof(MRAMmap_t)){
        printf("!!!Not enough MRAM in this IHU!!!");
    }
}

void DisplayTelemetry(uint32_t typeRequested)
{
    switch(typeRequested){
    case 0: {
        /*
         * Telemetry type 0 is actually status in the debug task
         */
        int i;

        printf("I2c device state:\n"
                "    PacSat CPU Temp: %d, Transmitter Temp: %d, RealTimeClock: %d\n",
                CpuTempIsOk(),TxTempIsOk(),RTCIsOk());

        printf("MRAM State Values:\n\r"
                " CommandedSafeMode=%d,Autosafe=%d\n\r"
                " CommandRcvd=%d,AllowAutoSafe=%d\n\r"
                " AX25 PB Enabled=%d,FTL0 Enabled=%d,Digi Enabled=%d\n\r",

               ReadMRAMBoolState(StateCommandedSafeMode),
               ReadMRAMBoolState(StateAutoSafe),
               ReadMRAMBoolState(StateCommandReceived),
               ReadMRAMBoolState(StateAutoSafeAllow),
               ReadMRAMBoolState(StatePbEnabled),
               ReadMRAMBoolState(StateUplinkEnabled),
               ReadMRAMBoolState(StateDigiEnabled));
        printf(" RX Modes:");
        for (i = 0; i < NUM_OF_RX_CHANNELS; i++)
            printf(" [%d] %x %s", i, ReadMRAMReceiverMode(i),
                   ReadMRAMReceiveSpeed(i) == DCT_SPEED_9600 ? "9600" : "1200");
        printf("\n Uncommanded Seconds in Orbit=%d\n\r",
                (unsigned int) ReadMRAMSecondsOnOrbit());
        // todo:  Have to put the on-orbit flag somewhere
        //        readNV(&onOrbit,sizeof(int),LocalEEPROMData,(int)&eepromMemMap->HaveWaitedInOrbit);
        //        printf(" On-orbit flag is ");
        //        if(onOrbit){
        //            printf("TRUE\n\r");
        //        } else {
        //            printf("FALSE\n\r");
        //        }
        printf("Errors:\n");
        printf(" IHU reset cause=0x%x,reset data=0x%x,IHUWdReport=0x%x,\n"
                " LastTask=%s\n"
                " Reason=%s\n",
                localErrorCollection.errorCode,
                localErrorCollection.errorData,
                localErrorCollection.wdReports,
                getTaskName(localErrorCollection.taskNumber),
                ErrorMessageString((ErrorType_t)localErrorCollection.errorCode));

        printf("Watchdog Reports in Errors.c:\n");

        int bit;

        for (bit = 0; bit < 9; bit ++) {
            printf("%20s = %d\n",TaskNames[bit+1],
                   (localErrorCollection.wdReports>>bit) & 0x01);
        }

        i = xPortGetFreeHeapSize();
        printf("Free heap size is %d\n\r", i);
        return;
    }

    default: {
        printf("????");
        break;
    }
    } /* End of switch */
}

void printID(void){
    printf("\nAMSAT-NA PacSat Console\n");
    printf("Flight Software %s (built on %s at %s)\n",
           PACSAT_FW_VERSION_STRING, __DATE__, __TIME__);
    //printf("   Built %s %s\n\n",__DATE__, __TIME__);//__GNUC__,__GNUC_MINOR__);

#if defined (COMPILE_DEBUG)
    printf("Compiled **DEBUG**\n");
#else
    printf("Compiled *RELEASE*\n");
#endif
#if !defined(UNDEFINE_BEFORE_FLIGHT) && !defined (ENGINEERING_MODEL) && !defined(COMPILE_DEBUG)
    printf("Configured for Flight\n");
#else
    printf("**X, E, U, or DEBUG Version: Not configured for flight**\n");
#endif
#ifdef WATCHDOG_ENABLE
    printf("Watchdog Enabled\n");
#else
    printf("Watchdog NOT Enabled\n");
#endif
    printf("Using FreeRTOS with"
#if !configUSE_PREEMPTION
            "OUT"
#endif
            " premption enabled;\n");


    printf("Free heap size is %d\n",xPortGetFreeHeapSize());
    {
        uint32_t *addr= (uint32_t *)0xf008015c,value,megs,kilos;
        value = ((*addr) & 0xFFFF);
        megs = value/1024;
        kilos = value % 1024;
        printf("Flash memory size %dMb+%dKb\n",megs,kilos);
        printf("MRAM config data partition size=%d, file system size=%d\n",
               getSizeNV(NVConfigData), getSizeNV(NVFileSystem));
    }

    //    printf("Previous reboot reason=%d (%s), WD reports=%x,task=%d\n",localErrorCollection.LegErrorCode,
    //            ErrMsg[localErrorCollection.LegErrorCode],
    //            localErrorCollection.LegWdReports,localErrorCollection.LegTaskNumber);
}

void printHelp(char *search,commandPairs *commands, int numberOfCommands)
{
    int commandNum,sizeCmd,numSpace,space;

    for (commandNum = 0; commandNum < numberOfCommands; commandNum++) {
        char *command = commands[commandNum].typedCommand;
        char *helpText = commands[commandNum].help;

        if (strstr(command, search) == NULL &&
                strstr(helpText, search) == NULL) {
            continue;
        }

        sizeCmd = strlen(command);
        numSpace = 20 - sizeCmd;
        printf("%s", command);
        for(space=0; space < numSpace; space++){
            printf(" ");
        }
        printf("%s\n", commands[commandNum].help);
    }
}
