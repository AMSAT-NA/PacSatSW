/*
 * consoleRoutines.c
 *
 *  Created on: Sep 21, 2020
 *      Author: bfisher
 */
#include <ctype.h>
#include <stdarg.h>
#include <stdlib.h>
#include <strings.h>

#include "pacsat.h"
#include "TMS570Hardware.h"
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

char *next_token(char **str)
{
    char *s = *str, *s2;

    while (isspace(*s))
        s++;
    if (!*s)
        return NULL;
    s2 = s + 1;
    while (*s2 && !isspace(*s2))
        s2++;
    if (*s2) {
        *s2 = '\0';
        s2++;
    }
    *str = s2;
    return s;
}

uint16_t parseNumber(char **str)
{
    char *t = next_token(str);

    if (!t)
        return 0;
    return (uint16_t) strtol(t, NULL, 0);
}

uint32_t parseNumber32(char **str)
{
    char *t = next_token(str);

    if (!t)
        return 0;
    return (uint32_t) strtoul(t, NULL, 0);
}

int parse_uint8(char **str, uint8_t *num, int base)
{
    char *t = next_token(str), *end;
    uint32_t v;

    if (!t || !*t)
        return -1;
    v = strtoul(t, &end, base);
    if (*end != '\0')
        return -2;
    if (v > 255)
        return -3;
    *num = v;
    return 0;
}

int parse_uint16(char **str, uint16_t *num, int base)
{
    char *t = next_token(str), *end;
    uint32_t v;

    if (!t || !*t)
        return -1;
    v = strtoul(t, &end, base);
    if (*end != '\0')
        return -2;
    if (v > 65535)
        return -3;
    *num = v;
    return 0;
}

int parse_uint32(char **str, uint32_t *num, int base)
{
    char *t = next_token(str), *end;
    uint32_t v;

    if (!t || !*t)
        return -1;
    v = strtoul(t, &end, base);
    if (*end != '\0')
        return -1;
    *num = v;
    return 0;
}

int parse_freq(char **str, uint32_t *num)
{
    char *t = next_token(str), *end;
    uint32_t v;

    if (!t || !*t)
        return -1;
    v = strtoul(t, &end, 0);
    if (*end == 'k' || *end == 'K')
        v *= 1000;
    else if (*end == 'm' || *end == 'M')
        v *= 1000000;
    else if (*end != '\0')
        return -1;
    *num = v;
    return 0;
}

int parse_bool(char **str, bool *val)
{
    char *t = next_token(str);

    if (!t || !*t)
        return -1;
    if (strcmp(t, "1") == 0 || strcmp(t, "true") == 0 || strcmp(t, "on") == 0)
        *val = true;
    else if (strcmp(t, "0") == 0 || strcmp(t, "false") == 0 ||
             strcmp(t, "off") == 0)
        *val = false;
    else
        return -1;
    return 0;
}

static char *next_token_chr(char **str, char *cstr)
{
    char *s = *str, *s2;

    while (isspace(*s))
        s++;
    if (!*s)
        return NULL;
    s2 = s + 1;
    while (*s2 && !isspace(*s2) && !strchr(cstr, *s2))
        s2++;
    if (*s2 && !strchr(cstr, *s2)) {
        *s2 = '\0';
        s2++;
    }
    *str = s2;
    return s;
}

int parse_uint16_range(char **str, uint16_t *num1, uint16 *num2, int base)
{
    char *t = next_token_chr(str, "-"), *s;
    bool has_range = false;
    int err;

    if (!t || !*t)
        return -1;
    s = *str;
    while (isspace(*s))
        s++;
    if (*s == '-') {
        has_range = true;
        *s = '\0';
        s++;
    }

    err = parse_uint16(&t, num1, base);
    if (err)
        return err;

    if (!has_range) {
        *num2 = *num1;
        return 0;
    }

    t = next_token(&s);
    if (!t || !*t)
        return -1;

    err = parse_uint16(&t, num2, base);
    if (err)
        return err;

    *str = s;
    return 0;
}

void receiveLine(COM_NUM ioCom, char *commandString, char prompt, bool echo)
{
    /*
     * Wait until something arrives in the queue - this routine will
     * block indefinitely provided INCLUDE_vTaskSuspend is set to 1 in
     * FreeRTOSConfig.h.
     */
    static char prevCommandString[COM_STRING_SIZE], prevSize = 0; /* For up-arrow */
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
                charNum = prevSize;
                commandString[charNum] = 0;
                printf("\r%c%s", prompt, commandString);

            } else if (receivedChar != '\n') {
                /*Ignore line feed.  Only watch for CR*/
                /* Echo the received character and record it in the buffer */
                escSeq = 0;
                if (charNum < COM_STRING_SIZE) {
                    if (echo)
                        // COM0, value, block time is unused
                        SerialPutChar(ioCom, receivedChar, 0);
                    /* Force it to be lower case, plus it won't change digits */
                    commandString[charNum++] = receivedChar | 0x20;
                }
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
                commandString[--charNum] = '\0'; // Replace the CR with a null terminator
                /* Now squeeze out extra spaces */
                while (commandString[src] != '\0') {
                    commandString[dest] = commandString[src++];
                    /*
                     * If we have multiple spaces, skip them.
                     */
                    if (commandString[dest] == ' ') {
                        while (commandString[src] == ' ')
                            src++;
                    }
                    dest++;
                }
                commandString[dest] = '\0';
                if (commandString[0] != 0) {
                    strncpy(prevCommandString, commandString, COM_STRING_SIZE);
                    prevSize = dest;
                }
                return;
            }

        }
    }
}

void PreflightInitNow(void)
{
    int size;

    size=SetupMRAM(); //This should be first.  It might overwrite part of MRAM.
    WriteMRAMBoolState(StateInOrbit, false);
//    printf("WOD Frequency,size set to Default, and WOD Indices initialized\n");

    printf("MRAM size found to be %d bytes\n", size);
    printf("Size of MRAM data structure is %d\n", sizeof(MRAMmap_t));
    if(size < sizeof(MRAMmap_t)){
        printf("!!!Not enough MRAM in this IHU!!!");
    }
}

bool radio_modulation_valid(radio_modulation mod)
{
    enum fec fec = RADIO_MODULATION_GET_FEC(mod);
    enum speed speed = RADIO_MODULATION_GET_SPEED(mod);
    enum modulation modulation = RADIO_MODULATION_GET_MOD(mod);

    if (mod < 0 || mod > 255)
        return false;
    if (speed == SPEED_SPECIAL && modulation == MODULATION_MSK)
        return true;
    return (fec <= FEC_COUNT
            && speed > SPEED_SPECIAL && speed < SPEED_COUNT
            && modulation < MODULATION_COUNT);
}

const char *modulation_fec_str(radio_modulation mod)
{
    switch (RADIO_MODULATION_GET_FEC(mod)) {
    case FEC_NONE:
        return "";

    case FEC_CONV:
        return "conv";

    default:
        return "unknown";
    }
}

const char *modulation_mod_str(radio_modulation mod)
{
    enum speed speed = RADIO_MODULATION_GET_SPEED(mod);
    enum modulation modulation = RADIO_MODULATION_GET_MOD(mod);

    if (modulation == MODULATION_MSK && speed == SPEED_SPECIAL)
        return "afsk";

    switch (modulation) {
    case MODULATION_MSK:
        return "msk";

    case MODULATION_PSK:
        return "psk";

    default:
        return "unknown";
    }
}

const char *modulation_speed_str(radio_modulation mod)
{
    enum speed speed = RADIO_MODULATION_GET_SPEED(mod);
    enum modulation modulation = RADIO_MODULATION_GET_MOD(mod);

    switch (speed) {
    case SPEED_SPECIAL:
        if (modulation != MODULATION_MSK)
            return "unknown";
        /* fallthrough */
    case SPEED_1200:
        return "1200";

    case SPEED_2400:
        return "2400";

    case SPEED_4800:
        return "4800";

    case SPEED_9600:
        return "9600";

    case SPEED_19200:
        return "19200";

    case SPEED_25K:
        return "25k";

    case SPEED_50K:
        return "50k";

    case SPEED_100K:
        return "100k";

    default:
        return "unknown";
    }
}

bool str_to_fec(const char **str, enum fec *fec)
{
    if (strcmp(*str, "") == 0) {
        *fec = FEC_NONE;
        return true;
    }
    if (strcasecmp(*str, "conv") == 0) {
        *str += 4;
        *fec = FEC_CONV;
        return true;
    }
    return false;
}

bool str_to_mod(const char **str, enum modulation *mod, enum speed *speed)
{
    *speed = SPEED_1200;
    if (strncasecmp(*str, "afsk", 4) == 0) {
        *str += 4;
        *mod = MODULATION_MSK;
        *speed = SPEED_SPECIAL;
        return true;
    }
    if (strncasecmp(*str, "msk", 3) == 0) {
        *str += 3;
        *mod = MODULATION_MSK;
        return true;
    }
    if (strncasecmp(*str, "psk", 3) == 0) {
        *str += 3;
        *mod = MODULATION_PSK;
        return true;
    }
    return false;
}

bool str_to_speed(const char **str, enum modulation mod, enum speed *speed)
{
    if (mod == MODULATION_MSK && *speed == SPEED_SPECIAL) {
        if (strncasecmp(*str, "1200", 4) != 0)
            return false;
        *str += 4;
        return true;
    }
    if (strncasecmp(*str, "1200", 4) == 0) {
        *str += 4;
        *speed = SPEED_1200;
        return true;
    }
    if (strncasecmp(*str, "2400", 4) == 0) {
        *str += 4;
        *speed = SPEED_2400;
        return true;
    }
    if (strncasecmp(*str, "4800", 4) == 0) {
        *str += 4;
        *speed = SPEED_4800;
        return true;
    }
    if (strncasecmp(*str, "9600", 4) == 0) {
        *str += 4;
        *speed = SPEED_9600;
        return true;
    }
    if (strncasecmp(*str, "19200", 5) == 0) {
        *str += 5;
        *speed = SPEED_19200;
        return true;
    }
    if (strncasecmp(*str, "25K", 3) == 0) {
        *str += 3;
        *speed = SPEED_25K;
        return true;
    }
    if (strncasecmp(*str, "50K", 3) == 0) {
        *str += 3;
        *speed = SPEED_50K;
        return true;
    }
    if (strncasecmp(*str, "100K", 4) == 0) {
        *str += 4;
        *speed = SPEED_100K;
        return true;
    }
    return false;
}

#ifdef DEBUG
static const char *getTaskName(unsigned int task)
{
    if (task >= NUM_TASKS)
        return "Invalid";
    return TaskNames[task];
}

void DisplayTelemetry(uint32_t typeRequested)
{
    radio_modulation mod;

    printf("Mode: %s\n", getSpacecraftModeStr());
    switch(typeRequested){
    case 0: {
        /*
         * Telemetry type 0 is actually status in the debug task
         */
        int i;

        printf("I2c device state:\n"
                "  PacSat CPU Temp: %d, Transmitter Temp: %d, RealTimeClock: %d\n",
                CpuTempIsOk(),TxTempIsOk(),RTCIsOk());

        printf("MRAM State Values:\n\r"
                "  TX Inhibit=%d\n\r"
                "  CommandedSafeMode=%d,Autosafe=%d\n\r"
                "  CommandRcvd=%d,AllowAutoSafe=%d\n\r"
                "  Enabled: PB=%d,FTL0=%d,Digi=%d,Telem=%d,Time=%d,WOD=%d,Err WOD=%d\n\r",
               ReadMRAMBoolState(StateTransmitInhibit),
               ReadMRAMBoolState(StateCommandedSafeMode),
               ReadMRAMBoolState(StateAutoSafe),
               ReadMRAMBoolState(StateCommandReceived),
               ReadMRAMBoolState(StateAutoSafeAllow),
               ReadMRAMBoolState(StatePbEnabled),
               ReadMRAMBoolState(StateUplinkEnabled),
               ReadMRAMBoolState(StateDigiEnabled),
               ReadMRAMBoolState(StateTelemBroadcastEnabled),
               ReadMRAMBoolState(StateTimeBroadcastEnabled),
               ReadMRAMBoolState(StateWodEnabled),
               ReadMRAMBoolState(StateErrWodEnabled));

        printf("MRAM Telem Values:\n\r"
                "  PB Status Period(s)=%d, PB Timeout(s)=%d, Uplink Status Period(s)=%d\n\r"
                "  Period(s): Time=%d, Telem=%d, WOD=%d, Err WOD=%d\n\r"
                "  Max FileSize(bytes) WOD=%d, Err WOD=%d, Exp=%d\n\r",
                ReadMRAMPBStatusFreq(),
                ReadMRAMPBClientTimeout(),
                ReadMRAMFTL0StatusFreq(),
                ReadMRAMTimeFreq(),
                ReadMRAMTelemFreq(),
                ReadMRAMWODFreq(),
                ReadMRAMErrWODFreq(),
                ReadMRAMWODMaxFileSize4kBlocks()*4096,
                ReadMRAMErrWODMaxFileSize4kBlocks()*4096,
                ReadMRAMExpMaxFileSize4kBlocks()*4096);
        mod = ReadMRAMModulation(FIRST_TX_CHANNEL);
        printf("  TX Mode: %s%s%s\n", modulation_mod_str(mod),
               modulation_speed_str(mod), modulation_fec_str(mod));
        printf("  RX Modes:");
        for (i = 0; i < NUM_RX_CHANNELS; i++) {
            mod = ReadMRAMModulation(i);
            printf(" [%d] %s", i, modulation_mod_str(mod),
                   modulation_speed_str(mod), modulation_fec_str(mod));
        }
        printf("\n  Uncommanded Seconds in Orbit=%d\n\r",
                (unsigned int) ReadMRAMSecondsOnOrbit());
                bool onOrbit = ReadMRAMBoolState(StateInOrbit);
                printf("  On-orbit flag is: ");
                if(onOrbit){
                    printf("TRUE\n\r");
                } else {
                    printf("FALSE\n\r");
                }
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

        for (i = 0; i < 9; i++) {
            printf(" %s=%d,", TaskNames[i+1],
                   (localErrorCollection.wdReports >> i) & 0x01);
        }
        printf("\n");
        printf("Free heap size is %d\n\r", xPortGetFreeHeapSize());
        return;
    }

    default: {
        printf("????");
        break;
    }
    } /* End of switch */
}
#endif

void printID(void)
{
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
        extern char _flash_last[], _flash_end[];
        value = ((*addr) & 0xFFFF);
        megs = value/1024;
        kilos = value % 1024;
        printf("Flash memory size %dMb+%dKb, %d free\n",megs,kilos,
               _flash_end - _flash_last);
        printf("MRAM config data partition size=%d, file system size=%d\n",
               getSizeNV(NVConfigData), getSizeNV(NVFileSystem));
    }

    printf("Previous reboot reason=%d (%s), WD reports=%x, task=%d\n",
           localErrorCollection.errorCode,
           ErrorMessageString((ErrorType_t) localErrorCollection.errorCode),
           localErrorCollection.wdReports,
           localErrorCollection.taskNumber);
}

void printHelp(char *search, const commandPairs *commands, int numberOfCommands)
{
    int commandNum, sizeCmd, numSpace;

    for (commandNum = 0; commandNum < numberOfCommands; commandNum++) {
        const char *command = commands[commandNum].typedCommand;
        const char *helpText = commands[commandNum].help;

        if (strstr(command, search) == NULL &&
                strstr(helpText, search) == NULL) {
            continue;
        }

        sizeCmd = strlen(command);
        numSpace = 20 - sizeCmd;
        printf("%s", command);
        printf("%*s%s\n", numSpace, "", commands[commandNum].help);
        if (commands[commandNum].help2) {
            printf("%22s%s %s\n", "", command, commands[commandNum].help2);
        }
    }
}
