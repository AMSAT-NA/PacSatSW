/*
 * ConsoleTask.c
 *
 *  Created on: Mar 4, 2019
 *      Author: burns
 */
/*
 * Standard headers for Golf
 */
#include <ax5043-ax25.h>
#include <pacsat.h>
#include "stdarg.h"
#include "stdlib.h"
#include "nonvolManagement.h"
#include "sys_common.h"
#include "serialDriver.h"
#include "UplinkCommands.h"
#include "CommandTask.h"
#include "TelemetryRadio.h"
#include "consoleRoutines.h"
#include "ax5043.h"
#include "ax5043_access.h"
#include "system.h"
#include "sys_core.h"
#include "gio.h"
#include "pinmux.h"
#include "spi.h"
#include "het.h"
#include "i2c.h"
#include "rti.h"
#include "esm.h"
#include "adc_proc.h"
#include "mram.h"

//FreeRTOS headers
#include "FreeRTOS.h"
#include "os_task.h"

//Flight software headers
#include "downlink.h"
#include "SystemServiceEntry.h"
#include "spiDriver.h"
#include "i2cDriver.h"
#include "serialDriver.h"
#include "TMS570Hardware.h"
#include "ADS7828.h"
#include "I2cPoll.h"
#include "MET.h"
#include "nonvol.h"
#include "ax5043.h"
#include "Max31331Rtc.h"
#include "GPIO9539.h"
#include "inet.h"
#include "errors.h"
#include "gpioDriver.h"
#include "LinearInterp.h"
#include "Buzzer.h"
#include "keyfile.h"

#ifdef BLINKY_HARDWARE
#include "Max31725Temp.h"
#endif

#include "CANTask.h" // For test routines
#include "TxTask.h" // for test routines
#include "PbTask.h" // for test routines
#include "pacsat_header.h" // for test routines
#include "pacsat_dir.h" // for dir commands and test routines
#include "redposix.h"
#include "Ax25Task.h"
#include "UplinkTask.h"


//Extern definition
extern uint8_t SWCmdRing[SW_CMD_RING_SIZE], SWCmdIndex;

/* Must be long enough for worst case: Uploadtest20xxxxxx  which is 86 */
static char commandString[COM_STRING_SIZE];

/* Current and default frequencies. */
static uint32_t DCTFreq[NUM_CHANNELS];
#if NUM_CHANNELS == 5
const uint32_t DCT_DEFAULT_FREQ[NUM_CHANNELS] =
    { 145780000, 145810000, 145840000, 145870000, 435760000 };
#else
    { 145780000, 435760000 };
#endif

const enum radio_modulation DCT_DEFAULT_MODULATION[NUM_CHANNELS] = {
    MODULATION_AFSK_1200, MODULATION_AFSK_1200,
    MODULATION_AFSK_1200, MODULATION_AFSK_1200,
    MODULATION_AFSK_1200,
};

// default all channels - TODO - is this used?
const uint8_t DCT_DEFAULT_MODE[NUM_CHANNELS] = { 0x15, 0x15, 0x15, 0x15, 15 };

extern bool InSafeMode, InScienceMode, InHealthMode;
extern bool TransponderEnabled, onOrbit, SimDoppler;
extern resetMemory_t SaveAcrossReset;
extern char *ErrMsg[];
extern bool monitorPackets;

bool TestPlayDead = false;


enum {
    nada=0,

    getTemp,
    getVoltages,
    getPowerFlags,
    RfPowPrint,
    BoardVersion,
    reset,
    startWD,
    preflight,
    time,
    SwVersion,
    clrMinMax,
    getI2cState,
    internalWDTimeout,
    externalWDTimeout,
    telem0,
    pollI2c,
    readMRAMsr,
    writeMRAMsr,
    dropBus,
    healthMode,
    Tx,
    getState,
    testLED,
    doClearMRAM,
    enbCanPrint,
    dsbCanPrint,
    EnableComPr,
    DisableComPr,
    MountFS,
    UnMountFS,
    FormatFS,
    LsFS,
    RmFS,
    mkdirFS,
    rmdirFS,
    SelectRFPowerLevels,
    SetDCTDrivePower,
    Freq,
    TxPow,
    SaveFreq,
    LoadKey,
    showDownlinkSize,
    ignoreUmb,
    noticeUmb,
    initSaved,
    TestMemScrub,
    GetCommands,
    Gpio,
    SendCANMsg,
    SetCANLoopback,
    TraceCAN,
    GetCANCounts,
    getax5043,
    AxReg,
    getRSSI,
    testFreq,
    testPLLrange,
    testAX5043,
    HelpAll,
    HelpDevo,
    HelpSetup,
    HelpPacsat,
    Help,
    MRAMWrEn,
    testAllMRAM,
    sizeMRAM,
    mramSleep,
    mramAwake,
    startRx,
    stopRx,
#ifdef DEBUG
    testPacsat,
    testCallsigns,
    testTx,
    testPbOk,
    testPbStatus,
    testPbList,
    testPbClearList,
    testPfh,
    testPfhFile,
    testDecode,
    makePfhFiles,
    testDir,
    sendUplinkStatus,
    testRetransmission,
    testUploadTable,
    listUploadTable,
#endif
    Monitor,
    pbShut,
    pbOpen,
    uplinkShut,
    uplinkOpen,
    digiShut,
    digiOpen,
    mramHxd,
    dirLoad,
    dirClear,
    listDir,
    getNextFileNumber,
    resetNextFileNumber,
    heapFree,
    Time,
    getRtc,
    regRtc,
    Modulation,
    setDigi,
};

/*
 * These commands should only be required to setup a new board and
 * configure it.
 */
commandPairs setupCommands[] = {
    { "init new proc",
      "Init DCT and MRAM stuff that will be set once for each unit",
      initSaved},
    { "freq",
      "Set/raise/lower/get the frequency\r\n"
      "                       freq <devnum> [+|-]value[k|m]",
      Freq},
    { "save freq",
      "Save the current frequency in MRAM",
      SaveFreq},
    { "test freq",
      "xmit on frequency of specified device",
      testFreq},
    { "txpow",
      "Get/set the tx power as a percentage 0-100\r\n"
      "                       txpow <devnum> [0-100]",
      TxPow },
    { "test internal wd",
      "Force internal watchdog to reset CPU",
      internalWDTimeout},
    { "test external wd",
      "Force external watchdog to reset CPU",
      externalWDTimeout},
    { "test leds",
      "Flash the LEDs in order",
      testLED},
    { "test mram",
      "Write and read mram with n blocksize (4,8,16,32,64,128)",testAllMRAM},
    {"load key",
     "Load an authorization key for uplink commands",
     LoadKey},
};

/*
 * These commands are used to operate the PB and the FTL0 system
 * through the console.
 */
commandPairs pacsatCommands[] = {
    { "monitor",
      "Enable/disable monitoring of sent and received packets\r\n"
      "                       monitor [on|off",
      Monitor},
    { "shut pb",
      "Shut the PB",
      pbShut},
    { "open pb",
      "Open the PB for use",
      pbOpen},
    { "shut uplink",
      "Shut the FTL0 Uplink",
      uplinkShut},
    { "open uplink",
      "Open the FTL0 Uplink for use",
      uplinkOpen},
    { "shut digi",
      "Disable the Digipeater",
      digiShut},
    { "open digi",
      "Enable the Digipeater",
      digiOpen},
    { "send pb status",
      "Immediately transmit the status of the PB",
      testPbStatus},
    { "hxd",
      "Display Hex for file number",
      mramHxd},
    { "load dir",
      "Load the directory from MRAM",
      dirLoad},
    { "clear dir",
      "Clear the directory but leave the files in MRAM",
      dirClear},
    { "list dir",
      "List the Pacsat Directory.",
      listDir},
    { "next filenumber",
      "Show the next file number that the Dir will assign to an uploaded file",
      getNextFileNumber},
    { "reset filenumber",
      "Reset the next Dir file number to zero.",
      resetNextFileNumber},
};

/*
 * These commands should only be required when writing the software.
 * They should not be required for setting up new boards, configuring
 * the flight unit or an other task performed once the software is
 * complete.
 */
commandPairs debugCommands[] = {
    { "test scrub",
      "Run the memory scrub routine once",
      TestMemScrub},
    { "test ax",
      "Read the revision and scratch registers",
      testAX5043},
    { "test pll",
      "test ax5043 PLL frequency range",
      testPLLrange},
    { "start rx",
      "Start up the 5043 receiver",
      startRx},
    { "stop rx",
      "Stop the 5043 receiver",
      stopRx},
    { "clear mram",
      "Initializes MRAM state,WOD,Min/max but does not clear InOrbit--testing only",
     doClearMRAM},
    { "poll i2c",
      "Poll to see which I2c devices are there",
      pollI2c},
    { "tx",
      "Simulate FCC command to turn on/off tx\r\n"
      "                       tx [on|off]",
      Tx},
    { "get mram sr",
      "Get the MRAM status register",
      readMRAMsr},
    { "get downlink size",
      "Debug-get sizes of downlink payloads and frames",
      showDownlinkSize},
    { "mram wren",
      "Write enable MRAM",
      MRAMWrEn},
    { "mram wake",
      "Send wake command to MRAM",
      mramAwake},
    { "mram sleep",
      "Send sleep command to MRAM",
      mramSleep},
#ifdef DEBUG
    { "test pacsat",
      "Run all of the PACSAT self tst routines",
      testPacsat},
    { "test callsigns",
      "Test the AX25 callsign routines",
      testCallsigns},
    { "test tx",
      "Test the Pacsat TX Packet routines",
      testTx},
    { "test pb ok",
      "Test the Pacsat Broadcast by sending OK packet",
      testPbOk},
    { "test pb list",
      "Test the PB List add and remove functions",
      testPbList},
    { "clear pb list",
      "Clear the PB List add remove all stations",
      testPbClearList},
    { "test pfh",
      "Test the Pacsat File Header Routines",
      testPfh},
    { "test psf",
      "Test the Pacsat Files in MRAM",
      testPfhFile},
    { "test decode",
      "Test decode of AX25 packets",
      testDecode},
    { "make psf",
      "Make a set of test Pacsat Files in MRAM",
      makePfhFiles},
    { "test dir",
      "Test the Pacsat Directory.  The command 'make psf' must already have been run",
     testDir},
    { "send uplink status",
      "Send Uplink status",
      sendUplinkStatus},
    { "test retransmission",
      "Test the AX25 I frame retransmission",
      testRetransmission},
    { "test upload table",
      "Test the storage of Upload records in the MRAM table",
      testUploadTable},
    { "list upload table",
      "List the Upload records in the MRAM table",
      listUploadTable},
    { "set digi",
      "Set the digipeater mode",
      setDigi},
#endif
};

commandPairs commonCommands[] = {
    { "reset ihu",
      "Reset this processor",
      reset},
    { "get time",
      "Display reset number and seconds",
      time},
    { "get status",
      "Get some general status info",
      telem0},
    { "get rssi",
      "Get the current RSSI reading from the AX5043 Rx",
      getRSSI},
    { "temp",
      "Print board temperatures",
      getTemp},
    { "volt",
      "Print board voltages",
      getVoltages},
    { "powfl",
      "Print power good flags",
      getPowerFlags},
    { "rfpowpr",
      "Enable/disable printing RF power\r\n"
      "                       rfpowpr [on|off]",
      RfPowPrint},
    { "board version",
      "Get board version number",
      BoardVersion},
    { "get state",
      "Mainly for debug: Get the current state of the downlink state machine",
      getState},
    { "sw version",
      "Get the software version number and build time",
      SwVersion},
    { "get mram size",
      "Get the total size of all MRAMs",
      sizeMRAM},
    { "get i2c",
      "What I2c devices are working?",
      getI2cState},
    { "heap free",
      "Show free bytes in the heap.",
      heapFree},
    { "time",
      "Get/set the number of seconds since the Unix epoch\r\n"
      "                       time [<value>]",
      Time},
    { "regrtc",
      "Read the specified register in the rtc\r\n"
      "                       regrtc [<reg> [<value>]]",
      regRtc},
    { "rtc",
      "Get the status and time from the Real Time Clock",
      getRtc},
    { "modul",
      "Get/set the radio to 1200 bps AFSK or 9600 GMSK\r\n"
      "                       mode [<dev> [1200|9600]]",
      Modulation},
    { "mount fs",
      "Mount the filesystem",
      MountFS},
    { "unmount fs",
      "unmount the filesystem",
      UnMountFS},
    { "format fs",
      "Format the filesystem",
      FormatFS},
    { "ls",
      "List files and directories in the filesystem",
      LsFS},
    { "rm",
      "Remove a file from the filesystem",
      RmFS},
    { "mkdir",
      "Make a directory in the filesystem",
      mkdirFS},
    { "rmdir",
      "Remove a directory from the filesystem",
      rmdirFS},
    { "get commands",
      "Get a list the last 4 s/w commands",
      GetCommands},
    { "gpio",
      "Display/set the values of GPIOS\r\n"
      "                       gpio [<num> [<value>]]",
      Gpio},
    { "send can",
      "Send a CAN bus message\r\n"
      "                       send can <canbus> <prio> <type> <id> <dest> [<byte1> [<byte2> [...]]]",
      SendCANMsg },
    { "set can loopback",
      "Enable/Disable the CAN loopback",
      SetCANLoopback },
    { "trace can",
      "Enable/Disable printing CAN messages",
      TraceCAN },
    { "get can counts",
      "Display can message and error counts",
      GetCANCounts },
    { "get ax",
      "Get ax5043 status",
      getax5043},
    { "axreg",
      "Read ax5043 reg n from device\r\n"
      "                       axreg <dev> <startreg>[-<endreg>] [value]",
      AxReg},
    { "health mode",
      "Set health mode",
      healthMode},
#if 0
     { "set dct drive power",
       // "Set drive power for high and low power",
       SetDCTDrivePower},
    { "select dct power",
      "Set rf power used in safe or normal modes",
      SelectRFPowerLevels},
#endif
    { "set mram sr",
      "Set the MRAM status register",
      writeMRAMsr},
    { "start watchdog",
      "Start the watchdog",
      startWD},
    { "preflight init",
      "Initialize MRAM etc for flight",
      preflight},
#if 0
    { "clear minmax",
      "Clear the min, max, and CIU counts",
      clrMinMax},
    { "drop bus",
      "Assert a fail briefly so the bus switches drop out",
      dropBus},
    { "ignore umbilical",
      "Allow radios to go on with umbilical connected",
      ignoreUmb},
    { "heed umbilical",
      "Reverse effects of ignore umbilical",
      noticeUmb},
    { "enable canprint",
      "Debug can--number follows",
      enbCanPrint},
    { "disable canprint",
      "Debug can--number follows",
      dsbCanPrint},
#endif
    { "enable comprint",
      "Show uplink commands",
      EnableComPr},
    { "disable comprint",
      "Turn off uplink command print",
      DisableComPr},
    { "helpall",
      "List all commands",
      HelpAll},
    { "helpdevo",
      "List commands for software developers",
      HelpDevo},
    { "helpsetup",
      "List commands for setting up new board",
      HelpSetup},
    { "helppacsat",
      "List commands for controlling the PB and Uploads (FTL0)",
      HelpPacsat},
    { "help",
      "List common commands",
      Help},
};

struct command_table {
    const char *name;
    commandPairs *pairs;
    unsigned int num_cmds;
} command_tables[] = {
#define HELP_COMMON_INDEX 0
    { "COMMON COMMANDS",
      commonCommands, sizeof(commonCommands) / sizeof(commandPairs) },
#define HELP_SETUP_INDEX 1
    { "SETUP COMMANDS",
      setupCommands, sizeof(setupCommands) / sizeof(commandPairs) },
#define HELP_PACSAT_INDEX 2
    { "PACSAT PB and Upload COMMANDS",
      pacsatCommands, sizeof(pacsatCommands) / sizeof(commandPairs) },
#define HELP_DEBUG_INDEX 3
    { "DEBUG COMMANDS",
      debugCommands, sizeof(debugCommands) / sizeof(commandPairs) },
    {}
};

char *ResetReasons[] = {
    NULL,
    NULL,
    NULL,
    "External",
    "Software",
    "CPU Reset",
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    "Watchdog",
    "Oscillator",
    "Power On"
};

static char *get_dev_modulation_str(uint8_t devb)
{
    return modulation_to_str(ReadMRAMModulation(devb));
}

static int parse_devnum_de(char **str, AX5043Device *dev, int off, bool doerr)
{
    uint8_t devb;
    int err = parse_uint8(str, &devb, 0);

    if (err) {
        if (doerr)
            printf("Invalid or missing device number\n");
        return err;
    }
    if (devb >= NUM_AX5043_SPI_DEVICES - off) {
        if (doerr)
            printf("Give a device number between 0 and %d\n",
                   NUM_AX5043_SPI_DEVICES - off);
        return -100;
    }
    *dev = (AX5043Device) devb;
    return 0;
}

static int parse_devnum_noerr(char **str, AX5043Device *dev, int off)
{
    return parse_devnum_de(str, dev, off, false);
}

static int parse_devnum(char **str, AX5043Device *dev, int off)
{
    return parse_devnum_de(str, dev, off, true);
}

void RealConsoleTask(void)
{
    /* Block for 500ms. */
    char *afterCommand;
    bool DoEcho = true;
    unsigned int i;

    for (i = 0; i < NUM_CHANNELS; i++) {
        DCTFreq[i] = ReadMRAMFreq(i);
        if ((DCTFreq[i] < 999000) || (DCTFreq[i] > 600000000))
            DCTFreq[0] = DCT_DEFAULT_FREQ[i];
        quick_setfreq((AX5043Device) i, DCTFreq[i]);
    }

    // For watchdog when it is called with "current task"
    vTaskSetApplicationTaskTag((xTaskHandle) 0, (pdTASK_HOOK_CODE) ConsoleTsk);

    while (true) {
        int tablenum, commandNumber, index = 0;

        if (DoEcho) {
            SerialPutString(PRINTF_COM, "Pacsat", 0);
        }
        receiveLine(PRINTF_COM, commandString, '>', DoEcho);
        if (commandString[0] == 0)
            continue; /* Ignore blank lines */

        /*
         * Start of command parsing.  Go through each table, then
         * search the array of command string for one that matches the
         * beginning of the typed line.  When it is found, dispatch on
         * the string number.  There are enums which match the strings
         * to use for the case statements.
         */
        for (tablenum = 0;
             index == 0 && command_tables[tablenum].pairs;
             tablenum++) {
            struct command_table *table = &command_tables[tablenum];

            for (commandNumber = 0;
                 index == 0 && commandNumber < table->num_cmds;
                 commandNumber++) {
                commandPairs *cmd = &table->pairs[commandNumber];

                /*
                 * If a command is found at the start of the type
                 * commands, leave the loop.
                 */
                if (strstr(commandString, cmd->typedCommand) == commandString) {
                    index = cmd->indexVal;
                    afterCommand = commandString + strlen(cmd->typedCommand);
                }
            }
        }

        /*
         * Here is the main command dispatch based on which command
         * number was found above.
         */
        switch (index) {
        case nada: {
            printf("Unknown command\n");
            break;
        }

        case sizeMRAM: {
            int i;
            printf("MRAM Address Size=%d\n", getMRAMAddressSize());
            printf("Partition 0 size=%d, partition 1=%d\n",
                   getMRAMPartitionSize(0), getMRAMPartitionSize(1));
            for (i = 0; i < PACSAT_MAX_MRAMS;) {
                printf("MRAM%d size is %dKBytes", i, getMRAMSize(i) / 1024);
                i++;
                if(i%2 == 0){
                    printf("\n");
                } else {
                    printf(", ");
                }
            }
#if 0
            printf("Size of MRAM0 is %dKB\n",getMRAMSize(0)/1024);
            printf("Size of MRAM1 is %dKB\n",getMRAMSize(1)/1024);
            printf("Size of MRAM2 is %dKB\n",getMRAMSize(MRAM2Dev)/1024);
            printf("Size of MRAM3 is %dKB\n",getMRAMSize(MRAM3Dev)/1024);
#endif
            break;
        }

        case testAllMRAM:{
            int add = parseNumber(&afterCommand);
            int secret = parseNumber(&afterCommand);

            if (add == 0)
                add = 4;
            printf("NOTE:  This test will wipe out the file system and configuration values in MRAM.\n\n");

            if (secret != 42) {
                printf("  To confirm you must give the command with a second argument of 42, for example 'test mram %d 42'\n", add);
                break;
            }
            testMRAM(add);
            break;
        }

        case MRAMWrEn: {
            bool stat;
            int num = parseNumber(&afterCommand);

            stat = writeEnableMRAM(num);
            printf("stat for MRAM %d is %d; sr is %x\n", num, stat,
                   readMRAMStatus(num));
            //readNV(data,8,NVConfigData,0);
            //printf("MRAM at address 0 and 1 are now now %d and %d\n",data[0],data[1]);
            break;
        }

#ifdef DEBUG
        case Gpio: {
            uint16_t gpio;
            int err = parse_uint16(&afterCommand, &gpio, 0);
            Gpio_Use i;
            bool val;

            if (err) {
                for (i = (Gpio_Use) 0; i < NumberOfGPIOs; i++)
                    printf("%s(%d):%d\n", GPIOToName(i), i, GPIOIsOn(i));
                break;
            }

            i = (Gpio_Use) gpio;

            if (i >= NumberOfGPIOs) {
                printf("Invalid GPIO number %d, use 'gpio' to find the number\n",
                       gpio);
                break;
            }

            err = parse_bool(&afterCommand, &val);
            if (err) {
                printf("%s(%d):%d\n", GPIOToName(i), i, GPIOIsOn(i));
                break;
            }

            if (val)
                GPIOSetOn(i);
            else
                GPIOSetOff(i);
            printf("Set %s(%d) to %d\n", GPIOToName(i), i, GPIOIsOn(i));
            break;
        }

        case SendCANMsg: {
            uint8_t canNum, priority, type, dest;
            int err = parse_uint8(&afterCommand, &canNum, 0);
            uint32_t id;
            uint8_t msg[64];
            unsigned int i;

            if (err) {
                printf("Invalid or missing can bus\n");
                break;
            }
            if (canNum >= NUM_CAN_BUSSES) {
                printf("Invalid can bus: %d\n", canNum);
                break;
            }

            err = parse_uint8(&afterCommand, &priority, 0);
            if (err) {
                printf("Invalid or missing priority\n");
                break;
            }

            err = parse_uint8(&afterCommand, &type, 0);
            if (err) {
                printf("Invalid or missing type\n");
                break;
            }

            err = parse_uint32(&afterCommand, &id, 0);
            if (err) {
                printf("Invalid or missing id\n");
                break;
            }

            err = parse_uint8(&afterCommand, &dest, 0);
            if (err) {
                printf("Invalid or missing dest\n");
                break;
            }

            for (i = 0; ; i++) {
                uint8_t v;

                err = parse_uint8(&afterCommand, &v, 0);
                if (err == -1)
                    break;
                if (err) {
                    printf("Invalid message byte %d\n", i);
                    i = 65;
                    break;
                }
                if (i > 64) {
                    printf("Message too long, 64 bytes max\n");
                    break;
                }
                msg[i] = v;
            }
            if (i > 64)
                break;
            if (!CANSend(canNum, priority, type, id, dest, msg, i))
                printf("Error sending CAN message.\n");
            break;
        }

        case SetCANLoopback: {
            uint16_t canNum;
            int err = parse_uint16(&afterCommand, &canNum, 0);
            bool enable;

            err |= parse_bool(&afterCommand, &enable);

            if (err) {
                printf("Invalid format: set can loopback <num> 0|1\n");
                break;
            }
            if (canNum >= NUM_CAN_BUSSES) {
                printf("Invalid can bus: %d\n", canNum);
                break;
            }

            CANEnableLoopback(canNum, enable);
            printf("CAN loopback %s\n", enable ? "enabled" : "disabled");
            break;
        }

        case TraceCAN: {
            extern bool trace_can;
            bool enable;
            int err = parse_bool(&afterCommand, &enable);

            if (err) {
                printf("Must specify on/off to enable/disable CAN trace\n");
                break;
            }

            trace_can = enable;
            printf("CAN tracing %s\n", enable ? "enabled" : "disabled");
            break;
        }

        case GetCANCounts: {
            uint16_t canNum;
            int err = parse_uint16(&afterCommand, &canNum, 0);
            struct can_counts c;

            if (err) {
                printf("No can bus number given\n", canNum);
                break;
            }
            if (canNum >= NUM_CAN_BUSSES) {
                printf("Invalid can bus: %d\n", canNum);
                break;
            }

            CANGetCounts(canNum, &c);
            printf(" RX messages: %u\n", c.rx_msgs);
            printf(" RX long messages: %u\n", c.rx_long_msgs);
            printf(" RX messages lost: %u\n", c.rx_msgs_lost);
            printf(" RX invalid start seq: %u\n", c.rx_msgs_invalid_start_seq);
            printf(" RX invalid long msg: %u\n", c.rx_msgs_unsupported_long_msg);
            printf(" RX sequence mismatch: %u\n", c.rx_msgs_sequence_mismatch);
            printf(" RX msg too long: %u\n", c.rx_msgs_long_msg_too_long);
            printf(" RX new msg in msg: %u\n", c.rx_msgs_new_source_in_long_msg);
            printf(" RX bit stuff: %u\n", c.rx_err_bit_stuff_count);
            printf(" RX form: %u\n", c.rx_err_form_count);
            printf(" RX crc: %u\n", c.rx_err_crc_count);

            printf(" TX message: %u\n", c.tx_msgs);
            printf(" TX long : %u\n", c.tx_long_msgs);
            printf(" TX bit0: %u\n", c.tx_err_bit0_count);
            printf(" TX bit1: %u\n", c.tx_err_bit1_count);
            printf(" TX no ack: %u\n", c.tx_err_no_ack_count);
            break;
        }
#endif
        case GetCommands: {
            extern uint8_t SWCmdRing[SW_CMD_RING_SIZE];
            int i = 0;

            printf("Commands received since reset: Hw=%d,Sw=%d\n\r",
                   GetHWCmdCount(), GetSWCmdCount());

            printf("\n\rSoftware Commands received:\n\r");
            for (i = 0; i < SW_CMD_RING_SIZE; i++){
                printf("%d/%d ", SWCmdRing[i]>>5, SWCmdRing[i] & 0x1f);
            }

            printf("\n\r");
            break;
        }

        case TestMemScrub:{
            ScrubECCMemory((uint64_t *) 0x08000000, 0x20000);
            break;
        }

        case Freq: {
            AX5043Device dev;
            int err = parse_devnum_noerr(&afterCommand, &dev, 0);
            uint32_t val, freq;
            unsigned int i;
            char *t;

            if (err) {
                for (i = 0; i < NUM_CHANNELS; i++)
                    printf("%s%d--MRAM: %d, Memory: %d\n",
                           is_tx_chan(i) ? "Tx" : "Rx", i,
                           ReadMRAMFreq(i), DCTFreq[i]);
                break;
            }

            t = next_token(&afterCommand);
            if (!t) {
                printf("%s%d--MRAM: %d Memory: %d\n", dev,
                       is_tx_chan(dev) ? "Tx" : "Rx", dev,
                       ReadMRAMFreq(dev), DCTFreq[dev]);
                break;
            }

            freq = DCTFreq[dev];
            
            if (*t == '+') {
                t++;
                err = parse_freq(&t, &val);
                if (err) {
                    printf("Invalid value\n");
                    break;
                }
                freq += val;
            } else if (*t == '-') {
                t++;
                err = parse_freq(&t, &val);
                if (err) {
                    printf("Invalid value\n");
                    break;
                }
                freq -= val;
            } else {
                err = parse_freq(&t, &freq);
                if (err) {
                    printf("Invalid value\n");
                    break;
                }
            }
            DCTFreq[dev] = freq;
            quick_setfreq(dev, DCTFreq[dev]);

            printf("Set %s%d=%d\n", dev,
                   is_tx_chan(dev) ? "Tx" : "Rx", dev, DCTFreq[dev]);
            break;
        }

        case SaveFreq: {
            int i;

            for (i = 0; i < NUM_AX5043_RX_DEVICES; i++) {
                printf("Saving %s%d frequency %d to MRAM\n",
                       is_tx_chan(i) ? "Tx" : "Rx", i, DCTFreq[i]);
                WriteMRAMFreq(i, DCTFreq[i]);
            }
            break;
        }

        case TxPow: {
            AX5043Device dev;
            uint8_t power;
            int err = parse_devnum(&afterCommand, &dev, 0);

            if (err)
                break;
            err = parse_uint8(&afterCommand, &power, 0);
            if (err) {
                power = get_tx_power(dev);
                printf("dev%u power = %d%%\n", dev, power);
                break;
            }

            set_tx_power(dev, power);
            printf("dev%u power set to %d%%\n", dev, power);
            break;
        }

        case initSaved: {
            // Init this thing from scratch (address size, data size,
            // partitions etc).
            initMRAM(true);
            IHUInitSaved(); //Init stuff that we won't want to change on reboot
            SetupMRAM();    //Init stuff that do change (epoch number etc)
            break;
        }

        case LoadKey: {
            uint8_t key[AUTH_KEY_SIZE], i;
            uint32_t magic = ENCRYPTION_KEY_MAGIC_VALUE, checksum;
            const MRAMmap_t *LocalFlash = 0;
            bool stat;

            for (i = 0; i < sizeof(key); i++) {
                if (!parse_uint8(&afterCommand, &key[i], 16)) {
                    printf("Not enough numbers or invalid number on item %d",
                           i);
                    break;
                }
            }
            if (i < sizeof(key))
                break;

            checksum = key_checksum(key);
            printf("\n");
            if (i == sizeof(key)) {
                printf("Writing key...");
                stat = writeNV(key, sizeof(LocalFlash->AuthenticateKey.key),
                               NVConfigData,
                               (int) &LocalFlash->AuthenticateKey.key);
            } else {
                stat = false;
            }
            if (stat) {
                printf("Writing checksum=%x...",checksum);
                stat = writeNV(&checksum,
                               sizeof(LocalFlash->AuthenticateKey.keyChecksum),
                               NVConfigData,
                               (int) &LocalFlash->AuthenticateKey.keyChecksum);
            }
            if (stat) {
                printf("Writing valid\n");
            } else {
                magic = 0;
                printf("Invalidating stored key\n");
            }
            stat = writeNV(&magic, sizeof(LocalFlash->AuthenticateKey.magic),
                           NVConfigData,
                           (int) &LocalFlash->AuthenticateKey.magic);
            break;
        }

        case MountFS: {
            if (red_mount("/") == -1) {
                printf("Unable to mount filesystem: %s\n",
                       red_strerror(red_errno));
            } else {
                printf("Filesystem mounted\n");
            }
            break;
        }

        case UnMountFS: {
            if (red_umount("/") == -1) {
                printf("Unable to unmount filesystem: %s\n",
                       red_strerror(red_errno));
            } else {
                printf("Filesystem unmounted\n");
            }
            break;
        }

        case FormatFS: {
            if (red_format("/") == -1) {
                printf("Unable to format filesystem: %s\n",
                       red_strerror(red_errno));
            } else {
                printf("Filesystem formatted\n");
            }
            break;
        }

        case LsFS: {
            REDDIR *pDir;
            char *t = next_token(&afterCommand);

            printf("%s:\n", t);
            printf("Name       Blks  Size \n");
            printf("---------- ----- --------\n");
            if (strlen(t) == 0){
                pDir = red_opendir("//");
            } else {
                pDir = red_opendir(t);
            }
            if (pDir == NULL) {
                printf("Unable to open dir: %s\n", red_strerror(red_errno));
                printf("Make sure the path is fully qualified and starts with //\n");
                break;
            }

            uint32_t total_bytes = 0;
            uint32_t total_blocks = 0;
            REDDIRENT *pDirEnt;

            /*
             * Set error to zero so we can distinguish between a real
             * error and the end of the DIR,
             */
            red_errno = 0;
            pDirEnt = red_readdir(pDir);
            while (pDirEnt != NULL) {
                printf("%10s %5d %8d\n", pDirEnt->d_name,
                       pDirEnt->d_stat.st_blocks,
                       (int) pDirEnt->d_stat.st_size);
                total_bytes += pDirEnt->d_stat.st_size;
                total_blocks += pDirEnt->d_stat.st_blocks;
                pDirEnt = red_readdir(pDir);
            }
            if (red_errno != 0) {
                printf("Error reading directory: %s\n",
                       red_strerror(red_errno));
            }

            int32_t rc = red_closedir(pDir);
            if (rc != 0) {
                printf("Unable to close dir: %s\n", red_strerror(red_errno));
            }
            printf("Total File Blocks: %d Bytes: %d\n",
                   total_blocks, total_bytes);

            // Now check and print the available disk space
            REDSTATFS redstatfs;
            rc = red_statvfs("/", &redstatfs);
            if (rc != 0) {
                printf("Unable to check disk space with statvfs: %s\n",
                       red_strerror(red_errno));
            } else {
                printf("Free blocks: %d of %d.  Free Bytes: %d\n",
                       redstatfs.f_bfree, redstatfs.f_blocks,
                       redstatfs.f_frsize * redstatfs.f_bfree);
                printf("Available File Ids: %d of %d.  \n",
                       redstatfs.f_ffree, redstatfs.f_files);
            }
            break;
        }

        case RmFS:
        {
            char *t = next_token(&afterCommand);

            if (!t || strlen(t) == 0) {
                printf("Usage: rm <file name with path>\n");
                break;
            }

            int32_t fp = red_unlink(t);

            if (fp == -1) {
                printf("Unable to remove file %s : %s\n",
                       t, red_strerror(red_errno));
            }
            break;
        }

        case SelectRFPowerLevels:
        {
            uint16_t args[4];

            args[0] = parseNumber(&afterCommand);
            args[1] = parseNumber(&afterCommand);
            args[2] = parseNumber(&afterCommand);
            args[3] = parseNumber(&afterCommand);

            printf("Select %s power for safe, %s power for normal on primary\n",
                   args[0] ? "HIGH" : "LOW", args[1] ? "HIGH" : "LOW");
            printf("Select %s power for safe, %s power for normal on secondary\n",
                   args[2] ? "HIGH" : "LOW", args[3] ? "HIGH" : "LOW");
            SimulateSwCommand(SWCmdNSSpaceCraftOps,SWCmdOpsSelectDCTRFPower,args,4); //Send to others
            break;
        }
        case SetDCTDrivePower:
        {
            uint16_t args[4];

            args[0] = parseNumber(&afterCommand);
            args[1] = parseNumber(&afterCommand);
            args[2] = parseNumber(&afterCommand);
            args[3] = parseNumber(&afterCommand);
            printf("Set primary low power to %d, secondary low power to %d\n",
                   args[0], args[1]);
            printf("Set primary high power to %d, secondary high power to %d\n",
                   args[2], args[3]);
            //SimulateSwCommand(SWCmdNSTelemetry,SWCmdTlmDCTDrivePwr,args,4);
            break;
        }

        case showDownlinkSize:{
#define memberSize(type, member) sizeof(((type *)0)->member)
            printf("\nPayload Sizes: Header=%d,"
                   "RTHealth=%d (common=%d,common2=%d,specific=%d),"
                   "\nmin=%d,max=%d,WODHealth=%d\n",
                   sizeof(header_t), sizeof(realTimePayload_t),
                   sizeof(commonRtMinmaxWodPayload_t),
                   sizeof(commonRtWodPayload_t),
                   sizeof(realtimeSpecific_t),
                   sizeof(minValuesPayload_t),
                   sizeof(maxValuesPayload_t),
                   sizeof(WODHousekeepingPayload_t));

            printf("Frame sizes: \n"
                    "      Payload Only,   Current Filler   Current Size\n");

            printf("RT1        %03d           \n",
#if 0
                   "RT2        %03d          \n",
                   sizeof(realTimeFrame_t),
#endif
                   sizeof(realTimeFrame_t));
#if 0
            sizeof(realTimeMinFrame_t) - memberSize(realTimeMinFrame_t, filler),
            memberSize(realTimeMinFrame_t, filler), sizeof(realTimeMinFrame_t),
            sizeof(realTimeMaxFrame_t) - memberSize(realTimeMaxFrame_t, filler),
            memberSize(realTimeMaxFrame_t, filler), sizeof(realTimeMaxFrame_t)
#endif
            printf("AllWOD1    %03d       \n",
                   sizeof(allWOD1Frame_t));
#if 0
            sizeof(allWOD1Frame_t) - memberSize(allWOD1Frame_t, filler),
            memberSize(allWOD1Frame_t, filler), sizeof(allWOD1Frame_t),
            sizeof(allWOD2Frame_t) - memberSize(allWOD2Frame_t, filler),
            memberSize(allWOD2Frame_t, filler), sizeof(allWOD2Frame_t),
            sizeof(allWOD3Frame_t) - memberSize(allWOD3Frame_t, filler),
            memberSize(allWOD3Frame_t, filler), sizeof(allWOD3Frame_t)
#endif
            printf("SafeData1  %03d       \n"
                   "SafeWOD    %03d      \n",
                   sizeof(safeData1Frame_t),
                   sizeof(safeWODFrame_t));
#if 0
            sizeof(safeData1Frame_t) - memberSize(safeData1Frame_t, filler),
            memberSize(safeData1Frame_t, filler), sizeof(safeData1Frame_t),
            sizeof(safeWODFrame_t) - memberSize(safeWODFrame_t, filler),
            memberSize(safeWODFrame_t, filler), sizeof(safeWODFrame_t)
#endif

            break;
        }

        case mramSleep: {
            int num = parseNumber(&afterCommand);

            MRAMSleep(num);
            break;
        }

        case mramAwake: {
            int num = parseNumber(&afterCommand);

            MRAMWake(num);
            break;
        }

        case doClearMRAM:{
            SetupMRAM();
            // Don't get confused by in orbit state!
            WriteMRAMBoolState(StateInOrbit, true);
            break;
        }

        case ignoreUmb:
            //OverrideUmbilical(true);
            break;

        case noticeUmb:
            //OverrideUmbilical(false);
            break;

        case testLED: {
            GPIOSetOn(LED1);
            GPIOSetOn(LED2);
            vTaskDelay(SECONDS(2));
            GPIOSetOff(LED1);
            vTaskDelay(SECONDS(2));
            GPIOSetOn(LED1);
            GPIOSetOff(LED2);
            vTaskDelay(SECONDS(2));
            GPIOSetOff(LED1);
            break;
        }

        case Tx: {
            bool val;
            int err = parse_bool(&afterCommand, &val);

            if (!err) {
                if (val) {
                    SimulateHWCommand(CMD_ALL_ON);
                    inhibitTransmit = false;
                } else {
                    SimulateHWCommand(CMD_TX_OFF);
                    inhibitTransmit = true;
                }
            }
            printf("Transmit is %s\n",
                   inhibitTransmit ? "disabled" : "enabled");
            break;
        }

        case EnableComPr:
            EnableCommandPrint(true);
            break;

        case DisableComPr:
            EnableCommandPrint(false);
            break;

        case healthMode:{
            SimulateSwCommand(SWCmdNSSpaceCraftOps,SWCmdOpsHealthMode,
                              NULL, 0);
            break;
        }

        case readMRAMsr: {
            int i;

            for (i = 0; i < PACSAT_MAX_MRAMS; ) {
                printf("MRAM%d: status %x", i, readMRAMStatus(i));
                i++;
                if(i%2 == 0){
                    printf("\n");
                } else {
                    printf(", ");
                }
            }
            break;
        }

        case writeMRAMsr: {
            uint8_t stat = parseNumber(&afterCommand);
            int i;

            for (i = 0; i < PACSAT_MAX_MRAMS; i++){
                writeMRAMStatus(i,stat);
            }
            break;
        }

        case internalWDTimeout: {
            ForceInternalWatchdogTrigger();
            break;
        }

        case externalWDTimeout: {
            ForceExternalWatchdogTrigger();
            break;
        }

        case pollI2c: {
            I2CDevicePoll();
            break;
        }

        case getI2cState: {
            printf("I2c device state: (1 is ok)\n"
                    "   PacSat CPU Temp: %d, Tx Temp: %d    RTC: %d\n",
                    CpuTempIsOk(),TxTempIsOk(),RTCIsOk());
            break;
        }

        case telem0: {
            DisplayTelemetry(0);
            break;
        }

        case startRx:{
            AX5043Device dev;
            int err = parse_devnum(&afterCommand, &dev, 1);

            if (err)
                break;
            ax5043StartRx(dev);
            break;
        }

        case stopRx: {
            AX5043Device dev;
            int err = parse_devnum(&afterCommand, &dev, 1);

            if (err)
                break;
            ax5043StopRx(dev);
            break;
        }

        case SwVersion:{
            printID();
            break;
        }

        case time: {
            logicalTime_t time;

            getTime(&time);
            printf("IHU time:  Resets=%i,seconds=%i\n",
                   time.IHUresetCnt,time.METcount);
            getTimestamp(&time);
            printf("Timestamp time:  Epoch=%i,seconds=%i\n",
                   time.IHUresetCnt, time.METcount);
            printf("Poweron Time since preflight: %d seconds\n",
                   getSecondsInOrbit());
            printf("Unix time in secs: %d\n", getUnixTime());
            if (!time_valid)
                printf("***Unix Time is not valid\n");

            printf("Short boot count: %d, short boot flag %d\n\r",
                   SaveAcrossReset.fields.earlyResetCount,
                   SaveAcrossReset.fields.wasStillEarlyInBoot);
            break;
        }
        case preflight: {
            // We will take Any_Node to mean send to everyone
            PreflightInitNow(Any_Node);
            break;

        }
        case startWD: {
            SWISetWDTimer();
            dwdReset();
            SWIStartWatchdog();
            break;
        }
        case reset: {
            vTaskDelay(CENTISECONDS(50));
            ProcessorReset();
            break;
        }

        case RfPowPrint: {
            bool enable;
            int err = parse_bool(&afterCommand, &enable);

            if (err) {
                printf("rf power print is %s\n", print_rf_power ? "on" : "off");
                break;
            }
            print_rf_power = enable;
            printf("Set rf power print %s\n", enable ? "on" : "off");
            break;
        }

        case getTemp: {
#ifdef BLINKY_HARDWARE
            uint8_t temp8;
            if(Get8BitTemp31725(CpuTemp,&temp8)){
                printf("Cpu temp: ");
                print8BitTemp(temp8);
            } else {
                printf("CPU Temp request failed\n");
            }
            if(Get8BitTemp31725(TxTemp,&temp8)){
                printf("  Transmitter temp: ");
                print8BitTemp(temp8);
                printf("\n");
            } else {
                printf("\nTransmitter temp request failed\n");
            }
#elif defined AFSK_HARDWARE
            printf("CPU temp: %d\n", board_temps[TEMPERATURE_VAL_CPU]);
            printf("PA temp: %d\n", board_temps[TEMPERATURE_VAL_PA]);
            printf("Power temp: %d\n", board_temps[TEMPERATURE_VAL_POWER]);
#else
            printf("\nNo temperature available\n");
#endif
            break;
        }

        case getVoltages: {
#ifdef AFSK_HARDWARE
            printf("3.3V: %d\n", board_voltages[VOLTAGE_VAL_3v3]);
            printf("1.2V: %d\n", board_voltages[VOLTAGE_VAL_1v2]);
            printf("5V: %d\n", board_voltages[VOLTAGE_VAL_5v]);
            printf("Battery: %d\n", board_voltages[VOLTAGE_VAL_BATTERY]);
#else
            printf("\nNo voltages available\n");
#endif
            break;
        }

        case getPowerFlags: {
#ifdef AFSK_HARDWARE
            printf("5V: %d\n", board_power_flags[POWER_FLAG_5V]);
            printf("LNA: %d\n", board_power_flags[POWER_FLAG_LNA]);
            printf("SSPA: %d\n", board_power_flags[POWER_FLAG_SSPA]);
            printf("AX5043: %d\n", board_power_flags[POWER_FLAG_AX5043]);
#else
            printf("\nNo power flags available\n");
#endif
            break;
        }

        case BoardVersion: {
            printf("Board version: %d\n", board_version);
            break;
        }

        case testAX5043: {
            int i;

            for (i = 0; i < NUM_AX5043_SPI_DEVICES; i++)
                ax5043Test((AX5043Device)i);
            break;
        }

        case getax5043: {
            AX5043Device dev;
            int err = parse_devnum(&afterCommand, &dev, 0);

            if (err)
                break;
            ax5043Dump(dev);
            break;
        }

        case AxReg: {
            AX5043Device dev;
            int err = parse_devnum(&afterCommand, &dev, 0);
            uint16_t regstart;
            uint16_t regend;
            uint16_t i;
            uint8 val;

            if (err)
                break;

            err = parse_uint16_range(&afterCommand, &regstart, &regend, 0);
            if (err) {
                printf("Invalid start or end register\n");
                break;
            }

            err = parse_uint8(&afterCommand, &val, 0);
            if (err) {
                for (i = regstart; i <= regend; i++) {
                    unsigned int val = ax5043ReadReg(dev, i);

                    printf("ax5043 %d reg 0x%3.3x = %3d (0x%2.2x)\n",
                           dev, i, val, val);
                }
                break;
            }
            if (regstart != regend) {
                printf("Cannot set multiple registers\n");
                break;
            }

            ax5043WriteReg(dev, regstart, val);
            break;
        }

        case getRSSI: {
            AX5043Device dev;
            int err = parse_devnum(&afterCommand, &dev, 0);

            if (err)
                break;
            int rssi = get_rssi(dev);
            printf("AX5043 Dev: %d RSSI is %02x = %d dBm\n",
                   dev, rssi, ((int16_t)rssi) - 255);
            break;
        }

        case testFreq: {
            // This is so we can find what the receive frequency is on
            // first build
            AX5043Device dev;
            int err = parse_devnum(&afterCommand, &dev, 0);
            uint32_t freq;

            if (err)
                break;

            freq = DCTFreq[dev];

            printf("Testing TX for AX5043 Dev: %d\n", dev);
            test_freq(dev, freq, MODULATION_AFSK_1200, 0);
            break;
        }

        case testPLLrange: {
            AX5043Device dev;
            int err = parse_devnum(&afterCommand, &dev, 0);

            if (err)
                break;
            printf("Testing the PLL range for device: %d\n", dev);

            // test the range of the receiver on 2m
            test_pll_2m_range(dev, MODULATION_GMSK_9600, 0);
            break;
        }

#ifdef DEBUG
        /* G0KLA TEST ROUTINES
         * This is a subset of the self tests.  They do not cause a transmission
         * It does create a test file in the directory that needs to be removed manually
         */
        case testPacsat: {
            if (!pb_test_callsigns()) {
                debug_print("### Callsign TEST FAILED\n");
                break;
            }
            if (!test_ax25_util_decode_packet()) {
                debug_print("### Packet Decode TEST FAILED\n");
                break;
            }
            if (! pb_test_list()) {
                debug_print("### pb list TEST FAILED.  ** was PB Enabled?? Use 'open pb' to enable it ** \n");
                break;
            }
            if (!pb_clear_list()) {
                debug_print("### pb list clear TEST FAILED\n");
                break;
            }
            if (!tx_test_make_packet()) {
                debug_print("### tx make packet TEST FAILED\n");
                break;
            }
            if (!test_pfh()) {
                debug_print("### pfh TEST FAILED\n");
                break;
            }
            if (!test_pfh_file()) {
                debug_print("### pfh TEST FILE FAILED\n");
                break;
            }
            if (!test_ftl0_upload_table()) {
                debug_print("### FTL0 Upload Table TEST FAILED\n");
                break;
            }

            debug_print("### ALL TESTS PASSED\n");
            break;
        }

        case testCallsigns: {
            bool rc = pb_test_callsigns();
            break;
        }

        case testPbOk: {
            bool rc = pb_test_ok();
            break;
        }

        case testPbStatus: {
            pb_send_status();
            break;
        }

        case testPbList: {
            bool rc = pb_test_list();
            if (rc != TRUE)
                debug_print (".. was PB Enabled?? Use 'open pb' to enable it \n");
            break;
        }

        case testPbClearList: {
            bool rc = pb_clear_list();
            break;
        }

        case testTx: {
            bool rc = tx_test_make_packet();
            break;
        }

        case testPfh: {
            bool rc = test_pfh();
            break;
        }

        case testPfhFile: {
            bool rc = test_pfh_file();
            break;
        }

        case makePfhFiles: {
            bool rc = test_pfh_make_files();
            break;
        }

        case testDir: {
            bool rc = test_pacsat_dir();
            break;
        }

        case testDecode: {
            bool rc = test_ax25_util_decode_packet();
            break;
        }

        case sendUplinkStatus: {
            ax25_send_status();
            break;
        }

        case testRetransmission: {
            bool rc = test_ax25_retransmission();
            break;
        }

        case testUploadTable: {
            bool rc = test_ftl0_upload_table();
            break;
        }

        case listUploadTable: {
            bool rc = ftl0_debug_list_upload_table();
            break;
        }

#endif /* DEBUG */

        case Monitor: {
            bool val;
            int err = parse_bool(&afterCommand, &val);

            if (err) {
                printf("Packet monitor is %s\n", monitorPackets ? "on" : "off");
                break;
            }
            monitorPackets = val;
            printf("Setting packet monitor %s\n", val ? "on" : "off");
            break;
        }

        case pbShut: {
            WriteMRAMBoolState(StatePbEnabled, false);
            printf("PB SHUT\n");
            break;
        }

        case pbOpen: {
            WriteMRAMBoolState(StatePbEnabled, true);
            printf("PB OPEN\n");
            break;
        }

        case uplinkShut: {
            WriteMRAMBoolState(StateUplinkEnabled, false);
            printf("UPLINK SHUT\n");
            break;
        }

        case uplinkOpen: {
            WriteMRAMBoolState(StateUplinkEnabled, true);
            printf("UPLINK OPEN\n");
            break;
        }

        case digiShut: {
            WriteMRAMBoolState(StateDigiEnabled, false);
            printf("Digipeater Disabled\n");
            break;
        }

        case digiOpen: {
            WriteMRAMBoolState(StateDigiEnabled, true);
            printf("Digipeater Enabled\n");
            break;
        }

        case mkdirFS: {
            char *t = next_token(&afterCommand);

            if (!t && strlen(t) == 0) {
                printf("Usage: mkdir <file name with path>\n");
                break;
            }

            int32_t rc = red_mkdir(t);
            if (rc == -1) {
                printf("Unable make dir %s: %s\n", t, red_strerror(red_errno));
            }
            break;
        }

        case rmdirFS: {
            char *t = next_token(&afterCommand);

            if(!t || strlen(t)== 0){
                printf("Usage: rmdir <file name with path>\n");
                break;
            }

            int32_t rc = red_rmdir(t);

            if (rc == -1) {
                if (red_errno == RED_EBUSY)
                    printf("Unable remove dir that contains other files or directories: %s\n",
                           t, red_strerror(red_errno));
                else
                    printf("Unable remove dir %s: %s\n",
                           t, red_strerror(red_errno));
            }
            break;
        }


        case mramHxd: {
            char *t = next_token(&afterCommand);

            if (!t || strlen(t) == 0) {
                printf("Usage: hxd <file name with path>\n");
                break;
            }

            char read_buffer[512];
            int32_t fp = red_open(t, RED_O_RDONLY);

            if (fp != -1) {
                int32_t numOfBytesRead = red_read(fp, read_buffer,
                                                  sizeof(read_buffer));

                printf("Read returned: %d\n",numOfBytesRead);
                if (numOfBytesRead == -1) {
                    printf("Unable to read file: %s\n",
                           red_strerror(red_errno));
                } else {
                    int q;

                    for (q = 0; q < numOfBytesRead; q++) {
                        printf("%02x ", read_buffer[q]);
                        if (q != 0 && q % 20 == 0)
                            printf("\n");
                    }
                }

                int32_t rc = red_close(fp);
                if (rc != 0) {
                    printf("Unable to close file: %s\n",
                           red_strerror(red_errno));
                }
            } else {
                printf("Unable to open %s for reading: %s\n",
                       t, red_strerror(red_errno));
            }
            printf("\n");
            break;
        }

        case dirLoad:{
            bool rc = dir_load();
            break;
        }

        case dirClear:{
            dir_free();
            break;
        }

        case listDir:{
            // pass NULL to print from the head of the list
            dir_debug_print(NULL);
            break;
        }

        case getNextFileNumber: {
            uint32_t next_file_id = dir_next_file_number();
            printf("Next File Number from the Dir will be: %04x\n",
                   next_file_id);
            break;
        }

        case resetNextFileNumber: {
            WriteMRAMHighestFileNumber(0);
            printf("Next file number reset to zero\n");
            break;
        }

        case heapFree: {
            printf("Free heap size: %d\n",xPortGetFreeHeapSize());
            break;
        }

        case Time: {
            uint32_t t;
            int err = parse_uint32(&afterCommand, &t, 0);

            if (err) {
                printf("Unix time in secs: %d\n", getUnixTime());
                if (!time_valid)
                    printf("***Unix Time is not valid\n");
            } else {
                printf("Setting unix time to: %d\n", t);
                setUnixTime(t);
                time_valid = true;
                //printf("Setting RTC\n");
                if (RTCIsOk()) {
                    bool set = SetRtcTime31331(&t);
                    if (set) {
                        printf("Setting RTC\n");
                    } else {
                        printf("Failed to set RTC\n");
                    }
                }
            }
            break;
        }

        case getRtc: {
            bool rc = false;
            uint32_t time;

            rc = GetRtcTime31331(&time);
            if (rc == FALSE)
                printf(" Error, time unavailable\n");
            else {
                printf("Time: %d vs IHU Unix time %d\n", time, getUnixTime());
                if (!time_valid)
                    printf("***Unix time is not valid\n");
            }
            break;
        }

        case regRtc: {
            uint8_t readReg;
            int err = parse_uint8(&afterCommand, &readReg, 0);
            bool status;
            uint8_t data;

            if (err) {
                uint8_t data[8];

                readReg = 0;
                status = I2cSendCommand(MAX31331_PORT,MAX31331_ADDR,
                                        &readReg, 1, &data, 8);
                printf("Status=%d, reg values are: "
                       "%2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x\n",
                       status, data[0], data[1], data[2], data[3],
                       data[4], data[5], data[6], data[7]);
                break;
            }

            status = I2cSendCommand(MAX31331_PORT,MAX31331_ADDR,
                                    &readReg, 1, &data, 1);
            printf("Status=%d, reg %d value: 0x%x\n",
                   status,readReg,data);
            break;
        }

        case Modulation: {
            uint8_t devb;
            char *modstr;
            enum radio_modulation mod;
            int err;

            err = parse_uint8(&afterCommand, &devb, 0);
            if (err) {
                for (devb = 0; devb < InvalidAX5043Device; devb++) {
                    modstr = get_dev_modulation_str(devb);
                    printf("device %d: %s\n", devb, modstr);
                }
                break;
            }
            if (devb >= InvalidAX5043Device) {
                printf("Give a device number between 0 and %d\n",
                       NUM_AX5043_SPI_DEVICES - 1);
                break;
            }
            modstr = next_token(&afterCommand);
            if (!modstr) {
                modstr = get_dev_modulation_str(devb);
                printf("device %d: %s\n", devb, modstr);
                break;
            }

            if (strcmp(modstr, "1200") == 0) {
                mod = MODULATION_AFSK_1200;
            } else if (strcmp(modstr, "9600") == 0) {
                mod = MODULATION_GMSK_9600;
            } else {
                printf("Invalid modulation %s, must be 1200 or 9600\n",
                       modstr);
                break;
            }

            printf("Setting Radio %u to %s.\n", devb, modstr);

            WriteMRAMModulation(devb, mod);
            if (is_tx_chan(devb)) {
                /*
                 * Transmit task will set the mode as necessary so it
                 * doesn't change while transmitting.
                 */
                tx_modulation = mod;
            } else {
                ax5043StopRx((AX5043Device) devb);
                ax5043_ax25_set_modulation((AX5043Device) devb, mod, false);
                ax5043StartRx((AX5043Device) devb);
            }
            break;
        }

        case HelpAll: {
            char *t = next_token(&afterCommand);
            int i;

            if (!t || strlen(t) == 0){
                printf("\n***LIST OF ALL COMMANDS:\n");
                t = "";
            } else {
                printf("\n***LIST OF ALL COMMANDS CONTAINING %s:\n", t);
            }
            for (i = 0; command_tables[i].pairs; i++) {
                printf("\n%s:\n", command_tables[i].name);
                printHelp(t, command_tables[i].pairs,
                          command_tables[i].num_cmds);
            }
            break;
        }

        case Help:
        case HelpPacsat:
        case HelpSetup:
        case HelpDevo: {
            struct command_table *table = NULL;
            char *t = next_token(&afterCommand);

            if (index == Help)
                table = &command_tables[HELP_COMMON_INDEX];
            else if (index == HelpPacsat)
                table = &command_tables[HELP_PACSAT_INDEX];
            else if (index == HelpSetup)
                table = &command_tables[HELP_SETUP_INDEX];
            else if (index == HelpDevo)
                table = &command_tables[HELP_DEBUG_INDEX];

            if (!table)
                break;

            if (!t || strlen(t) == 0){
                printf("\n***LIST OF %s COMMANDS:\n\n", table->name);
                t = "";
            } else {
                printf("\n***LIST OF %s COMMANDS CONTAINING %s:\n\n",
                       table->name, t);
            }
            printHelp(t, table->pairs, table->num_cmds);
            break;
        }

        default:
            printf("Unknown command\n");
        }
    }
}
