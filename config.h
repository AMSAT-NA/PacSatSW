/*
 * config.h
 *
 *  Created on: Feb 26, 2019
 *      Author: burns
 *
 *      All configuration info is here.  For example task stack sizes, timer values, etc.
 *      If you want to change the way the software behaves, come here.
 *
 */

#ifndef GOLFCONFIG_H_
#define GOLFCONFIG_H_

#define PACSAT_NUMBER "0" // This is PacSat.
#define GOLF_ID (PACSAT_NUMBER[0] - '0') /*Must change if number greater than 9*/
//#define ENGINEERING_MODEL
#define UNDEFINE_BEFORE_FLIGHT
#define DEBUG

#if defined(UNDEFINE_BEFORE_FLIGHT)
//#define WATCHDOG_ENABLE
#define SOFTWARE_TYPE "X" // Always X if undefine_before_flight is on.  Do not change this one
#elif defined(ENGINEERING_MODEL)
#define WATCHDOG_ENABLE
#    define SOFTWARE_TYPE "E"
#else
#define WATCHDOG_ENABLE
#   define SOFTWARE_TYPE "V"
#endif
#define VERSION "1a" /*Exactly 2 characters will show in the diagnostic downlink*/
#define PACSAT_FW_VERSION_STRING SOFTWARE_TYPE PACSAT_NUMBER "." VERSION

#define PACSAT_MAX_MRAMS 4

////////////////////////////////////////////This is I/O
#define PRINTF_COM COM2
#define COM1_BAUD 38400
#define COM2_BAUD 38400
#ifdef DEBUG
#define DEBUG_PRINT
#endif

// This is for Golf-Tee when this IHU is a backup
#define THIS_IHU_IS_DEFAULT

#ifdef DEBUG_PRINT
#define debug_print printf
#else
#define debug_print //
#endif
/*
 * Lots of time definitions
 */

#define CENTISECONDS(HowMany) ((portTickType)((HowMany)*(10/portTICK_RATE_MS))) /* A Centisecond is 10 milliseconds */
#define SECONDS(HowMany) ((portTickType)((HowMany)*(1000/portTICK_RATE_MS)))
#define MINUTES(HowMany) SECONDS(60*(HowMany))
#define HOURS(HowMany) SECONDS(3600*(HowMany))

#define NO_TIMEOUT ((portTickType)0)
#define SHORT_WAIT_TIME SECONDS(2)
#define WATCHDOG_SHORT_WAIT_TIME SECONDS(5)
#define WATCHDOG_MAX_WAIT_TIME SECONDS(5)
#define I2C_TIMEOUT CENTISECONDS(150)
#define SPI_USER_TIMEOUT SECONDS(20) /*Fully expect a w/d timeout here */
#define SPI_MRAM_RETRIES 3
#define COORD_STATUS_TIMER SECONDS(5)
#define RETRY_TIMEOUT CENTISECONDS(50) /* If something feils we want to retry in 1/2 second*/
#define ANTENNA_DEPLOY_WAIT_TIME SECONDS(5)  /* Burn wires run for 5 seconds */
/*
 * For authenticated software commands the command will include a timestamp.  The timestamp is intended to
 * be the LAST SATELLITE TIME that the command will be accepted.  But we don't want it to be forever, so
 * MAX_AHEAD is the maximum number of seconds ahead of the satellite time that will be accepted.  MAX_BEFORE
 * is the maximum number of seconds behind the satellite time accepted.
 */

#define SECONDS_AUTHENTICATION_MAX_BEHIND 0
#define SECONDS_AUTHENTICATION_MAX_AHEAD 300

/*
 * PACSAT Constants
 */
/* If you wish to transmit beyond the test bench then these need to be updated with a valid callsign
 * In theory we could put these in MRAM and have a command to set them, but the Flight Unit should probably
 * have these stored permanently in flash */
#define BBS_CALLSIGN "VE2TCP-12"
#define BROADCAST_CALLSIGN "VE2TCP-11"
#define DIGI_CALLSIGN "VE2TCP-1"

#define NUM_OF_TX_CHANNELS 1 // The number of transmitters we have
#define NUM_OF_RX_CHANNELS 1 // The number of receivers we have - TODO this should tie to the Device numbers for AX5043s

/* Radio Channels */
typedef enum {
    Channel_A,
    Channel_B,
    Channel_C,
    Channel_D,
} rx_channel_t;

#define AX25_MAX_DATA_LEN 240 /* This is the maximum number of bytes a packet can have */
#define AX25_MAX_INFO_BYTES_LEN 223 /* This is the maximum number of info bytes a packet can have */
#define AX25_PKT_BUFFER_LEN 260 /* This is the length of the buffers in the TX RX queues */
#define MAX_CALLSIGN_LEN 10 /* Length of the String for an AX25 callsign including dash, 2 digit Digi, and null termination */
#define MAX_PB_HOLES_LIST_BYTES 222 /* The max length of a holes list = ( AX25_MAX_DATA_LEN - 17 ) to nearest 6 */
#define MAX_FILENAME_WITH_PATH_LEN 25 /* Max length of a filename with its path.  This requires a shallow dir structure. */

#define PB_MAX_PERIOD_FOR_CLIENTS_IN_SECONDS 600  // TODO - Should be in MRAM and commandable.  10 mins
#define MAX_PKTS_IN_TX_PKT_QUEUE_FOR_TNC_TO_BE_BUSY 2 // TODO - Should be in MRAM and commandable. 2

/* T1 is the timeout for outstanding I frame or P bit.  Traditionally set to the Smoothed Rountrip Time (SRT), which is
 * calculated for the channel.  For a full duplex sat this is perhaps not appropriate.
 * TODO - what is the optimal value for this? */
#define AX25_TIMER_T1_PERIOD SECONDS(3)
#define AX25_TIMER_T3_PERIOD SECONDS(30) /* Idle timeout if nothing heard */
#define AX25_RETRIES_N2 10 /* Number of retries permitted by the Data Link State Machine */
/*
 * TASK INFORMATION This is the info like stack sizes and priorities for the tasks
 */
#define CONSOLE_STACK_SIZE configMINIMAL_STACK_SIZE*11
#define CONSOLE_PRIORITY (tskIDLE_PRIORITY + 1)

#define COMMAND_STACK_SIZE configMINIMAL_STACK_SIZE*15
#define COMMAND_PRIORITY (tskIDLE_PRIORITY + 1)

#define TELEMETRY_STACK_SIZE configMINIMAL_STACK_SIZE*15//*6
#define TELEMETRY_PRIORITY (tskIDLE_PRIORITY + 1)

#define COORD_STACK_SIZE configMINIMAL_STACK_SIZE*11//*6
#define COORD_PRIORITY (tskIDLE_PRIORITY + 1)

#define CAN_STACK_SIZE configMINIMAL_STACK_SIZE*15//*6
#define CAN_PRIORITY (tskIDLE_PRIORITY + 1)

#define DOWNLINK_STACK_SIZE configMINIMAL_STACK_SIZE*6
#define DOWNLINK_PRIORITY (tskIDLE_PRIORITY + 1)

#define EXPERIMENT_STACK_SIZE configMINIMAL_STACK_SIZE*6
#define EXPERIMENT_PRIORITY (tskIDLE_PRIORITY + 2)

#define RADIO_STACK_SIZE configMINIMAL_STACK_SIZE*11
#define RADIO_PRIORITY (tskIDLE_PRIORITY + 2)

// PACSAT Task stack sizes and priorities
#define RX_STACK_SIZE configMINIMAL_STACK_SIZE*11
#define RX_PRIORITY (tskIDLE_PRIORITY + 4)
#define RX_PACKET_QUEUE_LEN 5
#define RX_EVENT_QUEUE_LEN 5

#define TX_STACK_SIZE configMINIMAL_STACK_SIZE*11
#define TX_PRIORITY (tskIDLE_PRIORITY + 3)
#define TX_PACKET_QUEUE_LEN 5

#define AX25_STACK_SIZE configMINIMAL_STACK_SIZE*11
#define AX25_PRIORITY (tskIDLE_PRIORITY + 2)

#define PB_STACK_SIZE configMINIMAL_STACK_SIZE*11
#define PB_PRIORITY (tskIDLE_PRIORITY + 2)
#define PB_PACKET_QUEUE_LEN 5

#define UPLINK_STACK_SIZE configMINIMAL_STACK_SIZE*11
#define UPLINK_PRIORITY (tskIDLE_PRIORITY + 2)
#define UPLINK_PACKET_QUEUE_LEN 5


// Telemetry constants
#define SYNC_LFSR_LONG (0x47cd215d)
#define SYNC_LFSR_LONG_BITS 31
#define SYNC10b0 (0xfa) // 8b10b sync.
#define SYNC10b1 (0x305)
#ifdef UNDEFINE_BEFORE_FLIGHT
#define IDLE_TIMEOUT (SECONDS(20))
#define SCIENCE_TIMEOUT (MINUTES(2))
#else
#define SCIENCE_TIMEOUT (MINUTES(15):
#define IDLE_TIMEOUT (MINUTES(2))
#endif
#define BEACON_TIMEOUT (CENTISECONDS(1550)) /* Add some time to make up for wait time for carrier frequency stabilization */
#define SHORT_BEACON_TIMEOUT (SECONDS(10))
#define BEACON_SILENCE (CENTISECONDS(250)) /* Silent carrier at start of a beacon--give time to reach right freq */
#define FRAMES_IN_SAFE_MODE 2


/*
 * Downlink timing and modulation constants
 */

/*
 * Rough value of TXPWRCOEFFB to power output of 5043 and PA
 * Note that PA has a power restriction right now, so higher values are not yet known.
 *
 * dBm  TXPWRCOEFFB PA Output dBm
 * -15  95          10.6
 * -14  108         12.3
 * -13  123         13.5
 * -12  139         14.8
 * -11  158         16.2
 * -10  179         17.2
 * -9   203         18.3
 * -8   230         19.3
 * -7   261         19.9
 * -6   297         20.6
 * -5   336         21.12
 * -4   382
 * -3   433
 * -2   491
 * -1   557
 *  0   632
 *
 */

/**
 * AX5043 radio constants
 */
#define AX5043_USES_TCXO // If this is not defined then we are using an XTAL
#define DCT_DEFAULT_TX_FREQ 435760000 // Actual frequency.  Converted later to ax5043 register value
#define DCT_DEFAULT_RX_FREQ 436800000 // The Flight frequency will be in 2m. e.g. 145835000
// For now, we want the output to be something like 100mW (20dBm) and 500mW (27dBm)
// I believe this makes the DCT output be about -7dBM and +3dBM
#define DCT_DEFAULT_LOW_POWER 261   // This seems about right for 20dBm
#define DCT_DEFAULT_HIGH_POWER 632 // todo:  This should be defined so we get about 27dBm out of the PA.
/*
 * MET Constants.  The MET timer goes off every second.  Other time constants
 * mask a counter incremented every 1 second.
 */
#define TELEM_COLLECT_SECONDS 4 /* TELEM_COLLECT_SECONDS and the mask below must match */
#define TELEM_COLLECT_MASK 2 /* Collect telemetry every time the bottom 2 bits are all set, i.e. every 4 seconds*/
#define HALF_DUPLEX_RX_TIME SECONDS(20)/*Time receiver is on if in half duplex mode */
#define MIN_RSSI_OK -150 /* If RSSI is lower than this, assume receiver is dead */
#ifdef UNDEFINE_BEFORE_FLIGHT
#define IN_ORBIT_UPDATE_PERIOD 30 /* Update the in-orbit time every 30 seconds */
#define CHECK_SCHEDULE_PERIOD 60
#define CHECK_ECLIPSE_PERIOD 5
#else
#define IN_ORBIT_UPDATE_PERIOD 30 /* Update the in-orbit time every 30 seconds */
#define CHECK_SCHEDULE_PERIOD 60
#define CHECK_ECLIPSE_PERIOD 30
#endif
#define MET_STABLE_TIME 30 /* How long after boot do we assume it takes for everything to stabilize? */
#define TIMEOUT_NONE 0xFFFFFFFF

#define MIN_MAX_CLEAR_SECONDS (3600*24*7) /* Once per week */

#ifdef UNDEFINE_BEFORE_FLIGHT
#define SW_COMMAND_TIME_TIMEOUT (120) /*If no commands for this long after enabling time check, disable it*/
#define NO_COMMAND_TIMEOUT (3600*24*14) /* If no commands for 2 weeks, do something */
#define MAX_DIAG_AGE 120 /* Two minutes for testing */

#else
#define SW_COMMAND_TIME_TIMEOUT (3600*24*2) /*If no commands for this long (2 days) after enabling time check, disable it*/
#define NO_COMMAND_TIMEOUT (3600*24*14) /* If no commands for 2 weeks, do something */
#define MAX_DIAG_AGE 900 /* Don't mark diag telemetry from other IHUs invalid for 15 minutes */

#endif
#define RESETS_BEFORE_POWER_CYCLE 5 /* How many "short" resets before we power cycle? */
#define DEFAULT_WOD_FREQUENCY 15 /* How many telemetry collections before we do a WOD save */
#define DEFAULT_NUM_WOD_SAVED 150 /* This is what we think the "ideal" size is based on simulations */
#define MAX_WOD_SAVED 600 /* This is the size of the arrays */

#ifdef UNDEFINE_BEFORE_FLIGHT
#define POST_LAUNCH_WAIT_TIME 2 /* Wait this long after we reach orbit before doing anything */
#else
#define POST_LAUNCH_WAIT_TIME 50  /* Specified in minutes--Wait this long after we reach orbit before doing anything */
#endif

/*
 * Default voltages
 *
 * These voltages are based on the CIU voltage divider and a/d converter so that 99 is 5.2V.
 *
 */
#define DEFAULT_AUTOSAFE_INTO 92
#define DEFAULT_AUTOSAFE_OUTOF 98
#define AUTOSAFE_MIN_HYSTERESIS 6
/*
 * The following is the ADC 8-bit solar panel voltage that will be deemed enough to consider us in the sun
 */
#define SOLAR_VOLTS_IN_SUN_MIN 125

#endif /* GOLFCONFIG_H_ */
