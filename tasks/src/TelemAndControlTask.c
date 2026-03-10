/*
 * TelemAndControlTask.c
 *
 *  Created on: Jun 5, 2023
 *      Author: Chris Thompson G0KLA / VE2TCP / AC2CZ
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */

#include <time.h>
#include "pacsat.h"
#include "MET.h"
#include "MRAMmap.h"
#include "nonvolManagement.h"
#include "redposix.h"
#include "PbTask.h"
#include "Ax25Task.h"
#include "RxTask.h"
#include "TxTask.h"
#include "pacsat_dir.h"
#include "UplinkTask.h"
#include "errors.h"
#include "radio.h"
#include "ax5043.h"
#include "inet.h"
#include "str_util.h"
#include "adc_proc.h"
#include "CANTask.h"
#include "TelemAndControlTask.h"
#include "exp_interface.h"
#include "CommandTask.h"

#ifdef BLINKY_HARDWARE
#include "Max31725Temp.h"
#endif

#include "downlink.h"
#include "rt1ErrorsDownlink.h"

typedef struct {
    header_t header;
    commonRtMinmaxWodPayload_t common;
    commonRtWodPayload_t common2;
    realtimeSpecific_t realTimeData;
    wodSpecific_t wodInfo;
    minValuesPayload_t minVals;
    maxValuesPayload_t maxVals;
    rt1Errors_t primaryErrors;
} telem_buffer_t;

realTimeFrame_t realtimeFrame;
WODFrame_t wodFrame;
errWODFrame_t errwodFrame;

/* Forward declarations */
void tac_pb_status_callback(TimerHandle_t xTimer);
void tac_ftl0_status_callback(TimerHandle_t xTimer);
void tac_telem_timer_callback(TimerHandle_t xTimer);
void tac_time_timer_callback(TimerHandle_t xTimer);
void tac_wod_save_timer_callback(TimerHandle_t xTimer);
void tac_adc_timer_callback(TimerHandle_t xTimer);
void tac_maintenance_timer_callback(TimerHandle_t xTimer);
void tac_check_file_queues_timer_callback(TimerHandle_t xTimer);
void tac_science_mode_timer_callback(TimerHandle_t xTimer);
void tac_stop_science_mode_timer();
void tac_collect_telemetry(telem_buffer_t *buffer);
void tac_send_telemetry(telem_buffer_t *buffer);
void tac_send_time();
void tac_store_wod();

/* Local variables */
char wod_file_name_with_path[MAX_FILENAME_WITH_PATH_LEN];

/* timer to send the telemetry periodically */
static xTimerHandle timerTelemSend;

/* timer to send the time periodically */
static xTimerHandle timerTimeSend;

/* timer to send the roll the WOD file periodically */
static xTimerHandle timerWodSave;

/* timer to perform maintenance periodically */
static xTimerHandle timerMaintenance;

/* timer to perform file queue checks periodically */
static xTimerHandle timerCheckFileQueues;

/* timer to send the PB status periodically */
static xTimerHandle timerPbStatus;

/* timer to read the ADC periodically */
static xTimerHandle timerADC;

/* timer to end science mode */
static xTimerHandle timerScienceMode;

/* timer to send the uplink status */
static xTimerHandle timerUplinkStatus;

// Storage used to send messages to the Telemetry and Control task
static Intertask_Message statusMsg;


/* A buffer to store telemetry values as they are collected.  We do not need to double buffer as they are copied to
 * a TX queue before being sent */
static telem_buffer_t telem_buffer;

/* Globals we need access to */
extern bool InSafeMode;
extern rt1Errors_t localErrorCollection;


/**
 * The Telemetry and Control task ...
 *
 */
portTASK_FUNCTION_PROTO(TelemAndControlTask, pvParameters)
{
    vTaskSetApplicationTaskTag((xTaskHandle) 0,
                               (pdTASK_HOOK_CODE)TelemetryAndControlWD);
    InitInterTask(ToTelemetryAndControl, 10);
    localErrorCollection.valid = 1;

    ResetAllWatchdogs();
//    debug_print("Initializing Telem and Control Task\n");


    /* Setup a timer to send the PB status periodically */

    /*
     * create a RTOS software timer - TODO period should be in MRAM
     * and changeable from the ground using xTimerChangePeriod()
     */
    timerPbStatus = xTimerCreate("PB STATUS", SECONDS(ReadMRAMPBStatusFreq()),
                                 TRUE, NULL,
                                 tac_pb_status_callback); // auto reload timer
    // Block time of zero as this can not block
    portBASE_TYPE timerStatus = xTimerStart(timerPbStatus, 0);
    if (timerStatus != pdPASS) {
        ReportError(RTOSfailure, FALSE, CharString,
                    (int)"ERROR: Failed in starting PB Status Timer");
        // TODO - it's possible this might fail.  Somehow we should
        // recover from that.
    }

    /* Setup a timer to send the uplink status periodically */
    /*
     * create a RTOS software timer - TODO period should be in MRAM
     * and changeable from the ground using xTimerChangePeriod()
     */
     timerUplinkStatus = xTimerCreate("UPLINK STATUS",
                                      ReadMRAMFTL0StatusFreq(), TRUE,
                                      NULL,
                                      tac_ftl0_status_callback);
     // Block time of zero as this can not block
     timerStatus = xTimerStart(timerUplinkStatus, 0);
     if (timerStatus != pdPASS) {
         ReportError(RTOSfailure, FALSE, CharString,
                     (int)"ERROR: Failed in starting Uplink Status Timer");
         debug_print("ERROR: Failed in init PB Status Timer\n");
         // TODO - it's possible this might fail.  Somehow we should
         // recover from that.
     }

    /* Create a periodic timer to send telemetry */
    timerTelemSend = xTimerCreate("TelemSend",
                                  ReadMRAMTelemFreq(), TRUE,
                                  NULL, tac_telem_timer_callback);
    // Block time of zero as this can not block
    timerStatus = xTimerStart(timerTelemSend, 0);
    if (timerStatus != pdPASS) {
        ReportError(RTOSfailure, FALSE, CharString,
                    (int)"ERROR: Failed in starting Telem Timer");
    }

    /* Create a periodic timer to send time */
    timerTimeSend = xTimerCreate("TimeSend",
                                  ReadMRAMTimeFreq(), TRUE,
                                  NULL, tac_time_timer_callback);
    // Block time of zero as this can not block
    timerStatus = xTimerStart(timerTimeSend, 0);
    if (timerStatus != pdPASS) {
        ReportError(RTOSfailure, FALSE, CharString,
                    (int)"ERROR: Failed in starting Send Time Timer");
    }


    /* Create a periodic timer to save to the WOD file */
    timerWodSave = xTimerCreate("WodSave",
                                ReadMRAMWODFreq(), TRUE,
                                  NULL, tac_wod_save_timer_callback);
    // Block time of zero as this can not block
    timerStatus = xTimerStart(timerWodSave, 0);
    if (timerStatus != pdPASS) {
        ReportError(RTOSfailure, FALSE, CharString,
                    (int)"ERROR: Failed in starting WOD Save Timer");
    }

    /* Create a periodic timer for maintenance */
    timerMaintenance = xTimerCreate("Maintenance",
                                    TAC_TIMER_MAINTENANCE_PERIOD, TRUE,
                                    NULL, tac_maintenance_timer_callback);
    // Block time of zero as this can not block
    timerStatus = xTimerStart(timerMaintenance, 0);
    if (timerStatus != pdPASS) {
        ReportError(RTOSfailure, FALSE, CharString,
                    (int)"ERROR: Failed in starting Maintenance Timer");
    }

    /* Create a periodic timer for file queues */
    timerCheckFileQueues = xTimerCreate("Check File Queues",
                                    TAC_TIMER_CHECK_FILE_QUEUES_PERIOD, TRUE,
                                    NULL, tac_check_file_queues_timer_callback);
    // Block time of zero as this can not block
    timerStatus = xTimerStart(timerCheckFileQueues, 0);
    if (timerStatus != pdPASS) {
        ReportError(RTOSfailure, FALSE, CharString,
                    (int)"ERROR: Failed in starting File Queue Check Timer");
    }

    /* Create a periodic timer for reading the ADC */
    timerADC = xTimerCreate("ADC",
                            TAC_TIMER_ADC_PERIOD, TRUE,
                            NULL, tac_adc_timer_callback);
    // Block time of zero as this can not block
    timerStatus = xTimerStart(timerADC, 0);
    if (timerStatus != pdPASS) {
        ReportError(RTOSfailure, FALSE, CharString,
                    (int)"ERROR: Failed in starting ADC Timer");
    }


    /*
     * After a short delay for things to settle, send the status to
     * indicate we have rebooted
     */
    vTaskDelay(WATCHDOG_SHORT_WAIT_TIME);
    if (getSpacecraftMode() == SpacecraftFileSystemMode)
        pb_send_status();

    ReportToWatchdog(TelemetryAndControlWD);

    /* Wait a bit longer and then send uplink status */
    vTaskDelay(WATCHDOG_SHORT_WAIT_TIME);
    if (getSpacecraftMode() == SpacecraftFileSystemMode)
        ax25_send_status();
    ReportToWatchdog(TelemetryAndControlWD);

    //TODO - include any checks needed here to make sure hardware is available
    // I2CDevicePoll(); // verify that ICR, CIU, etc are communicating over I2c
    //ReportToWatchdog(TelemetryAndControlWD);

    /*
     * When everything is settled after boot we can start collecting
     * telemetry.  We don't want uninitialized values to corrupt the
     * telemetry, especially WOD or min/max.
     */
    METTelemetryReady();

    CANRegisterReceiveHandler(CANA, exp_can_handler);  // bus index 0 = CANA, 1 = CANB

    while(1) {
        Intertask_Message messageReceived;
        int status;
        uint32_t now = 0;
        uint16_t mins = 0;
        portBASE_TYPE timerStatus;

        ReportToWatchdog(CurrentTaskWD);
        status = WaitInterTask(ToTelemetryAndControl,
                               WATCHDOG_SHORT_WAIT_TIME, &messageReceived);
        ReportToWatchdog(CurrentTaskWD);
        if (status == pdFAIL) {
            ReportError(RTOSfailure, false, ReturnAddr,
                        (int) TelemAndControlTask);
        } else {
            //int waiting=WaitingInterTask(ToTelemetryAndControl);
            //if(waiting != 0){
            //debug_print("MessagesWaiting=%d\n",
            //            WaitingInterTask(ToTelemetryAndControl));

            switch(messageReceived.MsgType) {
            case TacEnterSafeMode:
                if (getSpacecraftMode() != SpacecraftSafeMode) {
                    if (getSpacecraftMode() == SpacecraftScienceMode) {
                        tac_stop_science_mode_timer();
                    }
                    setSpacecraftMode(SpacecraftSafeMode);
                    debug_print("Entering SAFE mode\n");
                }
                break;
            case TacEnterFileSystemMode:
                if (getSpacecraftMode() != SpacecraftFileSystemMode) {
                    if (getSpacecraftMode() == SpacecraftScienceMode) {
                        tac_stop_science_mode_timer();
                    }
                    setSpacecraftMode(SpacecraftFileSystemMode);
                    debug_print("Entering FS mode\n");
                    pb_send_status();
                    ReportToWatchdog(TelemetryAndControlWD);
                    /* Wait a bit longer and then send uplink status */
                    vTaskDelay(WATCHDOG_SHORT_WAIT_TIME);
                    ax25_send_status();
                }
                break;
            case TacEnterScienceMode:
                if (getSpacecraftMode() != SpacecraftScienceMode) {
                    setLastSpacecraftMode(getSpacecraftMode());
                    setSpacecraftMode(SpacecraftScienceMode);
                    mins = messageReceived.data[0];
                    if (mins > TAC_MAX_EXPERIMENT_TIMEOUT_MINS)
                        mins = TAC_MAX_EXPERIMENT_TIMEOUT_MINS;
                    debug_print("Entering SCIENCE Mode with timeout: %d mins\n",mins);
                    /* Start the science mode timer */
                    timerScienceMode = xTimerCreate("Science Mode",
                                                    SECONDS(mins*60), FALSE,
                                                    NULL, tac_science_mode_timer_callback);
                    // Block time of zero as this can not block
                    timerStatus = xTimerStart(timerScienceMode, 0);
                    if (timerStatus != pdPASS) {
                        ReportError(RTOSfailure, FALSE, CharString,
                                    (int)"ERROR: Failed in starting Science Mode Timer");
                        setSpacecraftMode(getLastSpacecraftMode());
                    }
                }
                break;
            case TacEndScienceMode:
                debug_print("Telem & Control: Ending Science Mode\n");
                switch(getLastSpacecraftMode()) {
                case SpacecraftFileSystemMode:
                    statusMsg.MsgType = TacEnterFileSystemMode;
                    NotifyInterTaskFromISR(ToTelemetryAndControl, &statusMsg);
                    break;
                default:
                    statusMsg.MsgType = TacEnterSafeMode;
                    NotifyInterTaskFromISR(ToTelemetryAndControl, &statusMsg);
                    break;
                }
                break;
            case TacSendPbStatus:
                //debug_print("Telem & Control: Send the PB Status\n");
                if (getSpacecraftMode() == SpacecraftFileSystemMode)
                    pb_send_status();
                break;

            case TacUpdatePbTimer:
                if (timerPbStatus != NULL) {
                    timerStatus = xTimerChangePeriod(timerPbStatus, SECONDS(ReadMRAMPBStatusFreq()), pdMS_TO_TICKS(100));
                    if (timerStatus != pdPASS) {
                        ReportError(RTOSfailure, FALSE, CharString,
                                    (int)"ERROR: Failed to change Telem Timer period");
                    }
                }
                break;
            case TacSendUplinkStatus:
                //debug_print("Telem & Control: Send the FTL0 Status\n");
                if (getSpacecraftMode() == SpacecraftFileSystemMode)
                    ax25_send_status();
                break;

            case TacMaintenanceMsg:
                //debug_print("TAC: Running DIR Maintenance\n");
                dir_maintenance();
                //debug_print("TAC: Running FTL0 Maintenance\n");
                ftl0_maintenance();
                break;

            case TacCheckFileQueuesMsg:
                now = getUnixTime(); // Get the time in seconds since the unix epoch
                dir_file_queue_check(now, WOD_FOLDER, PFH_TYPE_WL, WOD_DESTINATION, DIR_MAX_WOD_FILE_AGE);
                dir_file_queue_check(now, TXT_FOLDER, PFH_TYPE_ASCII, TXT_DESTINATION, DIR_MAX_WOD_FILE_AGE);
                dir_file_queue_check(now, EXP_FOLDER, PFH_TYPE_CAN_PACKETS, EXP_DESTINATION, DIR_MAX_WOD_FILE_AGE);
                break;

            case TacCollectMsg:
                tac_collect_telemetry(&telem_buffer);
                break;

            case TacSendRealtimeMsg:
                if (getSpacecraftMode() != SpacecraftScienceMode) {
                    /*
                     * For real time telemetry we collect it and send it
                     * at the same time.  This is sent in both SAFE and File System
                     * Mode.
                     */
                    tac_collect_telemetry(&telem_buffer);
                    tac_send_telemetry(&telem_buffer);
                }
                break;

            case TacSendTimeMsg:
                if (getSpacecraftMode() == SpacecraftFileSystemMode)
                    tac_send_time();
                break;

            case TacSaveWodMsg:
                tac_store_wod();
                break;

            case TacADCStartMsg:
                adc_start_conversion();
                break;

            case TacADCProcessMsg:
                adc_process_data();
                break;
            }
        }
    }
}


/**
 * tac_pb_status_callback()
 *
 * This is called from a timer whenever the status of the PB should be
 * sent.  The actual status is assembled and sent to the TX by the
 * Telemetry and Control task
 *
 */
void tac_pb_status_callback(TimerHandle_t xTimer)
{
    statusMsg.MsgType = TacSendPbStatus;
    NotifyInterTaskFromISR(ToTelemetryAndControl, &statusMsg);
}

/**
 * tac_ftl0_status_callback()
 *
 * This is called from a timer whenever the status of the Uplink should be sent.  The actual status is assembled and
 * sent to the TX by the Telemetry and Control task
 *
 */
void tac_ftl0_status_callback(TimerHandle_t xTimer)
{
    statusMsg.MsgType = TacSendUplinkStatus;
    NotifyInterTaskFromISR(ToTelemetryAndControl, &statusMsg);
}

/**
 * tac_telem_timer_callback()
 *
 * This is called from a timer whenever the telemetry should be sent.
 */
void tac_telem_timer_callback(TimerHandle_t xTimer)
{
    statusMsg.MsgType = TacSendRealtimeMsg;
    NotifyInterTaskFromISR(ToTelemetryAndControl, &statusMsg);
}

/**
 * tac_time_timer_callback()
 *
 * This is called from a timer whenever the time should be sent.
 */
void tac_time_timer_callback(TimerHandle_t xTimer)
{
    statusMsg.MsgType = TacSendTimeMsg;
    NotifyInterTaskFromISR(ToTelemetryAndControl, &statusMsg);
}


/**
 * tac_wod_roll_timer_callback()
 *
 * This is called from a timer whenever the WOD file should be rolled.
 */
void tac_wod_save_timer_callback(TimerHandle_t xTimer)
{
    statusMsg.MsgType = TacSaveWodMsg;
    NotifyInterTaskFromISR(ToTelemetryAndControl, &statusMsg);
}

/**
 * tac_void_timer_callback()
 *
 * This is called from a timer whenever the ADC conversion process
 * needs to start.
 */
void tac_adc_timer_callback(TimerHandle_t xTimer)
{
    statusMsg.MsgType = TacADCStartMsg;
    NotifyInterTaskFromISR(ToTelemetryAndControl, &statusMsg);
}

/**
 * tac_maintenance_timer_callback()
 *
 * This is called from a timer whenever directory and upload table
 * maintenance should be run.
 */
void tac_maintenance_timer_callback(TimerHandle_t xTimer)
{
    statusMsg.MsgType = TacMaintenanceMsg;
    NotifyInterTaskFromISR(ToTelemetryAndControl, &statusMsg);
}

/**
 * tac_file_queue_check_timer_callback()
 *
 * This is called from a timer whenever the file queues should be checked
 */
void tac_check_file_queues_timer_callback(TimerHandle_t xTimer)
{
    statusMsg.MsgType = TacCheckFileQueuesMsg;
    NotifyInterTaskFromISR(ToTelemetryAndControl, &statusMsg);
}

/**
 * tac_science_mode_timer_callback()
 *
 * This is called from a timer whenever science mode ends
 */
void tac_science_mode_timer_callback(TimerHandle_t xTimer)
{
    statusMsg.MsgType = TacEndScienceMode;
    NotifyInterTaskFromISR(ToTelemetryAndControl, &statusMsg);
}


/**
 * tac_stop_science_mode_timer()
 *
 * Stop the science mode timer if it is running
 */
void tac_stop_science_mode_timer() {
    portBASE_TYPE timerStatus;

    // Stop the Science mode timer, but f this timer is not actually runnning we will crash. So check first.
    if (timerScienceMode != NULL && xTimerIsTimerActive(timerScienceMode) != pdFALSE) {
        timerStatus = xTimerStop(timerScienceMode, pdMS_TO_TICKS(100));
        if (timerStatus != pdPASS) {
            ReportError(RTOSfailure, FALSE, CharString,
                        (int)"ERROR: Failed to stop Science Mode Timer");
            // TODO - log this or perhaps try again with longer delay and then reboot if that fails.
        }
    }

}

void tac_collect_telemetry(telem_buffer_t *buffer)
{
    logicalTime_t time;

    //debug_print("Telem & Control: Collect RT telem\n");

    if (!IsStabilizedAfterBoot())
        return;

    getTimestamp(&time);
    buffer->header.uptime = htotl(time.METcount);
    buffer->header.resetCnt = htots(time.IHUresetCnt);
    buffer->header.protocolVersion = 0;
    buffer->header.versionMajor = DownlinkVersionMajor;
    buffer->header.versionMinor = DownlinkVersionMinor;
    buffer->header.spacecraftMode = spacecraftMode;

// debug_print("Telem & Control: Collect RT telem at: %d/%d\n",time.IHUresetCnt, time.METcount);

    /********** commonRtMinmaxWodPayload_t - These values also go into min / max ***********/
    // TODO - put these in MIN MAX.  Make sure Min Max initialized with preflight init or similar

    /* File Storage */
    REDSTATFS redstatfs;
    bool rc = red_statvfs("/", &redstatfs);
    if (rc != 0) {
        printf("TAC: Unable to check disk space with statvfs: %s\n",
               red_strerror(red_errno));
    } else {
        //printf("Free blocks: %d of %d.  Free Bytes: %d\n",
        //       redstatfs.f_bfree, redstatfs.f_blocks,
        //       redstatfs.f_frsize * redstatfs.f_bfree);
        //printf("Available File Ids: %d of %d.  \n",
        //       redstatfs.f_ffree, redstatfs.f_files);
        buffer->common.FSAvailable = htotl(redstatfs.f_bfree); // blocks free
        buffer->common.FSTotalFiles = htots((uint16)(redstatfs.f_files - redstatfs.f_ffree));
    }

    if (buffer->minVals.common.FSAvailable > buffer->common.FSAvailable)
        buffer->minVals.common.FSAvailable = buffer->common.FSAvailable;
    if (buffer->maxVals.common.FSAvailable < buffer->common.FSAvailable)
        buffer->maxVals.common.FSAvailable = buffer->common.FSAvailable;

    ReportToWatchdog(CurrentTaskWD);

    short upload_kb = (uint16_t)(ftl0_get_space_reserved_by_upload_table()/1024);
    buffer->common.UploadQueueBytes = htots(upload_kb); // in kilobytes
    //debug_print("UploadBytes: %d",upload_kb);
    buffer->common.UploadQueueFiles = ftl0_get_num_of_files_in_upload_table();

    /* TX Telemetry */
    uint16_t rf_pwr = (ax5043ReadReg(FIRST_TX_CHANNEL, AX5043_TXPWRCOEFFB0)
            + (ax5043ReadReg(FIRST_TX_CHANNEL, AX5043_TXPWRCOEFFB1) << 8));
    buffer->common.TXPower = rf_pwr;
    buffer->common.TXPwrMode = ax5043ReadReg(FIRST_TX_CHANNEL, AX5043_PWRMODE);

    /* RX0 Telemetry */
    uint8_t rssi = get_rssi(FIRST_RX_CHANNEL);
    buffer->common.RX0RSSI = rssi;
    buffer->common.RX0PwrMode = ax5043ReadReg(FIRST_RX_CHANNEL, AX5043_PWRMODE);
#if NUM_RX_CHANNELS == 4
    rssi = get_rssi(FIRST_RX_CHANNEL+1);
    buffer->common.RX1RSSI = rssi;
    buffer->common.RX1PwrMode = ax5043ReadReg(FIRST_RX_CHANNEL+1, AX5043_PWRMODE);
    rssi = get_rssi(FIRST_RX_CHANNEL+2);
    buffer->common.RX2RSSI = rssi;
    buffer->common.RX2PwrMode = ax5043ReadReg(FIRST_RX_CHANNEL+2, AX5043_PWRMODE);
    rssi = get_rssi(FIRST_RX_CHANNEL+3);
    buffer->common.RX3RSSI = rssi;
    buffer->common.RX3PwrMode = ax5043ReadReg(FIRST_RX_CHANNEL+3, AX5043_PWRMODE);
#endif

    ReportToWatchdog(CurrentTaskWD);

#ifdef BLINKY_HARDWARE
    uint8_t temp8;
    if (Get8BitTemp31725(CpuTemp, &temp8)) {
        buffer->common.IHUTemp = temp8;
    } else {
        //debug_print("TAC: ERROR I2C temp request failed\n");
    }
#endif
#ifdef AFSK_HARDWARE
    // TODO - these need to be in a suitable format for the telemetry.  e.g. 0-255 is -30 to 105.
    // The value in the array is in C and is signed
    printf("CPU temp: %d\n", board_temps[TEMPERATURE_VAL_CPU]);
    printf("PA temp: %d\n", board_temps[TEMPERATURE_VAL_PA]);
    printf("Power temp: %d\n", board_temps[TEMPERATURE_VAL_POWER]);

    buffer->common.IHUTemp = board_temps[TEMPERATURE_VAL_CPU];
    buffer->common.PATemp = board_temps[TEMPERATURE_VAL_PA];
    buffer->common.PowerTemp = board_temps[TEMPERATURE_VAL_POWER];
#else
    buffer->common.IHUTemp = 0;
#endif


    /********** commonRtWodPayload_t - These values are static values and not suitable to calculate min / max ***********/

    // TODO - all these MRAM vars can be cached in memory and not read over SPI for every telem check.  e,g, like spacecraftMode.
    buffer->common2.AutoSafeAllowed = ReadMRAMBoolState(StateAutoSafeAllow);
    buffer->common2.AutoSafeModeActive = ReadMRAMBoolState(StateAutoSafe);
    //debug_print("PB: %d\n", pb_state);
    buffer->common2.pbEnabled = ReadMRAMBoolState(StatePbEnabled);
    buffer->common2.uplinkEnabled = ReadMRAMBoolState(StateUplinkEnabled);
    buffer->common2.DigiEnabled = ReadMRAMBoolState(StateDigiEnabled);

    buffer->common2.LogLevel = 0; // TODO - implement when logging in place
    buffer->common2.TimePeriod = ReadMRAMTimeFreq();
    buffer->common2.TelemPeriod = ReadMRAMTelemFreq();
    buffer->common2.WodPeriod = ReadMRAMWODFreq();
    buffer->common2.MaxWodFileSize = ReadMRAMWODMaxFileSize();
    buffer->common2.MaxExpFileSize = ReadMRAMExpMaxFileSize();
    buffer->common2.PbStatusPeriod = ReadMRAMPBStatusFreq();
    buffer->common2.PbTimeout = ReadMRAMPBClientTimeout();
    buffer->common2.UplinkStatusPeriod = ReadMRAMFTL0StatusFreq();
    buffer->common2.TLMresets = 0; // TODO - implement with clearMinMax() function and command
    buffer->common2.swCmds = getCmdRingTelem();
    buffer->common2.swCmdCnt = GetSWCmdCount();
    buffer->common2.MRAMstatus = 0; // TODO connect this to status?  What is it?  Do we need 4?

    // Errors TODO - make sure that when these are written in error handling they were converted from host to little endian
    buffer->primaryErrors = localErrorCollection;

    // TODO - calculate min max and store in MRAM
}

/**
 * Send telemetry that we have collected over the last period.
 * Realtime telemetry is sent in a UI frame.  The frame type depends on the mode and the
 * sequence
 *
 * The telemetry to send is determined by the mode that we are in.
 *
 */
void tac_send_telemetry(telem_buffer_t *buffer)
{
    if (!IsStabilizedAfterBoot())
        return;

    int len = 0;
    uint8_t *frame;
    if (ReadMRAMBoolState(StateCommandedSafeMode)
                || ReadMRAMBoolState(StateAutoSafe)) {
        /* Setup Type 1 frame - This will copy the buffer into the frame */
        realtimeFrame.header = buffer->header;
        realtimeFrame.rtHealth.common = buffer->common;
        realtimeFrame.rtHealth.common2 = buffer->common2;
        realtimeFrame.rtHealth.realTimeData = buffer->realTimeData;
        len = sizeof(realtimeFrame);

        switch (spacecraftMode) {
            case SpacecraftSafeMode:
                debug_print("Telem & Control: Send SAFE telem at: %d/%d\n", ttohs(realtimeFrame.header.resetCnt), htotl(realtimeFrame.header.uptime));
                frame = (uint8_t *)&realtimeFrame;
                break;
            case SpacecraftFileSystemMode:
                debug_print("Telem & Control: Send HEALTH telem at: %d/%d\n", ttohs(realtimeFrame.header.resetCnt), htotl(realtimeFrame.header.uptime));
                frame = (uint8_t *)&realtimeFrame;
                break;
            case SpacecraftScienceMode:
                return;
            default:
                break;
        }

        //debug_print("Sending Type 1 Frame %d:%d\n",
        //            realtimeFrame.header.resetCnt,
        //            realtimeFrame.header.uptime);
        //debug_print("Bytes sent:");
        //int i=0;
        //for (i=0; i<11; i++) {
        //    debug_print("%0x ",frame[i]);
        //}
        //debug_print("\n");
    }
    if (len == 0 || len > AX25_MAX_INFO_BYTES_LEN) {
        debug_print("ERROR: Telemetry frame length of %d is not valid.  Frame not sent\n",len);
        return;
    }

    ReportToWatchdog(CurrentTaskWD);

    if (frame != NULL)
        tx_send_ui_packet(BROADCAST_CALLSIGN, TLMP1, PID_NO_PROTOCOL,
                      frame, len, BLOCK, MODULATION_INVALID);

}

void tac_send_time() {
    uint32_t t = getUnixTime();
    //uint8_t time_frame[11];
    //snprintf((char *)time_frame, 11, "%d",t);
    //len = strlen((char *)time_frame);

    int len = 4;
    t = htotl(t);
    uint8_t *time_frame;
    time_frame = (uint8_t *) &t;

    tx_send_ui_packet(BROADCAST_CALLSIGN, TIME, PID_NO_PROTOCOL,
                      time_frame, len, BLOCK, MODULATION_INVALID);
}


/**
 * Store one line of telemetry data into the WOD file
 */
void tac_store_wod() {
    telem_buffer_t *buffer = &telem_buffer;
    if(!IsStabilizedAfterBoot()) return;

    if (strlen(wod_file_name_with_path) == 0) {
        /* Make a new wod file name and start the file */
        char file_name[MAX_FILENAME_WITH_PATH_LEN];
        strlcpy(file_name, WOD_PREFIX, sizeof(file_name));
        strlcat(file_name, ".tmp", sizeof(file_name));

        strlcpy(wod_file_name_with_path, WOD_FOLDER, sizeof(wod_file_name_with_path));
        strlcat(wod_file_name_with_path, file_name, sizeof(wod_file_name_with_path));
    }

    int len = 0;
    uint8_t *frame;
    /* TODO - Use a Type 1 frame - but this should be the WOD layout */
    wodFrame.header = buffer->header;
    wodFrame.HKWod.common = buffer->common;
    wodFrame.HKWod.common2 = buffer->common2;
    wodFrame.HKWod.wodInfo = buffer->wodInfo;
    errwodFrame.errWod.primaryErrors = buffer->primaryErrors;
    //TODO - need to save the errWOD to another WOD file.

    len = sizeof(wodFrame);

    frame = (uint8_t *)&wodFrame;

    /* Write bytes to the file */
    int32_t fp;
    int32_t numOfBytesWritten = -1;
    int32_t rc;
    int32_t wod_file_length;

    fp = red_open(wod_file_name_with_path, RED_O_CREAT | RED_O_APPEND | RED_O_WRONLY);
    if (fp == -1) {
        debug_print("Unable to open %s for writing: %s\n", wod_file_name_with_path, red_strerror(red_errno));
        return;
    } else {

        numOfBytesWritten = red_write(fp, frame, len);
        if (numOfBytesWritten != len) {
            printf("Write returned: %d\n",numOfBytesWritten);
            if (numOfBytesWritten == -1) {
                printf("Unable to write to %s: %s\n", wod_file_name_with_path, red_strerror(red_errno));
                red_close(fp);
                return;
            }
        }
        wod_file_length = red_lseek(fp, 0, RED_SEEK_END);
        rc = red_close(fp);
        if (rc != 0) {
            printf("Unable to close %s: %s\n", wod_file_name_with_path, red_strerror(red_errno));
            return;
        }
    }

    ReportToWatchdog(CurrentTaskWD);

    debug_print("Telem & Control: Stored WOD: %d/%d size:%d\n", ttohs(realtimeFrame.header.resetCnt), htotl(realtimeFrame.header.uptime),wod_file_length);
    if (wod_file_length > ReadMRAMWODMaxFileSize())
        tac_roll_file(wod_file_name_with_path, WOD_FOLDER, WOD_PREFIX);
}

/**
 * Rename the file with a timestamp so it will be processed in the file queue.
 */
void tac_roll_file(char *file_name_with_path, char *folder, char *prefix) {
    /* Make a new wod file name and start the file */
    char file_name[MAX_FILENAME_WITH_PATH_LEN];
    strlcpy(file_name, folder, sizeof(file_name));
    strlcat(file_name, prefix, sizeof(file_name));

    char file_id_str[14];
    uint32_t unixtime = getUnixTime(); // Get the time in seconds since the unix epoch
    if (unixtime < 1691675756) {
        // 10 Aug 2023 because that is when I wrote this line
        debug_print("Unix time seems to be in the past!");
        unixtime=0;
        strlcat(file_name, "---", sizeof(file_name));
    } else {
        struct tm *time;
        time_t t  = (time_t)(unixtime + 2208988800L - 6 * 60 * 60);
        // Adjust because TI Time library used Epoch of 1-1-1900 UTC - 6
        time = gmtime(&t);
        if (time != NULL) {
            strftime(file_id_str, sizeof(file_id_str), "%m%d%H%M", time);
            strlcat(file_name, file_id_str, sizeof(file_name));
        } else {
            strlcat(file_name, "---", sizeof(file_name));
        }
    }

    // Just in case the file already exists, we try to remove it and ignore any errors
    red_unlink(file_name);
    //TODO - use red_rename() here
    // Then we rename the file
    int rc = red_link(file_name_with_path, file_name);
    if (rc == -1) {
        debug_print("Unable to link que file: %s : %s\n", file_name, red_strerror(red_errno));
        switch (red_errno) {
        case RED_EINVAL: // not mounted
        case RED_EIO: // disk io, we hope it is temporary
            //TODO - log this error, so that if a count is reached we reboot to try to fix this
        case RED_ENOENT: // File does not exist, so we have been called incorrectly
        case RED_ENOSPC: //this is because we ran out of space.  So exit and we will try again next time
            return;
        default:
            // All other errors will be repeated in a loop, so we try to remove the file
            red_unlink(file_name_with_path);
            return;
        }
    }

    /* Otherwise File renamed, ready to be added to the dir.  Remove the tmp file*/
    rc = red_unlink(file_name_with_path);
    if (rc == -1) {
        debug_print("Rolled file but unable to remove tmp que file: %s : %s\n", file_name_with_path, red_strerror(red_errno));
        // TODO this is not fatal but there needs to be a way to clean this up or we will keep trying to add it to the dir
        return;
    }

    debug_print("Telem & Control: Rolled QUE file: %s\n", file_name);

}

#ifdef DEBUG

/**
 * TEST ROUTINES FOLLOW
 *
 */

bool tac_test_wod_file() {
    tac_store_wod();
    tac_roll_file(wod_file_name_with_path, WOD_FOLDER, WOD_PREFIX);
    printf("##### TEST MAKE WOD FILE: executed, check results in the dir\n");

    return true;
}

bool tac_test_txt_file() {
    char file_path[MAX_FILENAME_WITH_PATH_LEN];
    char tmp_file_path[MAX_FILENAME_WITH_PATH_LEN];

    strlcpy(file_path, TXT_FOLDER, sizeof(file_path));
    strlcat(file_path, "data123", sizeof(file_path));

    strlcpy(tmp_file_path, file_path, sizeof(tmp_file_path));
    strlcat(tmp_file_path, ".tmp", sizeof(tmp_file_path));

    char *msg = "Every PACSAT file will start with the byte 0xaa followed by the byte 0x55.  \n\
This flag is followed by the rest of the PACSAT File Header (PFH).  A valid \n\
PFH contains all of the items of the Mandatory Header (Section  3), and it may\n\
also contain all items of the Extended Header (Section 4) and any number of\n\
Optional Header items (Section 5).  All HEADER ITEMS are encoded using a \n\
standard syntax, described in Section 2. ";

    int rc = dir_fs_write_file_chunk(tmp_file_path, (uint8_t *)msg, strlen(msg), 0);
    if (rc == -1) {  debug_print("Write file data - FAILED\n"); return false; }

    // Rename it so the que watcher adds it to the dir
    rc = red_link(tmp_file_path, file_path);
    red_unlink(tmp_file_path);

    printf("##### TEST MAKE TXT FILE: executed, check results in the dir\n");

    return true;
}
#endif
