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
#include "mram.h"
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
    rt1Errors_t errors;
} telem_buffer_t;

bool trace_telem;

realTimeFrame_t realtimeFrame;
WODFrame_t wodFrame;
errFrame_t errFrame;
errWODFrame_t errwodFrame;

uint8_t payload_counter = 0;

/* List the payload send sequence.  These are sent in pairs.
 * TODO: We should store payloads and send the last and the latest if two of the same type are listed one after the other.
 * For now, do not list two payloads of the same type one after the other or we will send duplicates. */
uint8_t safe_mode_payload_sequence[] = {
     DIAGNOSTIC_PAYLOAD,
     RT_HK_PAYLOAD,
     RT_EXP_PAYLOAD,
     RT_HK_PAYLOAD,
     MAX_VALS_PAYLOAD,
     RT_HK_PAYLOAD,
     RT_EXP_PAYLOAD,
     RT_HK_PAYLOAD,
     MIN_VALS_PAYLOAD,
     RT_HK_PAYLOAD,
     RT_EXP_PAYLOAD,
     RT_HK_PAYLOAD,
};

uint8_t filesystem_mode_payload_sequence[] = {
     RT_HK_PAYLOAD,
     RT_EXP_PAYLOAD,
     RT_HK_PAYLOAD,
     MAX_VALS_PAYLOAD,
     RT_HK_PAYLOAD,
     RT_EXP_PAYLOAD,
     RT_HK_PAYLOAD,
     MIN_VALS_PAYLOAD,
};

uint8_t science_mode_payload_sequence[] = {
     RT_EXP_PAYLOAD,
     RT_HK_PAYLOAD,
};

/* Forward declarations */
void tac_pb_status_callback(TimerHandle_t xTimer);
void tac_ftl0_status_callback(TimerHandle_t xTimer);
void tac_telem_timer_callback(TimerHandle_t xTimer);
void tac_time_timer_callback(TimerHandle_t xTimer);
void tac_wod_save_timer_callback(TimerHandle_t xTimer);
void tac_errwod_save_timer_callback(TimerHandle_t xTimer);
void tac_adc_timer_callback(TimerHandle_t xTimer);
void tac_maintenance_timer_callback(TimerHandle_t xTimer);
void tac_check_file_queues_timer_callback(TimerHandle_t xTimer);
void tac_science_mode_timer_callback(TimerHandle_t xTimer);
void tac_stop_science_mode_timer();
void tac_collect_telemetry(telem_buffer_t *buffer);
void tac_send_telemetry(telem_buffer_t *buffer);
void tac_send_time();
void tac_store_wod();
void tac_store_errwod();

/* Local variables */
char wod_file_name_with_path[MAX_FILENAME_WITH_PATH_LEN];
char errwod_file_name_with_path[MAX_FILENAME_WITH_PATH_LEN];

/* timer to send the telemetry periodically */
static xTimerHandle timerTelemSend;

/* timer to send the time periodically */
static xTimerHandle timerTimeSend;

/* timer to send the roll the WOD file periodically */
static xTimerHandle timerWodSave;
static xTimerHandle timerErrWodSave;

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
static uint8_t minMaxResets = 0;

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

    portBASE_TYPE timerStatus = pdFAIL;
    /* Setup a timer to send the PB status periodically */
    timerPbStatus = xTimerCreate("PB STATUS", SECONDS(ReadMRAMPBStatusFreq()),
                                 TRUE, NULL,
                                 tac_pb_status_callback); // auto reload timer
    if (timerPbStatus != NULL) {
        // Block time of zero as this can not block
        timerStatus = xTimerStart(timerPbStatus, 0);
        if (timerStatus != pdPASS) {
            ReportError(RTOSfailure, FALSE, CharString,
                        (int)"TAC: ERROR: Could not start PB Timer");
        }
    } else {
        ReportError(RTOSfailure, FALSE, CharString,
                    (int)"TAC: ERROR: Could not create PB Timer");
    }

    /* Setup a timer to send the uplink status periodically */
    timerUplinkStatus = xTimerCreate("UPLINK STATUS",
                                     SECONDS(ReadMRAMFTL0StatusFreq()), TRUE,
                                     NULL,
                                     tac_ftl0_status_callback);
    if (timerUplinkStatus != NULL) {
        // Block time of zero as this can not block
        timerStatus = xTimerStart(timerUplinkStatus, 0);
        if (timerStatus != pdPASS) {
            ReportError(RTOSfailure, FALSE, CharString,
                        (int)"ERROR: Failed in starting Uplink Status Timer");
        }
    } else {
        ReportError(RTOSfailure, FALSE, CharString,
                    (int)"TAC: ERROR: Could not create Uplink Status Timer");
    }

    /* Create a periodic timer to send telemetry */
    timerTelemSend = xTimerCreate("TelemSend",
                                  SECONDS(ReadMRAMTelemFreq()), TRUE,
                                  NULL, tac_telem_timer_callback);
    if (timerTelemSend != NULL) {
        // Block time of zero as this can not block
        timerStatus = xTimerStart(timerTelemSend, 0);
        if (timerStatus != pdPASS) {
            ReportError(RTOSfailure, FALSE, CharString,
                        (int)"ERROR: Failed in starting Telem send Timer");
        }
    } else {
        ReportError(RTOSfailure, FALSE, CharString,
                    (int)"TAC: ERROR: Could not create Telem send Timer");
    }


    /* Create a periodic timer to send time */
    timerTimeSend = xTimerCreate("TimeSend",
                                 SECONDS(ReadMRAMTimeFreq()), TRUE,
                                 NULL, tac_time_timer_callback);
    if (timerTimeSend != NULL) {
        // Block time of zero as this can not block
        timerStatus = xTimerStart(timerTimeSend, 0);
        if (timerStatus != pdPASS) {
            ReportError(RTOSfailure, FALSE, CharString,
                        (int)"ERROR: Failed in starting Send Time Timer");
        }
    } else {
        ReportError(RTOSfailure, FALSE, CharString,
                    (int)"TAC: ERROR: Could not create Send Time Timer");
    }


    /* Create a periodic timer to save to the WOD files */
    timerWodSave = xTimerCreate("WodSave",
                                SECONDS(ReadMRAMWODFreq()), TRUE,
                                  NULL, tac_wod_save_timer_callback);
    if (timerWodSave != NULL) {
        // Block time of zero as this can not block
        timerStatus = xTimerStart(timerWodSave, 0);
        if (timerStatus != pdPASS) {
            ReportError(RTOSfailure, FALSE, CharString,
                        (int)"ERROR: Failed in starting WOD Save Timer");
        }
    } else {
        ReportError(RTOSfailure, FALSE, CharString,
                    (int)"TAC: ERROR: Could not create WOD Save Timer");
    }
    timerErrWodSave = xTimerCreate("ErrWodSave",
                                SECONDS(ReadMRAMErrWODFreq()), TRUE,
                                NULL, tac_errwod_save_timer_callback);
    if (timerErrWodSave != NULL) {
        // Block time of zero as this can not block
        timerStatus = xTimerStart(timerErrWodSave, 0);
        if (timerStatus != pdPASS) {
            ReportError(RTOSfailure, FALSE, CharString,
                        (int)"ERROR: Failed in starting ERR WOD Save Timer");
        }
    } else {
        ReportError(RTOSfailure, FALSE, CharString,
                    (int)"TAC: ERROR: Could not create WOD Save Timer");
    }
    /* Create a periodic timer for maintenance */
    timerMaintenance = xTimerCreate("Maintenance",
                                    TAC_TIMER_MAINTENANCE_PERIOD, TRUE,
                                    NULL, tac_maintenance_timer_callback);
    if (timerMaintenance != NULL) {
        // Block time of zero as this can not block
        timerStatus = xTimerStart(timerMaintenance, 0);
        if (timerStatus != pdPASS) {
            ReportError(RTOSfailure, FALSE, CharString,
                        (int)"ERROR: Failed in starting Maintenance Timer");
        }
    } else {
        ReportError(RTOSfailure, FALSE, CharString,
                    (int)"TAC: ERROR: Could not create Maintenance Timer");
    }

    /* Create a periodic timer for file queues */
    timerCheckFileQueues = xTimerCreate("Check File Queues",
                                    TAC_TIMER_CHECK_FILE_QUEUES_PERIOD, TRUE,
                                    NULL, tac_check_file_queues_timer_callback);
    if (timerCheckFileQueues != NULL) {
        // Block time of zero as this can not block
        timerStatus = xTimerStart(timerCheckFileQueues, 0);
        if (timerStatus != pdPASS) {
            ReportError(RTOSfailure, FALSE, CharString,
                        (int)"ERROR: Failed in starting File Queue Check Timer");
        }
    } else {
        ReportError(RTOSfailure, FALSE, CharString,
                    (int)"TAC: ERROR: Could not create File Queue Check Timer");
    }

    /* Create a periodic timer for reading the ADC */
    timerADC = xTimerCreate("ADC",
                            TAC_TIMER_ADC_PERIOD, TRUE,
                            NULL, tac_adc_timer_callback);
    if (timerADC != NULL) {
    // Block time of zero as this can not block
    timerStatus = xTimerStart(timerADC, 0);
    if (timerStatus != pdPASS) {
        ReportError(RTOSfailure, FALSE, CharString,
                    (int)"ERROR: Failed in starting ADC Timer");
    }
    } else {
        ReportError(RTOSfailure, FALSE, CharString,
                    (int)"TAC: ERROR: Could not create ADC Timer");
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
    if (getSpacecraftMode() == SpacecraftFileSystemMode) {
        uint32_t now = getUnixTime(); // Get the time in seconds since the unix epoch
        if (now > CLOCK_MIN_UNIX_SECS)
            ax25_send_status();
    }
    ReportToWatchdog(TelemetryAndControlWD);

    //TODO - include any checks needed here to make sure hardware is available
    // I2CDevicePoll(); // verify that ICR, CIU, etc are communicating over I2c
    //ReportToWatchdog(TelemetryAndControlWD);

    /* Clear the Min Max each boot */
    tac_clear_minmax();

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
                    if (trace_telem)
                        printf("Entering SAFE mode\n");
                }
                break;
            case TacEnterFileSystemMode:
                if (getSpacecraftMode() != SpacecraftFileSystemMode) {
                    if (getSpacecraftMode() == SpacecraftScienceMode) {
                        tac_stop_science_mode_timer();
                    }
                    setSpacecraftMode(SpacecraftFileSystemMode);
                    if (trace_telem) printf("Entering FS mode\n");
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
                    if (trace_telem)
                        printf("Entering SCIENCE Mode with timeout: %d mins\n",mins);
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
                if (trace_telem)
                    printf("Telem & Control: Ending Science Mode\n");
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
                if (ReadMRAMBoolState(StatePbEnabled))
                    if (getSpacecraftMode() == SpacecraftFileSystemMode)
                        pb_send_status();
                break;

            case TacUpdatePbTimer:
                if (timerPbStatus != NULL) {
                    timerStatus = xTimerChangePeriod(timerPbStatus, SECONDS(ReadMRAMPBStatusFreq()), pdMS_TO_TICKS(100));
                    if (timerStatus != pdPASS) {
                        ReportError(RTOSfailure, FALSE, CharString,
                                    (int)"ERROR: Failed to change PB Timer period");
                    }
                }
                break;
            case TacSendUplinkStatus:
                //debug_print("Telem & Control: Send the FTL0 Status\n");
                if (ReadMRAMBoolState(StateUplinkEnabled)) {
                    now = getUnixTime(); // Get the time in seconds since the unix epoch
                    if (now < CLOCK_MIN_UNIX_SECS)
                        break;
                    if (getSpacecraftMode() == SpacecraftFileSystemMode)
                        ax25_send_status();
                }
                break;
            case TacUpdateUplinkTimer:
                if (timerUplinkStatus != NULL) {
                    timerStatus = xTimerChangePeriod(timerUplinkStatus, SECONDS(ReadMRAMFTL0StatusFreq()), pdMS_TO_TICKS(100));
                    if (timerStatus != pdPASS) {
                        ReportError(RTOSfailure, FALSE, CharString,
                                    (int)"ERROR: Failed to change FTL0 Uplink Timer period");
                    }
                }
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
                dir_file_queue_check(now, ERRWOD_FOLDER, PFH_TYPE_WL, ERRWOD_DESTINATION, DIR_MAX_ERRWOD_FILE_AGE);
                dir_file_queue_check(now, TXT_FOLDER, PFH_TYPE_ASCII, TXT_DESTINATION, DIR_MAX_WOD_FILE_AGE);
                dir_file_queue_check(now, EXP_FOLDER, PFH_TYPE_CAN_PACKETS, EXP_DESTINATION, DIR_MAX_WOD_FILE_AGE);
                break;

            case TacCollectMsg:
                tac_collect_telemetry(&telem_buffer);
                break;

            case TacSendRealtimeMsg:
                if (ReadMRAMBoolState(StateTelemBroadcastEnabled)) {
                    if (getSpacecraftMode() != SpacecraftScienceMode) {
                        /*
                         * For real time telemetry we collect it and send it
                         * at the same time.  This is sent in both SAFE and File System
                         * Mode.
                         */
                        tac_collect_telemetry(&telem_buffer);
                        tac_send_telemetry(&telem_buffer);
                    }
                }
                break;
            case TacUpdateTelemTimer:
                if (timerTelemSend != NULL) {
                    timerStatus = xTimerChangePeriod(timerTelemSend, SECONDS(ReadMRAMTelemFreq()), pdMS_TO_TICKS(100));
                    if (timerStatus != pdPASS) {
                        ReportError(RTOSfailure, FALSE, CharString,
                                    (int)"ERROR: Failed to change Telem Timer period");
                    }
                }
                break;
            case TacSendTimeMsg:
                if (ReadMRAMBoolState(StateTimeBroadcastEnabled)) {
                    if (getSpacecraftMode() == SpacecraftFileSystemMode)
                        tac_send_time();
                }
                break;
            case TacUpdateTimeBroadcastTimer:
                if (timerTimeSend != NULL) {
                    timerStatus = xTimerChangePeriod(timerTimeSend, SECONDS(ReadMRAMTimeFreq()), pdMS_TO_TICKS(100));
                    if (timerStatus != pdPASS) {
                        ReportError(RTOSfailure, FALSE, CharString,
                                    (int)"ERROR: Failed to change Time Broadcast Timer period");
                    }
                }
                break;
            case TacSaveWodMsg:
                if (ReadMRAMBoolState(StateWodEnabled))
                    tac_store_wod();
                break;
            case TacUpdateWodTimer:
                if (timerWodSave != NULL) {
                    timerStatus = xTimerChangePeriod(timerWodSave, SECONDS(ReadMRAMWODFreq()), pdMS_TO_TICKS(100));
                    if (timerStatus != pdPASS) {
                        ReportError(RTOSfailure, FALSE, CharString,
                                    (int)"ERROR: Failed to change WOD save Timer period");
                    }
                }
                break;
            case TacSaveErrWodMsg:
                if (ReadMRAMBoolState(StateErrWodEnabled))
                    tac_store_errwod();
                break;
            case TacUpdateErrWodTimer:
                if (timerErrWodSave != NULL) {
                    timerStatus = xTimerChangePeriod(timerErrWodSave, SECONDS(ReadMRAMErrWODFreq()), pdMS_TO_TICKS(100));
                    if (timerStatus != pdPASS) {
                        ReportError(RTOSfailure, FALSE, CharString,
                                    (int)"ERROR: Failed to change ERR WOD save Timer period");
                    }
                }
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
 * tac_errwod_roll_timer_callback()
 *
 * This is called from a timer whenever the ERR WOD file should be rolled.
 */
void tac_errwod_save_timer_callback(TimerHandle_t xTimer)
{
    statusMsg.MsgType = TacSaveErrWodMsg;
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

    // Stop the Science mode timer, but if this timer is not actually runnning we will crash. So check first.
    if (timerScienceMode != NULL && xTimerIsTimerActive(timerScienceMode) != pdFALSE) {
        timerStatus = xTimerStop(timerScienceMode, pdMS_TO_TICKS(100));
        if (timerStatus != pdPASS) {
            ReportError(RTOSfailure, FALSE, CharString,
                        (int)"ERROR: Failed to stop Science Mode Timer");
        }
    }

}

uint8_t tac_encode_period_30s_blocks(uint16_t period) {
    if (2*period/60 > 255)
        return 255;
    return (uint8_t)(2*period/60);
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

    // TODO - it is not clear that we need these seperate timestamps.  They come from the FOX/GOLF scheme where the delayed WOPD payloads are transmitted later
    buffer->wodInfo.WODTimestampReset = htots(time.IHUresetCnt);
    buffer->wodInfo.WODTimestampUptime = htotl(time.METcount);

// debug_print("Telem & Control: Collect RT telem at: %d/%d\n",time.IHUresetCnt, time.METcount);

    /********** commonRtMinmaxWodPayload_t - These values also go into min / max ***********/
    // TODO - put these in MIN MAX.  Make sure Min Max initialized with preflight init or similar

    /* File Storage */
    REDSTATFS redstatfs;
    bool rc = red_statvfs("/", &redstatfs);
    if (rc != 0) {
        printf("TAC: Unable to check disk space with statvfs: %s\n",
               red_strerror(red_errno));
        ReportError(REDFSIOerror, FALSE, ErrorBits,(int)red_errno);
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

    uint8_t upload_kb = (uint8_t)(ftl0_get_space_reserved_by_upload_table()/1024);
    buffer->common.UploadQueueBytes = upload_kb; // in kilobytes
    //debug_print("UploadBytes: %d",upload_kb);
    buffer->common.UploadQueueFiles = ftl0_get_num_of_files_in_upload_table();

    /* TX Telemetry */
    uint16_t rf_pwr = (ax5043ReadReg(FIRST_TX_CHANNEL, AX5043_TXPWRCOEFFB0)
            + (ax5043ReadReg(FIRST_TX_CHANNEL, AX5043_TXPWRCOEFFB1) << 8));
    buffer->common.TXPower = rf_pwr;
    buffer->common.TXPwrMode = ax5043ReadReg(FIRST_TX_CHANNEL, AX5043_PWRMODE);
    buffer->common.TxModMode = ReadMRAMModulation(FIRST_TX_CHANNEL);
    /* RX0 Telemetry */
    uint8_t rssi = get_rssi(FIRST_RX_CHANNEL);
    buffer->common.RX0RSSI = rssi;
    buffer->common.RX0PwrMode = ax5043ReadReg(FIRST_RX_CHANNEL, AX5043_PWRMODE);
    buffer->common.RX0ModMode = ReadMRAMModulation(FIRST_RX_CHANNEL);
#if NUM_RX_CHANNELS == 4
    rssi = get_rssi(FIRST_RX_CHANNEL+1);
    buffer->common.RX1RSSI = rssi;
    buffer->common.RX1PwrMode = ax5043ReadReg(FIRST_RX_CHANNEL+1, AX5043_PWRMODE);
    buffer->common.RX1ModMode = ReadMRAMModulation(FIRST_RX_CHANNEL+1);
    rssi = get_rssi(FIRST_RX_CHANNEL+2);
    buffer->common.RX2RSSI = rssi;
    buffer->common.RX2PwrMode = ax5043ReadReg(FIRST_RX_CHANNEL+2, AX5043_PWRMODE);
    buffer->common.RX2ModMode = ReadMRAMModulation(FIRST_RX_CHANNEL+2);
    rssi = get_rssi(FIRST_RX_CHANNEL+3);
    buffer->common.RX3RSSI = rssi;
    buffer->common.RX3PwrMode = ax5043ReadReg(FIRST_RX_CHANNEL+3, AX5043_PWRMODE);
    buffer->common.RX3ModMode = ReadMRAMModulation(FIRST_RX_CHANNEL+3);
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
    // The temp value in the array is in deg C and is signed
//    printf("CPU temp: %d\n", board_temps[TEMPERATURE_VAL_CPU]);
//    printf("PA temp: %d\n", board_temps[TEMPERATURE_VAL_PA]);
//    printf("Power temp: %d\n", board_temps[TEMPERATURE_VAL_POWER]);

    // Scale 0-255 = -20C to +107.5C
    buffer->common.IHUTemp = (20+board_temps[TEMPERATURE_VAL_CPU])*2;
    buffer->common.PATemp = (20+board_temps[TEMPERATURE_VAL_PA])*2;
    buffer->common.PowerTemp = (20+board_temps[TEMPERATURE_VAL_POWER])*2;
#else
    buffer->common.IHUTemp = 0;
#endif

    /********** commonRtWodPayload_t - These values are static values and not suitable to calculate min / max ***********/

    buffer->common2.AutoSafeAllowed = ReadMRAMBoolState(StateAutoSafeAllow);
    buffer->common2.AutoSafeModeActive = ReadMRAMBoolState(StateAutoSafe);
    buffer->common2.pbEnabled = ReadMRAMBoolState(StatePbEnabled);
    buffer->common2.uplinkEnabled = ReadMRAMBoolState(StateUplinkEnabled);
    buffer->common2.DigiEnabled = ReadMRAMBoolState(StateDigiEnabled);
    //debug_print("PB: %d\n", buffer->common2.pbEnabled);

    buffer->common2.LogLevel = 0; // TODO - implement when logging in place
    buffer->common2.TimePeriod = tac_encode_period_30s_blocks(ReadMRAMTimeFreq());
    buffer->common2.TelemPeriod = tac_encode_period_30s_blocks(ReadMRAMTelemFreq());
    buffer->common2.WodPeriod = tac_encode_period_30s_blocks(ReadMRAMWODFreq());
    buffer->common2.MaxWodFileSize = ReadMRAMWODMaxFileSize4kBlocks();
    buffer->common2.MaxExpFileSize = ReadMRAMExpMaxFileSize4kBlocks();
    buffer->common2.PbStatusPeriod = tac_encode_period_30s_blocks(ReadMRAMPBStatusFreq());
    buffer->common2.PbTimeout = tac_encode_period_30s_blocks(ReadMRAMPBClientTimeout());
    buffer->common2.UplinkStatusPeriod = tac_encode_period_30s_blocks(ReadMRAMFTL0StatusFreq());
    buffer->common2.TLMresets = minMaxResets;
    buffer->common2.swCmds = htotl(getCmdRingTelem());
    buffer->common2.swCmdCnt = GetSWCmdCount();
    buffer->common2.MRAMstatus0 = readMRAMStatus(0);
    buffer->common2.MRAMstatus1 = readMRAMStatus(1);
    buffer->common2.MRAMstatus2 = readMRAMStatus(2);
    buffer->common2.MRAMstatus3 = readMRAMStatus(3);

    /* We need to make sure that when these are written in error handling they were converted from host to little endian
     * Note: that we do not do that for the TMS570 hardware values like RAMCorAddr1.  These are left big endian */
    buffer->errors = localErrorCollection;

    // TODO - calculate min max.  TODO - should this also be stored in MRAM so it survives across resets?  If so, remove the reset when this task starts and add to preflight init.
    // Note that an MRAM variable called MinMaxResetTimeout exists in MET.h and it indexes a timeout in MRAM. if not used, should be removed.
}

/**
 * Set the min values to their maximum and the max values to their minimum.  All of the
 * telem values are unsigned ints, so this works even for multi byte values
 */
void tac_clear_minmax() {
    /* First clear the max to zero */
    memset(&telem_buffer.maxVals, 0, sizeof(telem_buffer.maxVals));
    /* Then zero only the time stamps for min and set the min values to their maximum */
    memset(&telem_buffer.minVals.MinValuesData, 0, sizeof(telem_buffer.minVals.MinValuesData));
    memset(&telem_buffer.minVals.common, 0xff, sizeof(telem_buffer.minVals.common));
    minMaxResets++;
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

    char * to_callsign = TLMP1;
    int payload = RT_HK_PAYLOAD;
    int len = 0;
    uint8_t *frame;
//    if (ReadMRAMBoolState(StateCommandedSafeMode)
//                || ReadMRAMBoolState(StateAutoSafe)) {

        int i;
        for (i=0; i<PACKETS_PER_TELEMETRY_BEACON; i++) {
            switch (spacecraftMode) {
            case SpacecraftSafeMode:
                if (trace_telem)
                    printf("Telem & Control: Send SAFE telem");
                if (payload_counter >= sizeof(safe_mode_payload_sequence))
                    payload_counter=0;
                payload = safe_mode_payload_sequence[payload_counter++];
                break;
            case SpacecraftFileSystemMode:
                if (trace_telem)
                    printf("Telem & Control: Send HEALTH telem at: %d/%d\n", ttohs(realtimeFrame.header.resetCnt), htotl(realtimeFrame.header.uptime));
                if (payload_counter >= sizeof(filesystem_mode_payload_sequence))
                    payload_counter=0;
                payload = filesystem_mode_payload_sequence[payload_counter];
                break;
            case SpacecraftScienceMode:
                if (trace_telem)
                    printf("Telem & Control: Send SCIENCE telem at: %d/%d\n", ttohs(realtimeFrame.header.resetCnt), htotl(realtimeFrame.header.uptime));
                if (payload_counter >= sizeof(science_mode_payload_sequence))
                    payload_counter=0;
                payload = science_mode_payload_sequence[payload_counter];
                return;
            default:
                break;
            }


            switch (payload) {
            case RT_HK_PAYLOAD:
                to_callsign = TLMP1;
                realtimeFrame.header = buffer->header;
                realtimeFrame.rtHealth.common = buffer->common;
                realtimeFrame.rtHealth.common2 = buffer->common2;
                realtimeFrame.rtHealth.realTimeData = buffer->realTimeData;
                len = sizeof(realtimeFrame);
                frame = (uint8_t *)&realtimeFrame;
                break;
            case MAX_VALS_PAYLOAD:
                to_callsign = TLMP2;
                break;
            case MIN_VALS_PAYLOAD:
                to_callsign = TLMP3;
                break;
            case RT_EXP_PAYLOAD:
                to_callsign = TLMP5;
                break;
            case DIAGNOSTIC_PAYLOAD:
                to_callsign = TLMP_ERROR;
                errFrame.header = buffer->header;
                errFrame.err.errors = buffer->errors;
                len = sizeof(errFrame);
                frame = (uint8_t *)&errFrame;
                break;
            default:
                break;
            }

            if (trace_telem)
                printf("  Type %d Frame to %s at: %d/%d\n", payload, to_callsign,ttohs(buffer->header.resetCnt), htotl(buffer->header.uptime));

            //debug_print("Bytes sent:");
            //int i=0;
            //for (i=0; i<11; i++) {
            //    debug_print("%0x ",frame[i]);
            //}
            //debug_print("\n");
            //    }
            if (len == 0 || len > AX25_MAX_INFO_BYTES_LEN) {
                debug_print("ERROR: Telemetry frame length of %d is not valid.  Frame not sent\n",len);
                continue;
            }

            ReportToWatchdog(CurrentTaskWD);

            if (frame != NULL)
                tx_send_ui_packet(BROADCAST_CALLSIGN, to_callsign, PID_NO_PROTOCOL,
                                  frame, len, BLOCK, MODULATION_INVALID);
        }
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
    wodFrame.header = buffer->header;
    wodFrame.HKWod.common = buffer->common;
    wodFrame.HKWod.common2 = buffer->common2;

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
        ReportError(REDFSIOerror, FALSE, ErrorBits,(int)red_errno);
        return;
    } else {

        numOfBytesWritten = red_write(fp, frame, len);
        if (numOfBytesWritten != len) {
            printf("Write returned: %d\n",numOfBytesWritten);
            if (numOfBytesWritten == -1) {
                printf("Unable to write to %s: %s\n", wod_file_name_with_path, red_strerror(red_errno));
                ReportError(REDFSIOerror, FALSE, ErrorBits,(int)red_errno);
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

    if (trace_telem)
        printf("Telem & Control: Stored WOD: %d/%d size:%d\n", ttohs(wodFrame.header.resetCnt), htotl(wodFrame.header.uptime),wod_file_length);
    if (wod_file_length > (ReadMRAMWODMaxFileSize4kBlocks()*4096))
        tac_roll_file(wod_file_name_with_path, WOD_FOLDER, WOD_PREFIX);
}

/**
 * Store one line of telemetry data into the ERR WOD file
 */
void tac_store_errwod() {
//    debug_print("Storing ERR WOD..\n");
    telem_buffer_t *buffer = &telem_buffer;
    if(!IsStabilizedAfterBoot()) return;

    if (strlen(errwod_file_name_with_path) == 0) {
        /* Make a new wod file name and start the file */
        char file_name[MAX_FILENAME_WITH_PATH_LEN];
        strlcpy(file_name, ERRWOD_PREFIX, sizeof(file_name));
        strlcat(file_name, ".tmp", sizeof(file_name));

        strlcpy(errwod_file_name_with_path, ERRWOD_FOLDER, sizeof(errwod_file_name_with_path));
        strlcat(errwod_file_name_with_path, file_name, sizeof(errwod_file_name_with_path));
    }

    int len = 0;
    uint8_t *frame;
    errwodFrame.header = buffer->header;
    errwodFrame.errWod.errors = buffer->errors;

    len = sizeof(errwodFrame);

    frame = (uint8_t *)&errwodFrame;

    /* Write bytes to the file */
    int32_t fp;
    int32_t numOfBytesWritten = -1;
    int32_t rc;
    int32_t errwod_file_length;

    fp = red_open(errwod_file_name_with_path, RED_O_CREAT | RED_O_APPEND | RED_O_WRONLY);
    if (fp == -1) {
        debug_print("Unable to open %s for writing: %s\n", errwod_file_name_with_path, red_strerror(red_errno));
        ReportError(REDFSIOerror, FALSE, ErrorBits,(int)red_errno);
        return;
    } else {

        numOfBytesWritten = red_write(fp, frame, len);
        if (numOfBytesWritten != len) {
            printf("Write returned: %d\n",numOfBytesWritten);
            if (numOfBytesWritten == -1) {
                printf("Unable to write to %s: %s\n", errwod_file_name_with_path, red_strerror(red_errno));
                ReportError(REDFSIOerror, FALSE, ErrorBits,(int)red_errno);
                red_close(fp);
                return;
            }
        }
        errwod_file_length = red_lseek(fp, 0, RED_SEEK_END);
        rc = red_close(fp);
        if (rc != 0) {
            printf("Unable to close %s: %s\n", errwod_file_name_with_path, red_strerror(red_errno));
            return;
        }
    }

    ReportToWatchdog(CurrentTaskWD);

    if (trace_telem)
        printf("Telem & Control: Stored ERRWOD: %d/%d size:%d\n", ttohs(errwodFrame.header.resetCnt), htotl(errwodFrame.header.uptime),errwod_file_length);
    if (errwod_file_length > (ReadMRAMErrWODMaxFileSize4kBlocks()*4096))
        tac_roll_file(errwod_file_name_with_path, ERRWOD_FOLDER, ERRWOD_PREFIX);
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
    if (unixtime < CLOCK_MIN_UNIX_SECS) {
        debug_print("ERROR: Unix time seems to be in the past!");
        unixtime=0;
        logicalTime_t time;
        getTimestamp(&time);
        snprintf(file_id_str,sizeof(file_id_str),"%09d",time.METcount); // we put the uptime in the filename to make it unique.
        strlcat(file_name, file_id_str, sizeof(file_name));
    } else {
        struct tm *time;
        time_t t  = (time_t)(unixtime + 2208988800L - 6 * 60 * 60);
        // Adjust because TI Time library used Epoch of 1-1-1900 UTC - 6
        time = gmtime(&t);
        if (time != NULL) {
            strftime(file_id_str, sizeof(file_id_str), "%m%d%H%M", time);
            strlcat(file_name, file_id_str, sizeof(file_name));
        } else {
            strlcat(file_name, "---", sizeof(file_name)); // we put nothing in the filename as this was an error.
        }
    }

    // Just in case the file already exists, we try to remove it and ignore any errors
    red_unlink(file_name);
    //TODO - use red_rename() here?  Currently that feature is not enabled, but we are recreating it here
    // Then we rename the file
    int rc = red_link(file_name_with_path, file_name);
    if (rc == -1) {
        debug_print("Unable to link que file: %s : %s\n", file_name, red_strerror(red_errno));
        switch (red_errno) {
        case RED_EINVAL: // not mounted
        case RED_EIO: // disk io, we hope it is temporary
            //log this error, so that if a count is reached we reboot to try to fix this
            ReportError(REDFSIOerror, FALSE, ErrorBits,(int)red_errno);

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
        // This is not fatal but there needs to be a way to clean this up or we will keep trying to add it to the dir. So we log it
        // and if it happens repeatedly we will reboot to heopefully fix it
        ReportError(REDFSIOerror, FALSE, ErrorBits,(int)red_errno);
        return;
    }

    if (trace_telem)
        printf("Telem & Control: Rolled QUE file: %s\n", file_name);

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

bool tac_test_txt_file(char * dir) {
    char file_path[MAX_FILENAME_WITH_PATH_LEN];
    char tmp_file_path[MAX_FILENAME_WITH_PATH_LEN];

    strlcpy(file_path, dir, sizeof(file_path));
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
