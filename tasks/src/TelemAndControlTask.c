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

#ifdef BLINKY_HARDWARE
#include "Max31725Temp.h"
#endif

#include "downlink.h"
#include "rt1ErrorsDownlink.h"

typedef struct {
    header_t header;
    realTimePayload_t rtHealth;
    minValuesPayload_t minVals;
    maxValuesPayload_t maxVals;
} telem_buffer_t;

realTimeFrame_t realtimeFrame;


/* Forward declarations */
void tac_pb_status_callback(TimerHandle_t xTimer);
void tac_ftl0_status_callback(TimerHandle_t xTimer);
void tac_telem_timer_callback(TimerHandle_t xTimer);
void tac_wod_save_timer_callback(TimerHandle_t xTimer);
void tac_adc_timer_callback(TimerHandle_t xTimer);
void tac_maintenance_timer_callback(TimerHandle_t xTimer);
void tac_collect_telemetry(telem_buffer_t *buffer);
void tac_send_telemetry(telem_buffer_t *buffer);
void tac_send_time();
void tac_store_wod();
void tac_roll_wod_file();

/* Local variables */
char wod_file_name[MAX_FILENAME_WITH_PATH_LEN];
int wod_file_length = 0;

/* timer to send the telemetry periodically */
static xTimerHandle timerTelemSend;

/* timer to send the roll the WOD file periodically */
static xTimerHandle timerWodSave;

/* timer to perform maintenance periodically */
static xTimerHandle timerMaintenance;

/* timer to send the PB status periodically */
static xTimerHandle timerPbStatus;

/* timer to read the ADC periodically */
static xTimerHandle timerADC;

int pvtPbStatusTimerID = 0; // pb timer id
static xTimerHandle timerUplinkStatus;
int pvtUplinkStatusTimerID = 0; // uplink timer id

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
    timerPbStatus = xTimerCreate("PB STATUS", PB_TIMER_SEND_STATUS_PERIOD,
                                 TRUE, &pvtPbStatusTimerID,
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
                                      UPLINK_TIMER_SEND_STATUS_PERIOD, TRUE,
                                      &pvtUplinkStatusTimerID,
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
                                  TAC_TIMER_SEND_TELEMETRY_PERIOD, TRUE,
                                  NULL, tac_telem_timer_callback);
    // Block time of zero as this can not block
    timerStatus = xTimerStart(timerTelemSend, 0);
    if (timerStatus != pdPASS) {
        ReportError(RTOSfailure, FALSE, CharString,
                    (int)"ERROR: Failed in starting Telem Timer");
    }

    /* Create a periodic timer to save to the WOD file */
    timerWodSave = xTimerCreate("WodSave",
                                TAC_TIMER_SAVE_WOD_PERIOD, TRUE,
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
    pb_send_status();

    ReportToWatchdog(TelemetryAndControlWD);

    /* Wait a bit longer and then send uplink status */
    vTaskDelay(WATCHDOG_SHORT_WAIT_TIME);
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

    while(1) {
        Intertask_Message messageReceived;
        int status;
        uint32_t now = 0;

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
            case TacSendPbStatus:
                //debug_print("Telem & Control: Send the PB Status\n");
                pb_send_status();
                break;

            case TacSendUplinkStatus:
                //debug_print("Telem & Control: Send the FTL0 Status\n");
                ax25_send_status();
                break;

            case TacMaintenanceMsg:
                //debug_print("TAC: Running DIR Maintenance\n");
                dir_maintenance();
                //debug_print("TAC: Running FTL0 Maintenance\n");
                ftl0_maintenance();
                now = getUnixTime(); // Get the time in seconds since the unix epoch
                dir_file_queue_check(now, WOD_FOLDER, PFH_TYPE_WL, "WOD");
                break;

            case TacCollectMsg:
                tac_collect_telemetry(&telem_buffer);
                break;

            case TacSendRealtimeMsg:
                /*
                 * For real time telemetry we collect it and send it
                 * at the same time.
                 */
                tac_collect_telemetry(&telem_buffer);
                tac_send_telemetry(&telem_buffer);
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
    buffer->header.inScienceMode = 0;
    buffer->header.inHealthMode = 1;
    buffer->header.inSafeMode = 0;

// debug_print("Telem & Control: Collect RT telem at: %d/%d\n",time.IHUresetCnt, time.METcount);

    /**
     * Initial telemetry for the protoype booster board:
     * For each Radio RSSI
     *  Radio Power
     *  Radio mode
     *  Radio temperature?
     * PB enabled
     * FTL0 enabled
     * MODE - safe/health/exp
     * CPU temp
     * Errors
     *
     */

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
        buffer->rtHealth.common.FSAvailable = htotl(redstatfs.f_bfree); // blocks free
        buffer->rtHealth.common.FSTotalFiles = htots((uint16)(redstatfs.f_files - redstatfs.f_ffree));
    }
    short upload_kb = (uint16_t)(ftl0_get_space_reserved_by_upload_table()/1024);
    buffer->rtHealth.common.UploadQueueBytes = htots(upload_kb); // in kilobytes
    //debug_print("UploadBytes: %d",upload_kb);
    buffer->rtHealth.common.UploadQueueFiles = ftl0_get_num_of_files_in_upload_table();

    bool pb_state = ReadMRAMBoolState(StatePbEnabled);
    //debug_print("PB: %d\n", pb_state);
    buffer->rtHealth.common2.pbEnabled = pb_state;
    buffer->rtHealth.common2.uplinkEnabled = ReadMRAMBoolState(StateUplinkEnabled);

#ifdef BLINKY_HARDWARE
    uint8_t temp8;
    if (Get8BitTemp31725(CpuTemp, &temp8)) {
        buffer->rtHealth.common.IHUTemp = temp8;
    } else {
        //debug_print("TAC: ERROR I2C temp request failed\n");
    }
#elif ASFK_HARDWARE
    // TODO - add more temperatures here?
    buffer->rtHealth.common.IHUTemp = board_temps[TEMPERATURE_VAL_CPU];
#else
    buffer->rtHealth.common.IHUTemp = 0;
#endif

    /* TX Telemetry */
    uint16_t rf_pwr = (ax5043ReadReg(FIRST_TX_CHANNEL, AX5043_TXPWRCOEFFB0)
                       + (ax5043ReadReg(FIRST_TX_CHANNEL, AX5043_TXPWRCOEFFB1) << 8));
    buffer->rtHealth.common.TXPower = rf_pwr;
    buffer->rtHealth.common.TXPwrMode = ax5043ReadReg(FIRST_TX_CHANNEL, AX5043_PWRMODE);

    /* RX0 Telemetry */
    uint8_t rssi0 = get_rssi(FIRST_RX_CHANNEL);
    buffer->rtHealth.common.RX0RSSI = rssi0;
    buffer->rtHealth.common.RX0PwrMode = ax5043ReadReg(FIRST_RX_CHANNEL, AX5043_PWRMODE);

    // Errors TODO - make sure that when these are written in error handling they were converted from host to little endian
    buffer->rtHealth.primaryErrors = localErrorCollection;

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
        realtimeFrame.rtHealth = buffer->rtHealth;
        len = sizeof(realtimeFrame);

        debug_print("Telem & Control: Send SAFE telem at: %d/%d\n", ttohs(realtimeFrame.header.resetCnt), htotl(realtimeFrame.header.uptime));

        frame = (uint8_t *)&realtimeFrame;
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

    if (strlen(wod_file_name) == 0) {
        /* Make a new wod file name and start the file */
        char file_name[MAX_FILENAME_WITH_PATH_LEN];
        strlcpy(file_name, "wod.tmp", sizeof(file_name));

        strlcpy(wod_file_name, WOD_FOLDER, sizeof(wod_file_name));
        strlcat(wod_file_name, file_name, sizeof(wod_file_name));
    }

    int len = 0;
    uint8_t *frame;
    /* TODO - Use a Type 1 frame - but this should be the WOD layout */
    realtimeFrame.header = buffer->header;
    realtimeFrame.rtHealth = buffer->rtHealth;
    len = sizeof(realtimeFrame);

    frame = (uint8_t *)&realtimeFrame;

    /* Write bytes to the file */
    int32_t fp;
    int32_t numOfBytesWritten = -1;
    int32_t rc;

    fp = red_open(wod_file_name, RED_O_CREAT | RED_O_APPEND | RED_O_WRONLY);
    if (fp == -1) {
        debug_print("Unable to open %s for writing: %s\n", wod_file_name, red_strerror(red_errno));
    } else {

        numOfBytesWritten = red_write(fp, frame, len);
        if (numOfBytesWritten != len) {
            printf("Write returned: %d\n",numOfBytesWritten);
            if (numOfBytesWritten == -1) {
                printf("Unable to write to %s: %s\n", wod_file_name, red_strerror(red_errno));
            }
        }
        wod_file_length = red_lseek(fp, 0, RED_SEEK_END);
        rc = red_close(fp);
        if (rc != 0) {
            printf("Unable to close %s: %s\n", wod_file_name, red_strerror(red_errno));
        }
    }

    debug_print("Telem & Control: Stored WOD: %d/%d size:%d\n", ttohs(realtimeFrame.header.resetCnt), htotl(realtimeFrame.header.uptime),wod_file_length);
    if (wod_file_length > TAC_FILE_SIZE_TO_ROLL_WOD)
        tac_roll_wod_file();

}

/**
 * Rename the WOD file so it will be processed in the file queue.  Reset the WOD
 * file name so a new one is assigned next time we write data.
 */
void tac_roll_wod_file() {
    /* Make a new wod file name and start the file */
    char file_name[MAX_FILENAME_WITH_PATH_LEN];
    strlcpy(file_name, WOD_FOLDER, sizeof(file_name));
    strlcat(file_name, "wod", sizeof(file_name));

    char file_id_str[14];
    uint32_t unixtime = getUnixTime(); // Get the time in seconds since the unix epoch
    if (unixtime < 1691675756) {
        // 10 Aug 2023 because that is when I wrote this line
        debug_print("Unix time seems to be in the past!");
        unixtime=0;
    }
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

    int rc = red_link(wod_file_name, file_name);
    if (rc == -1) {
        debug_print("Unable to link wod file: %s : %s\n", file_name, red_strerror(red_errno));
        return; // TODO Check if this is because we ran out of space.  So exit and we will try again next time
    }

    /* Otherwise File renamed, ready to be added to the dir.  Remove the tmp file*/
    rc = red_unlink(wod_file_name);
    if (rc == -1) {
        debug_print("Unable to remove tmp wod file: %s : %s\n", wod_file_name, red_strerror(red_errno));
        // TODO this is not fatal but there needs to be a way to clean this up or we will keep trying to add it to the dir
    }

    wod_file_name[0] = 0; // clear the file name
    wod_file_length = 0;

    debug_print("Telem & Control: Rolled WOD file: %s\n", file_name);

}

#ifdef DEBUG

/**
 * TEST ROUTINES FOLLOW
 *
 */

int tac_test_wod_file() {
    tac_store_wod();
    tac_roll_wod_file();
    printf("##### TEST MAKE WOD FILE: success\n");

    return EXIT_SUCCESS;
}

#endif
