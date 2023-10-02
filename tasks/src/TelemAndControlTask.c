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
#include "Max31725Temp.h"
#include "ax5043_access.h"
#include "ax5043-ax25.h"
#include "inet.h"

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
void tac_telem_timer_callback();
void tac_maintenance_timer_callback();
void tac_collect_telemetry(telem_buffer_t *buffer);
void tac_send_telemetry(telem_buffer_t *buffer);



/* Local variables */
static xTimerHandle timerTelemSend;
static xTimerHandle timerMaintenance;
static Intertask_Message statusMsg; // Storage used to send messages to the Telemetry and Control task


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
portTASK_FUNCTION_PROTO(TelemAndControlTask, pvParameters)  {
    vTaskSetApplicationTaskTag((xTaskHandle) 0, (pdTASK_HOOK_CODE)TelemetryAndControlWD );
    InitInterTask(ToTelemetryAndControl, 10);
    localErrorCollection.valid = 1;

    ResetAllWatchdogs();
//    debug_print("Initializing Telem and Control Task\n");

    /* Create a periodic timer to send telemetry */
    timerTelemSend = xTimerCreate( "TelemSend", TAC_TIMER_SEND_TELEMETRY_PERIOD, TRUE, (void *)0, tac_telem_timer_callback);
    portBASE_TYPE timerStatus = xTimerStart(timerTelemSend, 0); // Block time of zero as this can not block
    if (timerStatus != pdPASS) {
        ReportError(RTOSfailure, FALSE, CharString, (int)"ERROR: Failed in starting Telem Timer"); /* failed to start the RTOS timer */
    }

    /* Create a periodic timer for maintenance */
    timerMaintenance = xTimerCreate( "Maintenance", TAC_TIMER_MAINTENANCE_PERIOD, TRUE, (void *)0, tac_maintenance_timer_callback);
    portBASE_TYPE timerStatus2 = xTimerStart(timerMaintenance, 0); // Block time of zero as this can not block
    if (timerStatus2 != pdPASS) {
        ReportError(RTOSfailure, FALSE, CharString, (int)"ERROR: Failed in starting Maintenance Timer"); /* failed to start the RTOS timer */
    }


    /* After a short delay for things to settle, send the status to indicate we have rebooted */
    vTaskDelay(WATCHDOG_SHORT_WAIT_TIME);
    pb_send_status();
    ReportToWatchdog(TelemetryAndControlWD); /* Tell the WD we are ok after that delay */

    /* Wait a bit longer and then send uplink status */
    vTaskDelay(WATCHDOG_SHORT_WAIT_TIME);
    ax25_send_status();
    ReportToWatchdog(TelemetryAndControlWD); /* Tell the WD we are ok after that delay */

    //TODO - include any checks needed here to make sure hardware is available
   // I2CDevicePoll(); /* verify that ICR, CIU, etc are communicating over I2c */
    ReportToWatchdog(TelemetryAndControlWD); /* Tell the WD we are ok after that delay */

    /* When everything is settled after boot we can start collecting telemetry.  We don't want uninitialized values
     * to corrupt the telemetry, especially WOD or min/max */
    METTelemetryReady();


    while(1) {
        Intertask_Message messageReceived;
        int status;
        ReportToWatchdog(CurrentTaskWD);
        status = WaitInterTask(ToTelemetryAndControl, WATCHDOG_SHORT_WAIT_TIME, &messageReceived);
        ReportToWatchdog(CurrentTaskWD);
        if (status == pdFAIL){
            ReportError(RTOSfailure, false, ReturnAddr,(int) TelemAndControlTask);
        } else {
            //            int waiting=WaitingInterTask(ToTelemetryAndControl);
            //            if(waiting != 0){
            //                debug_print("MessagesWaiting=%d\n",WaitingInterTask(ToTelemetryAndControl));
            //            }
            switch(messageReceived.MsgType){
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
                break;

            case TacCollectMsg:
                tac_collect_telemetry(&telem_buffer);
                break;

            case TacSendRealtimeMsg:
                tac_send_telemetry(&telem_buffer);
                break;
            }
        }
    }
}

/**
 * tac_telem_timer_callback()
 *
 * This is called from a timer whenever the telemetry should be sent.
 */
void tac_telem_timer_callback() {
    statusMsg.MsgType = TacSendRealtimeMsg;
    NotifyInterTaskFromISR(ToTelemetryAndControl,&statusMsg);
}

/**
 * tac_maintenance_timer_callback()
 *
 * This is called from a timer whenever directory and upload table maintenance should be run.
 */
void tac_maintenance_timer_callback() {
    statusMsg.MsgType = TacMaintenanceMsg;
    NotifyInterTaskFromISR(ToTelemetryAndControl,&statusMsg);
}


void tac_collect_telemetry(telem_buffer_t *buffer) {
    //debug_print("Telem & Control: Collect RT telem\n");


    if(!IsStabilizedAfterBoot()) return;

    logicalTime_t time;
    getTimestamp(&time);
    buffer->header.uptime = time.METcount;
    buffer->header.resetCnt = time.IHUresetCnt;
    buffer->header.protocolVersion = 0;
    buffer->header.versionMajor = DownlinkVersionMajor;
    buffer->header.versionMinor = DownlinkVersionMinor;
    buffer->header.inScienceMode = 0;
    buffer->header.inHealthMode = 0;
    buffer->header.inSafeMode = 0;

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
        printf("TAC: Unable to check disk space with statvfs: %s\n", red_strerror(red_errno));
    } else {
        //printf("Free blocks: %d of %d.  Free Bytes: %d\n",redstatfs.f_bfree, redstatfs.f_blocks, redstatfs.f_frsize * redstatfs.f_bfree);
        //printf("Available File Ids: %d of %d.  \n",redstatfs.f_ffree, redstatfs.f_files);
        buffer->rtHealth.common.FSAvailable = redstatfs.f_bfree;
        buffer->rtHealth.common.FSTotalFiles = redstatfs.f_files - redstatfs.f_ffree;
    }
    buffer->rtHealth.common.UploadQueueBytes = (uint16_t)ftl0_get_space_reserved_by_upload_table();
    buffer->rtHealth.common.UploadQueueFiles = ftl0_get_num_of_files_in_upload_table();

    buffer->rtHealth.common2.pbEnabled = ReadMRAMBoolState(StatePbEnabled);
    buffer->rtHealth.common2.uplinkEnabled = ReadMRAMBoolState(StateUplinkEnabled);

    uint8_t temp8;
    if(Get8BitTemp31725(&temp8)) {
        buffer->rtHealth.common.IHUTemp = temp8;
    } else {
        debug_print("TAC: ERROR I2C temp request failed\n");
    }

    /* TX Telemetry */
    uint16_t rf_pwr = ax5043ReadReg(TX_DEVICE, AX5043_TXPWRCOEFFB0)
            + (ax5043ReadReg(TX_DEVICE, AX5043_TXPWRCOEFFB1) << 8);
    buffer->rtHealth.common.TXPower = rf_pwr;
    buffer->rtHealth.common.TXPwrMode = ax5043ReadReg(TX_DEVICE, AX5043_PWRMODE);

    /* RX0 Telemetry */
    uint8_t rssi0 = get_rssi(RX0_DEVICE);
    buffer->rtHealth.common.RX0RSSI = rssi0;
    buffer->rtHealth.common.RX0PwrMode = ax5043ReadReg(RX0_DEVICE, AX5043_PWRMODE);

    // TODO - calculate min max and store in MRAM

}

/**
 * Send telemetry that we have collected over the last period.
 * Realtime telemetry is sent in a UI frame.  The frame type depends on the mode and the
 * sequence
 * WOD telemetry is saved to a file
 *
 * The telemetry to send is determined by the mode that we are in.
 *
 */
void tac_send_telemetry(telem_buffer_t *buffer) {
    if(!IsStabilizedAfterBoot()) return;

    int len = 0;
    uint8_t *frame;
    if (ReadMRAMBoolState(StateCommandedSafeMode) || ReadMRAMBoolState(StateAutoSafe)) {
        /* Setup Type 1 frame - This will copy the buffer into the frame */
        realtimeFrame.header = buffer->header;
        realtimeFrame.rtHealth = buffer->rtHealth;
        len = sizeof(realtimeFrame);

        frame = (uint8_t *)&realtimeFrame;
//        debug_print("Sending Type 1 Frame %d:%d\n",realtimeFrame.header.resetCnt, realtimeFrame.header.uptime);
//        debug_print("Bytes sent:");
//        int i=0;
//        for (i=0; i<11; i++) {
//            debug_print("%0x ",frame[i]);
//        }
//        debug_print("\n");
    }
    if (len == 0 || len > AX25_MAX_INFO_BYTES_LEN) {
        debug_print("ERROR: Telemetry frame length of %d is not valid.  Frame not sent\n",len);
        return;

    }
    int rc = tx_send_ui_packet(BROADCAST_CALLSIGN, TLMP1, PID_NO_PROTOCOL, frame, len, BLOCK);

    uint32_t t = getUnixTime();
    //uint8_t time_frame[11];
    //snprintf((char *)time_frame, 11, "%d",t);
    //len = strlen((char *)time_frame);

    len = 4;
    t = htotl(t);
    uint8_t *time_frame;
    time_frame = (uint8_t *)&t;

    rc = tx_send_ui_packet(BROADCAST_CALLSIGN, TIME, PID_NO_PROTOCOL, time_frame, len, BLOCK);

}

