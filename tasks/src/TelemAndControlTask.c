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

#include "MET.h"
#include "pacsat.h"
#include "PbTask.h"
#include "Ax25Task.h"

/**
 * The Telemetry and Control task ...
 *
 */
portTASK_FUNCTION_PROTO(TelemAndControlTask, pvParameters)  {
    vTaskSetApplicationTaskTag((xTaskHandle) 0, (pdTASK_HOOK_CODE)TelemetryAndControlWD );
    InitInterTask(ToTelemetryAndControl, 10);

    ResetAllWatchdogs();
    debug_print("Initializing Telem and Control Task\n");

    // TODO - this is just for testing
    //METTelemetryReady();

    while(1) {
        Intertask_Message messageReceived;
        int status;
        ReportToWatchdog(CurrentTaskWD);
        status = WaitInterTask(ToTelemetryAndControl, SECONDS(2), &messageReceived);
        ReportToWatchdog(CurrentTaskWD);
        if(status == 1){
            int waiting=WaitingInterTask(ToTelemetryAndControl);
            if(waiting != 0){
                debug_print("MessagesWaiting=%d\n",WaitingInterTask(ToTelemetryAndControl));
            }
            switch(messageReceived.MsgType){
            case TelemSendPbStatus:
                debug_print("Telem & Control: Send the PB Status\n");
                pb_send_status();
                break;

            case TelemSendUplinkStatus:
                debug_print("Telem & Control: Send the FTL0 Status\n");
                ax25_send_status();
                break;

            case TelemCollectMsg:
//                debug_print("Telem & Control: Collect RT telem\n");

                break;



            }
        }
    }
}



