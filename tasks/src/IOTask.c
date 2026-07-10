/*
 * IOTask.c
 *
 *  Created on: July 9, 2026
 *      Author: Corey Minyard  AE5KM
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
 *
 */

#include "pacsat.h"
#include "can.h"
#include "reg_can.h"
#include "errors.h"
#include "acp.h"
#include "canDriver.h"
#include "IOTask.h"

void init_IOTask(void)
{
    /* Set this up before interrupts are enabled. */
    InitInterTask(ToIOTask, 40);
}

portTASK_FUNCTION_PROTO(IOTask, pvParameters)
{
    vTaskSetApplicationTaskTag((xTaskHandle) 0, (pdTASK_HOOK_CODE)IOTaskWD);
    /* Enough for all message boxes and some for other purposes. */
    ReportToWatchdog(IOTaskWD);

    CANSetup();

    for (;;) {
        Intertask_Message msg;
        int status;

        ReportToWatchdog(CurrentTaskWD);
        status = WaitInterTask(ToIOTask, CENTISECONDS(10), &msg);
        ReportToWatchdog(CurrentTaskWD);

        if (status != 1)
            /* No message. */
            continue;

        switch (msg.MsgType) {
        case CANRxDataMsg:
            CANHandleReceive(msg.argument, msg.argument2);
            break;

        case CANUpdateIDMsg:
	    CANHandleUpdateID(msg.argument, msg.argument2);
            break;

        case CANErrorMsg:
	    CANHandleError(msg.argument, msg.argument2);
            break;

        case CANStatusMsg:
	    CANHandleStatus(msg.argument, msg.argument2);
            break;

#ifdef AFSK_HARDWARE3
        case ACPHandleMsg:
            /* We handle ACP processing here, too. */
            acp_runner();
            break;
#endif
        }
    }
}
