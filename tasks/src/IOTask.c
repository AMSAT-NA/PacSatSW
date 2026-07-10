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
#include "adc_proc.h"
#include "canDriver.h"
#include "TelemAndControlTask.h"
#include "IOTask.h"

/* timer to read the ADC periodically */
static xTimerHandle timerADC;
static Intertask_Message statusMsg;

void init_IOTask(void)
{
    /* Set this up before interrupts are enabled. */
    InitInterTask(ToIOTask, 40);
}

/**
 * adc_timer_callback()
 *
 * This is called from a timer whenever the ADC conversion process
 * needs to start.
 */
void adc_timer_callback(TimerHandle_t xTimer)
{
    statusMsg.MsgType = ADCStartMsg;
    NotifyInterTaskFromISR(ToIOTask, &statusMsg);
}

portTASK_FUNCTION_PROTO(IOTask, pvParameters)
{
    portBASE_TYPE timerStatus;

    vTaskSetApplicationTaskTag((xTaskHandle) 0, (pdTASK_HOOK_CODE)IOTaskWD);
    /* Enough for all message boxes and some for other purposes. */
    ReportToWatchdog(IOTaskWD);

    CANSetup();

    /* Create a periodic timer for reading the ADC */
    timerADC = xTimerCreate("ADC",
                            IO_TIMER_ADC_PERIOD, TRUE,
                            NULL, adc_timer_callback);
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

	case ADCStartMsg:
	    adc_start_conversion();
	    break;

	case ADCProcessMsg:
	    adc_process_data();
	    tac_check_auto_safe();
	    break;
        }
    }
}
