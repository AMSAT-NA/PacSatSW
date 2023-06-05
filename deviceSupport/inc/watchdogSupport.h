/*
 * watchdog.h
 *
 *  Created on: Jun 5, 2019
 *      Author: bfisher
 *  Updated with WD Code Oct 10, 2019 R.Gopstein
 */

#ifndef UTILITIES_INC_WATCHDOGSUPPORT_H_
#define UTILITIES_INC_WATCHDOGSUPPORT_H_

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "os_task.h"
#include "os_timer.h"

#define WATCHDOG_CHECK_FREQ_SECS 10

typedef enum {
    CurrentTaskWD=0,
    TelemetryAndControlWD,
    RxTaskWD,
    TxTaskWD,
    Ax25TaskWD,
    UplinkTaskWD,
    PBTaskWD,
    CommandWD,
    CANSupportWD,
    IdleWD, // Not used in RT
    /* Add any new tasks to watch in here.  Maximum number of tasks is 9. */
    /* The nine limit is the number of bits available in the downlink */
    LastWD,
    InterruptThd = LastWD,
    ConsoleTsk
    /*
     * NB: Task numbers must be the same in both RTIHU and LIHU and any changes must be recorded in FoxTelem's
     * golf_task_name.tab file.
     */
}WdReporters_t;
#if LastWD > 9
#error Too many watchdog tasks
#endif

void ReportToWatchdog(WdReporters_t reporter);
void CheckAndResetWatchdogs(xTimerHandle x);
void ResetExternalWatchdog(void);
void ResetInternalWatchdog(void);
void ResetAllWatchdogs(void);
void InitWatchdog(void);
void StartWatchdogTimer(void);
void WaitTaskWithWatchdog(TickType_t ticks);
void ForceInternalWatchdogTrigger(void);
void ForceExternalWatchdogTrigger(void);
void WaitSystemWithWatchdog(int time);
void ReportAllToWatchdog(void);

#define DWD_RESET_VALUE 0xfff /*Set internal hardware watchdog to maximum value, which about .42 seconds */
#define WD_TIMER_IN_CENTI 5 /*Set the timer to ever 10/100s of a second*/
#define WD_TIMER CENTISECONDS(WD_TIMER_IN_CENTI) /* Convert to the value the OS wants for the timer */
/*
 * We decrement a counter every WDSupportTimerFreq 100ths of a second so for the counter to 0
 * every CHECK_FREQ_SECS, it is initialized to CHECK_FREQ_SECS*100/TimerFreqCent
 */
#define WD_CHECK_REPORT_FREQ WATCHDOG_CHECK_FREQ_SECS*(100/WD_TIMER_IN_CENTI)

#endif /* UTILITIES_INC_WATCHDOGSUPPORT_H_ */
