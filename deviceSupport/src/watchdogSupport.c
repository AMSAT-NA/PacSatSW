/*
 * watchdog.c
 *
 *  Created on: Jun 5, 2019
 *      Author: bfisher, Rich Gopstein
 *  Updated with WD Code Oct 10, 2019 R.Gopstein
 */

 #include "FreeRTOS.h"
 #include "os_task.h"
 #include "os_timer.h"
 #include "MET.h"
 #include "errors.h"
 #include "watchdogSupport.h"
 #include "rti.h"

 #ifdef WD_DEBUG
 #include "serialDriver.h"
 #endif

static uint32_t WatchdogBits;
static bool DoExternalResetOk=true;
void StartWatchdogTimer(void) {
  xTimerHandle handle;
  volatile portBASE_TYPE timerStatus;
  int pvtID = 1;

  /* create a recurring RTOS software timer */
  handle = xTimerCreate( "Watchdog", WD_TIMER, TRUE, &pvtID, CheckAndResetWatchdogs);

  /* start the timer */
  timerStatus = xTimerStart(handle, 0);
  if (timerStatus != pdPASS){
      ReportError(RTOSfailure, FALSE, ReturnAddr, (int) StartWatchdogTimer); /* failed to create the RTOS timer */
  }
}


void InitWatchdog(void) {
    /*
     * Starts Watchdog - must be called when Privileged (i.e. in Main or timer routine)
     */
#ifdef WATCHDOG_ENABLE
    dwdInit(DWD_RESET_VALUE);
    dwdCounterEnable();
#endif
    DoExternalResetOk = true;
}
void ForceInternalWatchdogTrigger(void){
    while(1){ResetExternalWatchdog();}
}
void ForceExternalWatchdogTrigger(void){
    DoExternalResetOk = false;
    while(1){ResetInternalWatchdog();}
}
void ResetExternalWatchdog(void){
    static bool isOn=true;
    if(DoExternalResetOk){
        if(isOn){
            GPIOSetOff(WatchdogFeed);
        } else {
            GPIOSetOn(WatchdogFeed);
        }
        isOn = !isOn;
    }
}
void ResetInternalWatchdog(void){
#ifdef WATCHDOG_ENABLE    /* start up the watchdog - relies on CheckAndResetWatchdogs running faster than timeout */
    dwdReset();
#endif  /* start up the watchdog - relies on CheckAndResetWatchdogs running faster than timeout */
}
void ReportAllToWatchdog(void){
    /*
     * This reports in all tasks to the watchdogs.  So this would be used after the tasks are
     * going, but when there has been an intentional or expected holdup that might prevent tasks from running.
     *
     */

#ifdef WD_DEBUG
     SerialPutString(PRINTF_COM,"RAtW\r\n",0);
#endif

     WatchdogBits = (1<<(LastWD-1))-1; /* Say that everyone has reported in...for initialization mainly */
     ResetAllWatchdogs();
}

void ResetAllWatchdogs(void){
    /*
      *This will immediately force a reset on any watchdogs that exist.  So
      *this one is used before all the tasks are running to insure no watchdog
      *this trips.
    */

    ResetInternalWatchdog();
    ResetExternalWatchdog();
}

void ReportToWatchdog(WdReporters_t reporter){
#ifdef WD_DEBUG
  char rep[10];
#endif
    /*
     * This one is what an individual task calls to report that it is ok
     */

     /*
   	 * Note: If this is to be called by an ISR as well as a task,
   	 * we must disable interrupts
   	 */

   	/*
   	 * If the caller passed CurrentTaskWD, then use that tag associated with the current process.
   	 * This could be used in I/O for example.
   	 */
   	if(reporter==CurrentTaskWD)
   	    // All this casting  avoids a warning
   		reporter = (WdReporters_t)(((uint32_t)xTaskGetApplicationTaskTag(0)));

#ifdef WD_DEBUG
      SerialPutString(PRINTF_COM,"RtW ",0);
      rep[0] = '0' + (uint8_t) reporter;
      rep[1] = '\000';
      SerialPutString(PRINTF_COM, rep,0);
      SerialPutString(PRINTF_COM, "\r\n",0);
#endif

   	/*
   	 * Note that the WdBits_t values
   	 */
   	WatchdogBits |= (1<<(reporter-1));
}
void CheckAndResetWatchdogs(xTimerHandle x){

    /*
     * The timer calls this one regularly.  The idea here is that if the h/w watchdog is too fast
     * to be practical, we call this fast enough to reset the w/d but then every N times we call it
     * we check if all task have reported in, and if not, do not reset it.
     */
#ifdef WATCHDOG_ENABLE

     static int CheckReportCounter=WD_CHECK_REPORT_FREQ;
     /*
      * We are not going to require that the watchdog be reset by processes as frequently
      * as they need setting.  The external watchdog can't be set for as long as we would
      * like.  Therefore, we have the tasks set their bit in a bitmask and every
      * WATCHDOG_CHECK_FREQ_SECS seconds we make sure that they are all set or else we
      * do not reset the watchdog.
      */
     if(CheckReportCounter>0){
       ResetAllWatchdogs();
       CheckReportCounter--;
       SaveAcrossReset.fields.errorData &= 0xfffffffe; // Todo Clear the bottom bit to show no WD problem
       return;
     }
     SaveAcrossReset.fields.wdReports = WatchdogBits; // Todo Save the reporting status in case of a WD timeout
     SaveAcrossReset.fields.errorData |= 1;  // Todo Set the bottom bit to indicate potentially a real WD timeout
     if((WatchdogBits & ((1<<(LastWD-1))-1)) == (1<<(LastWD-1))-1){
       /*
        * Note that LastWD is 1 greater than the last WD that must
        * check in.  Thus we subtract 1, shift, and subtract 1 again
        * to get all the bits that must be set.
        * Everyone has checked in.  We can reset the WDs
        */
       ResetAllWatchdogs();
       WatchdogBits = 0; /* Start again */
       CheckReportCounter = WD_CHECK_REPORT_FREQ; /* Remember we get called twice per second */
     } // else don't reset the WD so it may die soon!
     else {
         printf("WD:%x\n",WatchdogBits);
     }
#else
     ResetAllWatchdogs(); // If WD is compiled not enabled, still reset the hardware regularly.
#endif
}

#ifdef WD_DEBUG

StartWatchdogResetDebug(void) {
  xTimerHandle handle;
  volatile portBASE_TYPE timerStatus;
  int pvtID = 1;

  /* create a recurring RTOS software timer */
  handle = xTimerCreate( "Debug", CENTISECONDS(100), TRUE, &pvtID, ResetAllWatchdogsStub);

  /* start the timer */
  timerStatus = xTimerStart(handle, 0);
  if (timerStatus != pdPASS){
      ReportError(RTOSfailure, FALSE, ReturnAddr, (int) StartWatchdogTimer); /* failed to create the RTOS timer */
  }
}
#endif

/*
 * Use this in place of just plain-old vTaskDelay to ensure there are sufficient calls to
 * the watchdog to keep it happy with your task
 */
void WaitTaskWithWatchdog(TickType_t ticks){
    ReportToWatchdog(CurrentTaskWD);
    while(ticks > WATCHDOG_SHORT_WAIT_TIME){
        ticks -= WATCHDOG_SHORT_WAIT_TIME;
        vTaskDelay(WATCHDOG_SHORT_WAIT_TIME);
        ReportToWatchdog(CurrentTaskWD);
    }
    vTaskDelay(ticks);
    ReportToWatchdog(CurrentTaskWD);
}
/*
 * And this one before tasks are actually started up (like when
 * releasing antennas
 */
void WaitSystemWithWatchdog(int time){
    while (time > 0){
        ResetAllWatchdogs();
        if (time > SECONDS(1)){
            vTaskDelay(SECONDS(1));
            time -= SECONDS(1);
        } else {
            vTaskDelay(time);
            time=0;
        }
    }
}

