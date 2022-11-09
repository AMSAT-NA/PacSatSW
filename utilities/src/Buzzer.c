/*
 * Buzzer.c
 *
 *  Created on: Jun 10, 2020
 *      Author: bfisher
 */
#include <pacsat.h>
#include "stdint.h"
#include "stdio.h"
#include "errors.h"
#include "FreeRTOS.h"
#include "os_task.h"
#include "gpioDriver.h"

#define POST_OS_LOOP_FACTOR 5000

void Buzzer(uint32_t burnTime,int pitch){
    int buzzCount;
    int timeLoop = (int)burnTime;
    int buzzLoop;
    portTickType stopTime = xTaskGetTickCount() + burnTime;
    if(pitch==0)pitch=1;
    xTaskGetTickCount();
    buzzLoop = POST_OS_LOOP_FACTOR/pitch;
    timeLoop *= pitch;
    while(xTaskGetTickCount() < stopTime){
        buzzCount=buzzLoop;
        GPIOSetOn(Alert);
        while(buzzCount > 0){buzzCount--;ResetAllWatchdogs();}
        GPIOSetOff(Alert);
        buzzCount=buzzLoop;
        while(buzzCount > 0){buzzCount--;ResetAllWatchdogs();}
        ResetAllWatchdogs();
        //taskYIELD();
    }
    ResetAllWatchdogs();
}

