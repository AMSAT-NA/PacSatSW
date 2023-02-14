/*
 * TncTask.c
 *
 *  Created on: Dec 1, 2022
 *      Author: g0kla
 */

#include "RxTask.h"
#include "FreeRTOS.h"
#include "os_task.h"

#include "ax5043_access.h"
//#include "ax5043-ax25.h"
#include "ax5043-2M-AFSK.h"
#include "ax5043-2M-AFSK-externs.h"

uint8_t getRssi();

static uint8_t PAPowerFlagCnt=0,DCTPowerFlagCnt=0;

portTASK_FUNCTION_PROTO(RxTask, pvParameters)  {

    vTaskSetApplicationTaskTag((xTaskHandle) 0, (pdTASK_HOOK_CODE)RadioWD ); // TODO - just reuse the Radio task name for now
    InitInterTask(ToRadio, 10);
    ResetAllWatchdogs();
    printf("Initializing Rx\n");

    /* Initialize the Radio RX */
    ax5043StartRx();

    while(1) {
        Intertask_Message messageReceived;
        int status = 0;
        int bytesRead = 0;
        uint8_t rssi = 0;

        ReportToWatchdog(CurrentTaskWD);
        status = WaitInterTask(ToRadio, CENTISECONDS(1), &messageReceived);  // This is triggered when there is RX data on the FIFO
        ReportToWatchdog(CurrentTaskWD);
        rssi = getRssi();
        if (rssi > 170) {
            debug_print("RSSI: %d   ",rssi);
            debug_print("FRMRX: %d   ",ax5043ReadReg(AX5043_FRAMING) & 0x80 ); // FRAMING Pkt start bit detected - will print 128
            debug_print("RADIO: %d\n",ax5043ReadReg(AX5043_RADIOSTATE) & 0xF ); // Radio State bits 0-3
        }
            if ((ax5043ReadReg(AX5043_FIFOSTAT) & 0x01) != 1) { // FIFO not empty
               debug_print("FIFO NOT EMPTY\n");
               uint8_t fifo_cmd = ax5043ReadReg(AX5043_FIFODATA); // read command
               uint8_t len = (fifo_cmd & 0xE0) >> 5; // top 3 bits encode payload len
               fifo_cmd &= 0x1F;
               debug_print("FIFO CMD: %d LEN:%d\n",fifo_cmd,len);
               while (len--) {
                   uint8_t b = ax5043ReadReg(AX5043_FIFODATA);
                   debug_print("%x ",b);
               }
               debug_print("\n");
               if (fifo_cmd == AX5043_FIFOCMD_DATA) {


               } else {

               }
            }

        if (status==1) { // We received a message
            switch(messageReceived.MsgType){
            case DCTPowerFlagMsg:
                debug_print("AX5043 Power Interrupted\n");
                DCTPowerFlagCnt++;
                break;
            case PAPowerFlagMsg:
                debug_print("Power Amp Power Interrupted\n");
                PAPowerFlagCnt++;
                break;

            case DCTInterruptMsg:

                if ((ax5043ReadReg(AX5043_PWRMODE) & 0x0F) == AX5043_PWRSTATE_FULL_RX) {

                    printf("Interrupt while in FULL_RX mode\n");
                    //printf("IRQREQUEST1: %02x\n", ax5043ReadReg(AX5043_IRQREQUEST1));
                    //printf("IRQREQUEST0: %02x\n", ax5043ReadReg(AX5043_IRQREQUEST0));
                    //printf("FIFOSTAT: %02x\n", ax5043ReadReg(AX5043_FIFOSTAT));
                    if ((ax5043ReadReg(AX5043_FIFOSTAT) & 0x01) == 0) {
                      //  printf("FIFO not empty\n");
                        //ax5043WriteReg(AX5043_FIFOSTAT, 3); // clear FIFO data & flags - Temporary until we can process the FIFO
                        //printf("After emptying FIFOSTAT: %02x\n", ax5043ReadReg(AX5043_FIFOSTAT));
                        if (rssi > 160) {
                            bytesRead = receive_packet_2m();
                            printf("bytes read from FIFO: %d\n", bytesRead);
                        }

                        //Clear status bits
                        //ax5043ReadReg(AX5043_RADIOEVENTREQ1);
                        //ax5043ReadReg(AX5043_RADIOEVENTREQ0);


 /////                       printf("bytes received: %02x %02x %02x\n", axradio_rxbuffer_2m[0], axradio_rxbuffer_2m[1], axradio_rxbuffer_2m[2]);

                        //incomingRawSoftwareCommand(axradio_rxbuffer_2m);


                    }
                    break;
                } else {
                    //printf("AX5043 Interrupt in pwrmode: %02x\n", ax5043ReadReg(AX5043_PWRMODE));
                }

          }
        }
    }
}

uint8_t getRssi() {
    int8_t byteVal;
    int16_t wordVal;
    byteVal = (int8_t)ax5043ReadReg(AX5043_RSSI);
    wordVal = (int16_t)byteVal;
    wordVal -=64;
    wordVal +=255;
    return (uint8_t)wordVal;
}

