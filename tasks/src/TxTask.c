/*
 * TncTask.c
 *
 *  Created on: Dec 1, 2022
 *      Author: g0kla
 */

#include "pacsat.h"
#include "ax5043_access.h"
#include "ax5043-ax25.h"
#include "TxTask.h"
#include "FreeRTOS.h"
#include "os_task.h"

void radio_set_power(uint32_t regVal);

static uint8_t tx_packet_buffer[AX25_PKT_BUFFER_LEN];
static SPIDevice device = DCTDev1;

/* Test Buffer PB Empty */
uint8_t byteBuf[] = {0xA0,0x84,0x98,0x92,0xA6,0xA8,0x00,0xA0,0x8C,0xA6,0x66,
                     0x40,0x40,0x17,0x03,0xF0,0x50,0x42,0x3A,0x20,0x45,0x6D,0x70,0x74,0x79,0x2E,0x0D};

portTASK_FUNCTION_PROTO(TxTask, pvParameters)  {

    vTaskSetApplicationTaskTag((xTaskHandle) 0, (pdTASK_HOOK_CODE)RadioWD ); // TODO - just reuse the Radio task name for now
    InitInterTask(ToTxTask, 10);
    ResetAllWatchdogs();
    printf("Initializing TX\n");

    /* This is defined in pacsat.h, declared here */
    xTxPacketQueue = xQueueCreate( TX_PACKET_QUEUE_LEN, AX25_PKT_BUFFER_LEN * sizeof( uint8_t ) );
    if (xTxPacketQueue == NULL) {
        /* The queue could not be created.  This is fatal and should only happen in test if we are short of memory at startup */
        debug_print("FATAL ERROR: Could not create TX Packet Queue\n");
    }


    ax5043StartTx(device);
    radio_set_power(0x020); // minimum power to test

    /* Set Power state to FULL_TX */
     ax5043WriteReg(device, AX5043_PWRMODE, AX5043_PWRSTATE_FULL_TX);

    while(1) {

        uint8_t pktstart_flag = 0x01;
        uint8_t pktend_flag = 0x02;
        uint8_t raw_no_crc_flag = 0x18; // Flag of 0x18 is RAW no CRC

        int numbytes = sizeof(byteBuf);
        ReportToWatchdog(CurrentTaskWD);
        BaseType_t xStatus = xQueueReceive( xTxPacketQueue, &tx_packet_buffer, CENTISECONDS(10) );  // TODO - adjust block time vs watchdog
        if( xStatus == pdPASS ) {
            /* Data was successfully received from the queue */
            //        vTaskDelay(pdMS_TO_TICKS(5*1000));
            ReportToWatchdog(CurrentTaskWD);

            //        printf("FIFO_FREE 1: %d\n",fifo_free());
            ax5043WriteReg(device, AX5043_FIFOSTAT, 3); // clear FIFO data & flags
            fifo_repeat_byte(device, 0x7E, 10, raw_no_crc_flag); // repeat the packet delimiter
            fifo_queue_buffer(device, tx_packet_buffer, numbytes, pktstart_flag|pktend_flag);
            //       printf("FIFO_FREE 2: %d\n",fifo_free());
            fifo_commit(device);
            //       printf("INFO: Waiting for transmission to complete\n");
            while (ax5043ReadReg(device, AX5043_RADIOSTATE) != 0) {
                vTaskDelay(1);
            }
            //       printf("INFO: Transmission complete\n");
        }
    }
}

/**
 * Pout = Pmax * txpwrcoeffb / (2^12 - 1)
 * Where Pmax = 0dBm
 *
 * FFF is max
 *
 */
void radio_set_power(uint32_t regVal) {
    ax5043WriteReg(device, AX5043_TXPWRCOEFFB0,regVal);
    ax5043WriteReg(device, AX5043_TXPWRCOEFFB1,regVal>>8);
}


//void radio_setup_9600bps_tx() {
//    ax5043WriteReg(AX5043_MODULATION              ,0x07); //(G)MSK Modulation
//
//    /* TXRATE = BITRATE/Fxtal * 2^24 + 1/2
//     * Where Fxtal = 16,000,000
//     */
//    /* Set the data rate to 0x2753 for 9600bps */
//    ax5043WriteReg(AX5043_TXRATE2                 ,0x00);
//    ax5043WriteReg(AX5043_TXRATE1                 ,0x27);
//    ax5043WriteReg(AX5043_TXRATE0                 ,0x53);
//
//    /* Set the Frequency Deviation */
//    /* Where FSKDEV = (m *1/2*bitrate) / Fxtal * 2^24 + 1/2
//     * and modulation index m = 0.5 (which makes it MSK)*/
//    /* Set FSK Deviation to 09D5 for 9600bps MSK */
//    ax5043WriteReg(AX5043_FSKDEV2,             0x00);
//    ax5043WriteReg(AX5043_FSKDEV1,             0x09);
//    ax5043WriteReg(AX5043_FSKDEV0,             0xD5);
//
//    /* Frequency Shape */
//    // No shaping for normal AX.25.  For GMSK we can use 0.5
//    ax5043WriteReg(AX5043_MODCFGF,             0x00); // TODO - should this be 0.5 (Guassian BT = 0.5) for normal G3RUH
//
//
//}

