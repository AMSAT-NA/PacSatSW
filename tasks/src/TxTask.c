/*
 * TncTask.c
 *
 *  Created on: Dec 1, 2022
 *      Author: g0kla
 */

#include <ax5043-ax25.h>
#include <TxTask.h>
#include "FreeRTOS.h"
#include "os_task.h"

#include "ax5043_access.h"

void radio_setup_16MHz_xtal_signal(bool txco, unsigned int xtal_cap);
void radio_setup_center_freq(bool vhf_band, uint32_t freq_reg_val);
void radio_setup_antenna_pins(bool use_amplifier);
void radio_set_power(uint32_t regVal);
void radio_setup_1200bps_tx();
void radio_setup_9600bps_tx();

portTASK_FUNCTION_PROTO(TxTask, pvParameters)  {

    vTaskSetApplicationTaskTag((xTaskHandle) 0, (pdTASK_HOOK_CODE)RadioWD ); // TODO - just reuse the Radio task name for now
    InitInterTask(ToRadio, 10);
    ResetAllWatchdogs();
    printf("Initializing TNC\n");

    // TODO - this code is here just for testing
    /* Initialize the Radio TX */

    ax5043StartTx();
 //   radio_setup_16MHz_xtal_signal(false, 5); /* Setup the oscillator for a real XTAL with 5pF capacitance added */
 //   radio_setup_center_freq(false, 0x1bd4cccd ); //0x1b3b5550 0x1bd4cccd - value from calc for 435800
 //   radio_setup_antenna_pins(false);
 //   radio_setup_1200bps_tx();
//
    radio_set_power(0x020); // minimum power to test

    /* Set Power state to FULL_TX */
     ax5043WriteReg(AX5043_PWRMODE, AX5043_PWRSTATE_FULL_TX);

    while(1) {

        uint8_t pktstart_flag = 0x01;
        uint8_t pktend_flag = 0x02;
        uint8_t raw_no_crc_flag = 0x18;
        // Flag of 0x18 is RAW no CRC

        /* Test Buffer PB Empty */
        uint8_t byteBuf[] = {0xA0,0x84,0x98,0x92,0xA6,0xA8,0x00,0xA0,0x8C,0xA6,0x66,
        0x40,0x40,0x17,0x03,0xF0,0x50,0x42,0x3A,0x20,0x45,0x6D,0x70,0x74,0x79,0x2E,0x0D};

        int numbytes = sizeof(byteBuf);
        ReportToWatchdog(CurrentTaskWD);
        vTaskDelay(pdMS_TO_TICKS(5*1000));
        ReportToWatchdog(CurrentTaskWD);

//        printf("FIFO_FREE 1: %d\n",fifo_free());
        ax5043WriteReg(AX5043_FIFOSTAT, 3); // clear FIFO data & flags
        fifo_repeat_byte(0x7E, 10, raw_no_crc_flag); // repeat the packet delimiter
        fifo_queue_buffer(byteBuf, numbytes, pktstart_flag|pktend_flag);
 //       printf("FIFO_FREE 2: %d\n",fifo_free());
        fifo_commit();
 //       printf("INFO: Waiting for transmission to complete\n");
        while (ax5043ReadReg(AX5043_RADIOSTATE) != 0) {
            vTaskDelay(1);
        }
 //       printf("INFO: Transmission complete\n");

    }
}

/**
 * Set the Fxtal signal.  Depends on TXCO vs Crystal
 * Sets the REFDIV to be less than 24.8MHz which is the signal into the Phase Comparator (PD)
 *
 * bool txco - true if this is a TXCO false for a crystal
 * unsigned int xtal_cap - The default capacitance is 8pF. This value adds 0.5 * xtal_cap pF.
 *
 * These values assume a 16MHz xtal
 */
void radio_setup_16MHz_xtal_signal(bool txco, unsigned int xtal_cap) {
    if (txco) {
        ax5043WriteReg(AX5043_XTALOSC                 ,0x04);
        ax5043WriteReg(AX5043_XTALAMPL                ,0x00);
    } else {
        ax5043WriteReg(AX5043_XTALOSC                 ,0x03); /* 03 for XTAL < 43MHz otherwise 0D */
        ax5043WriteReg(AX5043_XTALAMPL                ,0x07);
    }
    /* This sets XTAL_DIV. If the Crystal freq < 24.8MHz then XTAL_DIV is 1 otherwise XTAL_DIV is 2 and reg below would be 0x11*/
    ax5043WriteReg(AX5043_0xF35                   ,0x10);

    /* XTAL load capacitance is added by the chip and is not external on the board. So we need to set the
     * value here using formula C in pf = 8 + 0.5 * XTALCAP */
    ax5043WriteReg(AX5043_XTALCAP, xtal_cap);

#ifdef DEBUG
    /* By default this PIN outputs 1/16 of oscillator.  Set to higher rate to tweak capacitance */
    //ax5043WriteReg(AX5043_PINFUNCSYSCLK, 0x04); /* Output the Fxtal signal on SYSCLK GPIO */
    ax5043WriteReg(AX5043_PINFUNCSYSCLK, 0x06); /* Output the Fxtal signal / 4 on SYSCLK GPIO */
#endif

}

/**
 * The VCO and PLL need to be configured to set the center freq
 * This assumes an internal VCO.  An external VCO is needed for 525 - 800MHz
 *
 *
 * bool vhf_band - set to true for 2m of false for 70cm
 */
void radio_setup_center_freq(bool vhf_band, uint32_t freq_reg_val) {
    /*
     * For 27 - 262 MHz we need an external VCO inductor and RFDIV set to ON
     * For 54 - 526MHz we need an external VCO inductor and RFDIV set to OFF
     * For 400 - 525 MHz use internal inductor and RFDIV ON
     *
     * To set PLLVCODIV:
     * Bit 2 is RFDIV where 0 = OFF
     * Bit 4 is VCOSEL 1 = Use internal VCO2 with external inductor
     * Bit 5 is VCOINT2 1 = Use internal VCO2 with external inductor */

    if (vhf_band) {
        ax5043WriteReg(AX5043_0xF34                   ,0x08); /* PERFTUNE52 - RFDIV OFF */

        ax5043WriteReg(AX5043_PLLVCODIV               ,0x30);
    } else { // UHF
        ax5043WriteReg(AX5043_0xF34                   ,0x28); /* PERFTUNE52 - RFDIV ON */
        //ax5043WriteReg(AX5043_PLLVCODIV               ,0x04);
        ax5043WriteReg(AX5043_PLLVCODIV               ,0x24); // Per GOLF - use 24.  04 for internal inductor
    }

    /* PLL Loop filtering settings go here */
    // leave at defaults for now.  Per data sheet, we change this for faster start
    // up times, for lower phase noise with lower bandwidth and for FSK the bandwidth
    // must be of the order of the data rate

    /* Set the actual frequency based on formula: FREQA = Fcenter/Fxtal * 2^24 + 1/2 */
    /* Frequency should be avoided to be a multiple integer of the crystal
         * frequency, so we always set to 1 the LSB - per AX5043 documentation, to prevent synthesizer spurs */
    freq_reg_val |= 1;
    ax5043WriteReg(AX5043_FREQA0, freq_reg_val & 0xFF);
    ax5043WriteReg(AX5043_FREQA1, (freq_reg_val >> 8) & 0xFF);
    ax5043WriteReg(AX5043_FREQA2, (freq_reg_val >> 16) & 0xFF);
    ax5043WriteReg(AX5043_FREQA3, (freq_reg_val >> 24) & 0xFF);

    /* Then calibrate / auto range the PLL */
    /* First set bit 4 of AX5043_PLLRANGINGA to start auto ranging */
    unsigned int rng = ax5043ReadReg(AX5043_PLLRANGINGA);
    ax5043WriteReg(AX5043_PLLRANGINGA, rng & 0x10);
    /* Monitor that bit until it resets */
    while ((ax5043ReadReg(AX5043_PLLRANGINGA) & 0x10) != 0) {
        vTaskDelay(CENTISECONDS(1));
    }
    /* Check for lock*/
    unsigned int RNGERR = ax5043ReadReg(AX5043_PLLRANGINGA) & 0x20;
    unsigned int PLLLOCK = ax5043ReadReg(AX5043_PLLRANGINGA) & 0x40;
    printf("PLL Ranging Finished with RNGERR %d and PLLLOCK %d\n",RNGERR, PLLLOCK);
}

/**
 * This sets the antenna to use ANTP and ANTN if there is no amplifier. If
 * there is an amplifier it sets the output to ANTP1 and toggles the PWRAMP
 * PA outut pin - we probablly don't need this but it is a pin on the PI board
 *
 * TODO - MODCFGA sets bit shape as well, we should likely read first and just set the bits needed
 */
void radio_setup_antenna_pins(bool use_amplifier) {
    if (use_amplifier) {
        ax5043WriteReg(AX5043_PINFUNCANTSEL           ,0x01); // AMPLIFIER PA ON
        ax5043WriteReg(AX5043_MODCFGA                 ,0x06); //  Single Ended
    } else {
        ax5043WriteReg(AX5043_PINFUNCANTSEL           ,0x00); // AMPLIFIER PA OFF
        ax5043WriteReg(AX5043_MODCFGA                 ,0x05); // TX DIFFERENTIAL
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
    ax5043WriteReg(AX5043_TXPWRCOEFFB0,regVal);
    ax5043WriteReg(AX5043_TXPWRCOEFFB1,regVal>>8);
}

/**
 * Setup the modulation, bitrate, framing for TX
 *
 */
void radio_setup_1200bps_tx() {
    ax5043WriteReg(AX5043_MODULATION              ,0x0A); // AFSK Modulation

    /* TXRATE = BITRATE/Fxtal * 2^24 + 1/2
     * Where Fxtal = 16,000,000
     */
    /* Set the data rate to 0x04EB for 1200bps */
    ax5043WriteReg(AX5043_TXRATE2                 ,0x00);
    ax5043WriteReg(AX5043_TXRATE1                 ,0x04);
    ax5043WriteReg(AX5043_TXRATE0                 ,0xEA); // TODO - why is this EA and not EB in the GOLF code?

    /* Set the Frequency Deviation */
    /* Where FSKDEV = (0.858785 * Fdevisation) / Fxtal * 2^24 + 1/2
         * and Fdeviation = 3kHz  */
    /* Set FSK Deviation to 0A8E for 1200BPS FSK */
    ax5043WriteReg(AX5043_FSKDEV2,             0x00);
    ax5043WriteReg(AX5043_FSKDEV1,             0x0A);
    ax5043WriteReg(AX5043_FSKDEV0,             0x8E);

    /* For AFSK we need another pair of registers for MARK / SPACE
     * AFSK(Mark/Space) = Freq*2^18/Fxtal + 1/2
     * Freq Space = 2200Hz
     * Freq Mark = 1200Hz
     * */
    ax5043WriteReg(AX5043_AFSKSPACE1,          0x00);
    ax5043WriteReg(AX5043_AFSKSPACE0,          0x24); // 0r 25?  This was 24 in Golf
    ax5043WriteReg(AX5043_AFSKMARK1,           0x00);
    ax5043WriteReg(AX5043_AFSKMARK0,           0x14);

    /* Frequency Shape */
    // No shaping for normal AX.25.
    ax5043WriteReg(AX5043_MODCFGF,             0x00); // this is also the default value

    /* Can set Raised Cosine pre-distortion filter here */
    // NONE

    /* Set the power output */

}

void radio_setup_9600bps_tx() {
    ax5043WriteReg(AX5043_MODULATION              ,0x07); //(G)MSK Modulation

    /* TXRATE = BITRATE/Fxtal * 2^24 + 1/2
     * Where Fxtal = 16,000,000
     */
    /* Set the data rate to 0x2753 for 9600bps */
    ax5043WriteReg(AX5043_TXRATE2                 ,0x00);
    ax5043WriteReg(AX5043_TXRATE1                 ,0x27);
    ax5043WriteReg(AX5043_TXRATE0                 ,0x53);

    /* Set the Frequency Deviation */
    /* Where FSKDEV = (m *1/2*bitrate) / Fxtal * 2^24 + 1/2
     * and modulation index m = 0.5 (which makes it MSK)*/
    /* Set FSK Deviation to 09D5 for 9600bps MSK */
    ax5043WriteReg(AX5043_FSKDEV2,             0x00);
    ax5043WriteReg(AX5043_FSKDEV1,             0x09);
    ax5043WriteReg(AX5043_FSKDEV0,             0xD5);

    /* Frequency Shape */
    // No shaping for normal AX.25.  For GMSK we can use 0.5
    ax5043WriteReg(AX5043_MODCFGF,             0x00); // TODO - should this be 0.5 (Guassian BT = 0.5) for normal G3RUH


}

void radio_setup_ax25_framing() {

}
