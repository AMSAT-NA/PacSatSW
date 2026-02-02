/*
 * ax5043_access.c
 *
 *  Created on: Jun 15, 2019
 *      Author: bfisher
 *
 * This file contains the headers for AX5043 routines that are common
 * across modes and frequencies.  For example, it contains the
 * routines to read and write registers and the routines to start and
 * stop the radio.  It does not apply any configurations specific to
 * modes or frequency bands.  For PACSAT that is all contained in
 * ax5043_ax25.c
 *
 */


// Generic golf stuff--also include most CLIB things that are required

#include <pacsat.h>
#include "sys_common.h"
#include "spi.h"
#include "spi-replacement.h"
#include "gpioDriver.h"

//Pacsat headers
#include "spiDriver.h"
#include "ax5043_access.h"
#include "ax5043-ax25.h"

struct AX5043Info {
    SPIDevice spidev;

    bool on;
    bool rxing;
    bool txing;
};

static struct AX5043Info ax5043_info[NUM_CHANNELS] = {
    { .spidev = Rx1AX5043Dev },
#if NUM_CHANNELS > 2
    { .spidev = Rx2AX5043Dev },
    { .spidev = Rx3AX5043Dev },
    { .spidev = Rx4AX5043Dev },
#endif
    { .spidev = TxAX5043Dev },
};

static struct AX5043Info *ax5043_get_info(rfchan device)
{
    if (device >= NUM_CHANNELS)
        return NULL;
    return &ax5043_info[device];
}

void ax5043WriteReg(rfchan device, unsigned int reg, unsigned int val)
{
    //spiDAT1_t dc = {.WDEL = false, .CS_HOLD = true, .DFSEL = SPI_FMT_0, .CSNR = 1 };
    uint8_t srcbuf[3];
    struct AX5043Info *info = ax5043_get_info(device);

    if (!info)
        return;

    srcbuf[0] = 0x00f0 | ((reg & 0xf00) >> 8);
    srcbuf[1] = (reg & 0xff);
    srcbuf[2] = val & 0xff;

    SPISendCommand(info->spidev, 0, 0, srcbuf, 3, 0, 0);

#if 0
    if ((reg != AX5043_FIFODATA) && (reg != AX5043_FIFOSTAT)) {
        if ((reg < (sizeof(axregs)/sizeof(axregs[0]))) && (axregs[reg] != NULL)) {
            printf("ax5043WriteReg: %s: %02.2x\n", axregs[reg], val);
        } else {
            printf("ax5043WriteReg: %x: %02.2x\n", reg, val);
        }
    }
#endif
}

static unsigned int ax5043ReadLongreg(rfchan device,
                                      unsigned int reg, int bytes)
{
    uint8_t srcbuf[2];
    uint8_t dstbuf[4] = { 0, 0, 0, 0};
    unsigned int retval=0, i;
    struct AX5043Info *info = ax5043_get_info(device);

    if (!info)
        return 0;

    if (bytes > 4)
        bytes = 4;

    srcbuf[0] = 0x0070 | ((reg & 0xf00) >> 8);
    srcbuf[1] = (reg & 0xff);

    SPISendCommand(info->spidev, 0, 0, srcbuf, 2, dstbuf, bytes);
    for (i=0; i < bytes; i++) {
        retval <<= 8;
        retval |= dstbuf[i];
    }
    return retval;
}

unsigned int ax5043ReadReg(rfchan device, unsigned int reg)
{
    return ax5043ReadLongreg(device, reg, 1);
}

bool ax5043RxWorking(rfchan device)
{
    int power_mode = ax5043ReadReg(device, AX5043_PWRMODE);

    /* If Power Mode != 9 then RX is not on or working */
    return power_mode == AX5043_PWRSTATE_FULL_RX;
}

void ax5043Test(rfchan device)
{
    ax5043WriteReg(device, AX5043_SCRATCH, device);

    printf("AX5043 #%d: Revision=0x%x, scratch=%d\n", device,
           ax5043ReadReg(device, AX5043_SILICONREVISION),
           ax5043ReadReg(device, AX5043_SCRATCH));
}

static int32_t axradio_conv_freq_tohz(int32_t f)
{
    return (int32_t) (f / 1.048576);
}

void ax5043Dump(rfchan dev)
{
    uint32_t val;

    printf("AX5043 dev %d\n", dev);
    printf(" FIFOSTAT: %02x\n", ax5043ReadReg(dev, AX5043_FIFOSTAT));
    printf(" PWRMODE:: %02x\n", ax5043ReadReg(dev, AX5043_PWRMODE));
    printf(" XTALCAP: %d\n", ax5043ReadReg(dev, AX5043_XTALCAP));
    printf(" PLLLOOP: %02.2x\n", ax5043ReadReg(dev, AX5043_PLLLOOP));
    printf(" PLLCPI: %02.2x\n", ax5043ReadReg(dev, AX5043_PLLCPI));
    printf(" PLLVCOI: %02.2x\n", ax5043ReadReg(dev, AX5043_PLLVCOI));
    printf(" PLLRANGINGA: %02.2x\n", ax5043ReadReg(dev, AX5043_PLLRANGINGA));
    printf(" PLLVCODIV: %02.2x\n", ax5043ReadReg(dev, AX5043_PLLVCODIV));
    //            printf(" FREQ: %x", ax5043ReadReg(dev, AX5043_FREQA0));
    //            printf(" %x", ax5043ReadReg(dev, AX5043_FREQA1));
    //            printf(" %x", ax5043ReadReg(dev, AX5043_FREQA2));
    //            printf(" %x\n", ax5043ReadReg(dev, AX5043_FREQA3));
    val = ax5043ReadReg(dev, AX5043_FREQA0)
        + (ax5043ReadReg(dev, AX5043_FREQA1) << 8)
        + (ax5043ReadReg(dev, AX5043_FREQA2) << 16)
        + (ax5043ReadReg(dev, AX5043_FREQA3) << 24);
    printf(" FREQ %d Hz\n", axradio_conv_freq_tohz(val));
    printf(" MODULATION: %x\n", ax5043ReadReg(dev, AX5043_MODULATION));
    printf(" TXPWRCOEFFB0: %x\n", ax5043ReadReg(dev, AX5043_TXPWRCOEFFB0));
    printf(" TXPWRCOEFFB1: %x\n", ax5043ReadReg(dev, AX5043_TXPWRCOEFFB1));
}

/*
 * Common area to start and stop transmit and receive and to power off
 * for safe mode
 */

#ifdef AFSK_HARDWARE
static Gpio_Use ax5043_power_gpio[NUM_CHANNELS] = {
    AX5043_Rx1_Power,
#if NUM_CHANNELS > 2
    AX5043_Rx2_Power,
    AX5043_Rx3_Power,
    AX5043_Rx4_Power,
#endif
    AX5043_Tx_Power,
};
#endif

void ax5043PowerOn(rfchan device)
{
    struct AX5043Info *info = ax5043_get_info(device);

    if (!info)
        return;

#ifdef AFSK_HARDWARE
    GPIOSetOn(ax5043_power_gpio[device]);
#endif
    info->on = true;
    vTaskDelay(CENTISECONDS(1)); // Don't try to mess with it for a bit
}

void ax5043PowerOff(rfchan device)
{
    struct AX5043Info *info = ax5043_get_info(device);

    if (!info)
        return;

#ifdef AFSK_HARDWARE
    if (is_tx_chan(device))
        // Make sure the PA is off if we are turning off the TX 5043.
        GPIOSetOff(SSPAPower);
    GPIOSetOff(ax5043_power_gpio[device]);
#endif

    info->on = false;
    info->rxing = false;
    info->txing = false;
}

bool ax5043_rxing(rfchan device)
{
    struct AX5043Info *info = ax5043_get_info(device);

    if (!info)
        return false;

    return info->rxing;
}

static uint8_t ax5043_off_xtal(rfchan device)
{
    ax5043WriteReg(device, AX5043_PWRMODE, AX5043_PWRSTATE_XTAL_ON);
    ax5043WriteReg(device, AX5043_LPOSCCONFIG, 0x00); // LPOSC off

    return AXRADIO_ERR_NOERROR;
}

uint8_t ax5043_off(rfchan device)
{
    uint8_t retVal;

    retVal = ax5043_off_xtal(device);
    if (retVal != AXRADIO_ERR_NOERROR) {
        return retVal;
    }

    ax5043WriteReg(device, AX5043_PWRMODE, AX5043_PWRSTATE_POWERDOWN);

    return AXRADIO_ERR_NOERROR;
}

/**
 * TODO - this now starts the AX25 RX.  It should be setup to start
 * multiple receivers We also need another function to start the
 * command receiver.
 *
 */
void ax5043StartRx(rfchan device,
                   uint32_t freq, enum radio_modulation mod)
{
    struct AX5043Info *info = ax5043_get_info(device);

    if (!info)
        return;

    //printf("StartRx: Power=%d,Txing=%d,Rxing=%d\n",PowerOn,Txing,Rxing);
    ax5043StopTx(device);
    if (!info->on) {
        ax5043PowerOn(device);
        info->on = true;
    }
    if (!info->rxing) {
        start_ax25_rx(device, freq, mod, 0);
        info->rxing = true;
        info->txing = false;
    }
}

void ax5043StopRx(rfchan device)
{
    struct AX5043Info *info = ax5043_get_info(device);

    if (!info)
        return;

    //printf("StopRx: Power=%d,Txing=%d,Rxing=%d\n",PowerOn,Txing,Rxing);
    if (info->rxing & info->on) {
        ax5043PowerOff(device);
        info->on = false;
        info->txing = false;
        info->rxing = false;
    }
}

void ax5043StartTx(rfchan device,
                   uint32_t freq, enum radio_modulation mod)
{
    struct AX5043Info *info = ax5043_get_info(device);

    if (!info)
        return;

    //printf("StartTx: Power=%d,Txing=%d,Rxing=%d\n",PowerOn,Txing,Rxing);
    ax5043StopRx(device);

    if (!info->on) {
        ax5043PowerOn(device);
        info->on = true;
    }

    if (!info->txing) {
        start_ax25_tx(device, freq, mod, 0);
        info->txing = true;
        info->rxing = false;
    }
}

void ax5043StopTx(rfchan device)
{
    struct AX5043Info *info = ax5043_get_info(device);

    if (!info)
        return;

    //printf("StopTx: Power=%d,Txing=%d,Rxing=%d\n",PowerOn,Txing,Rxing);
    if (info->txing && info->on) {
        ax5043_off(device);
        info->txing = false;
        info->rxing = false;
    }
    ax5043PowerOff(device);
}
