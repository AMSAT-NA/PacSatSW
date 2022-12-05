// Portions of the code below are subject to the following licenses:

// Copyright (c) 2018-2019 Jonathan C. Brandenburg
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// Copyright (c) 2007,2008,2009,2010,2011,2012,2013, 2014 AXSEM AG
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1.Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     2.Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     3.Neither the name of AXSEM AG, Duebendorf nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//     4.All advertising materials mentioning features or use of this software
//       must display the following acknowledgement:
//       This product includes software developed by AXSEM AG and its contributors.
//     5.The usage of this source code is only granted for operation with AX5043
//       and AX8052F143. Porting to other radio or communication devices is
//       strictly prohibited.
//
// THIS SOFTWARE IS PROVIDED BY AXSEM AG AND CONTRIBUTORS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL AXSEM AG AND CONTRIBUTORS BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <pacsat.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "config.h"
#include "os_task.h"
#include "ax5043_access.h"
#include "ax5043-2M-AFSK.h"
#include "config-2M-AFSK.h"
#include "nonvolManagement.h"

int32_t axradio_conv_freq_fromhz(int32_t f) {

  return (int32_t) (f * 1.048576);
}

//int32_t axradio_conv_freq_tohz(int32_t f) {
//  return (int32_t) (f / 1.048576);
//}



void quick_setfreq(int32_t f) {
  int32_t f1 = axradio_conv_freq_fromhz(f);

  /* Set LSB, per AX5043 documentation, to prevent synthesizer spurs */
  f1 |= 1;
  
  ax5043WriteReg(AX5043_FREQA0, f1);
  ax5043WriteReg(AX5043_FREQA1, f1 >> 8);
  ax5043WriteReg(AX5043_FREQA2, f1 >> 16);
  ax5043WriteReg(AX5043_FREQA3, f1 >> 24);
}

static uint8_t ax5043_reset(void)
{
	//printf("INFO: Resetting AX5043 (ax5043_reset)\n");
	uint8_t i;
	// Initialize Interface
	// Reset Device
	ax5043WriteReg(AX5043_PWRMODE, 0x80);
	ax5043WriteReg(AX5043_PWRMODE, AX5043_PWRSTATE_POWERDOWN);
	// Wait some time for regulator startup
	vTaskDelay(CENTISECONDS(1));

	// Check Scratch
	i = ax5043ReadReg(AX5043_SILICONREVISION);
	i = ax5043ReadReg(AX5043_SILICONREVISION);

	if (i != SILICONREV1)
		return RADIO_ERR_REVISION;

	ax5043WriteReg(AX5043_SCRATCH, 0x55);
	if (ax5043ReadReg(AX5043_SCRATCH) != 0x55)
		return RADIO_ERR_COMM;

	ax5043WriteReg(AX5043_SCRATCH, 0xAA);
	if (ax5043ReadReg(AX5043_SCRATCH) != 0xAA)
		return RADIO_ERR_COMM;

	return RADIO_OK;
}


uint8_t ax5043_readfifo(uint8_t axradio_rxbuffer[], uint8_t len) {
	uint8_t loc = 0;
	while (len--) {
		axradio_rxbuffer[loc++] = ax5043ReadReg(AX5043_FIFODATA);
	}
	return loc;
}

extern const uint8_t axradio_phy_innerfreqloop;
extern uint8_t axradio_rxbuffer_2m[];

static uint8_t receive_loop(void);

uint8_t receive_packet_2m(void) {
    return receive_loop();
}

uint8_t receive_loop(void) {
    uint8_t fifo_cmd;
    uint8_t i;
    uint8_t b0 __attribute__((unused));
    uint8_t b1 __attribute__((unused));
    uint8_t b2 __attribute__((unused));
    uint8_t b3 __attribute__((unused));

    uint8_t len = ax5043ReadReg(AX5043_RADIOEVENTREQ0); // clear request so interrupt does not fire again. sync_rx enables interrupt on radio state changed in order to wake up on SDF detected

    uint8_t bytesRead = 0;

    while ((ax5043ReadReg(AX5043_FIFOSTAT) & 0x01) != 1) { // FIFO not empty
        fifo_cmd = ax5043ReadReg(AX5043_FIFODATA); // read command
        len = (fifo_cmd & 0xE0) >> 5; // top 3 bits encode payload len
        if (len == 7)
            len = ax5043ReadReg(AX5043_FIFODATA); // 7 means variable length, -> get length byte
        fifo_cmd &= 0x1F;
        switch (fifo_cmd) {
        case AX5043_FIFOCMD_DATA:
            if (!len)
                break;

            ax5043ReadReg(AX5043_FIFODATA); // Discard the flags
            --len;
            bytesRead = ax5043_readfifo(axradio_rxbuffer_2m, len);
        	break;

        case AX5043_FIFOCMD_RFFREQOFFS:
            if (axradio_phy_innerfreqloop || len != 3)
                goto dropchunk;
            i = ax5043ReadReg(AX5043_FIFODATA);
            i &= 0x0F;
            i |= 1 + (uint8_t)~(i & 0x08);

            b3 = ((int8_t)i) >> 8;
            b2 = i;
            b1 = ax5043ReadReg(AX5043_FIFODATA);
            b0 = ax5043ReadReg(AX5043_FIFODATA);

            //printf("INFO: RF Frequency Offset: 0x%02x%02x%02x%02x\n", b3, b2, b1, b0);
            break;

        case AX5043_FIFOCMD_FREQOFFS:
             if (!axradio_phy_innerfreqloop || len != 2)
                 goto dropchunk;

             b1 = ax5043ReadReg(AX5043_FIFODATA);
             b0 = ax5043ReadReg(AX5043_FIFODATA);

             //printf("INFO: Frequency offset: 0x%02x%02x\n", b1, b2);
             break;

        case AX5043_FIFOCMD_RSSI:
            if (len != 1)
                goto dropchunk;
            {
                int16_t rl;
                rl = ((int8_t)ax5043ReadReg(AX5043_FIFODATA)) - axradio_phy_rssioffset;

                debug_print("Command RSSI %d\n", (int)rl);
            }
            break;

        case AX5043_FIFOCMD_TIMER:
            if (len != 3)
                goto dropchunk;
            {
	      b2 = ax5043ReadReg(AX5043_FIFODATA);
	      b1 = ax5043ReadReg(AX5043_FIFODATA);
	      b0 = ax5043ReadReg(AX5043_FIFODATA);
	      //printf("INFO: Timer: %d, %d, %d\n", b2, b1, b0);
            }
            break;

        default:
	  fprintf(stderr, "ERROR: Unknown chunk in FIFO: %d\n", fifo_cmd);
        	dropchunk:
		  fprintf(stderr, "WARNING: Discarding chunk in FIFO\n");
            if (!len)
                break;
            i = len;
            do {
                ax5043ReadReg(AX5043_FIFODATA);        // purge FIFO
            }
            while (--i);
        	break;
        }
    }

    //printf("INFO: Done waiting for a packet\n");

    return bytesRead;
}

static volatile uint8_t axradio_mode = AXRADIO_MODE_UNINIT;
static volatile axradio_trxstate_t axradio_trxstate = trxstate_off;

static struct axradio_address_mask axradio_localaddr;
uint8_t axradio_rxbuffer_2m[PKTDATA_BUFLEN];

extern const uint32_t axradio_phy_chanfreq[];
extern const uint8_t axradio_phy_chanpllrnginit[];
extern uint8_t axradio_phy_chanpllrng[];
extern const uint8_t axradio_phy_vcocalib;
extern uint8_t axradio_phy_chanvcoi[];
extern const uint8_t axradio_phy_chanvcoiinit[];
extern const uint8_t axradio_framing_swcrclen;
extern const uint8_t axradio_phy_innerfreqloop;
extern const uint8_t axradio_phy_pn9;
extern const uint8_t axradio_framing_addrlen;
extern const uint8_t axradio_framing_destaddrpos;

static void axradio_setaddrregs(void)
{
//	uint8_t regValue;

    ax5043WriteReg(AX5043_PKTADDR0, axradio_localaddr.addr[0]);
    ax5043WriteReg(AX5043_PKTADDR1, axradio_localaddr.addr[1]);
    ax5043WriteReg(AX5043_PKTADDR2, axradio_localaddr.addr[2]);
    ax5043WriteReg(AX5043_PKTADDR3, axradio_localaddr.addr[3]);

    ax5043WriteReg(AX5043_PKTADDRMASK0, axradio_localaddr.mask[0]);
    ax5043WriteReg(AX5043_PKTADDRMASK1, axradio_localaddr.mask[1]);
    ax5043WriteReg(AX5043_PKTADDRMASK2, axradio_localaddr.mask[2]);
    ax5043WriteReg(AX5043_PKTADDRMASK3, axradio_localaddr.mask[3]);

    /* if (axradio_phy_pn9 && axradio_framing_addrlen) { */
    /*     uint16_t pn = 0x1ff; */
    /*     uint8_t inv = -(ax5043ReadReg(AX5043_ENCODING) & 0x01); */
    /*     if (axradio_framing_destaddrpos != 0xff) { */
    /*         pn = pn9_advance_bits(pn, axradio_framing_destaddrpos << 3); */
    /* 	} */
    /*     regValue = ax5043ReadReg(AX5043_PKTADDR0); */
    /*     regValue ^= pn ^ inv; */
    /*     ax5043WriteReg(AX5043_PKTADDR0, regValue); */
    /*     pn = pn9_advance_byte(pn); */

    /*     regValue = ax5043ReadReg(AX5043_PKTADDR1); */
    /*     regValue ^= pn ^ inv; */
    /*     ax5043WriteReg(AX5043_PKTADDR1, regValue); */
    /*     pn = pn9_advance_byte(pn); */

    /*     regValue = ax5043ReadReg(AX5043_PKTADDR2); */
    /*     regValue ^= pn ^ inv; */
    /*     ax5043WriteReg(AX5043_PKTADDR2, regValue); */
    /*     pn = pn9_advance_byte(pn); */

    /*     regValue = ax5043ReadReg(AX5043_PKTADDR3); */
    /*     regValue ^= pn ^ inv; */
    /*     ax5043WriteReg(AX5043_PKTADDR3, regValue); */
    /* } */
}

static void ax5043_init_registers(void)
{
	uint8_t regValue;

    ax5043_set_registers();

    regValue = ax5043ReadReg(AX5043_PKTLENOFFSET);
    /* regValue += axradio_framing_swcrclen; // add len offs for software CRC16 (used for both, fixed and variable length packets */
    ax5043WriteReg(AX5043_PKTLENOFFSET, regValue);

    ax5043WriteReg(AX5043_PINFUNCIRQ, 0x00); // No IRQ used for now
    ax5043WriteReg(AX5043_PKTSTOREFLAGS, axradio_phy_innerfreqloop ? 0x13 : 0x15); // store RF offset, RSSI and delimiter timing
    axradio_setaddrregs();
}

static void axradio_wait_for_xtal(void) {
	//printf("INFO: Waiting for crystal (axradio_wait_for_xtal)\n");
	while ((ax5043ReadReg(AX5043_XTALSTATUS) & 0x01) == 0) {
		vTaskDelay(CENTISECONDS(1));
	}
	//printf("INFO: Crystal is ready\n");
}

#if 0
static int16_t axradio_tunevoltage(void)
{
    int16_t r = 0;
    uint8_t cnt = 64;
    do {
    	ax5043WriteReg(AX5043_GPADCCTRL, 0x84);
        do {} while (ax5043ReadReg(AX5043_GPADCCTRL) & 0x80);
    } while (--cnt);
    cnt = 32;
    do {
    	ax5043WriteReg(AX5043_GPADCCTRL, 0x84);
        do {} while (ax5043ReadReg(AX5043_GPADCCTRL) & 0x80);
        {
            int16_t x = ax5043ReadReg(AX5043_GPADC13VALUE1) & 0x03;
            x <<= 8;
            x |= ax5043ReadReg(AX5043_GPADC13VALUE0);
            r += x;
        }
    } while (--cnt);
    return r;
}
#endif


#if 0
static uint8_t axradio_adjustvcoi(uint8_t rng)
{
    uint8_t offs;
    uint8_t bestrng;
    uint16_t bestval = ~0;
    rng &= 0x7F;
    bestrng = rng;
    for (offs = 0; offs != 16; ++offs) {
        uint16_t val;
        if (!((uint8_t)(rng + offs) & 0xC0)) {
            ax5043WriteReg(AX5043_PLLVCOI, 0x80 | (rng + offs));
            val = axradio_tunevoltage();
            if (val < bestval) {
                bestval = val;
                bestrng = rng + offs;
            }
        }
        if (!offs)
            continue;
        if (!((uint8_t)(rng - offs) & 0xC0)) {
        	ax5043WriteReg(AX5043_PLLVCOI, 0x80 | (rng - offs));
            val = axradio_tunevoltage();
            if (val < bestval) {
                bestval = val;
                bestrng = rng - offs;
            }
        }
    }
    // if we hit the lower rail, do not change anything
    if (bestval <= 0x0010)
        return rng | 0x80;
    return bestrng | 0x80;
}
#endif


#if 0
static uint8_t axradio_calvcoi(void)
{
    uint8_t i;
    uint8_t r = 0;
    uint16_t vmin = 0xffff;
    uint16_t vmax = 0x0000;
    for (i = 0x40; i != 0;) {
        uint16_t curtune;
        --i;
        ax5043WriteReg(AX5043_PLLVCOI, 0x80 | i);
        ax5043ReadReg(AX5043_PLLRANGINGA); // clear PLL lock loss
        curtune = axradio_tunevoltage();
        ax5043ReadReg(AX5043_PLLRANGINGA); // clear PLL lock loss
        ((uint16_t *)axradio_rxbuffer_2m)[i] = curtune;
        if (curtune > vmax)
            vmax = curtune;
        if (curtune < vmin) {
            vmin = curtune;
            // check whether the PLL is locked
            if (!(0xC0 & (uint8_t)~ax5043ReadReg(AX5043_PLLRANGINGA)))
                r = i | 0x80;
        }
    }
    if (!(r & 0x80) || vmax >= 0xFF00 || vmin < 0x0100 || vmax - vmin < 0x6000)
        return 0;
    return r;
}
#endif

uint8_t axradio_init_2m(int32_t freq)
{
	//uint8_t regValue;

	//printf("Inside axradio_init_2m\n");

    axradio_mode = AXRADIO_MODE_UNINIT;
    axradio_trxstate = trxstate_off;
    if (ax5043_reset())
        return AXRADIO_ERR_NOCHIP;
    ax5043_init_registers();
    ax5043_set_registers_tx();




    ax5043WriteReg(AX5043_PLLLOOP, 0x09); // default 100kHz loop BW for ranging
    ax5043WriteReg(AX5043_PLLCPI, 0x08);

    // range all channels
    ax5043WriteReg(AX5043_PWRMODE, AX5043_PWRSTATE_XTAL_ON);
    ax5043WriteReg(AX5043_MODULATION, 0x08);
    ax5043WriteReg(AX5043_FSKDEV2, 0x00);
    ax5043WriteReg(AX5043_FSKDEV1, 0x00);
    ax5043WriteReg(AX5043_FSKDEV0, 0x00);
    axradio_wait_for_xtal();


#if 0
    {
		uint32_t f = axradio_phy_chanfreq[0];
		ax5043WriteReg(AX5043_FREQA0, f);
		ax5043WriteReg(AX5043_FREQA1, f >> 8);
		ax5043WriteReg(AX5043_FREQA2, f >> 16);
		ax5043WriteReg(AX5043_FREQA3, f >> 24);
	}
#endif

    quick_setfreq(freq);     // DEBUG [RBG]



	axradio_trxstate = trxstate_pll_ranging;

#if 0

    printf("AX5043_PLLLOOP: %02.2x\n", ax5043ReadReg(AX5043_PLLLOOP));
    printf("AX5043_PLLCPI: %02.2x\n", ax5043ReadReg(AX5043_PLLCPI));

    printf("AX5043_PLLVCOI: %02.2x\n", ax5043ReadReg(AX5043_PLLVCOI));
    printf("AX5043_PLLRANGINGA: %02.2x\n", ax5043ReadReg(AX5043_PLLRANGINGA));
    printf("AX5043_PLLVCODIV: %02.2x\n", ax5043ReadReg(AX5043_PLLVCODIV));

    printf ("AX5043_FREQA0: %x\n", ax5043ReadReg(AX5043_FREQA0));
    printf ("AX5043_FREQA1: %x\n", ax5043ReadReg(AX5043_FREQA1));
    printf ("AX5043_FREQA2: %x\n", ax5043ReadReg(AX5043_FREQA2));
    printf ("AX5043_FREQA3: %x\n", ax5043ReadReg(AX5043_FREQA3));
#endif

	{
		uint8_t r;
		if( !(axradio_phy_chanpllrnginit[0] & 0xF0) ) { // start values for ranging available
			r = axradio_phy_chanpllrnginit[0] | 0x10;
		}
		else {
			r = 0x18;
		}
		ax5043WriteReg(AX5043_PLLRANGINGA, r); // init ranging process starting from "range"
	}
	//printf("INFO: Waiting for PLL ranging process\n");
	while ((ax5043ReadReg(AX5043_PLLRANGINGA) & 0x10) != 0) {
	    vTaskDelay(CENTISECONDS(1));
	}
	//printf("INFO: PLL ranging process complete\n");
	axradio_trxstate = trxstate_off;
	axradio_phy_chanpllrng[0] = ax5043ReadReg(AX5043_PLLRANGINGA);


#if 0
    // VCOI Calibration
    if (axradio_phy_vcocalib) {
        ax5043_set_registers_tx();
        ax5043WriteReg(AX5043_MODULATION, 0x08);
        ax5043WriteReg(AX5043_FSKDEV2, 0x00);
        ax5043WriteReg(AX5043_FSKDEV1, 0x00);
        ax5043WriteReg(AX5043_FSKDEV0, 0x00);

        regValue = ax5043ReadReg(AX5043_PLLLOOP);
        regValue |= 0x04;
        ax5043WriteReg(AX5043_PLLLOOP, regValue);
        {
            uint8_t x = ax5043ReadReg(AX5043_0xF35);
            x |= 0x80;
            if (2 & (uint8_t)~x)
                ++x;
            ax5043WriteReg(AX5043_0xF35, x);
        }
        ax5043WriteReg(AX5043_PWRMODE, AX5043_PWRSTATE_SYNTH_TX);
        {
            uint8_t vcoisave = ax5043ReadReg(AX5043_PLLVCOI);
            uint8_t j = 2;
			axradio_phy_chanvcoi[0] = 0;
			ax5043WriteReg(AX5043_PLLRANGINGA, axradio_phy_chanpllrng[0] & 0x0F);
			{
				uint32_t f = axradio_phy_chanfreq[0];
				ax5043WriteReg(AX5043_FREQA0, f);
				ax5043WriteReg(AX5043_FREQA1, f >> 8);
				ax5043WriteReg(AX5043_FREQA2, f >> 16);
				ax5043WriteReg(AX5043_FREQA3, f >> 24);
			}
			do {
				if (axradio_phy_chanvcoiinit[0]) {
					uint8_t x = axradio_phy_chanvcoiinit[0];
					if (!(axradio_phy_chanpllrnginit[0] & 0xF0))
						x += (axradio_phy_chanpllrng[0] & 0x0F) - (axradio_phy_chanpllrnginit[0] & 0x0F);
					axradio_phy_chanvcoi[0] = axradio_adjustvcoi(x);
				} else {
					axradio_phy_chanvcoi[0] = axradio_calvcoi();
				}
			} while (--j);
			j = 1;
			ax5043WriteReg(AX5043_PLLVCOI, vcoisave);
        }
    }
#endif

    ax5043WriteReg(AX5043_PWRMODE, AX5043_PWRSTATE_POWERDOWN);
    ax5043_init_registers();
    ax5043_set_registers_rx();
    ax5043WriteReg(AX5043_PLLRANGINGA, axradio_phy_chanpllrng[0] & 0x0F);

#if 0
    {
        uint32_t f = axradio_phy_chanfreq[0];
        ax5043WriteReg(AX5043_FREQA0, f);
        ax5043WriteReg(AX5043_FREQA1, f >> 8);
        ax5043WriteReg(AX5043_FREQA2, f >> 16);
        ax5043WriteReg(AX5043_FREQA3, f >> 24);
    }
#endif

    axradio_mode = AXRADIO_MODE_OFF;

#if 0

    printf("AX5043_PLLLOOP: %02.2x\n", ax5043ReadReg(AX5043_PLLLOOP));
    printf("AX5043_PLLCPI: %02.2x\n", ax5043ReadReg(AX5043_PLLCPI));

    printf("AX5043_PLLVCOI: %02.2x\n", ax5043ReadReg(AX5043_PLLVCOI));
    printf("AX5043_PLLRANGINGA: %02.2x\n", ax5043ReadReg(AX5043_PLLRANGINGA));
    printf("AX5043_PLLVCODIV: %02.2x\n", ax5043ReadReg(AX5043_PLLVCODIV));


#endif

	if (axradio_phy_chanpllrng[0] & 0x20)
		return AXRADIO_ERR_RANGING;
    return AXRADIO_ERR_NOERROR;
}

extern uint8_t axradio_phy_chanpllrng[];
extern const uint8_t axradio_phy_vcocalib;
extern uint8_t axradio_phy_chanvcoi[];
extern const uint8_t axradio_phy_chanvcoiinit[];
extern const uint8_t axradio_phy_chanpllrnginit[];
extern const int8_t axradio_phy_rssireference;

static uint8_t ax5043_init_registers_common(void);

uint8_t mode_tx_2m() {
	int retVal;

    retVal = ax5043_off();
	if (retVal != AXRADIO_ERR_NOERROR) {
		return retVal;
	}

	retVal = ax5043_init_registers_tx();
	if (retVal != AXRADIO_ERR_NOERROR) {
		return retVal;
	}

	return AXRADIO_ERR_NOERROR;
}

uint8_t mode_rx_2m() {
	int retVal;

    retVal = ax5043_off();
	if (retVal != AXRADIO_ERR_NOERROR) {
		return retVal;
	}

	retVal = ax5043_init_registers_rx();
	if (retVal != AXRADIO_ERR_NOERROR) {
		return retVal;
	}

	retVal = ax5043_receiver_on_continuous();
	if (retVal != AXRADIO_ERR_NOERROR) {
		return retVal;
	}

	return AXRADIO_ERR_NOERROR;
}


uint8_t ax5043_off(void)
{
    uint8_t retVal;

	retVal = ax5043_off_xtal();
	if (retVal != AXRADIO_ERR_NOERROR) {
		return retVal;
	}

	ax5043WriteReg(AX5043_PWRMODE, AX5043_PWRSTATE_POWERDOWN);

	return AXRADIO_ERR_NOERROR;
}

uint8_t ax5043_off_xtal(void)
{
    ax5043WriteReg(AX5043_PWRMODE, AX5043_PWRSTATE_XTAL_ON);
    ax5043WriteReg(AX5043_LPOSCCONFIG, 0x00); // LPOSC off
    return AXRADIO_ERR_NOERROR;
}

static uint8_t ax5043_init_registers_tx(void)
{
    ax5043_set_registers_tx();
    return ax5043_init_registers_common();
}

static uint8_t ax5043_init_registers_common(void)
{
    uint8_t rng = axradio_phy_chanpllrng[0];
    if (rng & 0x20)
        return AXRADIO_ERR_RANGING;
    if (ax5043ReadReg(AX5043_PLLLOOP) & 0x80) {
        ax5043WriteReg(AX5043_PLLRANGINGB, rng & 0x0F);
    } else {
        ax5043WriteReg(AX5043_PLLRANGINGA, rng & 0x0F);
    }
    rng = axradio_get_pllvcoi();
    if (rng & 0x80)
        ax5043WriteReg(AX5043_PLLVCOI, rng);

    return AXRADIO_ERR_NOERROR;
}

static uint8_t axradio_get_pllvcoi(void)
{
    if (axradio_phy_vcocalib) {
        uint8_t x = axradio_phy_chanvcoi[0];
        if (x & 0x80)
            return x;
    }
    {
        uint8_t x = axradio_phy_chanvcoiinit[0];
        if (x & 0x80) {
            if (!(axradio_phy_chanpllrnginit[0] & 0xF0)) {
                x += (axradio_phy_chanpllrng[0] & 0x0F) - (axradio_phy_chanpllrnginit[0] & 0x0F);
                x &= 0x3f;
                x |= 0x80;
            }
            return x;
        }
    }
    return ax5043ReadReg(AX5043_PLLVCOI);
}

static uint8_t ax5043_init_registers_rx(void) {
    ax5043_set_registers_rx();
    return ax5043_init_registers_common();

}

static uint8_t ax5043_receiver_on_continuous(void) {
    uint8_t regValue;

    ax5043WriteReg(AX5043_RSSIREFERENCE, axradio_phy_rssireference);
    ax5043_set_registers_rxcont();

    regValue = ax5043ReadReg(AX5043_PKTSTOREFLAGS);
    regValue &= (uint8_t)~0x40;
    ax5043WriteReg(AX5043_PKTSTOREFLAGS, regValue);


    ax5043WriteReg(AX5043_FIFOSTAT, 3); // clear FIFO data & flags
    ax5043WriteReg(AX5043_PWRMODE, AX5043_PWRSTATE_FULL_RX);

    return AXRADIO_ERR_NOERROR;
}

void start_rx() {
    debug_print("Starting RX\n");
    //uint8_t retVal;
    axradio_init_2m(ReadMRAMCommandFreq());
    mode_rx_2m();
    ax5043WriteReg(AX5043_PINFUNCIRQ, 0x0); //disable IRQs

    ax5043WriteReg(AX5043_IRQMASK0, 0x01); // FIFO not Empty
    ax5043WriteReg(AX5043_IRQMASK1, 0);    // FIFO not Empty
    ax5043WriteReg(AX5043_IRQINVERSION0, (ax5043ReadReg(AX5043_IRQINVERSION0) & 0xFE)); // Don't invert the interrupt

    ax5043WriteReg(AX5043_PINFUNCIRQ, 0x3); //enable IRQs

    // ax5043 should be receiving packets now and interrupting on FIFO not empty



    //}
}

void test_pll_range() {
    int32_t i;

    ax5043PowerOn();

    for (i = 134000000; i < 159000000; i+= 1000000) {
      printf("\n\nFreq: %d\n", i);
      uint8_t retVal = axradio_init_2m(i);
      printf("axradio_init_2m: %d\n",retVal);
    }

//    ax5043PowerOff();
}

uint8_t get_rssi() {
    int8_t byteVal;
    int16_t wordVal;
    byteVal = (int8_t)ax5043ReadReg(AX5043_RSSI);
    wordVal = (int16_t)byteVal;
    wordVal -=64;
    wordVal +=255;
    return (uint8_t)wordVal;
}
