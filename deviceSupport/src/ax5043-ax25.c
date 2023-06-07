// PSK Transmit routines for AMSAT GOLF-TEE
// R. Gopstein, Feb 2020

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

#include <ax5043-ax25.h>
#include <pacsat.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "config.h"
#include "os_task.h"
#include "ax5043_access.h"
#include "serialDriver.h"
#include "nonvolManagement.h"

/* Forward declarations */
static uint8_t ax5043_ax25_init_registers_common(AX5043Device device);
static uint8_t ax5043_ax25_init_registers_tx(AX5043Device device, bool band_vhf, bool rate_9600);
static uint8_t ax5043_ax25_init_registers_rx(AX5043Device device, bool band_vhf, bool rate_9600);

/// TODO - these variables need to be indeced by AX5043Device id

/* Variables */
static volatile uint8_t axradio_mode = AXRADIO_MODE_UNINIT;
static volatile axradio_trxstate_t axradio_trxstate = trxstate_off;

//static struct axradio_address_mask axradio_localaddr;
uint8_t axradio_rxbuffer_70cm[PKTDATA_BUFLEN];

// Global variables for radio physical layer.  These are used by the radio and read by telemetry or the console
// TODO - this needs to be a set of arrays, one for each radio
const uint32_t axradio_phy_chanfreq[2] = { 0x1b3b5550,0x1b3b5550}; //Primary and secondry frequencies.  There are two per radio
const uint8_t axradio_phy_chanpllrnginit[1] = { 0x09 };
uint8_t axradio_phy_chanpllrng[1];
const uint8_t axradio_phy_vcocalib = 0;
uint8_t axradio_phy_chanvcoi[1];
const uint8_t axradio_phy_chanvcoiinit[1] = { 0x97 };

// TODO - confirm these were LEGACY_GOLF and can be removed
//extern const uint8_t axradio_framing_swcrclen;
////extern const uint8_t axradio_phy_innerfreqloop;
//extern const uint8_t axradio_phy_pn9;
//extern const uint8_t axradio_framing_addrlen;
//extern const uint8_t axradio_framing_destaddrpos;

static const int8_t  axradio_phy_rssireference = 57;// 0xF9 + 64;
//extern const int8_t axradio_phy_rssireference;



/**
 * FIRST ALL OF THE SETTINGS THAT ARE COMMON TO BOTH BANDS AND FOR RX AND TX
 * Set all of the base registers
 */
static void ax5043_ax25_set_registers(AX5043Device device, bool band_vhf, bool rate_9600) {

    if (rate_9600) {
        /* NOTE that 0x07 GMSK does not work on receive.  Use GFSK 0x07 with BT = 0.5.  This works better than 0.3 for RX.
         * More testing needed for TX, which may work best with GMSK */
        ax5043WriteReg(device, AX5043_MODULATION              ,0x08); // AFSK.  0x04 is PSK. GMSK for 9600 is 0x07
        ax5043WriteReg(device, AX5043_ENCODING                ,0x07); // Differential encoding, bit inversion, no scrambler.  Use 0x07 for G3RUH scrambler
    } else {
        ax5043WriteReg(device, AX5043_MODULATION              ,0x0A); // AFSK.  0x04 is PSK. GMSK for 9600 is 0x07
        ax5043WriteReg(device, AX5043_ENCODING                ,0x03); // Differential encoding, bit inversion, no scrambler.  Use 0x07 for G3RUH scrambler
    }
  ax5043WriteReg(device, AX5043_FRAMING                 ,0x14); // 0x14 is HDLC framing with CRC. 0x04 is HDLC without CRC.  Was 0x06 for GOLF format
  ax5043WriteReg(device, AX5043_0xF72                   ,0x00); // Set PERTUNE114 to zero for CRC with HDLC.  Set to 0x06 for raw soft bits
  ax5043WriteReg(device, AX5043_PINFUNCSYSCLK           ,0x01); // This sets the the pin to output 1/16th of crystal freq.  Set to 06 for 1/4
  ax5043WriteReg(device, AX5043_PINFUNCDCLK             ,0x01);
  ax5043WriteReg(device, AX5043_PINFUNCDATA             ,0x01);
  ax5043WriteReg(device, AX5043_PINFUNCANTSEL           ,0x00); //Toggles Amplifier PA Pin to off, set to 01 if it is on
  ax5043WriteReg(device, AX5043_PINFUNCPWRAMP           ,0x07);
  ax5043WriteReg(device, AX5043_WAKEUPXOEARLY           ,0x01); // Wake up xtal oscillator early

  /**
   * Freq of the IF sets the center of the downconverted spectrum.  Set to BW / 2
   * 5200Hz for 1200 bps
   * 7200Hz for 9600 bps
   *
   * IFFREQ = 2^20 * Freq_of_if / Xtal_freq + 1/2
   * 1200bps = 0x0155
   * 9600bps = 0x01D8
   *
   * Note that Radio lab calculates 04CD, which is 1229, so this is the same freq as the bit rate..
   */
  if (rate_9600) {
      ax5043WriteReg(device, AX5043_IFFREQ1                 ,0x04);
      ax5043WriteReg(device, AX5043_IFFREQ0                 ,0xcd);
  } else {
      ax5043WriteReg(device, AX5043_IFFREQ1                 ,0x04);
      ax5043WriteReg(device, AX5043_IFFREQ0                 ,0xcd);

  }
  /**
   * RX DATA RATE and DECIMATION
   * For 1200bps AFSK the Bandwidth is 10.4kHz from 2 * (Fdev + Fmod) = 2 * (3000 + 2200)
   * For 9600bps the bandwidth is also 2 * (Fdev + Fmod) where Fdev = 0.5 * modulation_index * bitrate and Fmod = 0.5 * bitrate
   * modulation_index = 0.5 for GMSK.  So bandwidth is 14.4 kHz from 2 * (0.5*0.5*9600 + 0.5*9600)
   *
   * Min bw of lpf has to equal bw calculated above.  it is set by FILTIDX bits of PHASEGAIN register.  See prog manual.
   *
   * Decimation = 0.25 * Fxtal / (2^4 * BW)   -- Assuming FILTERIDX bits of PHASEGAIN0 set to default value of 0.25 using bits 11
   *
   * RadioLab sets the Decimation to 08
   */
  if (rate_9600) {
      ax5043WriteReg(device, AX5043_DECIMATION              ,0x0D);  // Set to 0c18 per black magic and 0x11 for 9600 bps
  } else {
      ax5043WriteReg(device, AX5043_DECIMATION              ,0x08);  // Set to 0c18 per black magic and 0x11 for 9600 bps
  }


  /**
   * Radiolab sets the 1200bps data rate to 034155, which is 213k and seems really wide...
   * Black magic uses 0115c8
   */
  if (rate_9600) {
      ax5043WriteReg(device, AX5043_RXDATARATE2             ,0x00);  // 0x003106 for 9600
      ax5043WriteReg(device, AX5043_RXDATARATE1             ,0x40);
      ax5043WriteReg(device, AX5043_RXDATARATE0             ,0x1A);
  } else {
      ax5043WriteReg(device, AX5043_RXDATARATE2             ,0x03);
      ax5043WriteReg(device, AX5043_RXDATARATE1             ,0x41);
      ax5043WriteReg(device, AX5043_RXDATARATE0             ,0x55);

  }

  /**
   * We can set to zero if our timing is stable and TX bitrate is within 1%
   * Otherwise set to MAXDROFFSET = 2^7 * Fxtal * delta_bitrate / bitrate^2 * decimation + 0.5
   * 1BC8 for 10% of bit rate might be interesting for testing, but note that pre-amble needs to be longer to cope
   */
  ax5043WriteReg(device, AX5043_MAXDROFFSET2            ,0x00); // These are set to 0 if timing of TX is within 1%.
  ax5043WriteReg(device, AX5043_MAXDROFFSET1            ,0x00);
  ax5043WriteReg(device, AX5043_MAXDROFFSET0            ,0x00);

  /**
   * AFC is bounded by MAXRFOFFSET = 2^24 * Foffset / Fxtal + 0.5
   * We cant set Foffset to more than BW / 4.  So the max is:
   * 1200bps = 2600Hz
   * 9600bps = 3600Hz
   *
   * So for 1200 we set to 0xAA7
   * For 9600 we set to 0x0EBF
   *
   * We also set bit 23 so that this is corrected at the first Local Oscillator rather than at the second
   *
   * Radio lab sets this to 0707
   */
  ax5043WriteReg(device, AX5043_MAXRFOFFSET2            ,0x80);  // set bit 23
  if (rate_9600) {
      ax5043WriteReg(device, AX5043_MAXRFOFFSET1            ,0x00);
      ax5043WriteReg(device, AX5043_MAXRFOFFSET0            ,0x00);
  } else {
      ax5043WriteReg(device, AX5043_MAXRFOFFSET1            ,0x07);
      ax5043WriteReg(device, AX5043_MAXRFOFFSET0            ,0x07);
  }

  /* Radio lab sets FSKDMAX and FSKDMIN, which are not set in the GOLF code.  They are set to 0 */
  ax5043WriteReg(device, AX5043_FSKDMAX1            ,0x00);
  ax5043WriteReg(device, AX5043_FSKDMAX0            ,0x00);
  ax5043WriteReg(device, AX5043_FSKDMIN1            ,0x00);
  ax5043WriteReg(device, AX5043_FSKDMIN0            ,0x00);

  /* AFSK detector bandwidth set as AFSKCTRL = 2 * log2 (Fxtal / 2^5 * bitrate * decimation)
   * Radio laB SETS TO 0C*/
  if (!rate_9600) {
      ax5043WriteReg(device, AX5043_AFSKCTRL,           0x0C); // not needed for 9600, only 1200bps AFSK
  }
  ax5043WriteReg(device, AX5043_AMPLFILTER              ,0x00);
  ax5043WriteReg(device, AX5043_RXPARAMSETS             ,0xF4);  // Was F4 in GOLF to have different param sets for each pattern matched

  /**
   * AGCGAIN contains two 4 bit numbers AGCDECAY AGCATTACK
   *
   * F3db = Fxtal / (2^5 * pi ) * (2^-AGCATTACK - 2^(-1-2*AGCATTACK))
   * AGCATTACK - F3db = bitrate
   * AGCDECAY - F3db = bitrate / 10
   *
   * We pick closest values
   * 0x7 = 1200
   * 0xA = 155
   *
   * 0x4 = 9636
   *
   * So for 1200bps ATTACK = 0x07 and DECAY = 0x0A
   * For 9600bps ATTACK = 0x04 and DECAY 0x07
   *
   * ATTACK is the lower 4 bits
   * We can tune this value by outputting RSSI or SAMPLE_ROT_I/SAMPLE_ROT_Q through the DAC
   *
   * Radio lab sets this to E8
   */
  if (rate_9600) {
      ax5043WriteReg(device, AX5043_AGCGAIN0                ,0xB5); // RX Only
  } else {
      ax5043WriteReg(device, AX5043_AGCGAIN0                ,0xE8); // RX Only
  }

  /* ADC for AGC has max value of 512.  This sets the max range.  Set to 75% or magnitude 384.  AGCTARGET = log2(Magnitude) * 16 = 0x89
   * Radio lab sets this to 84 */
  ax5043WriteReg(device, AX5043_AGCTARGET0              ,0x84); // RX only

  /**
   * TIMEGAIN contains two 4 bit numbers TIMEGAINxM and TIMEGAINxE
   *
   * TIMEGAINxE = min|log2(bitrate/TMGCORRFRAC*8)|
   * TIMEGAINxM = max|log2(bitrate/TMGCORRFRAC*2^TIMEGAINxE)|
   * TMGCORRFRAC should be at least 4.  Greater values reduce sampling jitter but slow timing lock-in
   *
   * So for 1200bps TIMEGAINxM = 0x9 and TIMEGAINxE = 0x5
   * For 9600bps TIMEGAINxM = 0x9 and TIMEGAINxE = 0x8
   * TIMEGAINxE is the lower 4 bits
   *
   * Radio lab sets TIMEGAIN0 to DC
   */
  if (rate_9600) {
      ax5043WriteReg(device, AX5043_TIMEGAIN0               ,0x89); // RX Only
  } else {
      ax5043WriteReg(device, AX5043_TIMEGAIN0               ,0xDC); // RX Only
  }

  /**
   * DRGAIN contains two 4 bit numbers DRGAINxM and DRGAINxE
   *
   * DRGAINxE = min|log2(bitrate/DRGCORRFRACx*8)|
   * DRGAINxM = max|log2(bitrate/DRGCORRFRACx*2^TIMEGAINxE)|
   * DRGCORRFRAC should be at least 64.  Greater values reduce datarate jitter but slow data rate aquisition
   *
   * So for 1200bps DRGAINxM = 0x9 and DRGAINxE = 0x1
   * For 9600 bps DRGAINxM = 0x9 and DRGAINxE = 0x4
   * DRGAINxE is lower 4 bits
   *
   * Radio lab sets DRGAIN0 to D6
   */
  if (rate_9600) {
      ax5043WriteReg(device, AX5043_DRGAIN0                 ,0x83); // RX Only
  } else {
      ax5043WriteReg(device, AX5043_DRGAIN0                 ,0xD6); // RX Only
  }

  /* Set FILTERIDX bits to 11 in PHASEGAIN - see DECIMATION calc above
   * Radio lab also sets this to default of C3*/
  ax5043WriteReg(device, AX5043_PHASEGAIN0              ,0xC3); // C3 is also the default

  /* From GOLF FREQUENCYGAINA0 = 46, B0 = 0A, C0 = qF, D0 = 1F
   * Radio lab sets FREQUENCYGAINA0 to 0F, B0 = 1F, C0 = 0D, D0 = 0D */

  if (rate_9600) {
      ax5043WriteReg(device, AX5043_FREQUENCYGAINA0         ,0x26);
      ax5043WriteReg(device, AX5043_FREQUENCYGAINB0         ,0x0A);
      ax5043WriteReg(device, AX5043_FREQUENCYGAINC0         ,0x1F);
      ax5043WriteReg(device, AX5043_FREQUENCYGAIND0         ,0x1F);
  } else {
      ax5043WriteReg(device, AX5043_FREQUENCYGAINA0         ,0x0F);
      ax5043WriteReg(device, AX5043_FREQUENCYGAINB0         ,0x1F);
      ax5043WriteReg(device, AX5043_FREQUENCYGAINC0         ,0x0D);
      ax5043WriteReg(device, AX5043_FREQUENCYGAIND0         ,0x0D);
  }
  /* AMPLITUDEGAIN0 set to 06 in GOLF and by RadioLab */
  ax5043WriteReg(device, AX5043_AMPLITUDEGAIN0          ,0x06);

  /* Set to 00 */
  ax5043WriteReg(device, AX5043_FREQDEV10               ,0x00);
  ax5043WriteReg(device, AX5043_FREQDEV00               ,0x00);

  /* Baseband gain compensation resistors Set to zero */
  ax5043WriteReg(device, AX5043_BBOFFSRES0              ,0x00);

  if (rate_9600) {
      ax5043WriteReg(device, AX5043_AGCGAIN1                ,0xB5); // RX Only?
  } else {
      ax5043WriteReg(device, AX5043_AGCGAIN1                ,0xE8); // RX Only?
  }
  ax5043WriteReg(device, AX5043_AGCTARGET1              ,0x84);
  ax5043WriteReg(device, AX5043_AGCAHYST1               ,0x00); // 0 is default value
  ax5043WriteReg(device, AX5043_AGCMINMAX1              ,0x00); // 0 is default value

  if (rate_9600) {
      ax5043WriteReg(device, AX5043_TIMEGAIN1               ,0x87); // GOLF was 94, DA per radio lab
      ax5043WriteReg(device, AX5043_DRGAIN1                 ,0x82); // GOLF was 90, D5 per radio lab
  } else {
      ax5043WriteReg(device, AX5043_TIMEGAIN1               ,0xDA); // GOLF was 94, DA per radio lab
      ax5043WriteReg(device, AX5043_DRGAIN1                 ,0xD5); // GOLF was 90, D5 per radio lab
  }

  ax5043WriteReg(device, AX5043_PHASEGAIN1              ,0xC3); // GOLF was 83, C3 per radio lab
  if (rate_9600) {
      ax5043WriteReg(device, AX5043_FREQUENCYGAINA1         ,0x26); // GOLF was 46, 0F per Radio lab
      ax5043WriteReg(device, AX5043_FREQUENCYGAINB1         ,0x0A); // GOLF was 0A, 1F per radio lab
      ax5043WriteReg(device, AX5043_FREQUENCYGAINC1         ,0x1F); // GOLF was 1F, 0D per radio lab
      ax5043WriteReg(device, AX5043_FREQUENCYGAIND1         ,0x1F); // GOLF was 1F, 0D per radio lab
  } else {
      ax5043WriteReg(device, AX5043_FREQUENCYGAINA1         ,0x0F); // GOLF was 46, 0F per Radio lab
      ax5043WriteReg(device, AX5043_FREQUENCYGAINB1         ,0x1F); // GOLF was 0A, 1F per radio lab
      ax5043WriteReg(device, AX5043_FREQUENCYGAINC1         ,0x1D); // GOLF was 1F, 0D per radio lab
      ax5043WriteReg(device, AX5043_FREQUENCYGAIND1         ,0x1D); // GOLF was 1F, 0D per radio lab
  }

  ax5043WriteReg(device, AX5043_AMPLITUDEGAIN1          ,0x06); // per radio lab
  ax5043WriteReg(device, AX5043_FREQDEV11               ,0x00); // per radio lab
  if (rate_9600) {
      ax5043WriteReg(device, AX5043_FREQDEV01               ,0x2D); // per radio lab
  } else {
      ax5043WriteReg(device, AX5043_FREQDEV01               ,0x00); // per radio lab
  }
  ax5043WriteReg(device, AX5043_FOURFSK1                ,0x16); // per radio lab

  ax5043WriteReg(device, AX5043_BBOFFSRES1              ,0x00); // per radio lab
  ax5043WriteReg(device, AX5043_AGCGAIN3                ,0xFF); // per radio lab
  ax5043WriteReg(device, AX5043_AGCTARGET3              ,0x84); // per radio lab
  ax5043WriteReg(device, AX5043_AGCAHYST3               ,0x00); // 0 is default value
  ax5043WriteReg(device, AX5043_AGCMINMAX3              ,0x00); // 0 is default value
  if (rate_9600) {
      ax5043WriteReg(device, AX5043_TIMEGAIN3               ,0x86); // GOLF was 93, D9 per radio lab
      ax5043WriteReg(device, AX5043_DRGAIN3                 ,0x81); // GOLF was 90, D4 per radio lab
  } else {
      ax5043WriteReg(device, AX5043_TIMEGAIN3               ,0xD9); // GOLF was 93, D9 per radio lab
      ax5043WriteReg(device, AX5043_DRGAIN3                 ,0xD4); // GOLF was 90, D4 per radio lab
  }
  ax5043WriteReg(device, AX5043_PHASEGAIN3              ,0xC3); // GOLF was 83, C3 per radio lab
  if (rate_9600) {
      ax5043WriteReg(device, AX5043_FREQUENCYGAINA3         ,0x26); // GOLF was 46, 0F per radio lab
      ax5043WriteReg(device, AX5043_FREQUENCYGAINB3         ,0x0A); // GOLF was 0A, 1F per radio lab
      ax5043WriteReg(device, AX5043_FREQUENCYGAINC3         ,0x1F); // GOLF was 1F, 0D per radio lab
      ax5043WriteReg(device, AX5043_FREQUENCYGAIND3         ,0x1F); // GOLF was 1F, 0D per radio lab
  } else {
      ax5043WriteReg(device, AX5043_FREQUENCYGAINA3         ,0x0F); // GOLF was 46, 0F per radio lab
      ax5043WriteReg(device, AX5043_FREQUENCYGAINB3         ,0x1F); // GOLF was 0A, 1F per radio lab
      ax5043WriteReg(device, AX5043_FREQUENCYGAINC3         ,0x0D); // GOLF was 1F, 0D per radio lab
      ax5043WriteReg(device, AX5043_FREQUENCYGAIND3         ,0x0D); // GOLF was 1F, 0D per radio lab
  }
  ax5043WriteReg(device, AX5043_AMPLITUDEGAIN3          ,0x06); // per radio lab
  ax5043WriteReg(device, AX5043_FREQDEV13               ,0x00); // per radio lab
  if (rate_9600) {
      ax5043WriteReg(device, AX5043_FREQDEV03               ,0x2D); // per radio lab
  } else {
      ax5043WriteReg(device, AX5043_FREQDEV03               ,0x00); // per radio lab
  }
  ax5043WriteReg(device, AX5043_FOURFSK3                ,0x16); // per radio lab
  ax5043WriteReg(device, AX5043_BBOFFSRES3              ,0x00); // per radio lab

  /* TODO - G0KLA - Signal filtering, for now per Jonathons code
   * 03 is Gaussian BT = 0.5.  02 is Gaussian with BT = 0.3 and 00 is no filtering. */
  if (rate_9600) {
      ax5043WriteReg(device, AX5043_MODCFGF                 ,0x03); // TX only  TODO - radio lab says 00, but is that for FSk or because I selected wrong.  Surely we want filtering
  } else {
      ax5043WriteReg(device, AX5043_MODCFGF                 ,0x03); // TX only
  }
  /* Set the Frequency Deviation  TODO -- THIS IS JUST THE TX VALUE?? */
  /* Where FSKDEV = (0.858785 * Fdeviation) / Fxtal * 2^24 + 1/2
       * and Fdeviation = 3kHz  */
  /* Set FSK Deviation to 0A8E for 1200BPS FSK and 09D5 for 9600
   * Radio lab sets this to 0547 but given TX is working, this is left at 0A8E */
  if (rate_9600) {
      ax5043WriteReg(device, AX5043_FSKDEV2                 ,0x00);
      ax5043WriteReg(device, AX5043_FSKDEV1                 ,0x09);
      ax5043WriteReg(device, AX5043_FSKDEV0                 ,0xD5);
  } else {
      ax5043WriteReg(device, AX5043_FSKDEV2                 ,0x00);
      ax5043WriteReg(device, AX5043_FSKDEV1                 ,0x0A);
      ax5043WriteReg(device, AX5043_FSKDEV0                 ,0x8E);
  }
  //TODO - G0KLA Sets up ANTENNA, MODCFGA sets bit shape as well, we should likely read first and just set the bits needed
  ax5043WriteReg(device, AX5043_MODCFGA                 ,0x05); // TX differential antenna
  //ax5043WriteReg(device, AX5043_MODCFGA                 ,0x06); // TX Single ended through antenna

  /* TXRATE = BITRATE/Fxtal * 2^24 + 1/2
   * Where Fxtal = 16,000,000
   */
  /* Set the data rate to 0x04EB for 1200bps.  Radio lab agrees with this value */
  if (rate_9600) {
      ax5043WriteReg(device, AX5043_TXRATE2                 ,0x00);
      ax5043WriteReg(device, AX5043_TXRATE1                 ,0x27);
      ax5043WriteReg(device, AX5043_TXRATE0                 ,0x52);
  } else {
      ax5043WriteReg(device, AX5043_TXRATE2                 ,0x00);
      ax5043WriteReg(device, AX5043_TXRATE1                 ,0x04);
      ax5043WriteReg(device, AX5043_TXRATE0                 ,0xEA);
  }


  //  ax5043WriteReg(device, AX5043_TXPWRCOEFFB1            ,0x0F); // max output pwr
  //  ax5043WriteReg(device, AX5043_TXPWRCOEFFB0            ,0xFF);
  //  ax5043WriteReg(device, AX5043_TXPWRCOEFFB1            ,0x02); // 1 mW output pwr
  // ax5043WriteReg(device, AX5043_TXPWRCOEFFB0            ,0x78); // 1 mW output pwr
  //ax5043WriteReg(device, AX5043_TXPWRCOEFFB1            ,0x00); // -10dBm output pwr
  //ax5043WriteReg(device, AX5043_TXPWRCOEFFB0            ,0xb3); // -10dBm output pwr
  ax5043WriteReg(device, AX5043_TXPWRCOEFFB1            ,0x00); // Start it out with totally minimum pwr
  ax5043WriteReg(device, AX5043_TXPWRCOEFFB0            ,0x20);

  /**
   * PLL VCO Current
   * GOLF sets this to 97, which is half the radio lab suggested value.  Given it works this is left as 97
   * Radio lab suggests AA ox 0x87 for 9600
   */
  ax5043WriteReg(device, AX5043_PLLVCOI                 ,0x97);

  ax5043WriteReg(device, AX5043_PLLRNGCLK               ,0x03); // per radio lab.  Unclear what this means due to misprint in programming manual
  ax5043WriteReg(device, AX5043_BBTUNE                  ,0x0F); // basegand tuning value - per radio lab
  ax5043WriteReg(device, AX5043_BBOFFSCAP               ,0x77); // baseband gain offset compensation capacitors - per radio lab

  /*
   * PKTADDRXFG
   * Send AX25 LSB first.  Disable FEC sync search.  Addr bytes set to 0.  Note this was 80 = MSB First for GOLF/FOX
   * Radio lab suggests 40, which is CRC skip first
   * This was set to 20 to disable FEC sync search, but maybe that is needed
   */
  ax5043WriteReg(device, AX5043_PKTADDRCFG              ,0x20); //  0x20 is right value for 1200bps, disables FEC sync search This work for 9600 too

  /**
   * Set to F0 for arbitrary length packets
   * Contains position and length of the length byte
   * Radio lab suggests 00 for HDLC Pkt
   */
  ax5043WriteReg(device, AX5043_PKTLENCFG               ,0x00);

  /**
   * GOLF sets this to 09.
   * Radio lab suggests 06
   * I had this at 00 for no offset, but that was wrong.  It is not ignored if we have no length byte
   */
  ax5043WriteReg(device, AX5043_PKTLENOFFSET            ,0x06);

  /**
   * Radio lab says F0 for PKTMAXLEN
   */
  ax5043WriteReg(device, AX5043_PKTMAXLEN               ,0xF0); // max 240 bytes for a packet

  /**
   * The match values are from radio lab, but likely only becuse I put 7E preamble byte in
   * the entry field.  Need to determine if this matters or is needed.
   */
  ax5043WriteReg(device, AX5043_MATCH0PAT3              ,0xAA);
  ax5043WriteReg(device, AX5043_MATCH0PAT2              ,0xCC);
  ax5043WriteReg(device, AX5043_MATCH0PAT1              ,0xAA);
  ax5043WriteReg(device, AX5043_MATCH0PAT0              ,0xCC);
//  ax5043WriteReg(device, AX5043_MATCH0LEN               ,0x00); // zero means pattern length is zero.  Match 0 used for Preamble 2
//  ax5043WriteReg(device, AX5043_MATCH0MAX               ,0x7E);
  ax5043WriteReg(device, AX5043_MATCH1PAT1              ,0x7E);  /// 7E for 1200  TODO - does this stay the same for 9600??
  ax5043WriteReg(device, AX5043_MATCH1PAT0              ,0x7E);
  if (rate_9600) // unclear if this needs to be different, but it is from radio lab
      ax5043WriteReg(device, AX5043_MATCH1LEN               ,0x0A); // Bit 7 - 1 = raw, 0 = scrambled
  else
      ax5043WriteReg(device, AX5043_MATCH1LEN               ,0x8A);

  ax5043WriteReg(device, AX5043_MATCH1MAX               ,0x0A);

  ax5043WriteReg(device, AX5043_TMGTXBOOST              ,0x32); // per radio lab
  ax5043WriteReg(device, AX5043_TMGTXSETTLE             ,0x14); // per radio lab
  ax5043WriteReg(device, AX5043_TMGRXBOOST              ,0x32);  // Receive PLL Boost Time.  Default is 0x32
  ax5043WriteReg(device, AX5043_TMGRXSETTLE             ,0x14); // Receive PLL settle time.  Default is 0x14
  ax5043WriteReg(device, AX5043_TMGRXOFFSACQ            ,0x00); // per radio lab
  ax5043WriteReg(device, AX5043_TMGRXCOARSEAGC          ,0x73); // per radio lab
  ax5043WriteReg(device, AX5043_TMGRXRSSI               ,0x03); // per radio lab
  ax5043WriteReg(device, AX5043_TMGRXPREAMBLE2          ,0x17); // Preamble 2 timeout - This was 0x35 for GOLF.  Radio lab says 17
  ax5043WriteReg(device, AX5043_RSSIABSTHR              ,0xE0); // This was DD in GOLF, radio lab says E0.  RSSI levels above this value indicate a busy channel.  We don't care with a full duplex radio
  ax5043WriteReg(device, AX5043_BGNDRSSITHR             ,0x00); // per radio lab
  ax5043WriteReg(device, AX5043_PKTCHUNKSIZE            ,0x0D); // Set FIFO chunk size to max of 240 bytes
  ax5043WriteReg(device, AX5043_PKTACCEPTFLAGS          ,0x20); // 3F for debug, otherwise Set to 0x20 Bit 5 LRGP - enables packets that span multiple fifo chunks.  We dont want that if we use HDLC framing
  ax5043WriteReg(device, AX5043_DACVALUE1               ,0x00);
  ax5043WriteReg(device, AX5043_DACVALUE0               ,0x00);
  ax5043WriteReg(device, AX5043_DACCONFIG               ,0x00);
  ax5043WriteReg(device, AX5043_REF                     ,0x03); // Per programming manual

  /**
   * Set the Fxtal signal.  Depends on TXCO vs Crystal
   * Sets the REFDIV to be less than 24.8MHz which is the signal into the Phase Comparator (PD)
   *
   * bool txco - true if this is a TXCO false for a crystal
   * unsigned int xtal_cap - The default capacitance is 8pF. This value adds 0.5 * xtal_cap pF.
   *
   * These values assume a 16MHz xtal
   */
#ifdef AX5043_USES_TCXO
  ax5043WriteReg(device, AX5043_XTALOSC                 ,0x04);  // 0x04 for TXCO
  ax5043WriteReg(device, AX5043_XTALAMPL                ,0x00);  // 0x00 for TXCO
#else
  ax5043WriteReg(device, AX5043_XTALOSC                 ,0x03);  // 0x03 for XTAL
  ax5043WriteReg(device, AX5043_XTALAMPL                ,0x07);  // 0x07 for XTAL
#endif

  ax5043WriteReg(device, AX5043_0xF1C                   ,0x07); // Programming manual specifies this value
  ax5043WriteReg(device, AX5043_0xF21                   ,0x68); // TODO - Programming manual specifies 5C
  ax5043WriteReg(device, AX5043_0xF22                   ,0xFF); // TODO - Programming manual specifies 53
  ax5043WriteReg(device, AX5043_0xF23                   ,0x84); // TODO - Programming manual specifies 76
  ax5043WriteReg(device, AX5043_0xF26                   ,0x98); // TODO - Programming manual specifies 92

  /**
   * PERFTUNE52
   *
   */
  if (band_vhf) {
      ax5043WriteReg(device, AX5043_0xF34                   ,0x08); /* PERFTUNE52 - Set  RFDIV OFF in reg PLLVCODIV  - Set to 08 for VHF */
  } else {
      ax5043WriteReg(device, AX5043_0xF34                   ,0x28); /* PERFTUNE52 - Set to 28 if RFDIV ON in reg PLLVCODIV for UHF */
  }
  ax5043WriteReg(device, AX5043_0xF35                   ,0x10); // XTAL div = 1

  ax5043WriteReg(device, AX5043_0xF44                   ,0x25); // In JB code set to 24 and in programming manual.  Was 25 in GOLF code and frpm Radio Lab.
  ax5043WriteReg(device, AX5043_MODCFGP                 ,0xE7); //  0xE7 disables PSK 0xE1 for PSK - but only needed if we actually want PSK?  Not in radio lab output

  ax5043WriteReg(device, AX5043_0xF0C                   ,0x00);  // Per programming manual, though 0x00 is the default

}

static void ax5043_ax25_init_registers(AX5043Device device, bool band_vhf, bool rate_9600) {
    ax5043_ax25_set_registers(device, band_vhf, rate_9600);

#ifdef LEGACY_GOLF
    uint8_t regValue;
    regValue = ax5043ReadReg(device, AX5043_PKTLENOFFSET);
    /* regValue += axradio_framing_swcrclen; // add len offs for software CRC16 (used for both, fixed and variable length packets */
    ax5043WriteReg(device, AX5043_PKTLENOFFSET, regValue);

    ax5043WriteReg(device, AX5043_PINFUNCIRQ, 0x00); // No IRQ used for now
//    ax5043WriteReg(device, AX5043_PKTSTOREFLAGS, axradio_phy_innerfreqloop ? 0x13 : 0x15); // store RF offset, RSSI and delimiter timing

    axradio_setaddrregs();
#endif
}

/**
 * TODO - why is this run after all the setup.  It appears to range the PLL again, but we have
 * already done that.  It also seems to read/write the VCO bias current
 *
 */
static uint8_t ax5043_ax25_init_registers_common(AX5043Device device) {
    uint8_t rng = axradio_phy_chanpllrng[0];
    if (rng & 0x20)
        return AXRADIO_ERR_RANGING;
    if (ax5043ReadReg(device, AX5043_PLLLOOP) & 0x80) {
        ax5043WriteReg(device, AX5043_PLLRANGINGB, rng & 0x0F);
    } else {
        ax5043WriteReg(device, AX5043_PLLRANGINGA, rng & 0x0F);
    }
    rng = axradio_get_pllvcoi(device);
    if (rng & 0x80)
        ax5043WriteReg(device, AX5043_PLLVCOI, rng);

    return AXRADIO_ERR_NOERROR;
}

static void axradio_wait_for_xtal(AX5043Device device) {
    //printf("INFO: Waiting for crystal (axradio_wait_for_xtal)\n");
    while ((ax5043ReadReg(device, AX5043_XTALSTATUS) & 0x01) == 0) {
        vTaskDelay(CENTISECONDS(1));
    }
    //printf("INFO: Crystal is ready\n");
}

/**
 * THEN THE SETTINGS THAT ARE JUST FOR THE TX
 */

static void ax5043_ax25_set_registers_tx(AX5043Device device, bool band_vhf, bool rate_9600) {
    ax5043WriteReg(device, AX5043_PLLLOOP                 ,0x0A);
    ax5043WriteReg(device, AX5043_PLLCPI                  ,0x10);
    if (band_vhf) {
        ax5043WriteReg(device, AX5043_PLLVCODIV               ,0x30);  // sets the internal inductor.  Set to 0x30 for external needed on vhf
    } else {
        ax5043WriteReg(device, AX5043_PLLVCODIV               ,0x24);  // sets the internal inductor. UHF uses internal inductor
    }
//    ax5043WriteReg(device, AX5043_0xF0D                   ,0x03);  // Per J Brandenburg

    if (!rate_9600) {
        /* For AFSK we need another pair of registers for MARK / SPACE
         * AFSK(Mark/Space) = Freq*2^18/Fxtal + 1/2
         * Freq Space = 2200Hz
         * Freq Mark = 1200Hz
         */
        ax5043WriteReg(device, AX5043_AFSKSPACE1,          0x00);
        ax5043WriteReg(device, AX5043_AFSKSPACE0,          0x24); // 0r 25?  This was 24 in Golf
        ax5043WriteReg(device, AX5043_AFSKMARK1,           0x00);
        ax5043WriteReg(device, AX5043_AFSKMARK0,           0x14);
    }
#ifdef AX5043_USES_TCXO
    ax5043WriteReg(device, AX5043_XTALCAP, 0x00);
#else
     /* XTAL load capacitance is added by the chip and is not external on the board. So we need to set the
        * value here using formula C in pf = 8 + 0.5 * XTALCAP */
     unsigned int xtal_cap = 7;
     ax5043WriteReg(device, AX5043_XTALCAP, xtal_cap);
#endif
     ax5043WriteReg(device, AX5043_0xF00                   ,0x0F);  // Per programming manual
     ax5043WriteReg(device, AX5043_0xF18                   ,0x06);

}

static uint8_t ax5043_ax25_init_registers_tx(AX5043Device device, bool band_vhf, bool rate_9600) {
    ax5043_ax25_set_registers_tx(device, band_vhf, rate_9600);
    return ax5043_ax25_init_registers_common(device);
}

void ax5043_prepare_tx(AX5043Device device, bool band_vhf, bool rate_9600) {
    ax5043WriteReg(device, AX5043_PWRMODE, AX5043_PWRSTATE_XTAL_ON);
    ax5043WriteReg(device, AX5043_PWRMODE, AX5043_PWRSTATE_FIFO_ON);
    ax5043_ax25_init_registers_tx(device, band_vhf, rate_9600);
    ax5043WriteReg(device, AX5043_FIFOTHRESH1, 0);
    ax5043WriteReg(device, AX5043_FIFOTHRESH0, 0x80);
    axradio_wait_for_xtal(device);
    ax5043ReadReg(device, AX5043_POWSTICKYSTAT); // clear pwr management sticky status --> brownout gate works
}


/**
 * THEN SETTINGS THAT ARE JUST FOR THE RX
 */

static void ax5043_ax25_set_registers_rx(AX5043Device device, bool band_vhf, bool rate_9600) {

    /* PLLLOOP configs PLL filter and sets freq A or B */
   ax5043WriteReg(device, AX5043_PLLLOOP                 ,0x0A);  // 0B - Use FREQ A and 500kHz loop filter.  Set to 0A for 200kHz using less current
   ax5043WriteReg(device, AX5043_PLLCPI                  ,0x10);
   if (band_vhf) {
       ax5043WriteReg(device, AX5043_PLLVCODIV               ,0x30);  // Set to 0x30 for external inductor needed on vhf
   } else {
       ax5043WriteReg(device, AX5043_PLLVCODIV               ,0x24);  // sets the internal inductor for UHF
   }
   if (!rate_9600) {
       /* For AFSK we need another pair of registers for MARK / SPACE
        * AFSK(Mark/Space) = Freq*DECIMATION*2^18/Fxtal + 1/2
        * Freq Space = 2200Hz
        * Freq Mark = 1200Hz
        * */
       ax5043WriteReg(device, AX5043_AFSKSPACE1,          0x00);
       ax5043WriteReg(device, AX5043_AFSKSPACE0,          0x48); // D9 Per black magic
       ax5043WriteReg(device, AX5043_AFSKMARK1,           0x00);
       ax5043WriteReg(device, AX5043_AFSKMARK0,           0x27); // 76 per black magic
   }
#ifdef AX5043_USES_TCXO
    ax5043WriteReg(device, AX5043_XTALCAP, 0x00);
#else

   /* XTAL load capacitance is added by the chip and is not external on the board. So we need to set the
      * value here using formula C in pf = 8 + 0.5 * XTALCAP */
   unsigned int xtal_cap = 7;  // Needs to be set for each crystal
   ax5043WriteReg(device, AX5043_XTALCAP, xtal_cap);
#endif
   ax5043WriteReg(device, AX5043_0xF00                   ,0x0F);  // Per programming manual
   ax5043WriteReg(device, AX5043_0xF18                   ,0x06); // I had 02, but not sure why or from where..

}

/**
 * This is called once the ranging is complete.  It finalizes the registers for receive
 */
uint8_t ax5043_ax25_init_registers_rx(AX5043Device device, bool band_vhf, bool rate_9600) {
    ax5043_ax25_set_registers_rx(device, band_vhf, rate_9600);
    return ax5043_ax25_init_registers_common(device);

}

static void ax5043_set_registers_rxcont(AX5043Device device){
    ax5043WriteReg(device, AX5043_TMGRXAGC,                 0x00);  // Receiver AGC settling time.  Default is 00
    ax5043WriteReg(device, AX5043_TMGRXPREAMBLE1,           0x00);  // Receiver preamble timeout - Default is 00
    ax5043WriteReg(device, AX5043_PKTMISCFLAGS,             0x00);  // 0 turns all these off
}

uint8_t ax5043_receiver_on_continuous(AX5043Device device) {
    uint8_t regValue;

    ax5043WriteReg(device, AX5043_RSSIREFERENCE, axradio_phy_rssireference); //
    ax5043_set_registers_rxcont(device);

    regValue = ax5043ReadReg(device, AX5043_PKTSTOREFLAGS);
    regValue &= (uint8_t)~0x40;                      // 40 stores RSSI and background noise measure at ant selection time
    ax5043WriteReg(device, AX5043_PKTSTOREFLAGS, regValue);


    ax5043WriteReg(device, AX5043_FIFOSTAT, 3); // clear FIFO data & flags
    ax5043WriteReg(device, AX5043_PWRMODE, AX5043_PWRSTATE_FULL_RX);

    return AXRADIO_ERR_NOERROR;
}
/*
 * Initialization routines
 */

/**
 * axradio_init
 *
 * This is run at startup.  It initializes the radio for TX and runs the PLL ranging
 * Then it initializes the radio for RX on the same frequency and ranges the PLL again.
 * If everything works it returns success - AXRADIO_ERR_NOERROR
 *
 */
uint8_t axradio_init(AX5043Device device, bool band_vhf, int32_t freq, bool rate_9600) {
    //printf("Inside axradio_init_70cm\n");

    /* Store the current state and reset the radio.  This makes sure everything is
     * clean as we start up */
    axradio_mode = AXRADIO_MODE_UNINIT;
    axradio_trxstate = trxstate_off;
    if (ax5043_reset(device)) // this also confirms that we can read/write to the chip
        return AXRADIO_ERR_NOCHIP;

    ax5043_ax25_init_registers(device, band_vhf, rate_9600);
    ax5043_ax25_set_registers_tx(device, band_vhf, rate_9600);

    /* Setup for PLL ranging to make sure we can lock onto the requested frequency */
    ax5043WriteReg(device, AX5043_PLLLOOP, 0x09); // default 100kHz loop BW for ranging
    ax5043WriteReg(device, AX5043_PLLCPI, 0x08);

    // range all channels
    ax5043WriteReg(device, AX5043_PWRMODE, AX5043_PWRSTATE_XTAL_ON);
    ax5043WriteReg(device, AX5043_MODULATION              ,0x08); // GOLF uses 08.  It should not matter as just fopr ranging.  07 is FSK. 0A is AFSK.  0x04 is PSK
    // TODO - should the freq dev be set to zero values here to make ranging work better??  It is in the GOLF code vs 0a8e
    ax5043WriteReg(device, AX5043_FSKDEV2                 ,0x00);
    ax5043WriteReg(device, AX5043_FSKDEV1                 ,0x00);
    ax5043WriteReg(device, AX5043_FSKDEV0                 ,0x00);
    axradio_wait_for_xtal(device);

    quick_setfreq(device, freq);

    axradio_trxstate = trxstate_pll_ranging;

#if 0

    printf("AX5043_PLLLOOP: %02.2x\n", ax5043ReadReg(device, AX5043_PLLLOOP));
    printf("AX5043_PLLCPI: %02.2x\n", ax5043ReadReg(device, AX5043_PLLCPI));

    printf("AX5043_PLLVCOI: %02.2x\n", ax5043ReadReg(device, AX5043_PLLVCOI));
    printf("AX5043_PLLRANGINGA: %02.2x\n", ax5043ReadReg(device, AX5043_PLLRANGINGA));
    printf("AX5043_PLLVCODIV: %02.2x\n", ax5043ReadReg(device, AX5043_PLLVCODIV));

    printf ("AX5043_FREQA0: %x\n", ax5043ReadReg(device, AX5043_FREQA0));
    printf ("AX5043_FREQA1: %x\n", ax5043ReadReg(device, AX5043_FREQA1));
    printf ("AX5043_FREQA2: %x\n", ax5043ReadReg(device, AX5043_FREQA2));
    printf ("AX5043_FREQA3: %x\n", ax5043ReadReg(device, AX5043_FREQA3));
#endif

    //debug_print("Before ranging: AX5043_PLLRANGINGA: %02.2x\n", ax5043ReadReg(device, AX5043_PLLRANGINGA)); //DEBUG RBG

    uint8_t r;
    if( !(axradio_phy_chanpllrnginit[0] & 0xF0) ) { // start values for ranging available
        r = axradio_phy_chanpllrnginit[0] | 0x10;
    }
    else {
        r = 0x18;
    }
    ax5043WriteReg(device, AX5043_PLLRANGINGA, r); // init ranging process starting from "range"

    //debug_print("Starting ranging from: %d\n", r); //RBG DEBUG

    //printf("INFO: Waiting for PLL ranging process\n");
    while ((ax5043ReadReg(device, AX5043_PLLRANGINGA) & 0x10) != 0) {
        vTaskDelay(CENTISECONDS(1));
    }

//    debug_print("After ranging: AX5043_PLLRANGINGA: %02.2x\n", ax5043ReadReg(device, AX5043_PLLRANGINGA)); //DEBUG RBG

    //printf("INFO: PLL ranging process complete\n");
    axradio_trxstate = trxstate_off;
    axradio_phy_chanpllrng[0] = ax5043ReadReg(device, AX5043_PLLRANGINGA);

#if 0
    // VCOI Calibration
    if (axradio_phy_vcocalib) {
        ax5043_set_registers_tx();
        ax5043WriteReg(device, AX5043_MODULATION, 0x08);
        ax5043WriteReg(device, AX5043_FSKDEV2, 0x00);
        ax5043WriteReg(device, AX5043_FSKDEV1, 0x00);
        ax5043WriteReg(device, AX5043_FSKDEV0, 0x00);

        regValue = ax5043ReadReg(device, AX5043_PLLLOOP);
        regValue |= 0x04;
        ax5043WriteReg(device, AX5043_PLLLOOP, regValue);
        {
            uint8_t x = ax5043ReadReg(device, AX5043_0xF35);
            x |= 0x80;
            if (2 & (uint8_t)~x)
                ++x;
            ax5043WriteReg(device, AX5043_0xF35, x);
        }
        ax5043WriteReg(device, AX5043_PWRMODE, AX5043_PWRSTATE_SYNTH_TX);
        {
            uint8_t vcoisave = ax5043ReadReg(device, AX5043_PLLVCOI);
            uint8_t j = 2;
            axradio_phy_chanvcoi[0] = 0;
            ax5043WriteReg(device, AX5043_PLLRANGINGA, axradio_phy_chanpllrng[0] & 0x0F);
            {
                uint32_t f = axradio_phy_chanfreq[0];
                ax5043WriteReg(device, AX5043_FREQA0, f);
                ax5043WriteReg(device, AX5043_FREQA1, f >> 8);
                ax5043WriteReg(device, AX5043_FREQA2, f >> 16);
                ax5043WriteReg(device, AX5043_FREQA3, f >> 24);
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
            ax5043WriteReg(device, AX5043_PLLVCOI, vcoisave);
        }
    }
#endif

    ax5043WriteReg(device, AX5043_PWRMODE, AX5043_PWRSTATE_POWERDOWN);
    ax5043_ax25_init_registers(device, band_vhf, rate_9600);
    ax5043_ax25_set_registers_rx(device, band_vhf, rate_9600);  // TODO - G0KLA - why is this RX?  Both TX and RX ranging is run??
    ax5043WriteReg(device, AX5043_PLLRANGINGA, axradio_phy_chanpllrng[0] & 0x0F);

    axradio_mode = AXRADIO_MODE_OFF;

#if 0

    printf("AX5043_PLLLOOP: %02.2x\n", ax5043ReadReg(device, AX5043_PLLLOOP));
    printf("AX5043_PLLCPI: %02.2x\n", ax5043ReadReg(device, AX5043_PLLCPI));

    printf("AX5043_PLLVCOI: %02.2x\n", ax5043ReadReg(device, AX5043_PLLVCOI));
    printf("AX5043_PLLRANGINGA: %02.2x\n", ax5043ReadReg(device, AX5043_PLLRANGINGA));
    printf("AX5043_PLLVCODIV: %02.2x\n", ax5043ReadReg(device, AX5043_PLLVCODIV));


#endif

    if (axradio_phy_chanpllrng[0] & 0x20) {
        debug_print("*** PLL RANGE ERROR!\n");
        return AXRADIO_ERR_RANGING;
    } else {
//        debug_print("PLL LOCK: %d\n",axradio_phy_chanpllrng[0] & 0x40);
    }
    return AXRADIO_ERR_NOERROR;
}


uint8_t mode_tx(AX5043Device device, bool band_vhf, bool rate_9600) {
    int retVal;

    retVal = ax5043_off(device);
    if (retVal != AXRADIO_ERR_NOERROR) {
        return retVal;
    }

    retVal = ax5043_ax25_init_registers_tx(device, band_vhf, rate_9600);
    if (retVal != AXRADIO_ERR_NOERROR) {
        return retVal;
    }

    return AXRADIO_ERR_NOERROR;
}


uint8_t mode_rx(AX5043Device device, bool band_vhf, bool rate_9600) {
    int retVal;

    retVal = ax5043_off(device);
    if (retVal != AXRADIO_ERR_NOERROR) {
        return retVal;
    }

    retVal = ax5043_ax25_init_registers_rx(device, band_vhf, rate_9600);
    if (retVal != AXRADIO_ERR_NOERROR) {
        return retVal;
    }

    retVal = ax5043_receiver_on_continuous(device);
    if (retVal != AXRADIO_ERR_NOERROR) {
        return retVal;
    }

    return AXRADIO_ERR_NOERROR;
}


/**
 * FUNCTIONS THAT ARE COMMON ACROSS MODES.
 */

/**
 * ax5043_reset()
 *
 * Reset the chip, then read the version and confirm we can write to scratch area.
 *
 * Returns RADIO_OK if all is well.
 */
static uint8_t ax5043_reset(AX5043Device device) {
	//printf("INFO: Resetting AX5043 (ax5043_reset)\n");
	uint8_t i;
	// Initialize Interface
	// Reset Device
	ax5043WriteReg(device, AX5043_PWRMODE, 0x80);
	ax5043WriteReg(device, AX5043_PWRMODE, AX5043_PWRSTATE_POWERDOWN);
	// Wait some time for regulator startup
	vTaskDelay(CENTISECONDS(1));

	// Check the version and that we can read/write to scratch.  Then we know the chip is connected
	i = ax5043ReadReg(device, AX5043_SILICONREVISION);
	i = ax5043ReadReg(device, AX5043_SILICONREVISION);

	if (i != SILICONREV1)
		return RADIO_ERR_REVISION;

	ax5043WriteReg(device, AX5043_SCRATCH, 0x55);
	if (ax5043ReadReg(device, AX5043_SCRATCH) != 0x55)
		return RADIO_ERR_COMM;

	ax5043WriteReg(device, AX5043_SCRATCH, 0xAA);
	if (ax5043ReadReg(device, AX5043_SCRATCH) != 0xAA)
		return RADIO_ERR_COMM;

	return RADIO_OK;
}




/**
 * TODO - need to understand what this does.
 * It seems to get the VCO bias current
 */
uint8_t axradio_get_pllvcoi(AX5043Device device)
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
    return ax5043ReadReg(device, AX5043_PLLVCOI);
}


void start_ax25_rx(AX5043Device device, bool rate_9600) {
    uint32_t freq;
    int status = 0;
    bool band_vhf = FALSE;
    freq = ReadMRAMCommandFreq();
    if (freq < 150000000)
        band_vhf = TRUE;
    debug_print("Starting RX with AX5043Device %d, vhf=%d 9600bps=%d, freq=%d\n", device, band_vhf, rate_9600, freq);
    //uint8_t retVal;
    ax5043WriteReg(device, AX5043_PINFUNCIRQ, 0x0); //disable IRQs
//    debug_print("In start_rx, Setting freq to %d\n", freq); //DEBUG RBG
    if ((status = axradio_init(device, band_vhf, freq, rate_9600)) != AXRADIO_ERR_NOERROR) {
        printf("ERROR: In start_rx, axradio_init returned: %d\n", status);
    }

    mode_rx(device, band_vhf, rate_9600);

    ax5043WriteReg(device, AX5043_IRQMASK0, 0x01); // FIFO not Empty
    ax5043WriteReg(device, AX5043_IRQMASK1, 0);    // FIFO not Empty
    ax5043WriteReg(device, AX5043_IRQINVERSION0, (ax5043ReadReg(device, AX5043_IRQINVERSION0) & 0xFE)); // Don't invert the interrupt

    ax5043WriteReg(device, AX5043_PINFUNCIRQ, 0x3); //enable IRQs

    // ax5043 should be receiving packets now and interrupting on FIFO not empty

}

void start_ax25_tx(AX5043Device device, bool rate_9600) {
    uint16_t irqreq;
    uint16_t irqs = 0;
    uint32_t freq;
    int status = 0;
    bool band_vhf = FALSE;

    freq = ReadMRAMTelemFreq();
    if (freq < 150000000)
        band_vhf = TRUE;
    debug_print("Starting TX with AX5043Device %d, vhf=%d 9600bps=%d, freq=%d\n", device, band_vhf, rate_9600, freq);

    /* Get ready for TX */
  //printf("Disabling IRQs\n");
  ax5043WriteReg(device, AX5043_PINFUNCIRQ, 0x0); //disable IRQs
//  debug_print("In start_tx, Setting freq to %d\n", freq); //DEBUG RBG
  if ((status = axradio_init(device, band_vhf, freq, rate_9600)) != AXRADIO_ERR_NOERROR) {
      printf("ERROR: In start_tx, axradio_init_70cm returned: %d", status);
      // TODO - what do we do if this error is returned?  Wait and try again?  Same issue for RX
  }

  //printf("axradio_init_70cm status: %d\n", status);

  mode_tx(device, band_vhf, rate_9600);
  ax5043_prepare_tx(device, band_vhf, rate_9600);

/* Set up IRQ on FIFO_FREE > THRESHOLD */
  ax5043WriteReg(device, AX5043_FIFOTHRESH1, 0);
  ax5043WriteReg(device, AX5043_FIFOTHRESH0, 150);  //Interrupt when FIFO has more than this many free bytes.
  
  irqreq = ax5043ReadReg(device, AX5043_IRQREQUEST0);
  irqreq |= (ax5043ReadReg(device, AX5043_IRQREQUEST1) << 8);
  //printf("AX5043_IRQREQUEST: %x\n", irqreq);

  /* Clear FIFO Flags */
  ax5043ReadReg(device, AX5043_RADIOEVENTREQ0); // make sure REVRDONE bit is cleared
  ax5043WriteReg(device, AX5043_FIFOSTAT, 3); // clear FIFO data & flags

  /* Set Power state to FULL_TX */
  ax5043WriteReg(device, AX5043_PWRMODE, AX5043_PWRSTATE_FULL_TX);
  //printf("Powerstate is FULL_TX\n");

  irqs = ax5043ReadReg(device, AX5043_IRQMASK0);
  irqs |= (ax5043ReadReg(device, AX5043_IRQMASK1) << 8);
  //printf("before: AX5043_IRQMASK: %x\n", irqs);

  irqs |= 0x08; //FifoFree > THRESHOLD

  ax5043WriteReg(device, AX5043_IRQMASK0, irqs & 0xff);
  ax5043WriteReg(device, AX5043_IRQMASK1, irqs >> 8);

  irqs = ax5043ReadReg(device, AX5043_IRQMASK0);
  irqs |= (ax5043ReadReg(device, AX5043_IRQMASK1) << 8);
  //printf("After: AX5043_IRQMASK: %x\n", irqs);
  
  //printf("Enabling IRQs\n");

  ax5043WriteReg(device, AX5043_PINFUNCIRQ, 0x3); //enable IRQs

}

void fifo_send_sync(AX5043Device device, int final) {
  uint8_t i;
  uint8_t flags = 0x1C; // RAW, NOCRC, RESIDUE
  //uint8_t pktstart = 0x01;
  uint8_t pktend = 0x02;
  uint8_t sync_left_just_with_stop_bit[4] = {0x8f,0x9a,0x42,0xbb};

  if (final) {
    flags |= pktend;
  }

  uint8_t len = 4;
  ax5043WriteReg(device, AX5043_FIFODATA, AX5043_FIFOCMD_DATA | (7 << 5));
  ax5043WriteReg(device, AX5043_FIFODATA, len+1); //len includes the flag byte
  ax5043WriteReg(device, AX5043_FIFODATA, flags);

  for (i=0; i<4; i++) {
    ax5043WriteReg(device, AX5043_FIFODATA, sync_left_just_with_stop_bit[i]);
  }

}

void fifo_commit(AX5043Device device) {
  ax5043WriteReg(device, AX5043_FIFOSTAT, 4);
  //  usleep(1000);
      vTaskDelay(1);

}  

void fifo_repeat_byte(AX5043Device device, uint8_t b, uint8_t count, uint8_t flags) {
  ax5043WriteReg(device, AX5043_FIFODATA, AX5043_FIFOCMD_REPEATDATA | (3 << 5));
  ax5043WriteReg(device, AX5043_FIFODATA, flags);
  ax5043WriteReg(device, AX5043_FIFODATA, count);
  ax5043WriteReg(device, AX5043_FIFODATA, b);
}

void fifo_queue_buffer(AX5043Device device, uint8_t *buf, uint8_t len, uint8_t flags) {
  ax5043WriteReg(device, AX5043_FIFODATA, AX5043_FIFOCMD_DATA | (7 << 5));
  ax5043WriteReg(device, AX5043_FIFODATA, len+1); //len includes the flag byte
  ax5043WriteReg(device, AX5043_FIFODATA, flags);
  while(len--) {
    ax5043WriteReg(device, AX5043_FIFODATA, *buf++);
  }
}

uint16_t fifo_free(AX5043Device device) {
  return (256*ax5043ReadReg(device, AX5043_FIFOFREE1) + ax5043ReadReg(device, AX5043_FIFOFREE0));
}

uint8_t get_rssi(AX5043Device device) {
    int8_t byteVal;
    int16_t wordVal;
    byteVal = (int8_t)ax5043ReadReg(device, AX5043_RSSI);
    wordVal = (int16_t)byteVal;
    wordVal -=64;
    wordVal +=255;
    return (uint8_t)wordVal;
}


int32_t axradio_conv_freq_fromhz(int32_t f) {

  return (int32_t) (f * 1.048576);
}


int32_t axradio_conv_freq_tohz(int32_t f) {
  return (int32_t) (f / 1.048576);
}


void quick_setfreq(AX5043Device device, int32_t f) {
  int32_t f1 = axradio_conv_freq_fromhz(f);

  /* Set LSB, per AX5043 documentation, to prevent synthesizer spurs */
  f1 |= 1;

  ax5043WriteReg(device, AX5043_FREQA0, f1);
  ax5043WriteReg(device, AX5043_FREQA1, f1 >> 8);
  ax5043WriteReg(device, AX5043_FREQA2, f1 >> 16);
  ax5043WriteReg(device, AX5043_FREQA3, f1 >> 24);
}

void test_rx_freq(AX5043Device device, uint32_t freq) {
    ax5043PowerOn(device);

     printf("Transmitting on receive freq: %d\n", freq);

     uint8_t retVal = axradio_init(device, BAND_VHF, freq, RATE_9600);
     printf("axradio_init: %d\n",retVal);

     retVal = mode_tx(device, BAND_VHF, RATE_9600);
     printf("mode_tx: %d\n",retVal);

     ax5043WriteReg(device, AX5043_PWRMODE, AX5043_PWRSTATE_FULL_TX);
     printf("Powerstate is FULL_TX\n");

     printf("AX5043_XTALCAP: %d\n", ax5043ReadReg(device, AX5043_XTALCAP));

     int i=0;
     while(i < 500) {
         fifo_repeat_byte(device, 0xAA, 200, 0);
         fifo_commit(device);
         i++;
     }
}
void test_pll_2m_range(AX5043Device device, bool rate_9600) {
    int32_t i;

    ax5043PowerOn(device);

    for (i = 134000000; i < 159000000; i+= 1000000) {
      printf("\n\nFreq: %d\n", i);
      uint8_t retVal = axradio_init(device, BAND_VHF, i, rate_9600);
      printf("axradio_init: %d\n",retVal);
    }

//    ax5043PowerOff();
}

