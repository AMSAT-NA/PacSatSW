
#ifdef LEGACY_GOLF

// TX: fcarrier=445.300MHz dev=  0.000kHz br=  1.200kBit/s pwr= 15.0dBm
// RX: fcarrier=445.300MHz bw=  1.200kHz br=  1.200kBit/s

void ax5043_set_frequency(uint32_t regVal){
        ax5043WriteReg(AX5043_FREQA0, regVal & 0xFF);
        ax5043WriteReg(AX5043_FREQA1, (regVal >> 8) & 0xFF);
        ax5043WriteReg(AX5043_FREQA2, (regVal >> 16) & 0xFF);
        ax5043WriteReg(AX5043_FREQA3, (regVal >> 24) & 0xFF);

}
void ax5043_set_power(uint32_t regVal){
    ax5043WriteReg(AX5043_TXPWRCOEFFB0,regVal);
    ax5043WriteReg(AX5043_TXPWRCOEFFB1,regVal>>8);
}


/**
 * Set all of the base registers
 */
static void ax5043_set_registers(void) {


  ax5043WriteReg(AX5043_MODULATION              ,0x0A); // AFSK.  0x04 is PSK

  ax5043WriteReg(AX5043_ENCODING                ,0x03); // Differential encoding, bit inversion, no scrambler.  Use 0x07 for G3RUH scrambler
  ax5043WriteReg(AX5043_FRAMING                 ,0x14); // HDLC framing with CRC.  Was 0x06 for GOLF format
  ax5043WriteReg(AX5043_PINFUNCSYSCLK           ,0x01); // This sets the the pin to output 1/16th of crystal freq.  Set to 06 for 1/4
  ax5043WriteReg(AX5043_PINFUNCDCLK             ,0x01);
  ax5043WriteReg(AX5043_PINFUNCDATA             ,0x01);
  ax5043WriteReg(AX5043_PINFUNCANTSEL           ,0x00); //Toggles Amplifier PA Pin to off, set to 01 if it is on
  ax5043WriteReg(AX5043_PINFUNCPWRAMP           ,0x07);
  ax5043WriteReg(AX5043_WAKEUPXOEARLY           ,0x01);
  ax5043WriteReg(AX5043_IFFREQ1                 ,0x00);
  ax5043WriteReg(AX5043_IFFREQ0                 ,0xCD);
  ax5043WriteReg(AX5043_DECIMATION              ,0x7F);
  ax5043WriteReg(AX5043_RXDATARATE2             ,0x00);
  ax5043WriteReg(AX5043_RXDATARATE1             ,0x34);
  ax5043WriteReg(AX5043_RXDATARATE0             ,0x7E);
  ax5043WriteReg(AX5043_MAXDROFFSET2            ,0x00);
  ax5043WriteReg(AX5043_MAXDROFFSET1            ,0x00);
  ax5043WriteReg(AX5043_MAXDROFFSET0            ,0x00);
  ax5043WriteReg(AX5043_MAXRFOFFSET2            ,0x80);
  ax5043WriteReg(AX5043_MAXRFOFFSET1            ,0x00);
  ax5043WriteReg(AX5043_MAXRFOFFSET0            ,0x00);
  ax5043WriteReg(AX5043_AMPLFILTER              ,0x00);
  ax5043WriteReg(AX5043_RXPARAMSETS             ,0xF4);
  ax5043WriteReg(AX5043_AGCGAIN0                ,0xE8);
  ax5043WriteReg(AX5043_AGCTARGET0              ,0x84);
  ax5043WriteReg(AX5043_TIMEGAIN0               ,0xD8);
  ax5043WriteReg(AX5043_DRGAIN0                 ,0xD2);
  ax5043WriteReg(AX5043_PHASEGAIN0              ,0x83);
  ax5043WriteReg(AX5043_FREQUENCYGAINA0         ,0x46);
  ax5043WriteReg(AX5043_FREQUENCYGAINB0         ,0x0A);
  ax5043WriteReg(AX5043_FREQUENCYGAINC0         ,0x1F);
  ax5043WriteReg(AX5043_FREQUENCYGAIND0         ,0x1F);
  ax5043WriteReg(AX5043_AMPLITUDEGAIN0          ,0x06);
  ax5043WriteReg(AX5043_FREQDEV10               ,0x00);
  ax5043WriteReg(AX5043_FREQDEV00               ,0x00);
  ax5043WriteReg(AX5043_BBOFFSRES0              ,0x00);
  ax5043WriteReg(AX5043_AGCGAIN1                ,0xE8);
  ax5043WriteReg(AX5043_AGCTARGET1              ,0x84);
  ax5043WriteReg(AX5043_AGCAHYST1               ,0x00);
  ax5043WriteReg(AX5043_AGCMINMAX1              ,0x00);
  ax5043WriteReg(AX5043_TIMEGAIN1               ,0xD6);
  ax5043WriteReg(AX5043_DRGAIN1                 ,0xD1);
  ax5043WriteReg(AX5043_PHASEGAIN1              ,0x83);
  ax5043WriteReg(AX5043_FREQUENCYGAINA1         ,0x46);
  ax5043WriteReg(AX5043_FREQUENCYGAINB1         ,0x0A);
  ax5043WriteReg(AX5043_FREQUENCYGAINC1         ,0x1F);
  ax5043WriteReg(AX5043_FREQUENCYGAIND1         ,0x1F);
  ax5043WriteReg(AX5043_AMPLITUDEGAIN1          ,0x06);
  ax5043WriteReg(AX5043_FREQDEV11               ,0x00);
  ax5043WriteReg(AX5043_FREQDEV01               ,0x00);
  ax5043WriteReg(AX5043_FOURFSK1                ,0x16);
  ax5043WriteReg(AX5043_BBOFFSRES1              ,0x00);
  ax5043WriteReg(AX5043_AGCGAIN3                ,0xFF);
  ax5043WriteReg(AX5043_AGCTARGET3              ,0x84);
  ax5043WriteReg(AX5043_AGCAHYST3               ,0x00);
  ax5043WriteReg(AX5043_AGCMINMAX3              ,0x00);
  ax5043WriteReg(AX5043_TIMEGAIN3               ,0xD5);
  ax5043WriteReg(AX5043_DRGAIN3                 ,0xD0);
  ax5043WriteReg(AX5043_PHASEGAIN3              ,0x83);
  ax5043WriteReg(AX5043_FREQUENCYGAINA3         ,0x46);
  ax5043WriteReg(AX5043_FREQUENCYGAINB3         ,0x0A);
  ax5043WriteReg(AX5043_FREQUENCYGAINC3         ,0x1F);
  ax5043WriteReg(AX5043_FREQUENCYGAIND3         ,0x1F);
  ax5043WriteReg(AX5043_AMPLITUDEGAIN3          ,0x06);
  ax5043WriteReg(AX5043_FREQDEV13               ,0x00);
  ax5043WriteReg(AX5043_FREQDEV03               ,0x00);
  ax5043WriteReg(AX5043_FOURFSK3                ,0x16);
  ax5043WriteReg(AX5043_BBOFFSRES3              ,0x00);

  /* Set the Frequency Deviation */
  /* Where FSKDEV = (0.858785 * Fdevisation) / Fxtal * 2^24 + 1/2
       * and Fdeviation = 3kHz  */
  /* Set FSK Deviation to 0A8E for 1200BPS FSK */
  ax5043WriteReg(AX5043_FSKDEV2                 ,0x00);
  ax5043WriteReg(AX5043_FSKDEV1                 ,0x0A);
  ax5043WriteReg(AX5043_FSKDEV0                 ,0x8E);

  //TODO - G0KLA Sets up ANTENNA, MODCFGA sets bit shape as well, we should likely read first and just set the bits needed
  ax5043WriteReg(AX5043_MODCFGA                 ,0x05); // TX differential antenna
  //ax5043WriteReg(AX5043_MODCFGA                 ,0x06); // TX Single ended through antenna

  /* TXRATE = BITRATE/Fxtal * 2^24 + 1/2
   * Where Fxtal = 16,000,000
   */
  /* Set the data rate to 0x04EB for 1200bps */
  ax5043WriteReg(AX5043_TXRATE2                 ,0x00);
  ax5043WriteReg(AX5043_TXRATE1                 ,0x04);
  ax5043WriteReg(AX5043_TXRATE0                 ,0xEA);


  //  ax5043WriteReg(AX5043_TXPWRCOEFFB1            ,0x0F); // max output pwr
  //  ax5043WriteReg(AX5043_TXPWRCOEFFB0            ,0xFF);
  //  ax5043WriteReg(AX5043_TXPWRCOEFFB1            ,0x02); // 1 mW output pwr
  // ax5043WriteReg(AX5043_TXPWRCOEFFB0            ,0x78); // 1 mW output pwr
  //ax5043WriteReg(AX5043_TXPWRCOEFFB1            ,0x00); // -10dBm output pwr
  //ax5043WriteReg(AX5043_TXPWRCOEFFB0            ,0xb3); // -10dBm output pwr
  ax5043WriteReg(AX5043_TXPWRCOEFFB1            ,0x00); // Start it out with totally minimum pwr
  ax5043WriteReg(AX5043_TXPWRCOEFFB0            ,0x20);

  ax5043WriteReg(AX5043_PLLVCOI                 ,0x97);
  ax5043WriteReg(AX5043_PLLRNGCLK               ,0x03);
  ax5043WriteReg(AX5043_BBTUNE                  ,0x0F);
  ax5043WriteReg(AX5043_BBOFFSCAP               ,0x77);
  ax5043WriteReg(AX5043_PKTADDRCFG              ,0x00); // Send AX25 LSB first.  Note this was 80 = MSB First for GOLF/FOX
  ax5043WriteReg(AX5043_PKTLENCFG               ,0xF0); // arbitrary length packets
  ax5043WriteReg(AX5043_PKTLENOFFSET            ,0x09);
  ax5043WriteReg(AX5043_PKTMAXLEN               ,0xFF); // max 255 bytes for a packet
  ax5043WriteReg(AX5043_MATCH0PAT3              ,0xAA);
  ax5043WriteReg(AX5043_MATCH0PAT2              ,0xCC);
  ax5043WriteReg(AX5043_MATCH0PAT1              ,0xAA);
  ax5043WriteReg(AX5043_MATCH0PAT0              ,0xCC);
  ax5043WriteReg(AX5043_MATCH0LEN               ,0x1F);
  ax5043WriteReg(AX5043_MATCH0MAX               ,0x1F);
  ax5043WriteReg(AX5043_MATCH1PAT1              ,0x55);
  ax5043WriteReg(AX5043_MATCH1PAT0              ,0x55);
  ax5043WriteReg(AX5043_MATCH1LEN               ,0x8A);
  ax5043WriteReg(AX5043_MATCH1MAX               ,0x0A);
  ax5043WriteReg(AX5043_TMGTXBOOST              ,0x32);
  ax5043WriteReg(AX5043_TMGTXSETTLE             ,0x14);
  ax5043WriteReg(AX5043_TMGRXBOOST              ,0x32);
  ax5043WriteReg(AX5043_TMGRXSETTLE             ,0x14);
  ax5043WriteReg(AX5043_TMGRXOFFSACQ            ,0x00);
  ax5043WriteReg(AX5043_TMGRXCOARSEAGC          ,0x73);
  ax5043WriteReg(AX5043_TMGRXRSSI               ,0x03);
  ax5043WriteReg(AX5043_TMGRXPREAMBLE2          ,0x35);
  ax5043WriteReg(AX5043_RSSIABSTHR              ,0xDD);
  ax5043WriteReg(AX5043_BGNDRSSITHR             ,0x00);
  ax5043WriteReg(AX5043_PKTCHUNKSIZE            ,0x0D); // Set FIFO chunk size to max of 240 bytes
  ax5043WriteReg(AX5043_PKTACCEPTFLAGS          ,0x20); //Bit 5 LRGP. 01 enables packets that span multiple fifo chunks.  We dont want that if we use HDLC framing
  ax5043WriteReg(AX5043_DACVALUE1               ,0x00);
  ax5043WriteReg(AX5043_DACVALUE0               ,0x00);
  ax5043WriteReg(AX5043_DACCONFIG               ,0x00);
  ax5043WriteReg(AX5043_REF                     ,0x03); // Per programming manual
  /**
   * Set the Fxtal signal.  Depends on TXCO vs Crystal
   * Sets the REFDIV to be less than 24.8MHz which is the signal into the Phase Comparator (PD)
   *
   * bool txco - true if this is a TXCO false for a crystal
   * unsigned int xtal_cap - The default capacitance is 8pF. This value adds 0.5 * xtal_cap pF.
   *
   * These values assume a 16MHz xtal
   */
  ax5043WriteReg(AX5043_XTALOSC                 ,0x03);  // 0x04 for TXCO
  ax5043WriteReg(AX5043_XTALAMPL                ,0x07);  // 0x00 for TXCO
  ax5043WriteReg(AX5043_0xF35                   ,0x10); // XTAL div = 1
  /* XTAL load capacitance is added by the chip and is not external on the board. So we need to set the
     * value here using formula C in pf = 8 + 0.5 * XTALCAP */
  unsigned int xtal_cap = 5;  // TODO - G0KLA this should be a define as it is set for each crystal
  ax5043WriteReg(AX5043_XTALCAP, xtal_cap);

  ax5043WriteReg(AX5043_0xF1C                   ,0x07); // Programming manual specifies this value
  ax5043WriteReg(AX5043_0xF21                   ,0x68); // TODO - Programming manual specifies 5C
  ax5043WriteReg(AX5043_0xF22                   ,0xFF); // TODO - Programming manual specifies 53
  ax5043WriteReg(AX5043_0xF23                   ,0x84); // TODO - Programming manual specifies 76
  ax5043WriteReg(AX5043_0xF26                   ,0x98); // TODO - Programming manual specifies 92
  ax5043WriteReg(AX5043_0xF34                   ,0x28); /* PERFTUNE52 - RFDIV ON - Set to 08 for VHF */

  ax5043WriteReg(AX5043_0xF44                   ,0x24); // In JB code but set to 24 and in programming manual.  Was 25 in GOLF code.
  ax5043WriteReg(AX5043_MODCFGP                 ,0xE7); //  0xE7 disables PSK 0xE1 for PSK

  ax5043WriteReg(AX5043_0xF00                   ,0x0F);  // Per programming manual
  ax5043WriteReg(AX5043_0xF0C                   ,0x00);  // Per programming manual, though 0x00 is the default

#ifdef LEGACY_GOLF
  ax5043WriteReg(AX5043_MODULATION              ,0x04);
  ax5043WriteReg(AX5043_ENCODING                ,0x03);
  ax5043WriteReg(AX5043_FRAMING                 ,0x06);
  ax5043WriteReg(AX5043_PINFUNCSYSCLK           ,0x01);
  ax5043WriteReg(AX5043_PINFUNCDCLK             ,0x01);
  ax5043WriteReg(AX5043_PINFUNCDATA             ,0x01);
  ax5043WriteReg(AX5043_PINFUNCANTSEL           ,0x01);
  ax5043WriteReg(AX5043_PINFUNCPWRAMP           ,0x07);
  ax5043WriteReg(AX5043_WAKEUPXOEARLY           ,0x01);
  ax5043WriteReg(AX5043_IFFREQ1                 ,0x00);
  ax5043WriteReg(AX5043_IFFREQ0                 ,0xCD);
  ax5043WriteReg(AX5043_DECIMATION              ,0x7F);
  ax5043WriteReg(AX5043_RXDATARATE2             ,0x00);
  ax5043WriteReg(AX5043_RXDATARATE1             ,0x34);
  ax5043WriteReg(AX5043_RXDATARATE0             ,0x7E);
  ax5043WriteReg(AX5043_MAXDROFFSET2            ,0x00);
  ax5043WriteReg(AX5043_MAXDROFFSET1            ,0x00);
  ax5043WriteReg(AX5043_MAXDROFFSET0            ,0x00);
  ax5043WriteReg(AX5043_MAXRFOFFSET2            ,0x80);
  ax5043WriteReg(AX5043_MAXRFOFFSET1            ,0x00);
  ax5043WriteReg(AX5043_MAXRFOFFSET0            ,0x00);
  ax5043WriteReg(AX5043_AMPLFILTER              ,0x00);
  ax5043WriteReg(AX5043_RXPARAMSETS             ,0xF4);
  ax5043WriteReg(AX5043_AGCGAIN0                ,0xE8);
  ax5043WriteReg(AX5043_AGCTARGET0              ,0x84);
  ax5043WriteReg(AX5043_TIMEGAIN0               ,0xD8);
  ax5043WriteReg(AX5043_DRGAIN0                 ,0xD2);
  ax5043WriteReg(AX5043_PHASEGAIN0              ,0x83);
  ax5043WriteReg(AX5043_FREQUENCYGAINA0         ,0x46);
  ax5043WriteReg(AX5043_FREQUENCYGAINB0         ,0x0A);
  ax5043WriteReg(AX5043_FREQUENCYGAINC0         ,0x1F);
  ax5043WriteReg(AX5043_FREQUENCYGAIND0         ,0x1F);
  ax5043WriteReg(AX5043_AMPLITUDEGAIN0          ,0x06);
  ax5043WriteReg(AX5043_FREQDEV10               ,0x00);
  ax5043WriteReg(AX5043_FREQDEV00               ,0x00);
  ax5043WriteReg(AX5043_BBOFFSRES0              ,0x00);
  ax5043WriteReg(AX5043_AGCGAIN1                ,0xE8);
  ax5043WriteReg(AX5043_AGCTARGET1              ,0x84);
  ax5043WriteReg(AX5043_AGCAHYST1               ,0x00);
  ax5043WriteReg(AX5043_AGCMINMAX1              ,0x00);
  ax5043WriteReg(AX5043_TIMEGAIN1               ,0xD6);
  ax5043WriteReg(AX5043_DRGAIN1                 ,0xD1);
  ax5043WriteReg(AX5043_PHASEGAIN1              ,0x83);
  ax5043WriteReg(AX5043_FREQUENCYGAINA1         ,0x46);
  ax5043WriteReg(AX5043_FREQUENCYGAINB1         ,0x0A);
  ax5043WriteReg(AX5043_FREQUENCYGAINC1         ,0x1F);
  ax5043WriteReg(AX5043_FREQUENCYGAIND1         ,0x1F);
  ax5043WriteReg(AX5043_AMPLITUDEGAIN1          ,0x06);
  ax5043WriteReg(AX5043_FREQDEV11               ,0x00);
  ax5043WriteReg(AX5043_FREQDEV01               ,0x00);
  ax5043WriteReg(AX5043_FOURFSK1                ,0x16);
  ax5043WriteReg(AX5043_BBOFFSRES1              ,0x00);
  ax5043WriteReg(AX5043_AGCGAIN3                ,0xFF);
  ax5043WriteReg(AX5043_AGCTARGET3              ,0x84);
  ax5043WriteReg(AX5043_AGCAHYST3               ,0x00);
  ax5043WriteReg(AX5043_AGCMINMAX3              ,0x00);
  ax5043WriteReg(AX5043_TIMEGAIN3               ,0xD5);
  ax5043WriteReg(AX5043_DRGAIN3                 ,0xD0);
  ax5043WriteReg(AX5043_PHASEGAIN3              ,0x83);
  ax5043WriteReg(AX5043_FREQUENCYGAINA3         ,0x46);
  ax5043WriteReg(AX5043_FREQUENCYGAINB3         ,0x0A);
  ax5043WriteReg(AX5043_FREQUENCYGAINC3         ,0x1F);
  ax5043WriteReg(AX5043_FREQUENCYGAIND3         ,0x1F);
  ax5043WriteReg(AX5043_AMPLITUDEGAIN3          ,0x06);
  ax5043WriteReg(AX5043_FREQDEV13               ,0x00);
  ax5043WriteReg(AX5043_FREQDEV03               ,0x00);
  ax5043WriteReg(AX5043_FOURFSK3                ,0x16);
  ax5043WriteReg(AX5043_BBOFFSRES3              ,0x00);
  ax5043WriteReg(AX5043_FSKDEV2                 ,0x00);
  ax5043WriteReg(AX5043_FSKDEV1                 ,0x00);
  ax5043WriteReg(AX5043_FSKDEV0                 ,0x00);
  ax5043WriteReg(AX5043_MODCFGA                 ,0x05);
  //ax5043WriteReg(AX5043_MODCFGA                 ,0x06); // TXSE
  ax5043WriteReg(AX5043_TXRATE2                 ,0x00);
  ax5043WriteReg(AX5043_TXRATE1                 ,0x04);
  ax5043WriteReg(AX5043_TXRATE0                 ,0xEA);
  //  ax5043WriteReg(AX5043_TXPWRCOEFFB1            ,0x0F); // max output pwr
  //  ax5043WriteReg(AX5043_TXPWRCOEFFB0            ,0xFF);
  //  ax5043WriteReg(AX5043_TXPWRCOEFFB1            ,0x02); // 1 mW output pwr
  // ax5043WriteReg(AX5043_TXPWRCOEFFB0            ,0x78); // 1 mW output pwr
  //ax5043WriteReg(AX5043_TXPWRCOEFFB1            ,0x00); // -10dBm output pwr
  //ax5043WriteReg(AX5043_TXPWRCOEFFB0            ,0xb3); // -10dBm output pwr
  ax5043WriteReg(AX5043_TXPWRCOEFFB1            ,0x00); // Start it out with totally minimum pwr
  ax5043WriteReg(AX5043_TXPWRCOEFFB0            ,0x20);
  ax5043WriteReg(AX5043_PLLVCOI                 ,0x97);
  ax5043WriteReg(AX5043_PLLRNGCLK               ,0x03);
  ax5043WriteReg(AX5043_BBTUNE                  ,0x0F);
  ax5043WriteReg(AX5043_BBOFFSCAP               ,0x77);
  ax5043WriteReg(AX5043_PKTADDRCFG              ,0x80);
  ax5043WriteReg(AX5043_PKTLENCFG               ,0x00);
  ax5043WriteReg(AX5043_PKTLENOFFSET            ,0x09);
  ax5043WriteReg(AX5043_PKTMAXLEN               ,0xC8);
  ax5043WriteReg(AX5043_MATCH0PAT3              ,0xAA);
  ax5043WriteReg(AX5043_MATCH0PAT2              ,0xCC);
  ax5043WriteReg(AX5043_MATCH0PAT1              ,0xAA);
  ax5043WriteReg(AX5043_MATCH0PAT0              ,0xCC);
  ax5043WriteReg(AX5043_MATCH0LEN               ,0x1F);
  ax5043WriteReg(AX5043_MATCH0MAX               ,0x1F);
  ax5043WriteReg(AX5043_MATCH1PAT1              ,0x55);
  ax5043WriteReg(AX5043_MATCH1PAT0              ,0x55);
  ax5043WriteReg(AX5043_MATCH1LEN               ,0x8A);
  ax5043WriteReg(AX5043_MATCH1MAX               ,0x0A);
  ax5043WriteReg(AX5043_TMGTXBOOST              ,0x32);
  ax5043WriteReg(AX5043_TMGTXSETTLE             ,0x14);
  ax5043WriteReg(AX5043_TMGRXBOOST              ,0x32);
  ax5043WriteReg(AX5043_TMGRXSETTLE             ,0x14);
  ax5043WriteReg(AX5043_TMGRXOFFSACQ            ,0x00);
  ax5043WriteReg(AX5043_TMGRXCOARSEAGC          ,0x73);
  ax5043WriteReg(AX5043_TMGRXRSSI               ,0x03);
  ax5043WriteReg(AX5043_TMGRXPREAMBLE2          ,0x35);
  ax5043WriteReg(AX5043_RSSIABSTHR              ,0xDD);
  ax5043WriteReg(AX5043_BGNDRSSITHR             ,0x00);
  ax5043WriteReg(AX5043_PKTCHUNKSIZE            ,0x0D);
  ax5043WriteReg(AX5043_PKTACCEPTFLAGS          ,0x20);
  ax5043WriteReg(AX5043_DACVALUE1               ,0x00);
  ax5043WriteReg(AX5043_DACVALUE0               ,0x00);
  ax5043WriteReg(AX5043_DACCONFIG               ,0x00);
  ax5043WriteReg(AX5043_REF                     ,0x03);
  ax5043WriteReg(AX5043_XTALOSC                 ,0x04);
  ax5043WriteReg(AX5043_XTALAMPL                ,0x00);
  ax5043WriteReg(AX5043_0xF1C                   ,0x07);
  ax5043WriteReg(AX5043_0xF21                   ,0x68);
  ax5043WriteReg(AX5043_0xF22                   ,0xFF);
  ax5043WriteReg(AX5043_0xF23                   ,0x84);
  ax5043WriteReg(AX5043_0xF26                   ,0x98);
  ax5043WriteReg(AX5043_0xF34                   ,0x28);
  ax5043WriteReg(AX5043_0xF35                   ,0x10);
  ax5043WriteReg(AX5043_0xF44                   ,0x25);
  ax5043WriteReg(AX5043_MODCFGP                 ,0xE1);

#endif
}

static void ax5043_set_registers_tx(void) {
    ax5043WriteReg(AX5043_PLLLOOP                 ,0x0B);
    ax5043WriteReg(AX5043_PLLCPI                  ,0x10);
    ax5043WriteReg(AX5043_PLLVCODIV               ,0x24);  // sets the internal inductor.  Set to 0x30 for external needed on vhf

//    ax5043WriteReg(AX5043_0xF0D                   ,0x03);  // Per J Brandenburg


    ax5043WriteReg(AX5043_0xF18                   ,0x06); // Unclear what this does but comes from radiolab
    /* For AFSK we need another pair of registers for MARK / SPACE
      * AFSK(Mark/Space) = Freq*2^18/Fxtal + 1/2
      * Freq Space = 2200Hz
      * Freq Mark = 1200Hz
      * */
     ax5043WriteReg(AX5043_AFSKSPACE1,          0x00);
     ax5043WriteReg(AX5043_AFSKSPACE0,          0x24); // 0r 25?  This was 24 in Golf
     ax5043WriteReg(AX5043_AFSKMARK1,           0x00);
     ax5043WriteReg(AX5043_AFSKMARK0,           0x14);

     /* TODO - G0KLA - Signal filtering, for now per Jonathons code */
     ax5043WriteReg(AX5043_MODCFGF                 ,0x03);



#ifdef LEGACY_GOLF
    ax5043WriteReg(AX5043_PLLLOOP                 ,0x0B);
    ax5043WriteReg(AX5043_PLLCPI                  ,0x10);
    ax5043WriteReg(AX5043_PLLVCODIV               ,0x24);
    ax5043WriteReg(AX5043_XTALCAP                 ,0x0F); // adjusted for v1.1 board
    ax5043WriteReg(AX5043_0xF00                   ,0x0F);
    ax5043WriteReg(AX5043_0xF18                   ,0x06);
#endif
}

static void ax5043_set_registers_rx(void) {
   ax5043WriteReg(AX5043_PLLLOOP                 ,0x0B);
   ax5043WriteReg(AX5043_PLLCPI                  ,0x10);
   ax5043WriteReg(AX5043_PLLVCODIV               ,0x24);
   ax5043WriteReg(AX5043_0xF00                   ,0x0F);
   ax5043WriteReg(AX5043_0xF18                   ,0x02);
   /* For AFSK we need another pair of registers for MARK / SPACE
     * AFSK(Mark/Space) = Freq*DECIMATION*2^18/Fxtal + 1/2
     * Freq Space = 2200Hz
     * Freq Mark = 1200Hz
     * */
    ax5043WriteReg(AX5043_AFSKSPACE1,          0x00);
    ax5043WriteReg(AX5043_AFSKSPACE0,          0xD9); // TODO -  This was 24 in Golf for RX?? Why same value as TX?
    ax5043WriteReg(AX5043_AFSKMARK1,           0x00);
    ax5043WriteReg(AX5043_AFSKMARK0,           0x76);

    ax5043WriteReg(AX5043_AFSKCTRL,           0x08);


#ifdef LEGACY_GOLF
   ax5043WriteReg(AX5043_PLLLOOP                 ,0x0B);
   ax5043WriteReg(AX5043_PLLCPI                  ,0x10);
   ax5043WriteReg(AX5043_PLLVCODIV               ,0x24);
   ax5043WriteReg(AX5043_XTALCAP                 ,0x0F); // adjusted for v1.1 board
   ax5043WriteReg(AX5043_0xF00                   ,0x0F);
   ax5043WriteReg(AX5043_0xF18                   ,0x02);
#endif
}

#endif // LEGACY_GOLF

// physical layer


static const uint32_t axradio_phy_chanfreq[2] = { 0x1b3b5550,0x1b3b5550};//Primary and secondry
static const uint8_t axradio_phy_chanpllrnginit[1] = { 0x09 };
static const uint8_t axradio_phy_chanvcoiinit[1] = { 0x97 };
static uint8_t axradio_phy_chanpllrng[1];
static uint8_t axradio_phy_chanvcoi[1];
static const uint8_t axradio_phy_vcocalib = 0;


#if 0
const uint8_t axradio_phy_pn9 = 0;
const uint8_t axradio_phy_nrchannels = 1;
//const uint32_t axradio_phy_chanfreq[1] = { 0x1bd4cccd };
const int32_t axradio_phy_maxfreqoffset = 315;
// axradio_phy_rssioffset is added to AX5043_RSSIREFERENCE and subtracted from chip RSSI value to prevent overflows (8bit RSSI only goes down to -128)
// axradio_phy_rssioffset is also added to AX5043_RSSIABSTHR
//const int8_t axradio_phy_rssireference = 0xFA + 64;
const int8_t axradio_phy_channelbusy = -99 + 64;
const uint16_t axradio_phy_cs_period = 7; // timer0 units, 10ms
const uint8_t axradio_phy_cs_enabled = 0;
const uint8_t axradio_phy_lbt_retries = 0;
const uint8_t axradio_phy_lbt_forcetx = 0;
const uint16_t axradio_phy_preamble_wor_longlen = 2; // wor_longlen + wor_len totals to 240.0ms plus 272bits
const uint16_t axradio_phy_preamble_wor_len = 48;
const uint16_t axradio_phy_preamble_longlen = 1;
const uint16_t axradio_phy_preamble_len = 16;
const uint8_t axradio_phy_preamble_byte = 0xaa;
const uint8_t axradio_phy_preamble_flags = 0x38;
const uint8_t axradio_phy_preamble_appendbits = 0;
const uint8_t axradio_phy_preamble_appendpattern = 0x00;

//framing
const uint8_t axradio_framing_maclen = 0;
const uint8_t axradio_framing_addrlen = 0;
const uint8_t axradio_framing_destaddrpos = 0;
const uint8_t axradio_framing_sourceaddrpos = 0xff;
const uint8_t axradio_framing_lenpos = 0;
const uint8_t axradio_framing_lenoffs = 9;
const uint8_t axradio_framing_lenmask = 0x00;
const uint8_t axradio_framing_swcrclen = 0;

const uint8_t axradio_framing_synclen = 32;
const uint8_t axradio_framing_syncword[] = { 0x33, 0x55, 0x33, 0x55};
const uint8_t axradio_framing_syncflags = 0x18;
const uint8_t axradio_framing_enable_sfdcallback = 0;

const uint32_t axradio_framing_ack_timeout = 204; // 316.3ms in wtimer0 units (640Hz)
const uint32_t axradio_framing_ack_delay = 313; // 1.0ms in wtimer1 units (20MHz/64)
const uint8_t axradio_framing_ack_retransmissions = 0;
const uint8_t axradio_framing_ack_seqnrpos = 0xff;

const uint8_t axradio_framing_minpayloadlen = 0; // must be set to 1 if the payload directly follows the destination address, and a CRC is configured
//WOR
const uint16_t axradio_wor_period = 128;

// synchronous mode
const uint32_t axradio_sync_period = 32768; // ACTUALLY FREQ, NOT PERIOD!
const uint32_t axradio_sync_xoscstartup = 49;
const uint32_t axradio_sync_slave_syncwindow = 98304; // 3.000s
const uint32_t axradio_sync_slave_initialsyncwindow = 5898240; //180.000s
const uint32_t axradio_sync_slave_syncpause = 19660800; // 600.000s
const int16_t axradio_sync_slave_maxperiod = 2020; // in (2^SYNC_K1) * wtimer0 units
const uint8_t axradio_sync_slave_resyncloss = 11;  // resyncloss is one more than the number of missed packets to cause a resync
// window 0 is the first window after synchronisation
// window 1 is the window normally used when there are no lost packets
// window 2 is used after one packet is lost, etc
const uint8_t axradio_sync_slave_nrrx = 3;
const uint32_t axradio_sync_slave_rxadvance[] = { 8978, 8963, 9224 };// 273.982ms, 273.525ms, 281.490ms
const uint32_t axradio_sync_slave_rxwindow[] = { 9031, 9001, 9523 }; // 275.600ms, 274.685ms, 290.615ms
const uint32_t axradio_sync_slave_rxtimeout = 10322; // 315.0ms, maximum duration of a packet

#endif
