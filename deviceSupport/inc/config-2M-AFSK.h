

// TX: fcarrier=145.835MHz dev=  1.500kHz br=  1.200kBit/s pwr= 15.0dBm
// RX: fcarrier=145.835MHz bw= 25.000kHz br=  1.200kBit/s

static void ax5043_set_registers(void)
{
	ax5043WriteReg(AX5043_MODULATION,          0x0A);
	ax5043WriteReg(AX5043_ENCODING,            0x00);
	ax5043WriteReg(AX5043_FRAMING,             0x06);
	//ax5043WriteReg(AX5043_PINFUNCSYSCLK,       0x90);// Was 0x1
	ax5043WriteReg(AX5043_PINFUNCDCLK,         0x01);
	ax5043WriteReg(AX5043_PINFUNCDATA,         0x01);
	ax5043WriteReg(AX5043_PINFUNCANTSEL,       0x01);
	ax5043WriteReg(AX5043_PINFUNCPWRAMP,       0x07);
	ax5043WriteReg(AX5043_WAKEUPXOEARLY,       0x01);
	ax5043WriteReg(AX5043_IFFREQ1,             0x05);
	ax5043WriteReg(AX5043_IFFREQ0,             0x55);
	ax5043WriteReg(AX5043_DECIMATION,          0x08);
	ax5043WriteReg(AX5043_RXDATARATE2,         0x03);
	ax5043WriteReg(AX5043_RXDATARATE1,         0x41);
	ax5043WriteReg(AX5043_RXDATARATE0,         0x55);
	ax5043WriteReg(AX5043_MAXDROFFSET2,        0x00);
	ax5043WriteReg(AX5043_MAXDROFFSET1,        0x00);
	ax5043WriteReg(AX5043_MAXDROFFSET0,        0x00);
	ax5043WriteReg(AX5043_MAXRFOFFSET2,        0x80);
	ax5043WriteReg(AX5043_MAXRFOFFSET1,        0x01);
	ax5043WriteReg(AX5043_MAXRFOFFSET0,        0x32);
	ax5043WriteReg(AX5043_FSKDMAX1,            0x00);
	ax5043WriteReg(AX5043_FSKDMAX0,            0x00);
	ax5043WriteReg(AX5043_FSKDMIN1,            0x00);
	ax5043WriteReg(AX5043_FSKDMIN0,            0x00);
	ax5043WriteReg(AX5043_AFSKCTRL,            0x0C);
	ax5043WriteReg(AX5043_AMPLFILTER,          0x00);
	ax5043WriteReg(AX5043_RXPARAMSETS,         0xF4);
	ax5043WriteReg(AX5043_AGCGAIN0,            0xE8);
	ax5043WriteReg(AX5043_AGCTARGET0,          0x84);
	ax5043WriteReg(AX5043_TIMEGAIN0,           0xDC);
	ax5043WriteReg(AX5043_DRGAIN0,             0xD6);
	ax5043WriteReg(AX5043_PHASEGAIN0,          0xC3);
	ax5043WriteReg(AX5043_FREQUENCYGAINA0,     0x0F);
	ax5043WriteReg(AX5043_FREQUENCYGAINB0,     0x1F);
	ax5043WriteReg(AX5043_FREQUENCYGAINC0,     0x0D);
	ax5043WriteReg(AX5043_FREQUENCYGAIND0,     0x0D);
	ax5043WriteReg(AX5043_AMPLITUDEGAIN0,      0x06);
	ax5043WriteReg(AX5043_FREQDEV10,           0x00);
	ax5043WriteReg(AX5043_FREQDEV00,           0x00);
	ax5043WriteReg(AX5043_BBOFFSRES0,          0x00);
	ax5043WriteReg(AX5043_AGCGAIN1,            0xE8);
	ax5043WriteReg(AX5043_AGCTARGET1,          0x84);
	ax5043WriteReg(AX5043_AGCAHYST1,           0x00);
	ax5043WriteReg(AX5043_AGCMINMAX1,          0x00);
	ax5043WriteReg(AX5043_TIMEGAIN1,           0xDA);
	ax5043WriteReg(AX5043_DRGAIN1,             0xD5);
	ax5043WriteReg(AX5043_PHASEGAIN1,          0xC3);
	ax5043WriteReg(AX5043_FREQUENCYGAINA1,     0x0F);
	ax5043WriteReg(AX5043_FREQUENCYGAINB1,     0x1F);
	ax5043WriteReg(AX5043_FREQUENCYGAINC1,     0x0D);
	ax5043WriteReg(AX5043_FREQUENCYGAIND1,     0x0D);
	ax5043WriteReg(AX5043_AMPLITUDEGAIN1,      0x06);
	ax5043WriteReg(AX5043_FREQDEV11,           0x00);
	ax5043WriteReg(AX5043_FREQDEV01,           0x00);
	ax5043WriteReg(AX5043_FOURFSK1,            0x16);
	ax5043WriteReg(AX5043_BBOFFSRES1,          0x00);
	ax5043WriteReg(AX5043_AGCGAIN3,            0xFF);
	ax5043WriteReg(AX5043_AGCTARGET3,          0x84);
	ax5043WriteReg(AX5043_AGCAHYST3,           0x00);
	ax5043WriteReg(AX5043_AGCMINMAX3,          0x00);
	ax5043WriteReg(AX5043_TIMEGAIN3,           0xD9);
	ax5043WriteReg(AX5043_DRGAIN3,             0xD4);
	ax5043WriteReg(AX5043_PHASEGAIN3,          0xC3);
	ax5043WriteReg(AX5043_FREQUENCYGAINA3,     0x0F);
	ax5043WriteReg(AX5043_FREQUENCYGAINB3,     0x1F);
	ax5043WriteReg(AX5043_FREQUENCYGAINC3,     0x0D);
	ax5043WriteReg(AX5043_FREQUENCYGAIND3,     0x0D);
	ax5043WriteReg(AX5043_AMPLITUDEGAIN3,      0x06);
	ax5043WriteReg(AX5043_FREQDEV13,           0x00);
	ax5043WriteReg(AX5043_FREQDEV03,           0x00);
	ax5043WriteReg(AX5043_FOURFSK3,            0x16);
	ax5043WriteReg(AX5043_BBOFFSRES3,          0x00);
	ax5043WriteReg(AX5043_MODCFGF,             0x00);
	ax5043WriteReg(AX5043_FSKDEV2,             0x00);
	ax5043WriteReg(AX5043_FSKDEV1,             0x05);
	ax5043WriteReg(AX5043_FSKDEV0,             0x47);
	ax5043WriteReg(AX5043_MODCFGA,             0x05);
	ax5043WriteReg(AX5043_TXRATE2,             0x00);
	ax5043WriteReg(AX5043_TXRATE1,             0x04);
	ax5043WriteReg(AX5043_TXRATE0,             0xEA);
	ax5043WriteReg(AX5043_TXPWRCOEFFB1,        0x0F);
	ax5043WriteReg(AX5043_TXPWRCOEFFB0,        0xFF);
	ax5043WriteReg(AX5043_PLLVCOI,             0x87);
	ax5043WriteReg(AX5043_PLLRNGCLK,           0x03);
	ax5043WriteReg(AX5043_BBTUNE,              0x0F);
	ax5043WriteReg(AX5043_BBOFFSCAP,           0x77);
	ax5043WriteReg(AX5043_PKTADDRCFG,          0x81);
	ax5043WriteReg(AX5043_PKTLENCFG,           0x00);
	ax5043WriteReg(AX5043_PKTLENOFFSET,        0x6c);
	ax5043WriteReg(AX5043_PKTMAXLEN,           0xF0);
	ax5043WriteReg(AX5043_MATCH0PAT3,          0xF1);
	ax5043WriteReg(AX5043_MATCH0PAT2,          0xBA);
	ax5043WriteReg(AX5043_MATCH0PAT1,          0x84);
	ax5043WriteReg(AX5043_MATCH0PAT0,          0xB3);
	ax5043WriteReg(AX5043_MATCH0LEN,           0x9F);
	ax5043WriteReg(AX5043_MATCH0MAX,           0x1D);
	ax5043WriteReg(AX5043_MATCH1PAT1,          0x55);
	ax5043WriteReg(AX5043_MATCH1PAT0,          0x55);
	ax5043WriteReg(AX5043_MATCH1LEN,           0x8A);
	ax5043WriteReg(AX5043_MATCH1MAX,           0x0A);
	ax5043WriteReg(AX5043_TMGTXBOOST,          0x32);
	ax5043WriteReg(AX5043_TMGTXSETTLE,         0x14);
	ax5043WriteReg(AX5043_TMGRXBOOST,          0x32);
	ax5043WriteReg(AX5043_TMGRXSETTLE,         0x14);
	ax5043WriteReg(AX5043_TMGRXOFFSACQ,        0x00);
	ax5043WriteReg(AX5043_TMGRXCOARSEAGC,      0x73);
	ax5043WriteReg(AX5043_TMGRXRSSI,           0x03);
	ax5043WriteReg(AX5043_TMGRXPREAMBLE2,      0x35);
	ax5043WriteReg(AX5043_RSSIABSTHR,          0xE6);
	ax5043WriteReg(AX5043_BGNDRSSITHR,         0x00);
	ax5043WriteReg(AX5043_PKTCHUNKSIZE,        0x0D);
	ax5043WriteReg(AX5043_PKTACCEPTFLAGS,      0x1C);
	ax5043WriteReg(AX5043_DACVALUE1,           0x00);
	ax5043WriteReg(AX5043_DACVALUE0,           0x00);
	ax5043WriteReg(AX5043_DACCONFIG,           0x00);
	ax5043WriteReg(AX5043_REF,                 0x03);
	ax5043WriteReg(AX5043_XTALOSC,             0x04);
	ax5043WriteReg(AX5043_XTALAMPL,            0x00);
	ax5043WriteReg(AX5043_0xF1C,               0x07);
	ax5043WriteReg(AX5043_0xF21,               0x68);
	ax5043WriteReg(AX5043_0xF22,               0xFF);
	ax5043WriteReg(AX5043_0xF23,               0x84);
	ax5043WriteReg(AX5043_0xF26,               0x98);
	ax5043WriteReg(AX5043_0xF34,               0x08);
	ax5043WriteReg(AX5043_0xF35,               0x10);
	ax5043WriteReg(AX5043_0xF44,               0x25);
}


static void ax5043_set_registers_tx(void)
{
	ax5043WriteReg(AX5043_PLLLOOP,             0x09);
	ax5043WriteReg(AX5043_PLLCPI,              0x02);
	ax5043WriteReg(AX5043_PLLVCODIV,           0x30);
	ax5043WriteReg(AX5043_AFSKSPACE1,          0x00);
	ax5043WriteReg(AX5043_AFSKSPACE0,          0x24);
	ax5043WriteReg(AX5043_AFSKMARK1,           0x00);
	ax5043WriteReg(AX5043_AFSKMARK0,           0x14);
	ax5043WriteReg(AX5043_XTALCAP,             0x0F); // adjusted for v1.1 board
	ax5043WriteReg(AX5043_0xF00,               0x0F);
	ax5043WriteReg(AX5043_0xF18,               0x06);
}


static void ax5043_set_registers_rx(void)
{
	ax5043WriteReg(AX5043_PLLLOOP,             0x0A);
	ax5043WriteReg(AX5043_PLLCPI,              0x10);
	ax5043WriteReg(AX5043_PLLVCODIV,           0x30);
	ax5043WriteReg(AX5043_AFSKSPACE1,          0x00);
	ax5043WriteReg(AX5043_AFSKSPACE0,          0x48);
	ax5043WriteReg(AX5043_AFSKMARK1,           0x00);
	ax5043WriteReg(AX5043_AFSKMARK0,           0x27);
	ax5043WriteReg(AX5043_XTALCAP,             0x0F); // adjusted for v1.1 board
	ax5043WriteReg(AX5043_0xF00,               0x0F);
	ax5043WriteReg(AX5043_0xF18,               0x06);
}

static void ax5043_set_registers_rxcont(void)
{
	ax5043WriteReg(AX5043_TMGRXAGC,                 0x00);
	ax5043WriteReg(AX5043_TMGRXPREAMBLE1,           0x00);
	ax5043WriteReg(AX5043_PKTMISCFLAGS,             0x00);
}

static const uint8_t axradio_phy_innerfreqloop = 0;

// physical layer
static const uint32_t  axradio_phy_chanfreq[1] = { 0x091d5c29 }; // ax5043 register value corresponding to 145.835
static const uint8_t  axradio_phy_chanpllrnginit[1] = { 0xFF };
static const uint8_t  axradio_phy_chanvcoiinit[1] = { 0x00 };
static uint8_t  axradio_phy_chanpllrng[1];
static uint8_t  axradio_phy_chanvcoi[1];
static const uint8_t  axradio_phy_vcocalib = 1;
static const int8_t  axradio_phy_rssireference = 57;// 0xF9 + 64;
static const int8_t axradio_phy_rssioffset = 64;

