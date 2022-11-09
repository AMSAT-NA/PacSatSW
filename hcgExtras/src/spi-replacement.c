/*
 * This is copied from the the original HALCoGen module in /hcg/src/spi.c.  This module
 * replace that one (mostly) for several reasons:  1) We really need to be able
 * to send/receive an array of bytes.  2) We really need to be able to send or
 * receive a long array without having to have an equivalent size for the opposite
 * direction.  3) The HALCoGen code does not have "user code" locations in the right
 * places to do all this.  So this module is very similar to the HALCoGen version
 * except with a few additions to do the above stuff.
 *
 * This is based on the following HALCoGen code.  Note that we use the init
 * routine from the original module.
 *
 */
/** @file spi.c
*   @brief SPI Driver Implementation File
*   @date 11-Dec-2018
*   @version 04.07.01
*/

/* 
* Copyright (C) 2009-2018 Texas Instruments Incorporated - www.ti.com 
* 
* 
*  Redistribution and use in source and binary forms, with or without 
*  modification, are permitted provided that the following conditions 
*  are met:
*
*    Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the   
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#include "spi.h"
#include "sys_vim.h"

#define MAX_SPI_DEVICES 5
static uint8_t dummyBuffer[MAX_SPI_DEVICES][2];
static inline void ForceDummy(unsigned int);

/** @struct g_spiPacket
*   @brief globals
*
*/
static volatile struct g_spiPacket
{
    spiDAT1_t g_spiDataFormat;
    uint32  tx_length;
    uint32  rx_length;
    uint8  * txdata_ptr;
    uint8  * rxdata_ptr;
    SpiDataStatus_t tx_data_status;
    SpiDataStatus_t rx_data_status;
    bool byte; /* Are we using bytes (true) or 16 bit shorts (false)*/
} g_spiPacket_t[5U];

/*
 *  AMSAT (WB1FJ) Notes:
 *  -We still use HALCoGen's spiInit.
 *  -The interrupt routine is here
 *  -The big changes made by WB1FJ are:
 *      -The arrays passed in are now bytes so that we can write out structures,
 *       buffers, and other stuff without separating it out into the LSB of a short.
 *      -I always turn on both Tx and Rx interrupts and we don't consider the I/O complete
 *       until both are complete.  This ensures that the tx is actually at the slave before
 *       we start an Rx for example.
 *      -The interrupt routines, beside being changed to work with bytes, now just pass the
 *       device specific info to a generic interrupt routine.  It's much easier to read, and
 *       only a single location to make code changes.
 *
 *
 */

/** @fn uint32 spiGetData(spiBASE_t *spi, spiDAT1_t *dataconfig_t, uint32 blocksize, uint16 * destbuff)
*   @brief Receives Data using interrupt method
*   @param[in] spi           - Spi module base address
*   @param[in] dataconfig_t    - Spi DAT1 register configuration
*   @param[in] blocksize    - number of data
*   @param[in] destbuff        - Pointer to the destination data (8 bit).
*/
void spiGetDataByte(spiBASE_t *spi, spiDAT1_t *dataconfig_t, uint32 blocksize, uint8 * destbuff)
{
     uint32 index = (spi == spiREG1) ? 0U :((spi==spiREG2) ? 1U : ((spi==spiREG3) ? 2U:((spi==spiREG4) ? 3U:4U)));

/* USER CODE BEGIN (8) */
/* USER CODE END */

     g_spiPacket_t[index].rx_length = blocksize;
     g_spiPacket_t[index].rxdata_ptr   = destbuff;
     g_spiPacket_t[index].g_spiDataFormat = *dataconfig_t;
     g_spiPacket_t[index].rx_data_status = SPI_PENDING;

     g_spiPacket_t[index].tx_length = blocksize;
     g_spiPacket_t[index].txdata_ptr   = 0;
     g_spiPacket_t[index].tx_data_status = SPI_PENDING;

     g_spiPacket_t[index].byte = true;

     spi->INT0 |= 0x0300U;

}

/** @fn void spiSendData(spiBASE_t *spi, spiDAT1_t *dataconfig_t, uint32 blocksize, uint16 * srcbuff)
*   @brief Transmits Data using interrupt method
*   @param[in] spi           - Spi module base address
*   @param[in] dataconfig_t  - Spi DAT1 register configuration
*   @param[in] blocksize      - number of data
*   @param[in] srcbuff        - Pointer to the source data ( 8 bit).
*
*   @return flag register value.
*
*/
void spiSendDataByte(spiBASE_t *spi, spiDAT1_t *dataconfig_t, uint32 blocksize, uint8 * srcbuff)
{
     uint32 index = (spi == spiREG1) ? 0U :((spi==spiREG2) ? 1U : ((spi==spiREG3) ? 2U:((spi==spiREG4) ? 3U:4U)));

     g_spiPacket_t[index].tx_length = blocksize;
     g_spiPacket_t[index].txdata_ptr   = srcbuff;
     g_spiPacket_t[index].g_spiDataFormat = *dataconfig_t;
     g_spiPacket_t[index].tx_data_status = SPI_PENDING;

     g_spiPacket_t[index].rx_length = blocksize;
     g_spiPacket_t[index].rxdata_ptr   = 0;
     g_spiPacket_t[index].rx_data_status = SPI_PENDING;
     g_spiPacket_t[index].byte = true;


     spi->INT0 |= 0x0300U;

}

/** @fn void spiSendAndGetDataByte(spiBASE_t *spi, spiDAT1_t *dataconfig_t, uint32 blocksize, uint16 * srcbuff, uint16 * destbuff)
*   @brief Initiate SPI Transmits and receive Data using Interrupt mode.
*   @param[in] spi           - Spi module base address
*   @param[in] dataconfig_t    - Spi DAT1 register configuration
*   @param[in] blocksize    - number of data
*   @param[in] srcbuff        - Pointer to the source data ( 8 bit).
*   @param[in] destbuff        - Pointer to the destination data ( 8 bit).
*
*/
void spiSendAndGetDataByte(spiBASE_t *spi, spiDAT1_t *dataconfig_t, uint32 blocksize, uint8 * srcbuff, uint8 * destbuff)
{

/* USER CODE BEGIN (17) */
/* USER CODE END */

    uint32 index = (spi == spiREG1) ? 0U :((spi==spiREG2) ? 1U : ((spi==spiREG3) ? 2U:((spi==spiREG4) ? 3U:4U)));

    g_spiPacket_t[index].tx_length       = blocksize;
    g_spiPacket_t[index].rx_length       = blocksize;
    g_spiPacket_t[index].txdata_ptr      = srcbuff;
    g_spiPacket_t[index].rxdata_ptr      = destbuff;
    g_spiPacket_t[index].g_spiDataFormat = *dataconfig_t;
    g_spiPacket_t[index].tx_data_status  = SPI_PENDING;
    g_spiPacket_t[index].rx_data_status  = SPI_PENDING;
    g_spiPacket_t[index].byte = true;

    spi->INT0 |= 0x0300U;

/* USER CODE BEGIN (18) */
/* USER CODE END */
}
#if 0
/*
 *  If we need 16-bit functions, we need to figure out how to make these work in combo with the 8-bit functions
 *
 *  My thought for an easy fix is a different interrupt routine that we switch to if spiPacket.byte is false
 *
 */
void spiGetData(spiBASE_t *spi, spiDAT1_t *dataconfig_t, uint32 blocksize, uint16 * destbuff)
{
     uint32 index = (spi == spiREG1) ? 0U :((spi==spiREG2) ? 1U : ((spi==spiREG3) ? 2U:((spi==spiREG4) ? 3U:4U)));

/* USER CODE BEGIN (8) */
/* USER CODE END */

     g_spiPacket_t[index].rx_length = blocksize;
     g_spiPacket_t[index].rxdata_ptr   = destbuff;
     g_spiPacket_t[index].g_spiDataFormat = *dataconfig_t;
     g_spiPacket_t[index].rx_data_status = SPI_PENDING;
     g_spiPacket_t[index].tx_length = blocksize;
     g_spiPacket_t[index].txdata_ptr   = destbuff;
     g_spiPacket_t[index].tx_data_status = SPI_PENDING;
     g_spiPacket_t[index].byte = false;

     spi->INT0 |= 0x0300U;

/* USER CODE BEGIN (9) */
/* USER CODE END */
}

/** @fn void spiSendData(spiBASE_t *spi, spiDAT1_t *dataconfig_t, uint32 blocksize, uint16 * srcbuff)
*   @brief Transmits Data using interrupt method
*   @param[in] spi           - Spi module base address
*   @param[in] dataconfig_t    - Spi DAT1 register configuration
*   @param[in] blocksize    - number of data
*   @param[in] srcbuff        - Pointer to the source data ( 16 bit).
*
*   @return flag register value.
*
*   This function transmits blocksize number of data from source buffer using interrupt method.
*/
/* SourceId : SPI_SourceId_006 */
/* DesignId : SPI_DesignId_006 */
/* Requirements : HL_SR132 */
void spiSendData(spiBASE_t *spi, spiDAT1_t *dataconfig_t, uint32 blocksize, uint16 * srcbuff)
{
        uint32 index = (spi == spiREG1) ? 0U :((spi==spiREG2) ? 1U : ((spi==spiREG3) ? 2U:((spi==spiREG4) ? 3U:4U)));

/* USER CODE BEGIN (12) */
 /* USER CODE END */

     g_spiPacket_t[index].tx_length = blocksize;
     g_spiPacket_t[index].txdata_ptr   = srcbuff;
     g_spiPacket_t[index].g_spiDataFormat = *dataconfig_t;
     g_spiPacket_t[index].tx_data_status = SPI_PENDING;

     g_spiPacket_t[index].rx_length = blocksize;
     g_spiPacket_t[index].rxdata_ptr   = srcbuff;
     g_spiPacket_t[index].rx_data_status = SPI_PENDING;

     g_spiPacket_t[index].byte = false;

     spi->INT0 |= 0x0300U;

/* USER CODE BEGIN (13) */
/* USER CODE END */
}


/* USER CODE BEGIN (16) */
/* USER CODE END */

/** @fn void spiSendAndGetData(spiBASE_t *spi, spiDAT1_t *dataconfig_t, uint32 blocksize, uint16 * srcbuff, uint16 * destbuff)
*   @brief Initiate SPI Transmits and receive Data using Interrupt mode.
*   @param[in] spi           - Spi module base address
*   @param[in] dataconfig_t    - Spi DAT1 register configuration
*   @param[in] blocksize    - number of data
*   @param[in] srcbuff        - Pointer to the source data ( 16 bit).
*   @param[in] destbuff        - Pointer to the destination data ( 16 bit).
*
*   Initiate SPI Transmits and receive Data using Interrupt mode..
*/
/* SourceId : SPI_SourceId_008 */
/* DesignId : SPI_DesignId_010 */
/* Requirements : HL_SR136 */
void spiSendAndGetData(spiBASE_t *spi, spiDAT1_t *dataconfig_t, uint32 blocksize, uint16 * srcbuff, uint16 * destbuff)
{

/* USER CODE BEGIN (17) */
/* USER CODE END */

    uint32 index = (spi == spiREG1) ? 0U :((spi==spiREG2) ? 1U : ((spi==spiREG3) ? 2U:((spi==spiREG4) ? 3U:4U)));

    g_spiPacket_t[index].tx_length       = blocksize;
    g_spiPacket_t[index].rx_length       = blocksize;
    g_spiPacket_t[index].txdata_ptr      = srcbuff;
    g_spiPacket_t[index].rxdata_ptr      = destbuff;
    g_spiPacket_t[index].g_spiDataFormat = *dataconfig_t;
    g_spiPacket_t[index].tx_data_status  = SPI_PENDING;
    g_spiPacket_t[index].rx_data_status  = SPI_PENDING;
    g_spiPacket_t[index].byte = false;

    spi->INT0 |= 0x0300U;

/* USER CODE BEGIN (18) */
/* USER CODE END */
}
#endif

/** @fn SpiDataStatus_t SpiTxStatus(spiBASE_t *spi)
*   @brief Get the status of the SPI Transmit data block.
*   @param[in] spi           - Spi module base address
*
*   @return Spi Transmit block data status.
*
*   Get the status of the SPI Transmit data block.
*/
/* SourceId : SPI_SourceId_009 */
/* DesignId : SPI_DesignId_013 */
/* Requirements : HL_SR139 */
SpiDataStatus_t SpiTxStatus(spiBASE_t *spi)
{

/* USER CODE BEGIN (19) */
/* USER CODE END */

    uint32 index = (spi == spiREG1) ? 0U :((spi==spiREG2) ? 1U : ((spi==spiREG3) ? 2U:((spi==spiREG4) ? 3U:4U)));
    return(g_spiPacket_t[index].tx_data_status);
}

/** @fn SpiDataStatus_t SpiRxStatus(spiBASE_t *spi)
*   @brief Get the status of the SPI Receive data block.
*   @param[in] spi           - Spi module base address
*
*   @return Spi Receive block data status.
*
*   Get the status of the SPI Receive data block.
*/
/* SourceId : SPI_SourceId_010 */
/* DesignId : SPI_DesignId_014 */
/* Requirements : HL_SR140 */
SpiDataStatus_t SpiRxStatus(spiBASE_t *spi)
{

    uint32 index = (spi == spiREG1) ? 0U :((spi==spiREG2) ? 1U : ((spi==spiREG3) ? 2U:((spi==spiREG4) ? 3U:4U)));
    return(g_spiPacket_t[index].rx_data_status);
}

/*
 * The following is a Generic SPI interrupt routine initially from the HALCoGen bus-specific
 * routine.  Following this are the actual interrupt routines that call it.
 */

static inline void GenericSPIInterrupt(spiBASE_t *thisSPI,int index,uint32 vector)
{
    uint32 flags = (thisSPI->FLG & 0x0000FFFFU) & (thisSPI->LVL & 0x035FU);

    ForceDummy(index); // Check whether there is a dummy buffer for tx and rx and reset it if so
    switch(vector)
    {

    case 0x24U: /* Receive Buffer Full Interrupt */
             {
                uint8 *destbuff;
                destbuff = g_spiPacket_t[index].rxdata_ptr;

                *destbuff = (uint8)thisSPI->BUF;
                /*SAFETYMCUSW 567 S MR:17.1,17.4 <APPROVED> "Pointer increment needed" */
                g_spiPacket_t[index].rxdata_ptr++;
                g_spiPacket_t[index].rx_length--;

                if(g_spiPacket_t[index].rx_length == 0U)
                {
                    thisSPI->INT0 = (thisSPI->INT0 & 0x0000FFFFU) & (~(uint32)0x0100U);
                    g_spiPacket_t[index].rx_data_status = SPI_COMPLETED;
                    spiEndNotification(thisSPI);
                }
                break;
             }

    case 0x28U: /* Transmit Buffer Empty Interrupt */
             {
                 volatile uint32 SpiBuf;
                 uint32 Chip_Select_Hold = 0U;
                 uint32 WDelay = (g_spiPacket_t[index].g_spiDataFormat.WDEL) ? 0x04000000U: 0U;
                 SPIDATAFMT_t DataFormat = g_spiPacket_t[index].g_spiDataFormat.DFSEL;
                 uint8 ChipSelect = g_spiPacket_t[index].g_spiDataFormat.CSNR;
                 uint16 Tx_Data = (uint16)*g_spiPacket_t[index].txdata_ptr;

                 g_spiPacket_t[index].tx_length--;

                 if(g_spiPacket_t[index].tx_length == 0U)
                 {
                    Chip_Select_Hold = 0U;
                 }
                 else
                 {
                    Chip_Select_Hold = (g_spiPacket_t[index].g_spiDataFormat.CS_HOLD) ? 0x10000000U : 0U;
                 }

                 thisSPI->DAT1 = (((uint32)DataFormat << 24U) |
                                  ((uint32)ChipSelect << 16U) |
                                  (WDelay)           |
                                  (Chip_Select_Hold) |
                                  (uint32)Tx_Data);

                 /*SAFETYMCUSW 567 S MR:17.1,17.4 <APPROVED> "Pointer increment needed" */
                 g_spiPacket_t[index].txdata_ptr++;
                 /* Dummy Receive read if no RX Interrupt enabled */
                 if(((thisSPI->INT0 & 0x0000FFFFU) & 0x0100U) == 0U)
                 {
                     if((thisSPI->FLG & 0x00000100U) == 0x00000100U)
                     {
                         SpiBuf = thisSPI->BUF;
                     }
                 }

                 if(g_spiPacket_t[index].tx_length == 0U)
                 {
                    thisSPI->INT0 =(thisSPI->INT0 & 0x0000FFFFU) & (~(uint32)0x0200U); /* Disable Interrupt */
                    g_spiPacket_t[index].tx_data_status = SPI_COMPLETED;
                    // I don't want to be notified when Tx is done.  Must wait for Rx!
                    //spiEndNotification(thisSPI);
                }
                break;
             }

    default: /* Clear Flags and return  */
             thisSPI->FLG = flags;
             spiNotification(thisSPI, flags & 0xFFU);
             break;
    }



}

#pragma CODE_STATE(mibspi1LowLevelInterrupt, 32)
#pragma INTERRUPT(mibspi1LowLevelInterrupt, IRQ)
void mibspi1LowLevelInterrupt(void){
    GenericSPIInterrupt(spiREG1,0,spiREG1->INTVECT1);
}

#pragma CODE_STATE(mibspi1HighLevelInterrupt, 32)
#pragma INTERRUPT(mibspi1HighLevelInterrupt, IRQ)
void mibspi1HighLevelInterrupt(void)
{
    GenericSPIInterrupt(spiREG1,0,spiREG1->INTVECT0);
}
#pragma CODE_STATE(mibspi3LowLevelInterrupt, 32)
#pragma INTERRUPT(mibspi3LowLevelInterrupt, IRQ)
void mibspi3LowLevelInterrupt(void)
{
    GenericSPIInterrupt(spiREG3,0,spiREG3->INTVECT1);
}


#pragma CODE_STATE(mibspi3HighInterruptLevel, 32)
#pragma INTERRUPT(mibspi3HighInterruptLevel, IRQ)
void mibspi3HighInterruptLevel(void)
{
    GenericSPIInterrupt(spiREG3,2,spiREG3->INTVECT0);

}
#pragma CODE_STATE(mibspi3LowLevelInterrupt, 32)
#pragma INTERRUPT(mibspi3LowLevelInterrupt, IRQ)
void mibspi5LowLevelInterrupt(void)
{
    GenericSPIInterrupt(spiREG5,0,spiREG5->INTVECT1);
}


#pragma CODE_STATE(mibspi3HighInterruptLevel, 32)
#pragma INTERRUPT(mibspi3HighInterruptLevel, IRQ)
void mibspi5HighInterruptLevel(void)
{
    GenericSPIInterrupt(spiREG5,2,spiREG5->INTVECT0);

}
static inline void ForceDummy(unsigned int SPIIndex){
    /*
     * If one of the buffers is pointing to the dummy, we just want to
     * continue to set the pointer back and rewrite or re-read the first
     * entry since that data is not important
     */

    if((g_spiPacket_t[SPIIndex].rxdata_ptr == &dummyBuffer[SPIIndex][1]) || (g_spiPacket_t[SPIIndex].rxdata_ptr == (void *)0)){
        g_spiPacket_t[SPIIndex].rxdata_ptr = &dummyBuffer[SPIIndex][0];
    }
    if(g_spiPacket_t[SPIIndex].txdata_ptr == (void *)0) {
        g_spiPacket_t[SPIIndex].txdata_ptr = &dummyBuffer[SPIIndex][0];
        dummyBuffer[SPIIndex][0] = dummyBuffer[SPIIndex][1] = 0;
    } else if((g_spiPacket_t[SPIIndex].txdata_ptr == &dummyBuffer[SPIIndex][1])){
        g_spiPacket_t[SPIIndex].txdata_ptr = &dummyBuffer[SPIIndex][0];
    }

    /* USER CODE END */
}




