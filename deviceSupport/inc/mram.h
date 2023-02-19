/*
 * fram.h
 *
 *  Created on: Feb 19, 2012
 *      Author: Burns
 */

#ifndef FRAM_H_
#define FRAM_H_

#include <pacsat.h>
#include "spiDriver.h"
#include "stdint.h"

#ifdef FRAM64K
#define ADDRESS_BYTES 2
#else
#define ADDRESS_BYTES 3
#endif

/*
 * These OPcode appear correct both for the RAMTRON F-RAM and
 * for the MRAM we will be using on the flight hardware.
 */
//#define MRAM
#define FRAM_OP_WREN  0b00000110 /* Write Enable */
#define FRAM_OP_WRDI  0b00000100 /* Write Disable */
#define FRAM_OP_RDSR  0b00000101 /* Read Status Register */
#define FRAM_ROP_RDSR  0b10100000 /* Try in reverse */
#define FRAM_OP_WRSR  0b00000001 /* Write Status Register */
#define FRAM_OP_READ  0b00000011 /* Read Memory Data */
#define FRAM_OP_WRITE 0b00000010 /* Write Memory Data */
#define FRAM_OP_SLEEP 0b10111001 /* Enter Sleep Mode */
#define MRAM_OP_WAKE  0b10101011 /* Wake up from sleep--only on MRAM, not FRAM*/
#define FRAM_OP_RDID  0b10011111 /* Read Device ID */

#define FRAM_1M_ID 0x24
#define FRAM_512K_ID 0x23
#define FRAM_1M_ADDRESS_LENGTH 3    /* This is for the M-RAM */
#define FRAM_1M_ADDRESS_MAX 0x1ffff
#define FRAM_4M_ADDRESS_LENGTH 3    /* This is for the M-RAM */
#define FRAM_4M_ADDRESS_MAX 0x7ffff

#define FRAM_512K_ADDRESS_LENGTH 2    /* This is for the RAMTRON F-RAM */
#define FRAM_512K_ADDRESS_MAX 0xffff

SPIDevice GetMRAMAndAddress(uint32_t *addr);
int getMRAMSize(SPIDevice mram);
uint8_t ReadMRAMStatus(SPIDevice mram);
void WriteMRAMStatus(SPIDevice mram,uint8_t newStat);
bool MRAMWriteEnable(SPIDevice mram);
int initMRAM(void);
bool MRAMSleep(int mramNum);
bool MRAMWake(int mramNum);
bool testMRAM(int add);
extern const SPIDevice MRAM_Devices[PACSAT_MAX_MRAMS];

#endif /* FRAM_H_ */
