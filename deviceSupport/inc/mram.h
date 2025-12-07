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


#define MRAM_OP_RCRS  0x46 /* Read config registers */
#define MRAM_OP_WCRS  0x87 /* Write config registers */

#define FRAM_1M_ID 0x24
#define FRAM_512K_ID 0x23
#define FRAM_1M_ADDRESS_LENGTH 3    /* This is for the M-RAM */
#define FRAM_1M_ADDRESS_MAX 0x1ffff
#define FRAM_4M_ADDRESS_LENGTH 3    /* This is for the M-RAM */
#define FRAM_4M_ADDRESS_MAX 0x7ffff

#define FRAM_512K_ADDRESS_LENGTH 2    /* This is for the RAMTRON F-RAM */
#define FRAM_512K_ADDRESS_MAX 0xffff

#define MAX_MRAM_PARTITIONS 3

#define MRAM_STATUS_ADDR_MASK 0x73
#define MRAM_STATUS_ADDR_2 0x23
#define MRAM_STATUS_ADDR_3 0x32
#define MRAM_STATUS_WEL 0x2 /*Pay attention to write protect bits*/
#define MRAM_STATUS_WPROT 0xC /*These bits clear to enable all banks in MRAM */

#define MRAM_MFG_ID_AVALANCHE 0xe6

/*
 * Size of partition 0.  This is the partition used by non-volatile
 * management for various statistics.
 *
 * Partition 1 is whatever is left over.
 */
#define MRAM_PARTITION_0_SIZE sizeof(MRAMmap_t)

/* Initialize the MRAM system. */
int initMRAM(bool newDevice);

/*
 * Address the MRAM data by partition.  The MRAM code makes all the
 * MRAM devices look like a single big device and parititions it.
 */
int getMRAMPartitionSize(int partition);
int getMRAMAddressSize(void);
bool writeMRAM(int partition,
	       void const * const data, uint32_t length, uint32_t address);
bool readMRAM(int partition,
	      void *data, uint32_t length, uint32_t address);

/* Commands for addressing individual MRAM devices, mostly for status. */
uint8_t readMRAMStatus(int mramNum);
void writeMRAMStatus(int mramNum, uint8_t newStat);
bool writeEnableMRAM(int mramNum);
int getMRAMSize(int mramNum);
bool MRAMSleep(int mramNum);
bool MRAMWake(int mramNum);

/* Test code. */
bool testMRAM(int add);

#endif /* FRAM_H_ */
