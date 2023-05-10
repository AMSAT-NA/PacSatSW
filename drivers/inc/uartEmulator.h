#if HET
/*
 * HET_EMU.h
 *
 *  Created on: Nov 28, 2012
 *      Author: a0866528
 */

#ifndef HET_EMU_H_
#define HET_EMU_H_
#include "het.h"
#define SCIRxReady (hetREG1->FLG & (1<<23))
bool HetUARTIsActive(hetRAMBASE_t *hetram);
void HetUARTEnableNotification(hetBASE_t *regs);
void HetUARTPutChar(hetRAMBASE_t *hetram,char Data);
void HetUART1PutText(char *text);
void HetUARTSetBaudrate(hetRAMBASE_t *hetram,unsigned int baud);
unsigned HetUART1Printf(const char *_format, ...);
char HetUART1GetChar(bool noWait);
void HetUARTInit(void);
void serialHETInterrupt(uint32 offset);

/*
 * The following are for the HET-emulated I2c
 */

#define I2C2RxReady (hetREG2->FLG & (1<<23))
void HetI2C2PutData(char Data, char IntEna);
void HetI2C2PutAddr(char Addr, char RW, char NumOfBytes, char IntEna, char StopBit);
//void HetI2CPutRepAddr(char Addr, char RW, char IntEna);
//void HetI2CPutText(char *text);
//unsigned HetI2UART1Printf(const char *_format, ...);
char HetI2C2GetChar(void);


#endif /* HET_EMU_H_ */
#endif
