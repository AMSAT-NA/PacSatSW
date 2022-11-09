/*
 * HET_EMU.h
 *
 *  Created on: Nov 28, 2012
 *      Author: a0866528
 */

#ifndef HET_I2C_EMU_H_
#define HET_I2C_EMU_H_
#include <pacsat.h>
#define I2C_EMU_READ 1
#define I2C_EMU_WRITE 0
#define I2CRxReady (hetREG1->FLG & (1<<23))
void HetI2CInit(void);
void HetI2CStop(void);
void HetI2CPutData(char Data, char IntEna);
void HetI2CPutAddr(char Addr, char RW, char NumOfBytes, char IntEna, char StopBit);
bool HetI2CGetChar(char *data);
void i2cHETInterrupt(uint32_t offset);

#endif /* HET_EMU_H_ */
