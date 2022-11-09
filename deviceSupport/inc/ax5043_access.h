/*
 * ax5043.h
 *
 *  Created on: Mar 11, 2019
 *      Author: burns
 */

#ifndef DRIVERS_INC_AX5043_ACCESS_H_
#define DRIVERS_INC_AX5043_ACCESS_H_
#include <stdbool.h>
bool IsRxing(void);
void ax5043StartRx(void);
void ax5043StopRx(void);
void ax5043StartTx(void);
void ax5043StopTx(void);
void ax5043PowerOn(void);
void ax5043PowerOff(void);
void ax5043WriteReg(unsigned int reg, unsigned int val);
void ax5043WriteRegMulti(unsigned int firstReg, uint8_t *writeVal,uint8_t length);
void ax5043ReadRegMulti(unsigned int firstReg, uint8_t *readVal,uint8_t length);
unsigned int ax5043ReadLongreg(unsigned int reg,int bytes);
unsigned int ax5043ReadReg(unsigned int reg);
bool ax5043SetClockout(void);

#endif /* DRIVERS_INC_AX5043_ACCESS_H_ */
