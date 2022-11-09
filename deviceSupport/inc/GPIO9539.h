/*
 * GPIO9539.h
 *
 *  Created on: Aug 20, 2019
 *      Author: bfisher
 */

#ifndef DEVICESUPPORT_INC_GPIO9539_H_
#define DEVICESUPPORT_INC_GPIO9539_H_

#include <pacsat.h>


#define PCA9539_ADDR (0x74)

#define PCA9539_REG_INPUT_ALL 0
#define PCA9539_REG_INPUT0 0
#define PCA9539_REG_INPUT1 1

#define PCA9539_REG_OUTPUT0 2
#define PCA9539_REG_OUTPUT1 3

#define PCA9539_REG_INVERT0 4
#define PCA9539_REG_INVERT1 5
#define PCA9539_REG_CONFIG0 6
#define PCA9539_REG_CONFIG1 7


#define PCA9539_PORT_0 (1<<0)
#define PCA9539_PORT_1 (1<<1)
#define PCA9539_PORT_2 (1<<2)
#define PCA9539_PORT_3 (1<<3)
#define PCA9539_PORT_4 (1<<4)
#define PCA9539_PORT_5 (1<<5)
#define PCA9539_PORT_6 (1<<6)
#define PCA9539_PORT_7 (1<<7)

#define PCA9539_BANK_READ 0xFF

/*
 * Note that we the intention of this routine is not to know anything about what the
 * GPIO bits are actually used for.  On Golf, "deployables.c" knows that and can use
 * these routines as appropriate.
 */


bool CheckPCA9539(bool retry);
bool ReadPCA9539Config(uint8_t config[2]);
bool WritePCA9539Config(uint8_t config[2]);
bool ReadPCA9539Bits(int bank,uint8_t *gpioBits);
bool WritePCA9539Bits(int bank, uint8_t *gpioBits);

#endif /* DEVICESUPPORT_INC_GPIO9539_H_ */
