/*
 * reed_solomon.h
 *
 *  Created on: Feb 3, 2014
 *      Author: fox
 */

#ifndef REEDSOLOMON_H_
#define REEDSOLOMON_H_

void update_rs(
   unsigned char parity[32], // 32-byte encoder state; zero before each frame
   unsigned char c          // Current data byte to update
);

#endif /* REEDSOLOMON_H_ */
