#ifndef AESDECIPHER_H
#define AESDECIPHER_H
#include "stdint.h"
void DeCipher16(uint8_t*,uint8_t*);
void DeCipher32(uint8_t*,uint8_t*);
uint8_t K[16]; //key
void iSubBytes(void);
void iShiftRows(void);
void iMixColumns(void);
uint8_t iM(uint8_t,uint8_t);
void iFillRK(void);
void iRKApply(void);
void RKExpand(void);
void FillRK(void);
void RoundKey(uint8_t);
bool InitEncryption(void);
uint8_t iRound;
uint8_t RKe[11][16];        //expanded key
uint8_t S[16];  //working array message to cipher, cipher to message fill in column order 4x4


#endif // AESDECIPHER_H
