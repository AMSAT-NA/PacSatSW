#ifndef SHADIGEST_H
#define SHADIGEST_H
#include "stdint.h"

    void Pad(uint8_t *Str, uint16_t len);
    void SHA(uint8_t *Str,uint16_t len, uint32_t *Sout);
    uint32_t ROTR(uint32_t x, uint32_t n);
    uint32_t ROTL(uint32_t x, uint32_t n);
    uint32_t Ch(uint32_t x, uint32_t y, uint32_t z);
    uint32_t Maj(uint32_t x, uint32_t y, uint32_t z);
    uint32_t Sg0(uint32_t x);
    uint32_t Sg1(uint32_t x);
    uint32_t sg0(uint32_t x);
    uint32_t sg1(uint32_t x);
    uint32_t H[8];
    uint32_t a,b,c,d,e,f,g,h;
    uint32_t W[64];
    uint32_t T1,T2;
    uint8_t Mc[64];
    uint32_t M[16];

#endif // SHADIGEST_H
