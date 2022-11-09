#include "shadigest.h"

const uint32_t Ki[] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2
};

const uint32_t Hi[] = {
    0x6a09e667,
    0xbb67ae85,
    0x3c6ef372,
    0xa54ff53a,
    0x510e527f,
    0x9b05688c,
    0x1f83d9ab,
    0x5be0cd19
};


uint32_t ROTR(uint32_t x, uint32_t n)
{
    return((x >> n) | (x << (32-n)));
}

uint32_t ROTL(uint32_t x, uint32_t n)
{
    return((x << n) | (x >> (32-n)));
}

uint32_t Ch(uint32_t x, uint32_t y, uint32_t z)
{
    return (x & y) ^ (~x & z);
}

uint32_t Maj(uint32_t x, uint32_t y, uint32_t z)
{
    return (x & y) ^ (x & z) ^ (y & z);
}

uint32_t Sg0(uint32_t x)
{
    return (ROTR(x,2) ^ ROTR(x,13) ^ ROTR(x,22));
}

uint32_t Sg1(uint32_t x)
{
    return (ROTR(x,6) ^ ROTR(x,11) ^ ROTR(x,25));
}

uint32_t sg0(uint32_t x)
{
    return (ROTR(x,7) ^ ROTR(x,18) ^ (x >> 3));
}

uint32_t sg1(uint32_t x)
{
    return (ROTR(x,17) ^ ROTR(x,19) ^ (x >> 10));
}

void Pad(uint8_t *Str, uint16_t len)
{
    uint32_t i,j,l,n,z,p;

        l = len;
        if(l > 55) l = 55;                  //single block length of data is 55 bytes max
        z = 55-l;
        for(i=0,p=0;i<l;i++)
            Mc[p++]=Str[i];       
        Mc[p++] = 0x80;                     //add 1 at end and rest of a byte of zeros (min zeros is seven)
        for(i=0;i<z;i++)                    //add extra zeros to get to 56 bytes
            Mc[p++]=0;
        for(i=0;i<6;i++)                    //add length to end - 8 bytes allocated, only need 2 for a single block
            Mc[p++]=0;
        n=l*8;                              //length added is in bits
        Mc[p++]= (n>>8) & 0xff;
        Mc[p] = n & 0xff;

        for(i=0;i<16;i++)                   //change to 32 bit words big endian - these are in M[]
        {
            j=i*4;
            M[i] = (Mc[j]<<24) + (Mc[j+1]<<16) + (Mc[j+2]<<8) + (Mc[j+3]);
        }

        for(i=0;i<8;i++)                    //preload H with initial values from constants
            H[i] = Hi[i];

        for(i=0;i<16;i++)                   //load W vector up with values - first 16 are M
            W[i] = M[i];

        for(i=16;i<64;i++)
            W[i] = (sg1(W[i-2])+W[i-7]+sg0(W[i-15])+W[i-16])& 0xffffffff;

        a=H[0]; b=H[1]; c=H[2]; d=H[3]; e=H[4]; f=H[5]; g=H[6]; h=H[7];         //initialize working variables

}



void SHA(uint8_t *Str, uint16_t len, uint32_t *Sout)
{
    uint32_t i;
    Pad(Str,len);                                   //setup working spaces
    for(i=0;i<64;i++)
    {
        T1 = (h+Sg1(e)+Ch(e,f,g)+Ki[i]+W[i]) & 0xffffffff;
        T2 = (Sg0(a) + Maj(a,b,c)) & 0xffffffff;
        h=g;
        g=f;
        f=e;
        e = (d+T1) & 0xffffffff;
        d=c;
        c=b;
        b=a;
        a=(T1+T2) & 0xffffffff;
    }
    H[0] += a;
    H[1] += b;
    H[2] += c;
    H[3] += d;
    H[4] += e;
    H[5] += f;
    H[6] += g;
    H[7] += h;

    for(i=0;i<8;i++)
        Sout[i] = H[i];

}


