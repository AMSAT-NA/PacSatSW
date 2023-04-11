#include <pacsat.h>
#include "aesdecipher.h"
#include "keyfile.h"
#include "sboxes.h"
#include "nonvol.h"
#include "MRAMmap.h"
//********************************************************************************
//AES De-cipher
//********************************************************************************
uint8_t Rc[10] = {0x01,0x02,0x04,0x08,0x010,0x20,0x40,0x80,0x1b,0x36};      //for key expansion algorithm
static uint8_t CKey[16];
bool InitEncryption(void){
	uint8_t DefaultKey[16] = DEFAULT_KEY;
	static const MRAMmap_t *LocalFlash = 0;
	int i;
	bool stat;
	uint32_t magic,checksum;
	stat = readNV(&magic,sizeof(LocalFlash->AuthenticateKey.magic),NVStatisticsArea,(int)&LocalFlash->AuthenticateKey.magic);
	if(stat){
		stat = readNV(&checksum,sizeof(LocalFlash->AuthenticateKey.keyChecksum),NVStatisticsArea,
		              (int)&LocalFlash->AuthenticateKey.keyChecksum);
	}
	if(stat && (magic == ENCRYPTION_KEY_MAGIC_VALUE)){
		uint32_t calcChecksum=0;
		printf("Reading encryption key from NVram\n");
		stat = readNV(CKey,sizeof(LocalFlash->AuthenticateKey.key),NVStatisticsArea,(int)&LocalFlash->AuthenticateKey.key);
		for(i=0;i<sizeof(CKey);i++){
			calcChecksum += CKey[i];
		}
		if(checksum != calcChecksum){
			printf("Key Checksum failure; ");
			stat = false;
		}
	} else {
		printf("No encryption key in memory; ");
		stat = false;
	}
	if(!stat){
		printf("Using default key\n");
		for(i=0;i<16;i++){
			CKey[i] = DefaultKey[i];
		}

	}
	return stat;
}
bool WriteKey(uint8_t newKey){
	return false;
}
void iSubBytes(void)
{
    uint8_t i;
    for(i=0;i<16;i++)
        S[i]=iSBox[S[i]];
}

void iShiftRows(void)
{
    uint8_t R[4],i,j;
    for(i=1;i<4;i++)
    {
        for(j=0;j<4;j++)
        {
            R[j]=S[(j<<2)+i];
        }
        for(j=0;j<4;j++)
        {
            S[(j<<2)+i]=R[(j-i)&0x03];
        }
    }
}

void iMixColumns(void)
{
    uint8_t St[4],T[4],i,j;
    for(i=0;i<4;i++)
    {
        for(j=0;j<4;j++)
        {
            St[j]=S[(i<<2)+j];
        }
        T[0]=(iM(St[0],0x0e)^iM(St[1],0x0b)^iM(St[2],0x0d)^iM(St[3],0x09))&0xff;
        T[1]=(iM(St[0],0x09)^iM(St[1],0x0e)^iM(St[2],0x0b)^iM(St[3],0x0d))&0xff;
        T[2]=(iM(St[0],0x0d)^iM(St[1],0x09)^iM(St[2],0x0e)^iM(St[3],0x0b))&0xff;
        T[3]=(iM(St[0],0x0b)^iM(St[1],0x0d)^iM(St[2],0x09)^iM(St[3],0x0e))&0xff;
        for(j=0;j<4;j++)
        {
            S[(i<<2)+j]=T[j];
        }
    }
}

uint8_t iM(uint8_t a,uint8_t b)
{
uint16_t v,v1,m;
int i;
        v=0;
        for(i=7;i>=0;i--)
        {
            m=1<<i;
            if(m&b)
            {
                v ^= (a<<i);
            }
        }
        m=0x8000;
        v1 = 0x8d80;
        while(m > 0x80)
        {
            if(m&v) v ^= v1;
            m >>= 1;
            v1 >>= 1;
        }
    return(v&0xff);
}


void iRKApply(void)
{
    uint8_t i;
    for(i=0;i<16;i++)
        S[i] = S[i]^RKe[iRound][i];
}
void DeCipher32(uint8_t *msgIn, uint8_t *msgOut){
	DeCipher16(&msgIn[0],&msgOut[0]);
	DeCipher16(&msgIn[16],&msgOut[16]);
}
void DeCipher16(uint8_t *msgin, uint8_t *msgout)
{
uint8_t i;
    for(i=0;i<16;i++){
        K[i]=CKey[i];
    }
    for(i=0;i<16;i++)
         S[i]=msgin[i];
    RKExpand();
    iRound=10;
    iRKApply();      //apply to get first round input
    for(iRound=9;iRound>0;iRound--)
    {
        iShiftRows();
        iSubBytes();
        iRKApply();
        iMixColumns();
    }
    iShiftRows();
    iSubBytes();
    iRKApply();                      //cipher now in S[]
    for(i=0;i<16;i++)
        msgout[i]=S[i];
}

//*******************************************************
//AES Common to Cipher and De-cipher
//*******************************************************
void RKExpand(void)
{
    uint8_t Round,i;
    for(i=0;i<16;i++)
        RKe[0][i]=K[i];       //fill first cell with key
    for(Round=1;Round<11;Round++)
    {
        RoundKey(Round);
    }
}

void RoundKey(uint8_t Rnd)
{
    uint8_t Temp[4],i,j,k,t;
    for(i=0;i<4;i++)
    {
        j=(i-1)&0x03;              //temp column
        for(k=0;k<4;k++)        //fill temp
            Temp[k]=RKe[Rnd-1][j*4+k];
        if(i==0)
        {
            t=Temp[0];             //rotate in T
            for(k=0;k<3;k++)
                Temp[k]=Temp[k+1];
            Temp[3]=t;
            for(k=0;k<4;k++)    //sub bytes
                Temp[k]=SBox[Temp[k]];
            Temp[0] = Temp[0]^Rc[Rnd-1];
         }
         for(k=0;k<4;k++)
         {
             RKe[Rnd][i*4+k]=RKe[Rnd-1][i*4+k]^Temp[k];
         }
    }
}

