/*
 * TelemEncoding.h
 *
 *  Created on: Feb 3, 2014
 *      Author: fox
 */

#ifndef TELEMENCODING_H_
#define TELEMENCODING_H_

void update_rs(
   unsigned char parity[32], // 32-byte encoder state; zero before each frame
   unsigned char c          // Current data byte to update
);

int encode_8b10b(
    int32_t *state, // pointer to encoder state (run disparity, RD)
    int32_t data);

#define CHARACTER_BITS 10
#define CHARACTERS_PER_LONGWORD 3
#define CHARACTER_MASK ((1<<CHARACTER_BITS)-1)
#define SYNC_CHARACTER -1
#define SYNC10B 0x0fa

//extern int maxNewBuffer,maxOldBuffer; //For debugging
#if 1
static void inline Put10bInBuffer(
		uint32_t *bufferBase,
		int index,
		uint16_t data)
{
	/*
	 * We are going to put 3 10-bit data items (we'll call them characters) into each
	 * 32-bit word in the buffer with
	 * two bits left over at the most significant end.
	 */
	int bufferIndex = index/CHARACTERS_PER_LONGWORD; /* Which 32-bit word?*/
	int position = (index%CHARACTERS_PER_LONGWORD)*CHARACTER_BITS;    /* Which position within the 32 bits? */
	if(position == 0)bufferBase[bufferIndex] = 0; /* Initialize the 3-character buffer */
	bufferBase[bufferIndex] |= data << position;
	//if(bufferIndex > maxNewBuffer)maxNewBuffer = bufferIndex;
	//if(index > maxOldBuffer)maxOldBuffer = index;
}

static uint16_t inline Get10bFromBuffer(
		uint32_t *bufferBase,
		int index)
{
	int bufferIndex = index/CHARACTERS_PER_LONGWORD; /* Which 32-bit word?*/
	int position = (index%CHARACTERS_PER_LONGWORD)*CHARACTER_BITS;    /* Which position within the 32 bits? */
	uint16_t retChar;
	retChar = (bufferBase[bufferIndex]>>position) & CHARACTER_MASK;
	return retChar;

}
#else
static void inline Put10bInBuffer(
		uint32_t *bufferBase,
		int index,
		uint16_t data)
{
	uint16_t *buffer = (uint16_t *)bufferBase;
	buffer[index] = data;
	if(index > maxOldBuffer)maxOldBuffer = index;
}

static uint16_t inline Get10bFromBuffer(
		uint32_t *bufferBase,
		int index)
{
	uint16_t *buffer = (uint16_t *)bufferBase;
	return buffer[index];
}
#endif


#define PARITY_BYTES_PER_CODEWORD 32U     // Number of parity symbols in frame
#define NP 32U //For Phil's code
#define DATA_BYTES_PER_CODE_WORD 223

#endif /* TELEMENCODING_H_ */
