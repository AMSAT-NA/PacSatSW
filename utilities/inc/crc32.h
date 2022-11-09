/*
 * crc32.h
 *
 *  Created on: Jun 18, 2019
 *      Author: bfisher
 */

#ifndef UTILITIES_INC_CRC32_H_
#define UTILITIES_INC_CRC32_H_

uint32_t crc32(const void *buf, size_t size);


inline uint32_t crc32Single(uint32_t crc, uint8_t newByte){
	extern const uint32_t crc32_tab[];
	return (crc32_tab[(crc ^ newByte) & 0xFF] ^ (crc >> 8));
}



#endif /* UTILITIES_INC_CRC32_H_ */
