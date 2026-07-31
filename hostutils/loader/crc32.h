
#ifndef CRC_H
#define CRC_H

uint32_t crc32(const void *buf, unsigned int size);

uint32_t crc32_byte(uint32_t crc, uint8_t byte);

#endif /* CRC_H */
