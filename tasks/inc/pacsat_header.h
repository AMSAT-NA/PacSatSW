/*
 * pacsat_header.h
 *
 *  Created on: Mar 9, 2023
 *      Author: g0kla
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 *
 *
 */

#ifndef TASKS_INC_PACSAT_HEADER_H_
#define TASKS_INC_PACSAT_HEADER_H_


#include <stdint.h>

// Mandatory Header
#define  FILE_ID 0x01
#define  FILE_NAME 0x02
#define  FILE_EXT 0x03
#define  FILE_SIZE 0x04
#define  CREATE_TIME 0x05
#define  LAST_MOD_TIME 0x06
#define  SEU_FLAG 0x07
#define  FILE_TYPE 0x08
#define  BODY_CHECKSUM 0x09
#define  HEADER_CHECKSUM 0x0a
#define  BODY_OFFSET 0x0b

// Extended Header
#define  SOURCE 0x10
#define  AX25_UPLOADER 0x11
#define  UPLOAD_TIME 0x12 /* Note that this is a Mandatory item on all files */
#define  DOWNLOAD_COUNT 0x13
#define  DESTINATION 0x14
#define  AX25_DOWNLOADER 0x15
#define  DOWNLOAD_TIME 0x16
#define  EXPIRE_TIME 0x17
#define  PRIORITY 0x18

// Optional Header
#define  COMPRESSION_TYPE 0x19
#define  BBS_MSG_TYPE 0x20
#define  BULLETIN_ID_NUMBER 0x21
#define  TITLE 0x22
#define  KEYWORDS 0x23
#define  FILE_DESCRIPTION 0x24
#define  COMPRESSION_DESCRIPTION 0x25
#define  USER_FILE_NAME 0x26

// Compression types
#define  BODY_NOT_COMPRESSED 0x00
#define  BODY_COMPRESSED_PKARC 0x01
#define  BODY_COMPRESSED_PKZIP 0x02
#define  BODY_COMPRESSED_GZIP 0x03

#define PFH_TYPE_ASCII 0
#define PFH_TYPE_WOD 3
#define PFH_TYPE_IMAGES 211

#define FILE_SIZE_BYTE_POS 26
#define BODY_OFFSET_BYTE_POS 65
#define HEADER_CHECKSUM_BYTE_POS 60

#define PSF_FILE_EXT "act"

typedef struct {
  /* required Header Information */
  uint32_t fileId;              /* 0x01 */
  char          fileName[9];         /* 0x02 */
  char          fileExt[4];          /* 0x03 */
  uint32_t fileSize;            /* 0x04 */
  uint32_t createTime;          /* 0x05 */
  uint32_t modifiedTime;        /* 0x06 */
  unsigned char SEUflag;             /* 0x07 */
  unsigned char fileType;            /* 0x08 */
  uint16_t  bodyCRC;             /* 0x09 */
  uint16_t  headerCRC;           /* 0x0A */
  uint16_t  bodyOffset;          /* 0x0B */

  /* Extended Header Information */
  char          source[33];          /* 0x10 */
  char          uploader[7];         /* 0x11 */
  uint32_t uploadTime;          /* 0x12 */   /* Note that this is a Mandatory item on all files */
  uint8_t downloadCount;       /* 0x13 */
  char          destination[33];     /* 0x14 */
  char          downloader[7];       /* 0x15 */
  uint32_t downloadTime;        /* 0x16 */
  uint32_t expireTime;          /* 0x17 */
  uint8_t priority;            /* 0x18 */

  /* Optional Header Information */
  uint8_t compression;         /* 0x19 */
  char          BBSMessageType;      /* 0x20 */
  char          BID[33];             /* 0x21 */
  char          title[65];           /* 0x22 */
  char          keyWords[33];        /* 0x23 */
  char          file_description[33];     /* 0x24 */
  char          compressionDesc[33]; /* 0x25 */
  char          userFileName[33];    /* 0x26 */

  char          other1[33];    /* 0x42 */
  char          other2[33];    /* 0x43 */
  char          other3[33];    /* 0x44 */

}
HEADER;

int pfh_extract_header(HEADER  *hdr, uint8_t *buffer, uint16_t nBytes, uint16_t *size, bool *crc_passed);
int pfh_generate_header_bytes(HEADER *pfh, int body_size, uint8_t *header_bytes);
void pfh_debug_print(HEADER *pfh);
int test_pfh();
int test_pfh_file();
int test_pfh_make_files();

#endif /* TASKS_INC_PACSAT_HEADER_H_ */
