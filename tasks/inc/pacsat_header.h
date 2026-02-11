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

#define MAX_PFH_LENGTH 2048

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

// TODO - need a more flexible way to store additional fields
#define  WISP1 0x2a
#define  WISP2 0x2e
#define  WISP3 0x2f


// Compression types
#define  BODY_NOT_COMPRESSED 0x00
#define  BODY_COMPRESSED_PKARC 0x01
#define  BODY_COMPRESSED_PKZIP 0x02
#define  BODY_COMPRESSED_GZIP 0x03

#define UNCOMPRESSED_FILE_SIZE_LIMIT 200 /* Compress files over this size before header added */

#define PFH_TYPE_ASCII 0
//#define PFH_TYPE_WOD 3 // This was WOD on historical sats
#define PFH_TYPE_AL 223
#define PFH_TYPE_BL 202
#define PFH_TYPE_WL 203 // WOD Log
#define PFH_TYPE_IMAGES 211

// These offsets are to the start of the field, i.e. they point to the ID number not the data.
#define FILE_ID_BYTE_POS 2
#define UPLOAD_TIME_BYTE_POS_EX_SOURCE_LEN 82
#define FILE_SIZE_BYTE_POS 26
#define BODY_OFFSET_BYTE_POS 65
#define HEADER_CHECKSUM_BYTE_POS 60

//#define PSF_FILE_EXT "act"
//#define PSF_FILE_EXT ".act" // no need to store this extra info??
#define PSF_FILE_TMP ".tmp"

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
  char          source_length;       /* This is the actual length of the source field on disk, which may have been truncated when parsed */
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

  char          wisp1[33];    /* 0x2a */
  char          wisp2[33];    /* 0x2e */
  char          wisp3[33];    /* 0x2f */

}
HEADER;

int pfh_extract_header(HEADER  *hdr, uint8_t *buffer, uint16_t nBytes, uint16_t *size, bool *crc_passed);
int pfh_generate_header_bytes(HEADER *pfh, int body_size, uint8_t *header_bytes);
void pfh_debug_print(HEADER *pfh);
uint8_t * pfh_store_short(uint8_t *buffer, uint16_t n);
uint8_t * pfh_store_int(uint8_t *buffer, uint32_t n);
int pfh_make_internal_file(HEADER *pfh, char *dir_folder, char *body_filename, uint32_t file_size);
int pfh_make_internal_header(HEADER *pfh,uint32_t now, uint8_t file_type, unsigned int id, char *filename,
        char *source, char *destination, char *title, char *user_filename, uint32_t update_time,
        uint32_t expire_time, char compression_type);

int test_pfh();
int test_pfh_file();
int test_pfh_make_files();
int test_pfh_make_internal_file(char * filename);

#endif /* TASKS_INC_PACSAT_HEADER_H_ */
