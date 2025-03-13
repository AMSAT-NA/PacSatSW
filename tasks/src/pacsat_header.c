/*
 * pacsat_header.c
 *
 *  Created on: Mar 9, 2023
 *      Author: g0kla
 *
 * Based on header.c by John Melton (G0ORX/N6LYT)
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
 * The Pacsatfile header format is descibed in the Pacsat File Definition:
 * https://www.g0kla.com/pacsat/fhead.txt
 *
 * All values in the PFH are stored in little endian format.  Given our CPU
 * for the Pacsat is little endian, no byte manipulation is needed.  This
 * may not be the case on the ground where the file is received.
 *
 * The PFH consists of the following:
 *  Mandatory Header - these fields are always present
 *  Extended Header - these fields are present on all messages
 *  Optional Header - present if needed
 *
 *  The header must start with 0xaa55
 *
 *  Thus, there are 3 forms of PACSAT file header:
 *
 *    <0xaa><0x55><Mandatory hdr><Hdr end>
 *    <0xaa><0x55><Mandatory hdr><Extended hdr><Hdr end>
 *    <0xaa><0x55><Mandatory hdr><Extended hdr>[<Optional    Items> . . . ]<Hdr end>
 *
 */

#include <ctype.h>
#include "pacsat.h"
#include "pacsat_header.h"
#include "pacsat_dir.h"
#include "str_util.h"
#include "inet.h"
#include "MRAMmap.h"
#include "nonvol.h"
#include "nonvolManagement.h"
#include "MET.h"
#include "redposix.h"


/* Local forward headers */
void header_copy_to_str(uint8_t *header, int length, char *destination, int maxbytes);
uint8_t * add_mandatory_header(uint8_t *p, HEADER *pfh);
uint8_t * add_extended_header(uint8_t *p, HEADER *pfh);
uint8_t * add_optional_header(uint8_t *p, HEADER *pfh);
int pfh_save_pacsatfile(unsigned char * header, int header_len, char *filename, char *body_filename, int file_size);
uint8_t * pfh_store_char_field(uint8_t *buffer, uint16_t id, uint8_t val);
uint8_t * pfh_store_short_int_field(uint8_t *buffer, uint16_t id, uint16_t val);
uint8_t * pfh_store_int_field(uint8_t *buffer, uint16_t id, uint32_t val);
uint8_t * pfh_store_str_field(uint8_t *buffer, uint16_t id, uint8_t len, char* str);
bool make_test_header(HEADER *pfh, uint32_t fh, unsigned int file_id, char *filename, char *source, char *destination,
                      char *title, char *user_filename, char *msg1);
/**
 * pfh_new_header()
 *
 * Initialize a new Pacsat File Header structure.  The caller is responsible for
 * creating the space and must pass in a pointer.
 *
 */
// TODO - refactor name to INIT
void pfh_new_header(HEADER  *hdr) {

    if (hdr == NULL) {
        debug_print("ERROR: NULL Pointer passed to pfh_new_header\n");
    }
    /* Mandatory */
    hdr->fileId             = 0;
    hdr->fileName[0]        = '\0';
    hdr->fileExt[0]         = '\0';
    hdr->fileSize           = 0;
    hdr->createTime         = 0;
    hdr->modifiedTime       = 0;
    hdr->SEUflag            = 0;
    hdr->fileType           = 0;
    hdr->bodyCRC            = 0;
    hdr->headerCRC          = 0;
    hdr->bodyOffset         = 0;

    /* Extended */
    hdr->source[0]          = '\0';
    hdr->source_length      = 0;
    hdr->uploader[0]        = '\0';
    hdr->uploadTime         = 0;
    hdr->downloadCount      = 0;
    hdr->destination[0]     = '\0';
    hdr->downloader[0]      = '\0';
    hdr->downloadTime       = 0;
    hdr->expireTime         = 0;
    hdr->priority           = 0;

    /* Optional */
    hdr->compression        = 0;
    hdr->BBSMessageType     = ' ';
    hdr->BID[0]             = '\0';
    hdr->title[0]           = '\0';
    hdr->keyWords[0]        = '\0';
    hdr->file_description[0]     = '\0';
    hdr->compressionDesc[0] = '\0';
    hdr->userFileName[0]    = '\0';

    hdr->wisp1[0]    = '\0';
    hdr->wisp2[0]    = '\0';
    hdr->wisp3[0]    = '\0';
}


/**
 * pfh_extract_header()
 *
 * Extract the header from a byte buffer
 * The caller must pass in a pfh strcture
 *
 * TODO - if an empty or incorrect buffer is passed then we may read random memory.  The CRC will fail,
 * but we should have some other safety checks.  If we read an incorrect length for a field then we
 * could try to read invalid memory and crash.
 *
 * Returns: TRUE if we extracted a header, otherwise FALSE.
 */
int pfh_extract_header( HEADER  *hdr, uint8_t *buffer, uint16_t nBytes, uint16_t *size, bool *crc_passed) {
 //   uint8_t buffer [] = {0xAA, 0x55, 0x01, 0x00, 0x04, 0x47, 0x03, 0x00, 0x00, 0x02, 0x00};
    int i = 0;
    int bMore = 0;
    unsigned id;
    uint8_t length;
    uint16_t crc_result = 0;

    if (hdr == NULL ){
        return FALSE;
    }
    pfh_new_header(hdr);
    if (buffer[0] != 0xAA || buffer[1] != 0x55) {
        return FALSE;
    }

    bMore = 1;

    crc_result += buffer[0] & 0xff;
    crc_result += buffer[1] & 0xff;
    i = 2; /* skip over 0xAA 0x55 */

    while (bMore && i < nBytes) {
        crc_result += buffer[i] & 0xff;
        id = buffer[i++];
        crc_result += buffer[i] & 0xff;
        id += buffer[i++] << 8;
        crc_result += buffer[i] & 0xff;
        length = buffer[i++];

        if (id != HEADER_CHECKSUM) {
            int j;
            for (j=0; j<length; j++) {
                if (i+j >= nBytes) {
                    debug_print("ERROR extracting PFH.  Buffer overflow prevented for length %d of id %d\n",length,id);
                    return FALSE;
                }
                crc_result += buffer[i+j] & 0xff;
                //debug_print("%02x ",buffer[i+j]);
            }
        }

//        debug_print("ExtractHeader: id:%X length:%d \n", id, length);

        switch (id) {
        case 0x00:
            bMore = 0;
            break;
        case FILE_ID:
            if (length != 4) { debug_print ("PFH ERROR: File_id length %d\n", length); return FALSE; }
            hdr->fileId = *(uint32_t *)&buffer[i];
            hdr->fileId = ttohl(hdr->fileId);
            break;
        case FILE_NAME:
            if (length != 8) { debug_print ("PFH ERROR: File_name length %d\n", length); return FALSE; }
            header_copy_to_str(&buffer[i], length, hdr->fileName, 8);
            break;
        case FILE_EXT:
            if (length != 3) { debug_print ("PFH ERROR: File_ext length %d\n", length); return FALSE; }
            header_copy_to_str(&buffer[i], length, hdr->fileExt, 3);
            break;
        case FILE_SIZE:
            if (length != 4) { debug_print ("PFH ERROR: File_size length %d\n", length); return FALSE; }
            hdr->fileSize = ttohl(*(uint32_t *)&buffer[i]);
            break;
        case CREATE_TIME:
            if (length != 4) { debug_print ("PFH ERROR: create time length %d\n", length); return FALSE; }
            hdr->createTime = ttohl(*(uint32_t *)&buffer[i]);
            break;
        case LAST_MOD_TIME:
            if (length != 4) { debug_print ("PFH ERROR: last mod time length %d\n", length); return FALSE; }
            hdr->modifiedTime = ttohl(*(uint32_t *)&buffer[i]);
            break;
        case SEU_FLAG:
            if (length != 1) { debug_print ("PFH ERROR: SEU length %d\n", length); return FALSE; }
            hdr->SEUflag = buffer[i];
            break;
        case FILE_TYPE:
            if (length != 1) { debug_print ("PFH ERROR: File_type length %d\n", length); return FALSE; }
            hdr->fileType = buffer[i];
            break;
        case BODY_CHECKSUM:
            if (length != 2) { debug_print ("PFH ERROR: body cs length %d\n", length); return FALSE; }
            hdr->bodyCRC = ttohs(*(uint16_t *)&buffer[i]);
            break;
        case HEADER_CHECKSUM:
            if (length != 2) { debug_print ("PFH ERROR: header cs length %d\n", length); return FALSE; }
            hdr->headerCRC = ttohs(*(uint16_t *)&buffer[i]);
            break;
        case BODY_OFFSET:
            if (length != 2) { debug_print ("PFH ERROR: body off length %d\n", length); return FALSE; }
            hdr->bodyOffset = ttohs(*(uint16_t *)&buffer[i]);
            break;
        case SOURCE:
            header_copy_to_str(&buffer[i], length, hdr->source, 32);
            hdr->source_length = length; /* Store the actual source length in case it was truncated. */
            break;
        case AX25_UPLOADER:
            if (length != 6) { debug_print ("PFH ERROR: ax25 uploader length %d\n", length); return FALSE; }
            header_copy_to_str(&buffer[i], length, hdr->uploader, 6);
            break;
        case UPLOAD_TIME:
            if (length != 4) { debug_print ("PFH ERROR: upload time length %d\n", length); return FALSE; }
            hdr->uploadTime = ttohl(*(uint32_t *)&buffer[i]);
            //ConvertTime(&hdr->uploadTime);
            break;
        case DOWNLOAD_COUNT:
            if (length != 1) { debug_print ("PFH ERROR: download count length %d\n", length); return FALSE; }
            hdr->downloadCount = buffer[i];
            break;
        case DESTINATION:
            header_copy_to_str(&buffer[i], length, hdr->destination, 32);
            break;
        case AX25_DOWNLOADER:
            if (length != 6) { debug_print ("PFH ERROR: ax25 downloader length %d\n", length); return FALSE; }
            header_copy_to_str(&buffer[i], length, hdr->downloader, 6);
            break;
        case DOWNLOAD_TIME:
            if (length != 4) { debug_print ("PFH ERROR: download time length %d\n", length); return FALSE; }
            hdr->downloadTime = ttohl(*(uint32_t *)&buffer[i]);
            break;
        case EXPIRE_TIME:
            if (length != 4) { debug_print ("PFH ERROR: expire time length %d\n", length); return FALSE; }
            hdr->expireTime = ttohl(*(uint32_t *)&buffer[i]);
            break;
        case PRIORITY:
            if (length != 1) { debug_print ("PFH ERROR: priority length %d\n", length); return FALSE; }
            hdr->priority = buffer[i];
            break;
        case COMPRESSION_TYPE:
            if (length != 1) { debug_print ("PFH ERROR: compression type length %d\n", length); return FALSE; }
            hdr->compression = buffer[i];
            break;
        case BBS_MSG_TYPE:
            if (length != 1) { debug_print ("PFH ERROR: bbs msg type length %d\n", length); return FALSE; }
            hdr->BBSMessageType = buffer[i];
            break;
        case BULLETIN_ID_NUMBER:
            header_copy_to_str(&buffer[i], length, hdr->BID, 32);
            break;
        case TITLE:
            header_copy_to_str(&buffer[i], length, hdr->title, 64);
            break;
        case KEYWORDS:
            header_copy_to_str(&buffer[i], length, hdr->keyWords, 32);
            break;
        case FILE_DESCRIPTION:
            header_copy_to_str(&buffer[i], length, hdr->file_description, 32);
            break;
        case COMPRESSION_DESCRIPTION:
            header_copy_to_str(&buffer[i], length, hdr->compressionDesc, 32);
            break;
        case USER_FILE_NAME:
            header_copy_to_str(&buffer[i], length, hdr->userFileName, 32);
            break;
        case WISP1:
            header_copy_to_str(&buffer[i], length, hdr->wisp1, 32);
            break;
        case WISP2:
            header_copy_to_str(&buffer[i], length, hdr->wisp2, 32);
            break;
        case WISP3:
            header_copy_to_str(&buffer[i], length, hdr->wisp3, 32);
            break;

        default:
            debug_print("** Unknown header id %04x ** ", id);

            int n;
            for (n=0; n<length; n++) {
                if (isprint(buffer[i+n]))
                    debug_print("%c",buffer[i+n]);
            }
            debug_print(" |");
            for (n=0; n<length; n++)
                debug_print(" %02X",buffer[i+n]);
            debug_print("\n");

            break;
        }

        i+=length;
    }


    /* let the user know the size */
    *size = i;

    /* see if we ran out of space */
    if (bMore) {
        return FALSE;
    }

  //debug_print("CRC: %02x\n", crc_result);
    if (crc_result == hdr->headerCRC )
        *crc_passed = true;
    else
        *crc_passed = false;

    return TRUE;
}

/**
 * header_copy_to_str()
 *
 * Copy length bytes, but at most maxbytes, from a header into a string.  Terminate the string.
 *
 */
void header_copy_to_str(uint8_t *header, int length, char *destination, int maxbytes) {
    if (length > maxbytes) length = maxbytes;

    while (length > 0) {
        *destination++ = *header++;
        length--;
    }

    *destination = '\0';
}

/**
 * pfh_debug_print()
 *
 * Print key items from the pacsat header for debugging.  Prints nothing
 * if DEBUG is set to 0
 *
 */
void pfh_debug_print(HEADER *pfh) {
    if (pfh == NULL) return;
    debug_print("PFH: File:%4x - %s.%s ", (int)pfh->fileId, pfh->fileName, pfh->fileExt);

    debug_print("Source:%s ", pfh->source);
    debug_print("Dest:%s ", pfh->destination);
    debug_print("Crc:%4x ", pfh->headerCRC);
    debug_print("Size:%d\n", pfh->fileSize);
    debug_print("Title:'%s' ", pfh->title);
//    char buf[30];
//    time_t now = pfh->createTime;
//    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", gmtime(&now));
//    debug_print("Cr:%s ", buf);
    debug_print("Upload Time: %d ",pfh->uploadTime);
//    now = pfh->uploadTime;
//    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", gmtime(&now));
//    debug_print("Up:%s ", buf);
    debug_print(" Contains:%s\n", pfh->userFileName);
}

/**
 * pfh_generate_header_bytes()
 *
 * Generate the header bytes from the structure.  header_bytes should
 * be passed in and be sufficient length for the generated bytes. The
 * number of bytes generated is returned.  This should be equal to
 * the body_offset
 *
 * The checksums will be calculated.
 *
 * TODO - wer should pass in the maximum length of header_bytes and confirm before writing that we do not overflow at any point.
 *
 */
int pfh_generate_header_bytes(HEADER *pfh, int body_size, uint8_t *header_bytes) {
    /* Clean up data values.  These are usually callsigns.
     * The spec says source and destination can be mixed case, but typically they
     * are in upper case */
    int a;
    for (a=0; pfh->source[a]!=0; a++)
        pfh->source[a]=toupper(pfh->source[a]);
    for (a=0; pfh->destination[a]!=0; a++)
        pfh->destination[a]=toupper(pfh->destination[a]);
    for (a=0; pfh->uploader[a]!=0; a++)
        pfh->uploader[a]=toupper(pfh->uploader[a]);
    for (a=0; pfh->downloader[a]!=0; a++)
        pfh->downloader[a]=toupper(pfh->downloader[a]);

    uint8_t *p = &header_bytes[0];

    /* All PFHs start with 0xaa55 */
    *p++ = 0xaa;
    *p++ = 0x55;

    pfh->headerCRC = 0; /* Zero this out so it is recalculated correctly */
    p = add_mandatory_header(p, pfh);
    p = add_extended_header(p, pfh);
    p = add_optional_header(p, pfh);

    /* End the PFH */
    *p++  = 0x00;
    *p++ = 0x00;
    *p++ = 0x00;

    /* update some items in the mandatory header.  They are at a fixed offset because
     * all the items before them are mandatory
     */
    pfh->bodyOffset = p - &header_bytes[0];
    pfh->fileSize = pfh->bodyOffset + body_size;
    //debug_print("Body Offset: %02x\n",pfh->bodyOffset);
    //debug_print("File Size: %04x\n",pfh->fileSize);
    pfh_store_int_field(&header_bytes[FILE_SIZE_BYTE_POS] , FILE_SIZE, pfh->fileSize);
    pfh_store_short_int_field(&header_bytes[BODY_OFFSET_BYTE_POS] , BODY_OFFSET, pfh->bodyOffset);

    /* Now that all fields are populated we need to calculate the header checksum */
    short int header_checksum = 0;
    int i;
    for (i=0; i< pfh->bodyOffset; i++) {
        header_checksum += header_bytes[i] & 0xff;
    }
    pfh->headerCRC = header_checksum;
    pfh_store_short_int_field(&header_bytes[HEADER_CHECKSUM_BYTE_POS] , HEADER_CHECKSUM, header_checksum);
    return pfh->bodyOffset;
}

uint8_t * add_mandatory_header(uint8_t *p, HEADER *pfh) {
    /* Mandatory header */
    p = pfh_store_int_field(p, FILE_ID, pfh->fileId);
    p = pfh_store_str_field(p, FILE_NAME, 8, pfh->fileName);
    p = pfh_store_str_field(p, FILE_EXT, 3, pfh->fileExt);
    p = pfh_store_int_field(p, FILE_SIZE, pfh->fileSize); // this is the size of the header and file, so populated at end
    p = pfh_store_int_field(p, CREATE_TIME, pfh->createTime);
    p = pfh_store_int_field(p, LAST_MOD_TIME, pfh->modifiedTime);
    p = pfh_store_char_field(p, SEU_FLAG, pfh->SEUflag);
    p = pfh_store_char_field(p, FILE_TYPE, pfh->fileType);
    p = pfh_store_short_int_field(p, BODY_CHECKSUM, pfh->bodyCRC);
    p = pfh_store_short_int_field(p, HEADER_CHECKSUM, pfh->headerCRC);  //this is blank until header fully populated
    p = pfh_store_short_int_field(p, BODY_OFFSET, pfh->bodyOffset); // size of header so populated at end
    return p;
}

uint8_t * add_extended_header(uint8_t *p, HEADER *pfh) {
    /* Extended Header - all message files have this
     * If a Extended Header is present, it must immediately follow the final item in
     * the Mandatory Header.
     *
     * If any Extended Header item is present, all must be present.
     *
     * Extended Header items must be present in order of ascending value of <id>,
     * with the exception that multiple destinations are represented by multiple
     * occurrences of items 0x14, 0x15, and 0x16.
     */
    p = pfh_store_str_field(p, SOURCE, strlen(pfh->source), pfh->source);
    p = pfh_store_str_field(p, AX25_UPLOADER, 6, pfh->uploader);
    p = pfh_store_int_field(p, UPLOAD_TIME, pfh->uploadTime);
    p = pfh_store_char_field(p, DOWNLOAD_COUNT, pfh->downloadCount);
    p = pfh_store_str_field(p, DESTINATION, strlen(pfh->destination), pfh->destination);
    p = pfh_store_str_field(p, AX25_DOWNLOADER, 6, pfh->downloader);
    p = pfh_store_int_field(p, DOWNLOAD_TIME, pfh->downloadTime);
    p = pfh_store_int_field(p, EXPIRE_TIME, pfh->expireTime);
    p = pfh_store_char_field(p, PRIORITY, pfh->priority);
    return p;
}

uint8_t * add_optional_header(uint8_t *p, HEADER *pfh) {
    /* Optional Header items
     * The Mandatory Header and Extended Header may be followed by any number of
     * Optional Header items.  It is intended that any expansion of the PFH defini-
     * tion will involve only addition of Optional Items
     *
     * Optional Header items need not be presented in increasing order of <id>.
     */
    if (pfh->BBSMessageType != 0)
        p = pfh_store_char_field(p, BBS_MSG_TYPE, pfh->BBSMessageType);
    if (pfh->BID[0] != 0)
        p = pfh_store_str_field(p, BULLETIN_ID_NUMBER, strlen(pfh->BID), pfh->BID);
    if (pfh->compression != BODY_NOT_COMPRESSED)
        p = pfh_store_char_field(p, COMPRESSION_TYPE, pfh->compression);
    if (pfh->title[0] != 0)
        p = pfh_store_str_field(p, TITLE, strlen(pfh->title), pfh->title);
    if (pfh->keyWords[0] != 0)
        p = pfh_store_str_field(p, KEYWORDS, strlen(pfh->keyWords), pfh->keyWords);
    if (pfh->file_description[0] != 0)
        p = pfh_store_str_field(p, FILE_DESCRIPTION, strlen(pfh->file_description), pfh->file_description);
    if (pfh->compressionDesc[0] != 0)
        p = pfh_store_str_field(p, COMPRESSION_DESCRIPTION, strlen(pfh->compressionDesc), pfh->compressionDesc);
    if (pfh->userFileName[0] != 0)
            p = pfh_store_str_field(p, USER_FILE_NAME, strlen(pfh->userFileName), pfh->userFileName);
    if (pfh->wisp1[0] != 0)
        p = pfh_store_str_field(p, WISP1, strlen(pfh->wisp1), pfh->wisp1);
    if (pfh->wisp2[0] != 0)
        p = pfh_store_str_field(p, WISP2, strlen(pfh->wisp2), pfh->wisp2);
    if (pfh->wisp3[0] != 0)
        p = pfh_store_str_field(p, WISP3, strlen(pfh->wisp3), pfh->wisp3);
    return p;
}

/**
 * pfh_save_pacsatfile()
 *
 * Create a Pacsat file based on the PFH byte stream specified with header and the file specified
 * by body_filename.  The resulting pacsat file is saved in filename
 *
 * Returns: EXIT_SUCCESS or EXIT_FAILURE
 *
 */
int pfh_save_pacsatfile(unsigned char * header, int header_len, char *filename, char *body_filename, int file_size) {

    /* Save the header bytes */
    int32_t rc = dir_fs_write_file_chunk(filename, header, header_len, 0);
    if (rc == -1) {
        debug_print("pfh_save_pacsatfile:File I/O error writing chunk: %s\n", filename);
        return EXIT_FAILURE; // This is most likely caused by running out of file ids or space
    }

    /* Add the file contents */
    int32_t fp = red_open(body_filename, RED_O_RDONLY);
    if (fp == -1) {
        debug_print("pfh_save_pacsatfile: Unable to open %s for reading: %s\n", body_filename, red_strerror(red_errno));
        return EXIT_FAILURE;
    }
    uint8_t read_buffer[255];

    int bytes_written = 0;
    int loop_limit = 99999;
    while (bytes_written < file_size && loop_limit-- > 0) {
        int32_t numOfBytesRead = red_read(fp, read_buffer, sizeof(read_buffer));
        if (numOfBytesRead == -1) {
            debug_print("pfh_save_pacsatfile: Unable to read %s: %s\n", body_filename, red_strerror(red_errno));
            // remove partial file
            fp = red_unlink(filename); // try to unlink and ignore error if we can not
            return EXIT_FAILURE;
        }

        rc = dir_fs_write_file_chunk(filename, read_buffer, numOfBytesRead, header_len + bytes_written);
        if (rc == -1) {
            debug_print("pfh_save_pacsatfile:File I/O error writing chunk: %s\n", filename);
            // remove partial file
            fp = red_unlink(filename); // try to unlink and ignore error if we can not
            return EXIT_FAILURE; // This is most likely caused by running out of file ids or space
        }
        bytes_written = bytes_written + numOfBytesRead;
    }


    rc = red_close(fp);
    if (rc != 0) {
        printf("Unable to close %s: %s\n", body_filename, red_strerror(red_errno));
    }

    if (loop_limit == 0) {
        debug_print("Unable to complete file read: stuck in loop\n");
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}


/**
 * Store a little endian 16 bit int as two big endian bytes
 */
uint8_t * pfh_store_short(uint8_t *buffer, uint16_t n) {
    buffer[1] = (n >> 8) & 0xFF;
    buffer[0] = n & 0xFF;
    return &buffer[2];
}

/**
 * Store a little endian 4 byte int into the pacsat header
 * at the position of the passed pointer
 */
uint8_t * pfh_store_int(uint8_t *buffer, uint32_t n) {
    buffer[3] = (n >> 24) & 0xFF;
    buffer[2] = (n >> 16) & 0xFF;
    buffer[1] = (n >> 8) & 0xFF;
    buffer[0] = n & 0xFF;
    return &buffer[4];
}

uint8_t * pfh_store_char_field(uint8_t *buffer, uint16_t id, uint8_t val) {
    buffer = pfh_store_short(buffer,id);
    *buffer++ = 0x01;
    *buffer++ = val;
    return buffer;
}

uint8_t * pfh_store_short_int_field(uint8_t *buffer, uint16_t id, uint16_t val) {
    buffer = pfh_store_short(buffer,id);
    *buffer++ = 0x02;
    buffer = pfh_store_short(buffer, val);
    return buffer;
}

uint8_t * pfh_store_int_field(uint8_t *buffer, uint16_t id, uint32_t val) {
    buffer = pfh_store_short(buffer,id);
    *buffer++ = 0x04;
    buffer = pfh_store_int(buffer, val);
    return buffer;
}

uint8_t * pfh_store_str_field(uint8_t *buffer, uint16_t id, uint8_t len, char* str) {
    buffer = pfh_store_short(buffer,id);
    *buffer++ = len;
    int i;
    for (i=0; i < len; i++) {
        buffer[i] = str[i];
    }
    return buffer + len;
}


/**
 * pfh_make_internal_header()
 * Make a header for an internal file such as a log file or wod
 * Return a pointer to the header.  The caller is responsible for freeing
 *
 * the memory later.
 *
 * The id needs to be obtained from the dir before the header is created. This is then an
 * unused but allocated id until the file is added or discarded.  Similar to the upload
 * queue.
 *
 * An expire_time of 0 will use the default expire time for all files.
 *
 * Returns EXIT SUCCESS or EXIT FAILURE if the header could not be created.
 *
 */
int pfh_make_internal_header(HEADER *pfh,uint32_t now, uint8_t file_type, unsigned int id, char *filename,
        char *source, char *destination, char *title, char *user_filename, uint32_t update_time,
        uint32_t expire_time, char compression_type) {

    if (pfh != NULL) {
        pfh_new_header(pfh);
        /* Required Header Information */
        pfh->fileId = id;
        strlcpy(pfh->fileName,filename, sizeof(pfh->fileName));
//        strlcpy(pfh->fileExt,PSF_FILE_EXT, sizeof(pfh->fileExt));

        pfh->createTime = (uint32_t)update_time;
        pfh->modifiedTime = now;
        pfh->fileType = file_type;

        /* Extended Header Information */
        strlcpy(pfh->source,source, sizeof(pfh->source));

        strlcpy(pfh->destination,destination, sizeof(pfh->destination));
        if (expire_time != 0)
            pfh->expireTime = now + expire_time;

        /* Optional Header Information */
        strlcpy(pfh->title,title, sizeof(pfh->title));
        strlcpy(pfh->userFileName,user_filename, sizeof(pfh->userFileName));
        pfh->compression = compression_type;
        return EXIT_SUCCESS;
    }
    return EXIT_FAILURE;
}

/**
 * test_pfh_make_pacsat_file()
 *
 * Create a new PACSAT File based on a Header and the body file.  Save the new file
 * into out_filename which will be file_id.act and saved into dir_folder
 *
 * Returns: EXIT SUCCESS or EXIT_FAILURE
 */
int pfh_make_internal_file(HEADER *pfh, char *dir_folder, char *body_filename, uint32_t file_size) {
    if (pfh == NULL) return EXIT_FAILURE;

    char out_filename[MAX_FILENAME_WITH_PATH_LEN];

    dir_get_file_path_from_file_id(pfh->fileId, dir_folder, out_filename, MAX_FILENAME_WITH_PATH_LEN);

    /* Measure body_size and calculate body_checksum */
    short int body_checksum = 0;
    unsigned int body_size = 0;

    int32_t fp = red_open(body_filename, RED_O_RDONLY);
    if (fp == -1) {
        debug_print("pfh_make_internal_file: Unable to open %s for reading: %s\n", body_filename, red_strerror(red_errno));
        return EXIT_FAILURE;
    }

    int32_t rc;
    int i=0;
    char read_buffer[255];

    while (body_size < file_size) {
        int32_t numOfBytesRead = red_read(fp, read_buffer, sizeof(read_buffer));
        if (numOfBytesRead == -1) {
            debug_print("pfh_make_internal_file: Unable to read %s: %s\n", body_filename, red_strerror(red_errno));
            break;
        }
        if (numOfBytesRead == 0)
            break;

        for (i=0; i<numOfBytesRead; i++) {
            body_checksum += read_buffer[i] & 0xff;
            body_size++;
        }
    }

    rc = red_close(fp);
    if (rc != 0) {
        printf("pfh_make_internal_file: Unable to close %s: %s\n", body_filename, red_strerror(red_errno));
    }

    pfh->bodyCRC = body_checksum;

    debug_print("Body size: %d Body CRC %02x\n",body_size, body_checksum);

    /* Build Pacsat File Header */
    uint8_t buffer[MAX_PFH_LENGTH];
    int len = pfh_generate_header_bytes(pfh, body_size, buffer);

    rc = pfh_save_pacsatfile(buffer, len, out_filename, body_filename, body_size);

    return rc;
}



#ifdef DEBUG

/**
 * TEST ROUTINES FOLLOW
 */

int test_pfh() {
    printf("##### TEST PACSAT HEADER:\n");
    int rc = TRUE;

    uint8_t tst_buf[8];
    uint16_t v = 0x1234;
    pfh_store_short(tst_buf, v);
    int k;
    debug_print("Tst Buf: ");
    for (k=0; k<8; k++) {
        debug_print("%x ",tst_buf[k]);
    }
    debug_print("\n");
    if (tst_buf[0] != 0x34) {  debug_print("Store short byte 0 FAILED\n"); return FALSE; }
    if (tst_buf[1] != 0x12) {  debug_print("Store short byte 1 FAILED\n"); return FALSE; }

    uint32_t tst = 0xabcdef12;
//    uint32_t r = ttohl(tst);
    uint32_t r = ttohl(tst);
    debug_print("Swap result: %x\n",r);
    if (r != 0x12efcdab) {  debug_print("Swap32 FAILED\n"); return FALSE; }
    uint16_t tst2 = 0xabcd;
//    uint16_t r2 = ttohs(tst2);
    uint16_t r2 = ttohs(tst2);
    debug_print("Swap16 result: %x\n",r2);
    if (r2 != 0xcdab) {  debug_print("Swap 16 FAILED\n"); return FALSE; }

    debug_print("Test PFH with checksum: Expected CRC: 282b\n");
    uint8_t big_header [] = {0xAA, 0x55, 0x01, 0x00, 0x04, 0x47, 0x03, 0x00, 0x00, 0x02, 0x00
    , 0x08, 0x35, 0x61, 0x62, 0x39, 0x38, 0x34, 0x62,
            0x30, 0x03, 0x00, 0x03, 0x20, 0x20, 0x20, 0x04, 0x00, 0x04, 0xDE, 0x3D, 0x01, 0x00, 0x05, 0x00, 0x04, 0x47, 0x7D, 0xB9, 0x5A,
            0x06, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x01, 0x00, 0x08, 0x00, 0x01, 0x10, 0x09, 0x00, 0x02, 0x3E, 0x54,
            0x0A, 0x00, 0x02, 0x2B, 0x28,
            0x0B, 0x00, 0x02, 0xD8, 0x00, 0x10, 0x00, 0x05, 0x53, 0x54, 0x32, 0x4E, 0x48, 0x11, 0x00, 0x06, 0x53,
            0x54, 0x32, 0x4E, 0x48, 0x20, 0x12, 0x00, 0x04, 0x31, 0x85, 0xB9, 0x5A, 0x13, 0x00, 0x01, 0x00, 0x14, 0x00, 0x03, 0x41, 0x4C,
            0x4C, 0x15, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x48, 0x00, 0x16, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x17, 0x00, 0x04, 0xC7,
            0x71, 0xBD, 0x5A, 0x18, 0x00, 0x01, 0x00, 0x19, 0x00, 0x01, 0x00, 0x22, 0x00, 0x10, 0x4D, 0x59, 0x20, 0x53, 0x48, 0x41, 0x43,
            0x4B, 0x20, 0x41, 0x4E, 0x44, 0x20, 0x41, 0x4E, 0x54, 0x23, 0x00, 0x04, 0x3C, 0x57, 0x3E, 0x20, 0x26, 0x00, 0x11, 0x73, 0x74,
            0x32, 0x6E, 0x68, 0x20, 0x70, 0x69, 0x63, 0x20, 0x61, 0x6E, 0x74, 0x2E, 0x6A, 0x70, 0x67, 0x2A, 0x00, 0x07, 0x41, 0x57, 0x55,
            0x32, 0x2E, 0x31, 0x30, 0x2E, 0x00, 0x08, 0xAE, 0x47, 0xE1, 0x7A, 0x14, 0x2E, 0x2F, 0x40, 0x2F, 0x00, 0x08, 0xCD, 0xCC, 0xCC,
            0xCC, 0xCC, 0x4C, 0x40, 0xC0, 0x00, 0x00, 0x00};

    uint16_t size = 0;
    bool crc_passed = false;
    HEADER pfh;
    pfh_extract_header(&pfh, big_header, sizeof(big_header), &size, &crc_passed);
    pfh_debug_print(&pfh);
    if (pfh.fileId != 0x0347) {  debug_print("File id wrong - FAILED\n"); return FALSE; }
    if (pfh.fileSize != 81374) {  debug_print("Size wrong - FAILED\n"); return FALSE; }
    if (strcmp(pfh.source, "ST2NH")) {  debug_print("Source str wrong - FAILED\n"); return FALSE; }
    if (crc_passed)
        debug_print("CRC PASSED\n");
    else {
        debug_print("CRC FAILED\n");
        return FALSE;
    }

    debug_print("Generate Header Bytes\n");
    uint8_t buffer2[256];
    uint32_t body_size = 81374;
    uint16_t body_off = pfh_generate_header_bytes(&pfh, body_size, buffer2);

    HEADER pfh2;
    pfh_extract_header(&pfh2, big_header, sizeof(big_header), &size, &crc_passed);
    pfh_debug_print(&pfh2);
    if (pfh2.fileId != 0x0347) {  debug_print("File id wrong - FAILED\n"); return FALSE; }
    if (pfh2.fileSize != 81374) {  debug_print("Size wrong - FAILED\n"); return FALSE; }
    if (strcmp(pfh2.source, "ST2NH")) {  debug_print("Source str wrong - FAILED\n"); return FALSE; }
    if (crc_passed)
        debug_print("CRC PASSED\n");
    else {
        debug_print("CRC FAILED\n");
        return FALSE;
    }

    if (rc == TRUE)
        printf("##### TEST PACSAT HEADER: success:\n");
    else
        printf("##### TEST PACSAT HEADER: fail:\n");


    return rc;

}

int test_pfh_file() {
    printf("##### TEST PACSAT FILE:\n");
    int rc = TRUE;

    uint16_t size = 0;

    uint8_t header [] = {0xAA, 0x55, 0x01, 0x00, 0x04, 0x47, 0x03, 0x00, 0x00, 0x02, 0x00
    , 0x08, 0x35, 0x61, 0x62, 0x39, 0x38, 0x34, 0x62,
            0x30, 0x03, 0x00, 0x03, 0x20, 0x20, 0x20, 0x04, 0x00, 0x04, 0xDE, 0x3D, 0x01, 0x00, 0x05, 0x00, 0x04, 0x47, 0x7D, 0xB9, 0x5A,
            0x06, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x01, 0x00, 0x08, 0x00, 0x01, 0x10, 0x09, 0x00, 0x02, 0x3E, 0x54,
            0x0A, 0x00, 0x02, 0x2B, 0x28,
            0x0B, 0x00, 0x02, 0xD8, 0x00, 0x10, 0x00, 0x05, 0x53, 0x54, 0x32, 0x4E, 0x48, 0x11, 0x00, 0x06, 0x53,
            0x54, 0x32, 0x4E, 0x48, 0x20, 0x12, 0x00, 0x04, 0x31, 0x85, 0xB9, 0x5A, 0x13, 0x00, 0x01, 0x00, 0x14, 0x00, 0x03, 0x41, 0x4C,
            0x4C, 0x15, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x48, 0x00, 0x16, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x17, 0x00, 0x04, 0xC7,
            0x71, 0xBD, 0x5A, 0x18, 0x00, 0x01, 0x00, 0x19, 0x00, 0x01, 0x00, 0x22, 0x00, 0x10, 0x4D, 0x59, 0x20, 0x53, 0x48, 0x41, 0x43,
            0x4B, 0x20, 0x41, 0x4E, 0x44, 0x20, 0x41, 0x4E, 0x54, 0x23, 0x00, 0x04, 0x3C, 0x57, 0x3E, 0x20, 0x26, 0x00, 0x11, 0x73, 0x74,
            0x32, 0x6E, 0x68, 0x20, 0x70, 0x69, 0x63, 0x20, 0x61, 0x6E, 0x74, 0x2E, 0x6A, 0x70, 0x67, 0x2A, 0x00, 0x07, 0x41, 0x57, 0x55,
            0x32, 0x2E, 0x31, 0x30, 0x2E, 0x00, 0x08, 0xAE, 0x47, 0xE1, 0x7A, 0x14, 0x2E, 0x2F, 0x40, 0x2F, 0x00, 0x08, 0xCD, 0xCC, 0xCC,
            0xCC, 0xCC, 0x4C, 0x40, 0xC0, 0x00, 0x00, 0x00};


    bool crc_passed = false;

    HEADER pfh;
    pfh_extract_header(&pfh, header, sizeof(header), &size, &crc_passed);
    debug_print("PFH Extracted from buffer:\n");
    pfh_debug_print(&pfh);

    // Write the header into MRAM FS

    rc = dir_fs_write_file_chunk("//dir/0347",header,sizeof(header),0);
    if (rc == -1) {
        debug_print("FAILED to write header\n");
        return FALSE;
    }
    uint8_t buffer2[256];
    HEADER pfh2;
    int num_bytes_read = dir_fs_read_file_chunk("//dir/0347",buffer2,sizeof(buffer2),0);
    if (num_bytes_read == -1) {
        debug_print("ERROR reading header back from file system\n");
        return FALSE;
    }
    rc = pfh_extract_header(&pfh2, buffer2, sizeof(buffer2), &size, &crc_passed);
    if (rc == FALSE) {  debug_print("Could not extract header - FAILED\n"); return FALSE; }
    debug_print("PFH Extracted from MRAM:\n");
    pfh_debug_print(&pfh2);
    if (pfh2.fileId != 0x0347) {  debug_print("File id wrong - FAILED\n"); return FALSE; }
    if (pfh2.fileSize != 81374) {  debug_print("Size wrong - FAILED\n"); return FALSE; }
    if (strcmp(pfh2.source, "ST2NH")) {  debug_print("Source str wrong - FAILED\n"); return FALSE; }
    if (crc_passed)
        debug_print("CRC PASSED\n");
    else {
        debug_print("CRC FAILED\n");
        return FALSE;
    }

    if (rc == TRUE)
        printf("##### TEST PACSAT FILE: success:\n");
    else
        printf("##### TEST PACSAT FILE: fail:\n");
    return rc;
}

bool make_test_header(HEADER *pfh, uint32_t fh, unsigned int file_id, char *filename, char *source, char *destination,
                      char *title, char *user_filename, char *msg1) {
    if (pfh == NULL) return FALSE;
    /* Required Header Information */
    pfh->fileId = 0; //file_id;
    strlcpy(pfh->fileName,filename, sizeof(pfh->fileName));
//    strlcpy(pfh->fileExt,PSF_FILE_EXT, sizeof(pfh->fileExt));

    uint32_t now = getUnixTime();
    pfh->createTime = now;
    pfh->modifiedTime = now;
    pfh->SEUflag = 1;
    pfh->fileType = PFH_TYPE_ASCII;

    /* Extended Header Information */
    strlcpy(pfh->source,source, sizeof(pfh->source));

    pfh->uploadTime = 0;
    pfh->downloadCount = 0;
    strlcpy(pfh->destination,destination, sizeof(pfh->destination));
    pfh->downloadTime = 0;
    pfh->expireTime = now + 30*24*60*60;  // use for testing
    pfh->priority = 0;

    /* Optional Header Information */
    strlcpy(pfh->title,title, sizeof(pfh->title));
    strlcpy(pfh->userFileName,user_filename, sizeof(pfh->userFileName));

    uint32_t body_size = strlen(msg1);
    uint16_t body_checksum = 0;
    int j=0;
    /* Calc Body Checksum */
    while (j<body_size) {
        body_checksum += (uint8_t)msg1[j++] & 0xff;
    }
 //   debug_print("Body CRC: %04x\n",body_checksum);
    pfh->bodyCRC = body_checksum;

    uint8_t buffer3[256];
    uint16_t body_offset = pfh_generate_header_bytes(pfh, body_size, buffer3);

    char file_path[25];
    strlcpy(file_path,"//",sizeof(file_path));
    strlcat(file_path,filename,sizeof(file_path));

    /* Write the header */
//    bool rc = dir_mram_write_file(fh, buffer3, body_offset, file_id, pfh->uploadTime, body_offset, (10000 + fh * 1000));
    int32_t rc = dir_fs_write_file_chunk(file_path, buffer3, body_offset, 0);
    if (rc == -1) {  debug_print("Write PF header - FAILED\n"); return FALSE; }

    /* Then the file data */
//    rc = dir_mram_append_to_file(fh, (uint8_t *)msg1, body_size);
    rc = dir_fs_write_file_chunk(file_path, (uint8_t *)msg1, body_size, body_offset);
    if (rc == -1) {  debug_print("Write file data - FAILED\n"); return FALSE; }

    return TRUE;
}

int test_pfh_make_files() {
    // First reset the filnumbers as this is a test and we want them to start from 0
    WriteMRAMHighestFileNumber(0);
    printf("Next file number reset to zero\n");

    // Make a pacsat file to save
    HEADER pfh3;
    bool rc = TRUE;

    uint32_t numOfFiles = 10;
    debug_print("Make %d PPSF and save to MRAM\n",numOfFiles);
    char *msg1 = "Hi there,\nThis is a test message\n73 Chris\n";
    int i;
    for (i=0; i< numOfFiles; i++) {
        int id = (i + 1);
        pfh_new_header(&pfh3);
        char file_id_str[5];
        snprintf(file_id_str, 5, "%04x",id);
        char title_str[35];
        snprintf(title_str, 35, "Test Message %d",id);
        rc = make_test_header(&pfh3, i, id, file_id_str, "AC2CZ", "G0KLA", title_str, "userfile.txt", msg1);
        if (rc == FALSE) return FALSE;
    }

    char *msg2 = "Every PACSAT file will start with the byte 0xaa followed by the byte 0x55.  \n\
This flag is followed by the rest of the PACSAT File Header (PFH).  A valid \n\
PFH contains all of the items of the Mandatory Header (Section  3), and it may\n\
also contain all items of the Extended Header (Section 4) and any number of\n\
Optional Header items (Section 5).  All HEADER ITEMS are encoded using a \n\
standard syntax, described in Section 2.  \n\
\n\
The PFH is terminated by a special header item, after which the file body \n\
begins.\n\
\n\
Thus, there are 3 forms of PACSAT file header:\n\
\n\
<0xaa><0x55><Mandatory hdr><Hdr end>\n\
\n\
<0xaa><0x55><Mandatory hdr><Extended hdr><Hdr end>\n\
\n\
<0xaa><0x55><Mandatory hdr><Extended hdr>[<Optional    Items> . . . ]<Hdr end>\n\
            2.0 PACSAT HEADER ITEM SYNTAX\n\
            \n\
All PACSAT file header items follow a single format, simplifying both specifi-\n\
cation and implementation of the PACSAT File Header.  The format is:\n\
\n\
<id><length>[<data> . . . ]\n\
2.1 <id>\n\
The id is a 2-byte integer in which the bits have the following meaning:\n\
\n\
bit 15            0 this is an system-defined item.\n\
                 1 this is an experimental, user defined item.\n\
                 \n\
bits 0-14         form the 15-bit unsigned binary number identifying the\n\
                 item.\n\
                 \n\
The <id>, allows some 32,000 system-defined and 32,000 user defined items.\n\
<id> like all multi-byte integers is stored least-significant byte first.  \n\
Refer to the PACSAT Data Specification Standards document for further informa-\n\
tion.\n\
\n\
2.2 <length>\n\
\n\
This field is an 8-bit unsigned binary integer giving the number of <data>\n\
bytes present.  Even if the size of the data item is fixed, the length is \n\
still present.\n\
\n\
2.3 <data>\n\
\n\
The <data> bytes may hold any type of information.\n\
\n\
Encoding rules for system-defined items are found in this document.  User-\n\
defined items may adopt any internal encoding agreed by all users of the item.\n\
\n\
2.4 Presentation\n\
\n\
The PACSAT File Header must always be transmitted without data compression,\n\
even if compression is applied to the body of the attached file.\n\
\n\
2.5 Header Termination\n\
\n\
The end of the PACSAT File Header will always be indicated by a header item\n\
with <id> 0 and <length> 0.  The byte sequence is 0x00 0x00 0x00.\n\
\n\
3.0 THE PACSAT MANDATORY HEADER\n\
\n\
The first two bytes of a PACSAT file should always contain 0xaa followed by\n\
0x55 to confirm that the file contains a PACSAT file header.  \n\
\n\
The 0xaa, 0x55 sequence must be followed immediately by all items of the\n\
Mandatory Header.\n\
\n\
Mandatory Header items must be present in order of ascending value of <id>.\n\
\n\
When preparing files for uploading to PACSAT, groundstations must initialize\n\
header items as specified below.\n";

    rc = make_test_header(&pfh3, numOfFiles, numOfFiles+1, "psfhead", "AC2CZ", "VE2TCP", "Extract of PACSAT Header", "pfh_header.txt", msg2);
    if (rc == FALSE) return FALSE;
    numOfFiles++;

//    bool rc = writeNV(&numOfFiles,sizeof(uint32_t),NVConfigData,(int)&LocalFlash->NumberOfFiles);

    debug_print("Load the files and confirm\n");

    HEADER pfh4;
    pfh_new_header(&pfh4);

    uint16_t size;
    bool crc_passed = false;
    uint8_t buffer2[256];
    for (i=0; i< numOfFiles-1; i++) {
//        dir_mram_get_node(i, &mram_fh);
//        rc = readNV(buffer2, mram_fh.body_offset,NVStatisticsArea, mram_fh.address);

        char file_path_str[7];
        snprintf(file_path_str, 7, "//%04x",(i+1));
        int32_t num = dir_fs_read_file_chunk(file_path_str,buffer2,sizeof(buffer2),0);
        if (num == -1) {
            debug_print("Error reading file: %s\n",file_path_str);
            return FALSE;
        }
        pfh_extract_header(&pfh4, buffer2, sizeof(buffer2), &size, &crc_passed);
        if (!crc_passed) { debug_print("CRC FAILED\n"); return FALSE;}
        pfh_debug_print(&pfh4);
//        if (pfh4.fileId != (i+1)) {  debug_print("ID wrong for file %d - FAILED\n",i); return FALSE; }
//        if (pfh4.fileSize != 216) {  debug_print("Size %d wrong for file %d - FAILED\n",pfh4.fileSize,i); return FALSE; }
        if (strcmp(pfh4.source, "AC2CZ")) {  debug_print("Source str wrong - FAILED\n"); return FALSE; }
        if (strcmp(pfh4.destination, "G0KLA")) {  debug_print("Destination str wrong - FAILED\n"); return FALSE; }
//        if (mram_fh.body_offset != pfh4.bodyOffset) {  debug_print("Wrong body offset fh: %d pfh: %d- FAILED\n",mram_fh.body_offset, pfh4.bodyOffset); return FALSE; }
//        if (mram_fh.file_size != pfh4.fileSize) {  debug_print("Wrong file size fh: %d pfh: %d- FAILED\n",mram_fh.file_size, pfh4.fileSize); return FALSE; }
    }

    i = numOfFiles-1;
//    dir_mram_get_node(i, &mram_fh);
//    rc = readNV(buffer2, mram_fh.body_offset,NVConfigData, mram_fh.address);
    int32_t num = dir_fs_read_file_chunk("//psfhead", buffer2,sizeof(buffer2),0);
    if (num == -1) {
        debug_print("Error reading file: //psfhead\n");
        return FALSE;
    }
    pfh_extract_header(&pfh4, buffer2, sizeof(buffer2), &size, &crc_passed);
    if (!crc_passed) { debug_print("CRC FAILED\n"); return FALSE;}
    pfh_debug_print(&pfh4);
 //   if (pfh4.fileId != numOfFiles){  debug_print("ID wrong for file %d - FAILED\n",i); return FALSE; }
    if (pfh4.fileSize != 2892) {  debug_print("Size wrong for file %d is %d - FAILED\n",i,pfh4.fileSize); return FALSE; }
    if (strcmp(pfh4.source, "AC2CZ")) {  debug_print("Source str wrong - FAILED\n"); return FALSE; }
    if (strcmp(pfh4.destination, "VE2TCP")) {  debug_print("Destination str wrong - FAILED\n"); return FALSE; }
    if (strcmp(pfh4.title, "Extract of PACSAT Header")) {  debug_print("Title str wrong - FAILED\n"); return FALSE; }
//    if (mram_fh.body_offset != pfh4.bodyOffset) {  debug_print("Wrong body offset fh: %d pfh: %d- FAILED\n",mram_fh.body_offset, pfh4.bodyOffset); return FALSE; }
//    if (mram_fh.file_size != pfh4.fileSize) {  debug_print("Wrong file size fh: %d pfh: %d- FAILED\n",mram_fh.file_size, pfh4.fileSize); return FALSE; }
    if (strcmp(pfh4.fileName, "psfhead")) {  debug_print("File name str |%s| wrong - FAILED\n",pfh4.fileName); return FALSE; }

    if (rc == TRUE)
        printf("##### TEST PACSAT FILES: success:\n");
    else
        printf("##### TEST PACSAT FILES: fail:\n");


    return rc;
}

/**
 * test_pfh_make_internal_file()
 *
 * Test the mechanism to create an internal dir file
 *
 */
int test_pfh_make_internal_file(char * filename) {
    char *msg = "This is a test of the internal file system.  This file was created internally\
            by PACSAT and added automatically to the directory.  This mechansim will be used\
            for WOD, Log and Experiment files.\n\n\
            This file ends here with the word END";
    int rc = dir_fs_write_file_chunk(filename, (uint8_t *)msg, strlen(msg), 0);
    if (rc == -1) {  debug_print("PFH: Test Write file data for internal file test - FAILED\n"); return FALSE; }

    char psf_name[MAX_FILENAME_WITH_PATH_LEN];
    char psf_name_with_path[MAX_FILENAME_WITH_PATH_LEN];
    uint32_t now = getUnixTime(); // Get the time in seconds since the unix epoch
    HEADER pfh;
    uint32_t id = dir_next_file_number();
    debug_print("Next id is: %d\n",id);
    int ret = pfh_make_internal_header(&pfh, now, PFH_TYPE_ASCII, id, "", BBS_CALLSIGN, "TXT", "Title: Test", "user_file_test_file.txt",
                                       now, 0, BODY_NOT_COMPRESSED);
    if (ret == EXIT_FAILURE)
        return FALSE;

    pfh_debug_print(&pfh);
    dir_get_filename_from_file_id(id, psf_name, sizeof(psf_name_with_path));
    dir_get_file_path_from_file_id(id, DIR_FOLDER, psf_name_with_path, sizeof(psf_name_with_path));
    debug_print("Trying to create file %s in queue: %s from file %s\n",psf_name_with_path, "//",filename);

    REDSTAT stat;
    int32_t fp = red_open(filename, RED_O_RDONLY);
    if (fp == -1) { debug_print("Unable to open %s : %s\n", filename, red_strerror(red_errno));return FALSE; }
    ret = red_fstat(fp, &stat);
    if (ret == -1) { debug_print("Unable to stat %s : %s\n", filename, red_strerror(red_errno));return FALSE; }

    uint32_t file_size = stat.st_size;
    debug_print("file size: %d\n",file_size);
    rc = pfh_make_internal_file(&pfh, DIR_FOLDER, filename, file_size);
    if (rc != EXIT_SUCCESS) {
        printf("** Failed to make pacsat file %s\n", filename);
        rc = red_unlink(psf_name_with_path); // remove this in case it was partially written, ignore any error
        if (rc == -1) {
            debug_print("Unable to remove file: %s : %s\n", psf_name_with_path, red_strerror(red_errno));
        }
        return FALSE;
    }

    rc = dir_load_pacsat_file(psf_name);
    if (rc != TRUE) {
        rc = red_unlink(psf_name_with_path); // remove this in case it was partially written, ignore any error
        return FALSE;
    }

    printf("##### TEST MAKE INTERNAL FILE: success\n");

    return TRUE;
}

#endif /* DEBUG */
