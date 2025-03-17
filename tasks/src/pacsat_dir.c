/*
 * pacsat_dir.c
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
 *
 * This file also includes a simple MRAM File System:
 * This is a very simple file storage system THAT IS NOT SUITABLE FOR FLIGHT
 * that uses the file_id as the file handle.
 *
 */
#include <stdlib.h> //TODO - only added for strtol - hex conversion.  Is this really needed?
#include <assert.h>
#include "MRAMmap.h"
#include "nonvol.h"
#include "nonvolManagement.h"
#include "redposix.h"

#include "PbTask.h"
#include "pacsat_header.h"
#include "pacsat_dir.h"
#include "UplinkTask.h"
#include "inet.h"
#include "str_util.h"

#ifdef DEBUG
#include "time.h"
#endif

/* Dir variables */
static DIR_NODE *dir_head = NULL;  // the head of the directory linked list
static DIR_NODE *dir_tail = NULL;  // the tail of the directory linked list
static uint8_t data_buffer[AX25_MAX_DATA_LEN]; /* Static buffer used to store file bytes loaded from MRAM */

/* Forward declarations */
int32_t dir_check_folder(char *path);
void dir_delete_node(DIR_NODE *node);
void insert_after(DIR_NODE *p, DIR_NODE *new_node);
bool dir_fs_update_header(char *file_name_with_path, HEADER *pfh);
bool dir_fs_save_int(int32_t fp, uint32_t value, uint32_t offset);
bool dir_fs_save_short(int32_t fp, uint16_t value, uint32_t offset);

/* Local Variables */
static HEADER pfh_buffer; // Static allocation of a header to use when we need to load/save the header details
static uint8_t pfh_byte_buffer[MAX_BYTES_IN_PACSAT_FILE_HEADER]; /* Maximum size for a PFH */

/**
 * dir_next_file_number()
 *
 * This returns the next file number available for the upload process.  This is an important
 * process and must be reliable.  If something goes wrong or becomes corrupted then it could
 * return a file number that is already in use or return a filenumber that is much too large
 * potentially shortening the pool of available file handles.
 *
 * As a safety check the next file number should be based on the highest file number
 * actually used in the dir and the file numbers used by existing temporary upload files.  This
 * will benefit from a process that expires old temporary files that have become stale but it is
 * not required.
 *
 * This will not cope well with failed uploads if a successful file has been saved to the
 * dir after it.  Those ids will be lost and never used.  We are supposed to only "reserve" the
 * file number when a DATA command is actually received, but we need to allocate it before that.
 * So it is not clear how we reuse file numbers that are allocated but no bytes are ever received.
 *
 * This returns the next file id or zero if there is an error
 *
 */
uint32_t dir_next_file_number() {
    bool success = false;
    uint32_t file_id = ReadMRAMHighestFileNumber() + 1; /* Get the next number */
    while (!success) {
        /* Now check if we already have a temp file with this id */
        char file_name_with_path[MAX_FILENAME_WITH_PATH_LEN];
        dir_get_upload_file_path_from_file_id(file_id, file_name_with_path, MAX_FILENAME_WITH_PATH_LEN);
        int32_t fp = red_open(file_name_with_path, RED_O_RDONLY);
        if (fp == -1) {
            if (red_errno == RED_ENOENT) {
                /* No file so we can use this one */
                success = true;
            } else {
                /* Some other file-io error */
                debug_print("dir: err %s\n",red_strerror(red_errno));
                return 0;
            }
        } else {
            /* File exists, close it and check the next number */
            int32_t rc2 = red_close(fp);
            if (rc2 != 0) {
                debug_print("*** Unable to close tmp file: %s\n", red_strerror(red_errno));
                /* If we carry on here then we might use all the open files.  Something is wrong, return */
                return 0;
            }
            file_id = file_id + 1;
        }
    }
    return file_id;
}

/**
 * For a temporary file that is being uploaded and given a file id, create
 * the file name string with full path
 */
void dir_get_upload_file_path_from_file_id(uint32_t file_id, char *file_path, int max_len) {
    char file_id_str[5];
    snprintf(file_id_str, 5, "%04x",file_id);
    strlcpy(file_path, TMP_FOLDER, max_len);
    strlcat(file_path, file_id_str, max_len);
    strlcat(file_path, ".", max_len);
    strlcat(file_path, "tmp", max_len);
}

/**
 * Given a file id, create the file name string with full path
 */
void dir_get_file_path_from_file_id(uint32_t file_id, char *dir_name, char *file_path, int max_len) {
    char file_id_str[5];
    snprintf(file_id_str, 5, "%04x",file_id);
    strlcpy(file_path, dir_name, max_len);
//    strlcpy(file_path, DIR_FOLDER, max_len);
    strlcat(file_path, file_id_str, max_len);
//    strlcat(filename, PSF_FILE_EXT, MAX_FILE_PATH_LEN);
}


void dir_get_filename_from_file_id(uint32_t file_id, char *file_name, int max_len) {
    snprintf(file_name, max_len, "%04x",file_id);
}

uint32_t dir_get_file_id_from_filename(char *file_name) {
    char file_id_str[5];
    strlcpy(file_id_str,file_name,sizeof(file_id_str)); // copy first 4 chars
    int ret = strlen(file_id_str);
    if (ret != 4) return 0;
    uint32_t id = (uint32_t)strtol(file_id_str, NULL, 16);
    return id;
}

/**
 * Check that the required folders exist in the filesystem and create them if they
 * are missing.  If they can not be created return the file system error number,
 * otherwise return 0
 */
int32_t dir_check_folders() {
    int32_t ret;
    ret = dir_check_folder(DIR_FOLDER);
    if (ret == -1) return ret;
    ret = dir_check_folder(TMP_FOLDER);
    if (ret == -1) return ret;
    ret = dir_check_folder(WOD_FOLDER);
    if (ret == -1) return ret;

    return 0;
}

int32_t dir_check_folder(char *path) {
    REDDIR *pDir;
    int32_t rc = 0;
    pDir = red_opendir(path);
    if (pDir == NULL) {
        debug_print("Making folder %s\n",path);
        rc = red_mkdir(path);
        if (rc == -1) {
            debug_print("Unable to open dir: %s\n", red_strerror(red_errno));
        }
    } else {
        int32_t rc2 = red_closedir(pDir);
        if (rc2 != 0) {
            debug_print("*** Unable to close dir: %s\n", red_strerror(red_errno));
        }
    }

    return rc;
}

/**
 * dir_add_pfh()
 * Add a pacsat file header to the directory and return a pointer
 * to the inserted DIR_NODE in the linked list.
 *
 * This handles the situation where the list is empty and creates the first item.
 * In an existing list if this item does not have an upload time then it is new
 * and it is inserted at the end of the list with the current time.  If many items
 * are added at the same time then it is given an upload time 1 second after the
 * last item.
 * If this header already has an upload_time then we search backwards in the list
 * to find the insertion point.
 * If we find a header with the same upload_time then this must be a duplicate and it
 * is discarded.
 *
 * To update an item correctly, remove it from the list, set the upload_time to zero
 * and then call this routine to insert it at the end.
 *
 * If the upload_time was modified then the pacsat file header is resaved to disk
 *
 * This routine allocates memory for the DIR_NODE and the dir_delete_node function
 * deallocates it
 *
 * TODO Currently this memory is allocated on the heap with pvPortMalloc and pPortFree.  We may
 * want to change that to a completely static allocation and hold a free list for nodes that
 * are not used.  This could guarantee no memory fragmentation and the expense of complexity.
 *
 */
DIR_NODE * dir_add_pfh(char *file_name, HEADER *new_pfh) {
    int resave_as_new_file = false;
    DIR_NODE *new_node = (DIR_NODE *)pvPortMalloc(sizeof(DIR_NODE));
    new_node->file_id = new_pfh->fileId;
    strlcpy(new_node->filename, file_name, REDCONF_NAME_MAX+1U);
    new_node->body_offset = new_pfh->bodyOffset;
    new_node->upload_time = new_pfh->uploadTime;

    uint32_t now = getUnixTime(); // Get the time in seconds since the unix epoch
    if (new_node == NULL) return NULL; // ERROR
    if (dir_head == NULL) { // This is a new list
        dir_head = new_node;
        dir_tail = new_node;
        if (new_node->upload_time == 0) {
            new_node->upload_time = now;
            resave_as_new_file = true;
        }
        new_node->next = NULL;
        new_node->prev = NULL;
    } else if (new_node->upload_time == 0){
        /* Insert this at the end of the list as the newest item.  Make sure it has a unique upload time */
        if (dir_tail->upload_time >= now) {
            /* We have added more than one file within 1 second.  Add this at the next available second. */
            new_node->upload_time = dir_tail->upload_time+1;
        } else {
            new_node->upload_time = now;
        }
        insert_after(dir_tail, new_node);
        resave_as_new_file = true;
    } else {
        /* Insert this at the right point, searching from the back*/
        DIR_NODE *p = dir_tail;
        while (p != NULL) {
            if (p->upload_time == new_node->upload_time) {
                debug_print("ERROR: Attempt to insert duplicate PFH: ");
                //pfh_debug_print(mram_file);
                // TODO - IMPORTANT - Don't we need to free the new_node??  Memory leak??
                return NULL; // this is a duplicate
            } else if (p->upload_time < new_node->upload_time) {
                insert_after(p, new_node);
                break;
            } else if (p == dir_head) {
                // Insert at the head of the list
                new_node->next = p;
                p->prev = new_node;
                new_node->prev = NULL;
                dir_head = new_node;
                break;
            }
            p = p->prev;
        }
    }
    // Now re-save the file with the new time if it changed, this recalculates the checksums
    if (resave_as_new_file) {
        /* The length of the header and the file length do not change because we only change the upload time
         * and the file id.
         * So we can just re-write the header on top of the old with the new
         * upload time.  We must recalc the header checksum
         */
        if (new_node->file_id == 0) {
            // then we need a file_id.  This was not an upload
            new_node->file_id = dir_next_file_number();
        }

        /* store these values back in the header that the caller passed in */
        new_pfh->uploadTime = new_node->upload_time;
        new_pfh->fileId = new_node->file_id;

        char file_name_with_path[MAX_FILENAME_WITH_PATH_LEN];
        strlcpy(file_name_with_path, DIR_FOLDER, MAX_FILENAME_WITH_PATH_LEN);
        strlcat(file_name_with_path, new_node->filename, MAX_FILENAME_WITH_PATH_LEN);

        // Write the new file id, upload_time and recalc the checksum
        bool rc = dir_fs_update_header(file_name_with_path, new_pfh);
        if (rc == FALSE) {
            // we could not save this
            debug_print("** Could not update the header for fh: %d to dir\n",new_node->file_id);
            dir_delete_node(new_node);
            return NULL;
        } else {
            //debug_print("DIR: Saved File: %s\n",file_name_with_path);
        }
    }

    uint32_t file_id = ReadMRAMHighestFileNumber();
    if (new_node->file_id > file_id)
        WriteMRAMHighestFileNumber(new_node->file_id);

    return new_node;
}

/**
 * insert_after()
 * Insert new_node after node p in the linked list.
 * Handle the situation where p is the tail of the list
 * p may not be NULL
 *
 */
void insert_after(DIR_NODE *p, DIR_NODE *new_node) {
    assert(p != NULL);
    new_node->next = p->next; // which may be null if at end of list
    new_node->prev = p;
    if (p->next == NULL) // we are at the end of the list
        dir_tail = new_node;
    else
        p->next->prev = new_node;
    p->next = new_node;
}

/**
 * dir_delete_node()
 *
 * Remove an entry from the dir linked list and free the memory held by the node
 * and the pacsat file header.
 *
 * The files on disk are not removed.
 *
 */
void dir_delete_node(DIR_NODE *node) {
    if (node == NULL) return;
    if (node->prev == NULL && node->next == NULL) {
        // special case of only one item
        dir_head = NULL;
        dir_tail = NULL;
    } else if (node->prev == NULL) {
        // special case removing the head of the list
        dir_head = node->next;
        node->next->prev = NULL;
    } else if (node->next == NULL) {
        // special case removing the tail of the list
        dir_tail = node->prev;
        node->prev->next = NULL;

    } else {
        node->next->prev = node->prev;
        node->prev->next = node->next;
    }
//    debug_print("REMOVED: %d\n",node->mram_file->file_id);
//    pfh_debug_print(node->pfh);
    vPortFree(node);
}

/**
 * dir_free_list()
 *
 * Remove all entries from the dir linked list and free all the
 * memory held by the list and the pacsat file headers.
 */
void dir_free() {
    DIR_NODE *p = dir_head;
    while (p != NULL) {
        DIR_NODE *node = p;
        p = p->next;
        dir_delete_node(node);
    }
//    debug_print("Dir List Cleared\n");
}

/**
 * dir_load_pacsat_file()
 *
 * Load a PACSAT file from MRAM and store it in the directory
 */
bool dir_load_pacsat_file(char *file_name) {
//    debug_print("Loading: %d from addr: %d \n", mram_file->file_id, mram_file->address);

    char file_name_with_path[MAX_FILENAME_WITH_PATH_LEN];
    //snprintf(file_name_with_path, MAX_FILENAME_WITH_PATH_LEN, "//%s",file_name);

    strlcpy(file_name_with_path, DIR_FOLDER, sizeof(file_name_with_path));
    strlcat(file_name_with_path, file_name, sizeof(file_name_with_path));

    int len = dir_load_header(file_name_with_path, pfh_byte_buffer, sizeof(pfh_byte_buffer), &pfh_buffer);
    if (len == -1) {
        debug_print("** Could not extract header from %s\n", file_name_with_path);
        return FALSE;
    }
    /* TODO - do we want to validate the files every time we boot and load the dir?
     * We would need to take an action if it fails
     */
//    int err = dir_validate_file(&pfh_buffer, file_name_with_path);
//    if (err != ER_NONE) {
//        debug_print("Err: %d - validating: %s\n", err, file_name_with_path);
//        return FALSE;
//    }
    DIR_NODE *p = dir_add_pfh(file_name, &pfh_buffer);
    if (p == NULL) {
        debug_print("** Could not add %s to dir\n", file_name_with_path);
        return FALSE;
    }
    return TRUE;
}

/**
 * dir_load()
 *
 * Load the directory from the MRAM and store it in uptime sorted order in
 * the linked list.  This should only be run at boot.  Then it is maintained in
 * memory as a cache.
 *
 */
int dir_load() {
    dir_free();
    //WriteMRAMHighestFileNumber(0); /* Reset the next file id as we will calculate the highest file number */

    bool rc;
    REDDIR *pDir;
    char * path = DIR_FOLDER;
//    printf("Loading Directory from %s:\n",path);
    pDir = red_opendir(path);
    if (pDir == NULL) {
        debug_print("Unable to open dir: %s\n", red_strerror(red_errno));
        return FALSE;
    }

    REDDIRENT *pDirEnt;
    red_errno = 0; /* Set error to zero so we can distinguish between a real error and the end of the DIR */
    pDirEnt = red_readdir(pDir);
    while (pDirEnt != NULL) {
        if (!RED_S_ISDIR(pDirEnt->d_stat.st_mode)) {
            //debug_print("Loading: %s\n",pDirEnt->d_name);
            rc = dir_load_pacsat_file(pDirEnt->d_name);
            if (rc != TRUE) {
                debug_print("May need to remove potentially corrupt or duplicate PACSAT file\n");
                /* Don't automatically remove here, otherwise loading the dir twice actually deletes all the
                 * files! */
            }
        }
        pDirEnt = red_readdir(pDir);
    }
    if (red_errno != 0) {
        debug_print("*** Error reading directory: %s\n", red_strerror(red_errno));

    }
    int32_t rc2 = red_closedir(pDir);
    if (rc2 != 0) {
        debug_print("*** Unable to close dir: %s\n", red_strerror(red_errno));
    }

//    debug_print("DONE:\n");
    return TRUE;
}

int dir_load_header(char *file_name_with_path, uint8_t *byte_buffer, int buffer_len, HEADER *pfh) {
    // Read enough of the file to parse the PFH
    int32_t rc = dir_fs_read_file_chunk(file_name_with_path, byte_buffer, buffer_len, 0);
    if (rc == -1) {
        debug_print("Error reading file: %s\n",file_name_with_path);
        return -1;
    }
    uint16_t size;
    bool crc_passed = FALSE;
    if (!pfh_extract_header(pfh, byte_buffer, buffer_len, &size, &crc_passed)) {
        debug_print("PFH Extract FAILED\n");
        return -1;
    }

    if (!crc_passed) {
        debug_print("PFH CRC FAILED\n");
        return -1;
    }
    return size;
}

/**
 * dir_validate_file()
 * Validate a newly uploaded file to make sure that it has integrity with its Header.
 * This assumes that the header has already been loaded and validated.
 *
 * Returns ERR_NONE if everything is good.  Otherwise it returns an FTL0 error number.
 *
 */
int dir_validate_file(HEADER *pfh, char *file_name_with_path, WdReporters_t reporter) {
    //debug_print("DIR: Checking data in file: %s\n",file_name_with_path);

    /* Now check the body */
    uint16_t body_checksum = 0;
    uint32_t body_size = 0;
    uint32_t offset = pfh->bodyOffset;
    bool finished = FALSE;

    while (!finished) {
        int32_t num_of_bytes_read = dir_fs_read_file_chunk(file_name_with_path, data_buffer, AX25_MAX_DATA_LEN, offset);
        if (num_of_bytes_read == -1) {
            return ER_NO_SUCH_FILE_NUMBER;
        }

        int j;
        for (j=0; j<num_of_bytes_read;j++){
            body_checksum += data_buffer[j] & 0xff;
            body_size++;
        }
        if (num_of_bytes_read < AX25_MAX_DATA_LEN)
            finished = true;
        offset += num_of_bytes_read;
        //debug_print("%d\n",offset);
        ReportToWatchdog(reporter);
    }

    debug_print("File check loop done\n");
    if (pfh->bodyCRC != body_checksum) {
        debug_print("** Body check failed for %s\n",file_name_with_path);
        return ER_BODY_CHECK;
    }
    if (pfh->fileSize != pfh->bodyOffset + body_size) {
        debug_print("** Body check failed for %s\n",file_name_with_path);
        return ER_FILE_COMPLETE;
    }

    return ER_NONE;
}


/**
 * dir_get_pfh_by_date()
 *
 * Traverse the directory and return a pointer to the next DIR_NODE identified by the dates.
 * The node passed in is the previous node that we processed.  If there are no more nodes
 * then NULL is returned.
 *
 * The protocol defines the required functionality as:
 *
      For each PAIR, the server will transmit directories for those files with
      UPLOAD_TIME greater than or equal to <start> and less than or equal to <end>.

      If there are no files within the range, the directory for the first file with
      upload time greater than <end> will be transmitted. If there is no such file,
      the directory for this first file with upload time less than <start> will be
      transmitted. In either case, the <t_old> and <t_new> fields in this directory
      will indicate to the client that there are no entries between <start> and
      <end>.
 *
 * Returns a pointer to the DIR_NODE or NULL if none are found
 *
 */
DIR_NODE * dir_get_pfh_by_date(DIR_DATE_PAIR pair, DIR_NODE *p ) {
    if (p == NULL) {
        /* Then we are starting the search from the head.  TODO - could later optimize if search from head or tail */
        p = dir_head;
    }
    while (p != NULL) {
        DIR_NODE *node = p;
        p = p->next;
        if (node->upload_time >= pair.start && node->upload_time <= pair.end) {
            trace_pb("-> returning file id: %04x name: %s\n",node->file_id, node->filename);
            return node;
        }
    }

    return NULL;
}

/**
 * dir_get_node_by_id()
 * Search for and return a file based on its id. If the file can not
 * be found then return NULL
 *
 */
DIR_NODE * dir_get_node_by_id(int file_id) {
    DIR_NODE *p = dir_head;
    while (p != NULL) {
        if (p->file_id == file_id)
            return p;
        p = p->next;
    }
    return NULL;
}

// TODO - this should do one file and return (remembering where it is) otherwise it will block the TELEM TASK when dir is large
void dir_maintenance() {
    uint32_t now = getUnixTime();

    DIR_NODE *p = dir_head;
    while (p != NULL) {
        //debug_print("CHECKING: File id: %04x name: %s up:%d age:%d sec\n",p->file_id, p->filename, p->upload_time, now-p->upload_time);
        int32_t age = now-p->upload_time;
        if (age < 0) {
            // this looks wrong, something is corrupt.  Skip it
            p = p->next;
        } else if (pb_is_file_in_use(p->file_id)) {
            // This file is currently being broadcast then skip it until next time
            p = p->next;
        } else if (age > DIR_MAX_FILE_AGE) {
            // Remove this file it is over the max age
            char file_name_with_path[MAX_FILENAME_WITH_PATH_LEN];
            strlcpy(file_name_with_path, DIR_FOLDER, sizeof(file_name_with_path));
            strlcat(file_name_with_path, p->filename, sizeof(file_name_with_path));

            //debug_print("Purging: %s\n",file_name_with_path);
            int32_t fp = red_unlink(file_name_with_path);
            if (fp == -1) {
                // This was probablly open because it is being update or broadcast.  So it is OK to skip until next time
                debug_print("Unable to remove file: %s : %s\n", file_name_with_path, red_strerror(red_errno));
                p = p->next;
            } else {
                // Remove from the dir
                DIR_NODE *node = p;
                p = p->next;
                dir_delete_node(node);
            }
        } else {
            // TODO - Check if there is an expiry date in the header
            p = p->next;
        }
        ReportToWatchdog(CurrentTaskWD);
        vTaskDelay(CENTISECONDS(10)); // yield some time so that other things can do work
        ReportToWatchdog(CurrentTaskWD);

    }
}

/**
 * dir_file_queue_check()
 *
 * Check a folder to see if it contains any files.  If it does, each file is added to the directory.
 * If a file can not be added, then we have two choices:
 * 1. Skip it and try again later - error not related to the file itself.  e.g. the file system is full
 * 2. Delete it as it can never be added - file error.  It is corrupt, or can not be processed.
 *
 * We use the following file names in this:
 * de->d_name: is the name of the file in the folder without the path.  This becomes the user_file_name in
 *  the header
 * file_name: This is the path to the file in the folder.
 * psf_name: This is the name of the file in the dir with the path e.g. //dir/0034
 * file_to_load: This is the name of the file in the dir without the path e,g, 0034
 *
 */
void dir_file_queue_check(uint32_t now, char * folder, uint8_t file_type, char * destination) {
    debug_print("Checking for files in queue: %s\n",folder);
    REDDIR *pDir;
    pDir = red_opendir(folder);
    if (pDir == NULL) {
        debug_print("Unable to open dir:550 %s\n", red_strerror(red_errno));
        return;
    }

    REDDIRENT *de;
    red_errno = 0; /* Set error to zero so we can distinguish between a real error and the end of the DIR */

    char file_name[MAX_FILENAME_WITH_PATH_LEN];
//    char user_file_name[MAX_FILENAME_WITH_PATH_LEN];
    char psf_name[MAX_FILENAME_WITH_PATH_LEN];
    for (de = red_readdir(pDir); de != NULL; de = red_readdir(pDir)) {
        strlcpy(file_name, folder, sizeof(file_name));
//         strlcat(file_name, "/", sizeof(file_name));
        strlcat(file_name, de->d_name, sizeof(file_name));
        if (!RED_S_ISDIR(de->d_stat.st_mode)) {
            if (str_ends_with(de->d_name, PSF_FILE_TMP)) {
              debug_print("Skipping file: %s\n",de->d_name);
                continue;
            }

            uint32_t id = dir_next_file_number();  //TODO - if this fails, then should we roll this back
            char compression_type = BODY_NOT_COMPRESSED;

            uint32_t create_time = de->d_stat.st_mtime; /* We use the time of last modify as the create time.  So for a wod file this is the time the last data was written. */
            uint32_t file_size = de->d_stat.st_size;
            if (file_size > UNCOMPRESSED_FILE_SIZE_LIMIT) {
                //TODO - compression is not yet implemented for internally generated files
                /* Compress if more than 200 bytes used */
//                char zip_command[MAX_FILE_PATH_LEN];
//                char compressed_file_name[MAX_FILE_PATH_LEN];
//
//                strlcpy(compressed_file_name, file_name, sizeof(compressed_file_name));
//                strlcat(compressed_file_name,".zip", sizeof(compressed_file_name));
//
//                strlcpy(zip_command, "zip -j -q ",sizeof(zip_command)); // -j to "junk" or not store the paths.  -q quiet
//                strlcat(zip_command, compressed_file_name,sizeof(zip_command));
//                strlcat(zip_command, " ",sizeof(zip_command));
//                strlcat(zip_command, file_name,sizeof(zip_command));
//                int zip_rc = system(zip_command);
//                if (zip_rc == EXIT_SUCCESS) {
//                    /* We could compress it */
//                    strlcat(user_file_name,".zip", sizeof(user_file_name));
//                    compression_type = BODY_COMPRESSED_PKZIP;
//                    remove(file_name);
//                    strlcpy(file_name, compressed_file_name, sizeof(file_name));
//                }
                // NOTE - update file size here
            }
            HEADER pfh;
            int ret = pfh_make_internal_header(&pfh, now, file_type, id, "", BBS_CALLSIGN, destination, de->d_name, de->d_name,
                    create_time, 0, compression_type);
            if (ret == EXIT_FAILURE)
                continue;

            //pfh_debug_print(&pfh);
            dir_get_file_path_from_file_id(id, DIR_FOLDER, psf_name, sizeof(psf_name));
            debug_print("Trying to create file %s in queue: %s from file %s\n",psf_name, folder,file_name);

            int rc;
            rc = pfh_make_internal_file(&pfh, DIR_FOLDER, file_name, file_size);
            if (rc != EXIT_SUCCESS) {
                printf("** Failed to make pacsat file %s from file %s\n", psf_name, file_name);
                rc = red_unlink(psf_name); // remove this in case it was partially written, ignore any error
                if (rc == -1) {
                    debug_print("Unable to remove file: %s : %s\n", psf_name, red_strerror(red_errno));
                }
                continue;
            }

            char file_to_load[MAX_FILENAME_WITH_PATH_LEN];
            dir_get_filename_from_file_id(id, file_to_load, MAX_FILENAME_WITH_PATH_LEN);
            rc = dir_load_pacsat_file(file_to_load);
            if (rc != TRUE) {
                debug_print("Removing potentially corrupt file from queue to prevent multiple adds: %s\n", file_name);
                rc = red_unlink(psf_name); // remove this in case it was partially written, ignore any error
                continue;
            }

            rc = red_unlink(file_name); // remove the file now that it is processed
            if (rc == -1) {
                //TODO - this is a critical error as we will keep adding it and fill the file system
                debug_print("Unable to remove file: %s : %s\n", file_name, red_strerror(red_errno));
            }
        }
    }
    int32_t rc2 = red_closedir(pDir);
    if (rc2 != 0) {
        debug_print("*** Unable to close dir: %s\n", red_strerror(red_errno));
    }

}


#ifdef DEBUG

/**
 * dir_debug_print()
 *
 * If DEBUG is set then print out all of the entries of the dir linked list.
 *
 */
void dir_debug_print(DIR_NODE *p) {
    if (p == NULL)
        p = dir_head;

    if (p == NULL)
        debug_print("..Empty Dir List\n");
    while (p != NULL) {
        //pfh_debug_print(p->pfh);
        char buf[30];
         time_t now = p->upload_time + 2208988800L;
         strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", gmtime(&now));
        debug_print("File id: %04x name: %s up:%d %s\n",p->file_id, p->filename, p->upload_time,buf);
        p = p->next;
    }
}

#endif /* DEBUG */

/**
 * FS FILE functions
 *
 */

/**
 * Write a chunk of data to a file in the file system.
 * The full path to the file must be specified.
 *
 * If the file does not exist, then it is created.
 * If offset is specified then data is written at that offset in the file
 *
 * Returns the number of bytes written or -1 if there is an error.
 *
 */
int32_t dir_fs_write_file_chunk(char *file_name_with_path, uint8_t *data, uint32_t length, uint32_t offset) {
    int32_t fp;
    int32_t numOfBytesWritten = -1;
    int32_t rc;

    fp = red_open(file_name_with_path, RED_O_CREAT | RED_O_WRONLY);
    if (fp == -1) {
        debug_print("Unable to open %s for writing: %s\n", file_name_with_path, red_strerror(red_errno));
        return -1;
    }

    if (offset != 0) {
        rc = red_lseek(fp, offset, RED_SEEK_SET);
        if (rc == -1) {
            debug_print("Unable to seek %s  to offset %d: %s\n", file_name_with_path, offset, red_strerror(red_errno));

            rc = red_close(fp);
            if (rc != 0) {
                printf("Unable to close %s: %s\n", file_name_with_path, red_strerror(red_errno));
            }
            return -1;
        }
    }

    numOfBytesWritten = red_write(fp, data, length);
    if (numOfBytesWritten != length) {
        printf("Write returned: %d\n",numOfBytesWritten);
        if (numOfBytesWritten == -1) {
            printf("Unable to write to %s: %s\n", file_name_with_path, red_strerror(red_errno));
        }
    }
    rc = red_close(fp);
    if (rc != 0) {
        printf("Unable to close %s: %s\n", file_name_with_path, red_strerror(red_errno));
    }

    return numOfBytesWritten;
}

/**
 * Read a chunk of bytes from a file in MRAM File system.
 * The full path must be specified for the file.
 *
 * Returns the number of bytes read or -1 if there was an error.
 *
 */
int32_t dir_fs_read_file_chunk(char *file_name_with_path, uint8_t *read_buffer, uint32_t length, uint32_t offset) {
    int32_t rc;

    int32_t fp = red_open(file_name_with_path, RED_O_RDONLY);
    if (fp == -1) {
        debug_print("Unable to open %s for reading: %s\n", file_name_with_path, red_strerror(red_errno));
        return -1;
    }

    if (offset != 0) {
        rc = red_lseek(fp, offset, RED_SEEK_SET);
        if (rc == -1) {
            debug_print("Unable to seek %s  to offset %d: %s\n", file_name_with_path, offset, red_strerror(red_errno));

            rc = red_close(fp);
            if (rc != 0) {
                printf("Unable to close %s: %s\n", file_name_with_path, red_strerror(red_errno));
            }
            return -1;
        }
    }

    int32_t numOfBytesRead = red_read(fp, read_buffer, length);
    if (numOfBytesRead == -1) {
        debug_print("Unable to read %s: %s\n", file_name_with_path, red_strerror(red_errno));
    }

    rc = red_close(fp);
    if (rc != 0) {
        printf("Unable to close %s: %s\n", file_name_with_path, red_strerror(red_errno));
    }
    return numOfBytesRead;
}

/**
 * This saves a big endian 4 byte int into little endian format in the MRAM
 */
bool dir_fs_save_int(int32_t fp, uint32_t value, uint32_t offset) {
    int32_t numOfBytesWritten = -1;
    int32_t rc = red_lseek(fp, offset + 3, RED_SEEK_SET);
    if (rc == -1) {
        return -1;
    }
    uint8_t data[4];
    pfh_store_int(data, value);
    numOfBytesWritten = red_write(fp, data, sizeof(data));
    if (numOfBytesWritten != sizeof(data)) {
        printf("Write returned: %d\n",numOfBytesWritten);
        return -1;
    }
    return numOfBytesWritten;
}

/**
 * This saves a big endian 2 byte short into into little endian format in the MRAM
 */
bool dir_fs_save_short(int32_t fp, uint16_t value, uint32_t offset) {
    int32_t numOfBytesWritten = -1;
    int32_t rc = red_lseek(fp, offset + 3, RED_SEEK_SET);
    if (rc == -1) {
        return -1;
    }
    uint8_t data[2];
    pfh_store_short(data, value);
    numOfBytesWritten = red_write(fp, data, sizeof(data));
    if (numOfBytesWritten != sizeof(data)) {
        printf("Write returned: %d\n",numOfBytesWritten);
        return -1;
    }
    return numOfBytesWritten;
}

/**
 * Update the header in place in MRAM. This preserves any pacsat header fields that the spacecraft
 * does not understand, but which are important to the sender/receiver.
 *
 * This uses fixed offsets for the mandatory fields, which are defined in pacsat_header.h
 *
 * Returns TRUE or FALSE if there is an error
 */
//bool dir_fs_update_header(char *file_name_with_path, uint32_t file_id, uint32_t upload_time, uint16_t body_offset) {
bool dir_fs_update_header(char *file_name_with_path, HEADER *pfh) {
    int32_t fp;
    int32_t rc;

    fp = red_open(file_name_with_path, RED_O_CREAT | RED_O_WRONLY);
    if (fp == -1) {
        debug_print("Unable to open %s for writing: %s\n", file_name_with_path, red_strerror(red_errno));
        return FALSE;
    }
    rc = dir_fs_save_int(fp, pfh->fileId, FILE_ID_BYTE_POS);
    if (rc == -1) {
        debug_print("Unable to save fileid to %s with data at offset %d: %s\n", file_name_with_path, FILE_ID_BYTE_POS, red_strerror(red_errno));
        rc = red_close(fp);
        return FALSE;
    }
    rc = dir_fs_save_int(fp, pfh->uploadTime, UPLOAD_TIME_BYTE_POS_EX_SOURCE_LEN + pfh->source_length);
    if (rc == -1) {
        debug_print("Unable to save uploadtime to %s with data at offset %d: %s\n", file_name_with_path, UPLOAD_TIME_BYTE_POS_EX_SOURCE_LEN + pfh->source_length, red_strerror(red_errno));
        rc = red_close(fp);
        return FALSE;
    }

    /* Then recalculate the checksum and write it */
    uint16_t crc_result = 0;

    /* Read the header */
    int32_t num = dir_fs_read_file_chunk(file_name_with_path, pfh_byte_buffer, pfh->bodyOffset, 0);
    if (num != pfh->bodyOffset) {
        debug_print("Can not read the correct length of header bytes from %s \n", file_name_with_path);
        rc = red_close(fp);
        return FALSE;
    }
    /* First zero out the existing header checksum */
    pfh_byte_buffer[HEADER_CHECKSUM_BYTE_POS +3] = 0x00;
    pfh_byte_buffer[HEADER_CHECKSUM_BYTE_POS +4] = 0x00;

    int j;
    /* Then calculate the new one and save it back to MRAM */
    for (j=0; j<pfh->bodyOffset; j++)
        crc_result += pfh_byte_buffer[j] & 0xff;
    rc = dir_fs_save_short(fp, crc_result, HEADER_CHECKSUM_BYTE_POS);
    if (rc == -1) {
        debug_print("Unable to save header checksum to %s with data at offset %d: %s\n", file_name_with_path, HEADER_CHECKSUM_BYTE_POS, red_strerror(red_errno));
        rc = red_close(fp);
        return FALSE;
    }

    rc = red_close(fp);
    if (rc != 0) {
        printf("Unable to close %s: %s\n", file_name_with_path, red_strerror(red_errno));
    }

    return TRUE;
}


/**
 * Get the size of a file.
 *
 * Returns the size of the file or -1 if there is an error.
 *
 */
int32_t dir_fs_get_file_size(char *file_name_with_path) {
    int32_t rc;
    int64_t numOfBytesRead; // we need room for a 32 bit size and a negative number for an error.  TODO - we could limit to u32 bits if we never have files over 31 bits in size

    int32_t fp = red_open(file_name_with_path, RED_O_RDONLY);
    if (fp == -1) {
        debug_print("Unable to open %s for reading: %s\n", file_name_with_path, red_strerror(red_errno));
        return -1;
    }

    numOfBytesRead = red_lseek(fp, 0, RED_SEEK_END);
    if (numOfBytesRead == -1) {
        debug_print("Unable to seek %s to end: %s\n", file_name_with_path, red_strerror(red_errno));

        rc = red_close(fp);
        if (rc != 0) {
            printf("Unable to close %s: %s\n", file_name_with_path, red_strerror(red_errno));
        }
        return -1;
    }

    rc = red_close(fp);
    if (rc != 0) {
        printf("Unable to close %s: %s\n", file_name_with_path, red_strerror(red_errno));
    }
    return numOfBytesRead;
}


#ifdef DEBUG

/**
 * TEST ROUTINES FOLLOW
 *
 */

/**
 * Test the Pacsat dir.  The command "make psf" needs to have been run already to generate the
 * test files.
 *
 */
int test_pacsat_dir() {
    printf("##### TEST PACSAT DIR:\n");
    int rc = EXIT_SUCCESS;
    debug_print("TEST DIR LOAD\n");
    dir_load();
    dir_debug_print(dir_head);
    if (dir_head->file_id != 1) { printf("** Error creating file 1\n"); return FALSE; }
    if (dir_head->next->file_id != 2) { printf("** Error creating file 2\n"); return FALSE; }
    if (dir_tail->file_id != 4) { printf("** Error creating file 4\n"); return FALSE; }

    //TODO this test needs to be fixed
#ifdef REFACTOR
    debug_print("DELETE HEAD\n");
    dir_delete_node(dir_head);
    dir_debug_print(dir_head);
    if (dir_head->pfh->fileId != 2) { printf("** Error deleting head with file 2\n"); return EXIT_FAILURE; }
    if (dir_head->next->pfh->fileId != 3) { printf("** Error deleting head with file 3\n"); return EXIT_FAILURE; }
    dir_free();

    if (make_three_test_entries() == EXIT_FAILURE) { printf("** Could not make test files\n"); return EXIT_FAILURE; }
    debug_print("DELETE MIDDLE\n");
    dir_delete_node(dir_head->next);
    dir_debug_print(dir_head);
    if (dir_head->pfh->fileId != 1) { printf("** Error deleting middle with file 1\n"); return EXIT_FAILURE; }
    if (dir_head->next->pfh->fileId != 3) { printf("** Error deleting middle with file 3\n"); return EXIT_FAILURE; }
    dir_free();

    if (make_three_test_entries() == EXIT_FAILURE) { printf("** Could not make test files\n"); return EXIT_FAILURE; }
    debug_print("DELETE TAIL\n");
    dir_delete_node(dir_tail);
    dir_debug_print(dir_head);
    if (dir_head->pfh->fileId != 1) { printf("** Error deleting tail with file 1\n"); return EXIT_FAILURE; }
    if (dir_head->next->pfh->fileId != 2) { printf("** Error deleting tail with file 2\n"); return EXIT_FAILURE; }

    dir_free();

    if (make_three_test_entries() == EXIT_FAILURE) { printf("** Could not make test files\n"); return EXIT_FAILURE; }
    dir_debug_print(dir_head);
    dir_free(); // we just want the fresh files on disk, but the dir list to be empty

    /* Now load the dir from the folder and check that it is the same
     * This tests insert to blank list, insert at end and insert in the middle, assuming
     * the load order is file1, file3, file2
     */
    debug_print("LOAD DIR\n");
    //dir_load();
    if (dir_load_pacsat_file("/tmp/test_dir/0001.act") != EXIT_SUCCESS) {  printf("** Could not load psf 1\n"); return EXIT_FAILURE; }
    if (dir_load_pacsat_file("/tmp/test_dir/0002.act") != EXIT_SUCCESS) {  printf("** Could not load psf 2\n"); return EXIT_FAILURE; }
    if (dir_load_pacsat_file("/tmp/test_dir/0003.act") != EXIT_SUCCESS) {  printf("** Could not load psf 3\n"); return EXIT_FAILURE; }
    if (dir_load_pacsat_file("/tmp/test_dir/0004.act") != EXIT_SUCCESS) {  printf("** Could not load psf 4\n"); return EXIT_FAILURE; }

    if (dir_head == NULL) {printf("** Could not load head\n"); return EXIT_FAILURE; }
    if (dir_head->next == NULL) {printf("** Could not load head + 1\n"); return EXIT_FAILURE; }
    if (dir_tail == NULL) {printf("** Could not load fail\n"); return EXIT_FAILURE; }

    if (dir_head->pfh->fileId != 1) { printf("** Error loading file 1 as head\n"); return EXIT_FAILURE; }
    if (dir_head->next->pfh->fileId != 2) { printf("** Error loading file 2 as second entry\n"); return EXIT_FAILURE; }
    if (dir_tail->pfh->fileId != 4) { printf("** Error loading file 4 as tail\n"); return EXIT_FAILURE; }
    debug_print("LOADED DIR LIST\n");
    dir_debug_print(dir_head);
    debug_print("TEST DUPLICATE DIR LOAD - expecting load errors, but exit success\n");
    if (dir_load() != EXIT_SUCCESS) { printf("** Error testing duplicate insertion\n"); return EXIT_FAILURE; } // confirm duplicates not loaded

    /* Test search for file */
    if (dir_get_node_by_id(1) == NULL) { printf("** Error finding file 1\n"); return EXIT_FAILURE; }
    DIR_NODE * last = dir_get_node_by_id(4);
    if ( last == NULL) { printf("** Error finding file 4\n"); return EXIT_FAILURE; }
    if ( last->next != NULL) { printf("** Error duplicate insert after file 4\n"); return EXIT_FAILURE; }
    if (dir_get_node_by_id(9999) != NULL) { printf("** Error with search for missing file\n"); return EXIT_FAILURE; }

#endif

    if (rc == EXIT_SUCCESS)
        printf("##### TEST PACSAT DIR: success\n");
    else
        printf("##### TEST PACSAT DIR: fail\n");
    return rc;
}

#endif /* DEBUG */
