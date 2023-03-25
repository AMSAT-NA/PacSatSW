/*
 * pacsat_dir.c
 *
 *  Created on: Mar 9, 2023
 *      Author: g0kla
 *
 *
 *
 * This file also includes a simple MRAM File System:
 * This is a very simple file storage system THAT IS NOT SUITABLE FOR FLIGHT
 * that uses the file_id as the file handle.
 *
 */
#include <assert.h>
#include "MRAMmap.h"
#include "nonvol.h"
#include "PbTask.h"
#include "pacsat_header.h"
#include "pacsat_dir.h"

/* Dir variables */
static DIR_NODE *dir_head = NULL;  // the head of the directory linked list
static DIR_NODE *dir_tail = NULL;  // the tail of the directory linked list
static uint32_t next_file_id = 0; // This is incremented when we add files for upload.  Initialized when dir loaded.  TODO - keep only in MRAM

/* Forward declarations */
void dir_delete_node(DIR_NODE *node);
void insert_after(DIR_NODE *p, DIR_NODE *new_node);
void dir_debug_print(DIR_NODE *p);

/* This is used by the Simple MRAM FS */
static const MRAMmap_t *LocalFlash = 0;

/**
 * dir_next_file_number()
 *
 * This returns the next file number available for the upload process.
 * TODO - this will not cope well with failed uploads.  Those ids will be lost and
 * never used.  We are supposed to "reserve" the file number when a DATA command is
 * received, but we need to allocate it before that.
 *
 */
int dir_next_file_number() {
    next_file_id++;
    return next_file_id;
}

/**
 * dir_add_pfh()
 * Add an MRAM file header to the directory and return a pointer
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
 * To update an item correctly, remove it from the list, set the upload_time to zero and then call this routine to insert it at the end.
 *
 * If the upload_time was modified then the pacsat file header is resaved to disk
 *
 * The linked list takes care of allocating and deallocating memory for the DIR_NODEs and the
 * MRAM_FILEs inside the DIR_NODEs.  The MRAM_FILE that is passed in has its contents copied
 * into the allocated structure.
 *
 * TODO Currently this memory is allocated on the heap with pvPortMalloc and pPortFree.  We may
 * want to change that to a completely static allocation and hold a free list for nodes that
 * are not used.  This could guarantee no memory fragmentation and the expense of complexity.
 *
 */
DIR_NODE * dir_add_pfh(MRAM_FILE *new_mram_file) {
    int resave = false;
    DIR_NODE *new_node = (DIR_NODE *)pvPortMalloc(sizeof(DIR_NODE));
    MRAM_FILE *mram_file = (MRAM_FILE *)pvPortMalloc(sizeof(MRAM_FILE));
    new_node->mram_file = mram_file;
    mram_file->file_id = new_mram_file->file_id;
    mram_file->address = new_mram_file->address;
    mram_file->body_offset = new_mram_file->body_offset;
    mram_file->file_size = new_mram_file->file_size;
    mram_file->upload_time = new_mram_file->upload_time;

    uint32_t now = getSeconds(); // Get the system time in seconds since the epoch
    if (new_node == NULL) return NULL; // ERROR
    if (dir_head == NULL) { // This is a new list
        dir_head = new_node;
        dir_tail = new_node;
        if (mram_file->upload_time == 0) {
            mram_file->upload_time = now;
            resave = true;
        }
        new_node->next = NULL;
        new_node->prev = NULL;
    } else if (mram_file->upload_time == 0){
        /* Insert this at the end of the list as the newest item.  Make sure it has a unique upload time */
        if (dir_tail->mram_file->upload_time >= now) {
            /* We have added more than one file within 1 second.  Add this at the next available second. */
            mram_file->upload_time = dir_tail->mram_file->upload_time+1;
        } else {
            mram_file->upload_time = now;
        }
        insert_after(dir_tail, new_node);
        resave = true;
    } else {
        /* Insert this at the right point, searching from the back*/
        DIR_NODE *p = dir_tail;
        while (p != NULL) {
            if (p->mram_file->upload_time == mram_file->upload_time) {
                debug_print("ERROR: Attempt to insert duplicate PFH: ");
                //pfh_debug_print(mram_file);
                return NULL; // this is a duplicate
            } else if (p->mram_file->upload_time < mram_file->upload_time) {
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
    if (resave) {
        bool rc = FALSE;
        /////////////// TODO ------ NOT YET IMPLEMENTED!
        debug_print("** NOT YET IMPLEMENTED - cant update upload time for fh: %d \n",mram_file->file_id);
        //int rc = pfh_make_pacsat_file(mram_file, dir_folder);
        //rc = pfh_update_pacsat_header(mram_file, get_dir_folder(), filename);


        if (rc != TRUE) {
            // we could not save this
            debug_print("** Could not update the header for fh: %d to dir\n",mram_file->file_id);
            dir_delete_node(new_node);
            return NULL;
        } else {
            debug_print("Saved: %d\n",mram_file->file_id);
            //pfh_debug_print(new_pfh);
        }
    }
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
    debug_print("REMOVED: %d\n",node->mram_file->file_id);
//    pfh_debug_print(node->pfh);
    vPortFree(node->mram_file);
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
    debug_print("Dir List Cleared\n");
}

/**
 * dir_load_pacsat_file()
 *
 * Load a PACSAT file from MRAM and store it in the directory
 */
bool dir_load_pacsat_file(MRAM_FILE *mram_file) {
    debug_print("Loading: %d from addr: %d \n", mram_file->file_id, mram_file->address);
//    int err = dir_validate_file(pfh,psf_name);
//    if (err != ER_NONE) {
//        error_print("Err: %d - validating: %s\n", err, psf_name);
//        return FALSE;
//    }
    //pfh_debug_print(pfh);
    DIR_NODE *p = dir_add_pfh(mram_file);
    if (p == NULL) {
        debug_print("** Could not add to dir\n");
        return FALSE;
    }
    if (mram_file->file_id > next_file_id)
        next_file_id = mram_file->file_id;
    return TRUE;
}

/**
 * dir_load()
 *
 * Load the directory from the MRAM and store it in uptime sorted order in
 * the linked list
 *
 */
int dir_load() {
    MRAM_FILE mram_file;
    uint32_t numOfFiles = 0;
    bool rc;
    int file_handle = 0;

    debug_print("Loading directory from MRAM...\n");

    rc = readNV(&numOfFiles, sizeof(uint32_t),ExternalMRAMData, (int)&(LocalFlash->NumberOfFiles));
    if (!rc) {
        debug_print("Read MRAM number of files - FAILED\n");
        return FALSE;
    }
    debug_print("Loading %d files\n",numOfFiles);

    while (file_handle < numOfFiles) {
        rc = dir_mram_get_node(file_handle++,&mram_file);
        if (!rc) {
            debug_print("Read MRAM FAT - FAILED\n");
            return FALSE;
        }
        debug_print("%d: Id: %04x ",file_handle,mram_file.file_id);
        debug_print("Size: %d ",mram_file.file_size);
        debug_print("Address: %d ",mram_file.address);
        debug_print("Uploaded: %d\n",mram_file.upload_time);

        rc = dir_load_pacsat_file(&mram_file);
        if (rc != TRUE) {
            debug_print("May need to remove potentially corrupt or duplicate PACSAT file: %d  id: %d\n", file_handle, mram_file.file_id);
            /* Don't automatically remove here, otherwise loading the dir twice actually deletes all the
             * files! */
        }
    }
    debug_print("LOADED DIR:\n");
    dir_debug_print(dir_head);
    return TRUE;
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
        if (node->mram_file->upload_time >= pair.start && node->mram_file->upload_time <= pair.end)
            return node;
    }

    return NULL;
}
//uint32_t dir_get_pfh_by_date(DIR_DATE_PAIR pair, uint32_t file_handle, MRAM_FILE *pfh_node ) {
//    MRAM_FILE dir_node;
//    uint32_t numOfFiles = 0;
//    bool rc;
//
//    if (pfh_node == NULL) {
//        return NO_FILE; // error
//    }
//
//    rc = readNV(&numOfFiles, sizeof(uint32_t),ExternalMRAMData, (int)&(LocalFlash->NumberOfFiles));
//    if (!rc) {
//        debug_print("Read MRAM number of files - FAILED\n");
//        return NO_FILE;
//    }
//
//    while (file_handle < numOfFiles) {
//        rc = dir_mram_get_node(file_handle++,&dir_node);
//        if (!rc) {
//            debug_print("Read MRAM FAT - FAILED\n");
//            return NO_FILE;
//        }
//        debug_print("%d: Id: %04x ",file_handle,dir_node.file_id);
//        debug_print("Size: %d ",dir_node.file_size);
//        debug_print("Address: %d ",dir_node.address);
//        debug_print("Uploaded: %d\n",dir_node.upload_time);
//        if (dir_node.upload_time >= pair.start && dir_node.upload_time <= pair.end) {
////            memcpy(&dir_node, pfh_node, sizeof(DIR_NODE));
//            pfh_node->file_id = dir_node.file_id;
//            pfh_node->file_size = dir_node.file_size;
//            pfh_node->address = dir_node.address;
//            pfh_node->upload_time = dir_node.upload_time;
//            pfh_node->body_offset = dir_node.body_offset;
//            return file_handle;
//        }
//
//    }
//
//    return NO_FILE;
//}

/**
 * dir_get_node_by_id()
 * Search for and return a file based on its id. If the file can not
 * be found then return NULL
 *
 */
DIR_NODE * dir_get_node_by_id(int file_id) {
    DIR_NODE *p = dir_head;
    while (p != NULL) {
        if (p->mram_file->file_id == file_id)
            return p;
        p = p->next;
    }
    return NULL;
}
//uint32_t dir_get_node_by_id(int file_id, MRAM_FILE *pfh_node) {
//    MRAM_FILE dir_node;
//    uint32_t numOfFiles = 0;
//    uint32_t file_handle = 0;
//    bool rc;
//
//    if (pfh_node == NULL) {
//        return NO_FILE; // error
//    }
//
//    rc = readNV(&numOfFiles, sizeof(uint32_t),ExternalMRAMData, (int)&(LocalFlash->NumberOfFiles));
//    if (!rc) {
//        debug_print("Read MRAM number of files - FAILED\n");
//        return NO_FILE;
//    }
//
//    while (file_handle < numOfFiles) {
//        rc = dir_mram_get_node(file_handle++,&dir_node);
//        if (!rc) {
//            debug_print("Read MRAM FAT - FAILED\n");
//            return NO_FILE;
//        }
//        debug_print("%d: Id: %04x ",file_handle,dir_node.file_id);
//        debug_print("Size: %d ",dir_node.file_size);
//        debug_print("Uploaded: %d\n",dir_node.upload_time);
//        if (pfh_node->file_id == dir_node.file_id) {
//            pfh_node->file_id = dir_node.file_id;
//            pfh_node->file_size = dir_node.file_size;
//            pfh_node->address = dir_node.address;
//            pfh_node->upload_time = dir_node.upload_time;
//            pfh_node->body_offset = dir_node.body_offset;
//            return file_handle;
//        }
//    }
//
//    return NO_FILE;
//}

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
        debug_print("File id: %d up:%d\n",p->mram_file->file_id, p->mram_file->upload_time);
        p = p->next;
    }
}

/**
 * SIMPLE MRAM FILE SYSTEM FOLLOWS
 */


/**
 * Read a record from the file allocation table
 */
bool dir_mram_get_node(uint32_t file_handle, MRAM_FILE * dir_node) {
    bool rc = readNV(dir_node, sizeof(MRAM_FILE),ExternalMRAMData, (int)&(LocalFlash->MRAMFiles[file_handle]));
    if (!rc) {
        debug_print("MRAM FAT read - FAILED\n");
        return FALSE;
    }
    return TRUE;
}

/**
 * This saves data to a file based on the file id.  There is no automatic management of
 * space by this function, so we must know exactly where the data goes in the FAT and in the MRAM!
 *
 */
bool dir_mram_write_file(uint32_t file_handle, uint8_t *data, uint32_t length, uint32_t file_id, uint32_t upload_time,
                         uint16_t body_offset, uint32_t address) {

    bool rc;
    rc = writeNV(&file_id,sizeof(uint32_t),ExternalMRAMData,(int)&(LocalFlash->MRAMFiles[file_handle].file_id));
    if (!rc) {  debug_print("Write MRAM FAT file_id - FAILED\n");
        return FALSE; }
    rc = writeNV(&length,sizeof(uint32_t),ExternalMRAMData,(int)&(LocalFlash->MRAMFiles[file_handle].file_size));
    if (!rc) {  debug_print("Write MRAM FAT file_size - FAILED\n");
        return FALSE; }
    rc = writeNV(&address,sizeof(uint32_t),ExternalMRAMData,(int)&(LocalFlash->MRAMFiles[file_handle].address));
    if (!rc) {  debug_print("Write MRAM FAT address - FAILED\n");
        return FALSE; }
    rc = writeNV(&upload_time,sizeof(uint32_t),ExternalMRAMData,(int)&(LocalFlash->MRAMFiles[file_handle].upload_time));
    if (!rc) {  debug_print("Write MRAM FAT upload_time - FAILED\n");
        return FALSE; }
    rc = writeNV(&body_offset,sizeof(uint16_t),ExternalMRAMData,(int)&(LocalFlash->MRAMFiles[file_handle].body_offset));
    if (!rc) {  debug_print("Write MRAM FAT body_offset - FAILED\n"); return FALSE; }

    rc = writeNV(data,length,ExternalMRAMData,(int)address);
    if (!rc) {  debug_print("Write MRAM file data - FAILED\n");
        return FALSE; }
    return TRUE;
}

bool dir_mram_append_to_file(uint32_t file_handle, uint8_t *data, uint32_t length ) {

    bool rc;
    uint32_t address;
    uint32_t size;

    // Read the address for the start of the file
    rc = readNV(&address, sizeof(uint32_t),ExternalMRAMData, (int)&(LocalFlash->MRAMFiles[file_handle].address));
    if (!rc) {  debug_print("Write MRAM FAT header - FAILED\n"); return FALSE; }

    // Read the existing size
    rc = readNV(&size, sizeof(uint32_t),ExternalMRAMData, (int)&(LocalFlash->MRAMFiles[file_handle].file_size));
    if (!rc) {  debug_print("Write MRAM FAT header - FAILED\n"); return FALSE; }

    // Append the data
    rc = writeNV(data,length,ExternalMRAMData,(int)address+size);
    if (!rc) {  debug_print("Write MRAM file data - FAILED\n"); return FALSE; }

    // Write the new length
    size = size + length;
    rc = writeNV(&size,sizeof(uint32_t),ExternalMRAMData,(int)&(LocalFlash->MRAMFiles[file_handle].file_size));
    if (!rc) {  debug_print("Write MRAM FAT header - FAILED\n"); return FALSE; }

    return TRUE;
}

/**
 *
 */
bool dir_mram_read_file_chunk(uint32_t file_handle, uint8_t *data, uint32_t chunk_length, uint32_t offset) {

    return TRUE;
}

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
    if (dir_head->mram_file->file_id != 1) { printf("** Error creating file 1\n"); return EXIT_FAILURE; }
    if (dir_head->next->mram_file->file_id != 2) { printf("** Error creating file 2\n"); return EXIT_FAILURE; }
    if (dir_tail->mram_file->file_id != 4) { printf("** Error creating file 4\n"); return EXIT_FAILURE; }

#ifdef 0
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
