/*
 * pacsat_dir.h
 *
 *  Created on: Feb 27, 2023
 *      Author: g0kla
 */

#ifndef UTILITIES_INC_PACSAT_DIR_H_
#define UTILITIES_INC_PACSAT_DIR_H_

#include "MRAMmap.h"
#include "redconf.h"
#include "PbTask.h"

#define NO_FILE 0xFFFFFFFF

/*
 * We implement the directory as a doubly linked list.  Most operations involve traversing the list
 * in order to retrieve dir fills. When we add items they are near the end of the list (updated
 * files), so we search backwards to find the insertion point.
 *
 * Each dir_node stores MRAM file header together with the list pointers
 */
typedef struct dir_node {
    char filename[REDCONF_NAME_MAX+1U]; /* The name of the file on disk */
    uint32_t file_id; /* Cached from the PFH to allow searching by id */
    uint16_t body_offset; /* Cached from the PFH to allow load of PFH without having to overread the bytes */
    uint32_t upload_time; /* Cached from the PFH and used to sort the directory */
    struct dir_node *next;
    struct dir_node *prev;
} DIR_NODE;

int dir_next_file_number();
void dir_free();
int dir_load();

int32_t dir_fs_write_file_chunk(char *file_name_with_path, uint8_t *data, uint32_t length, uint32_t offset);
int32_t dir_fs_read_file_chunk(char *file_name_with_path, uint8_t *read_buffer, uint32_t length, uint32_t offset);
int32_t dir_fs_get_file_size(char *file_name_with_path);
DIR_NODE * dir_get_pfh_by_date(DIR_DATE_PAIR pair, DIR_NODE *p );
DIR_NODE * dir_get_node_by_id(int file_id);
void dir_debug_print(DIR_NODE *p);

/* Test functions */
int test_pacsat_dir();

#endif /* UTILITIES_INC_PACSAT_DIR_H_ */
