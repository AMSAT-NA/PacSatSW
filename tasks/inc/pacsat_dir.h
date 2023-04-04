/*
 * pacsat_dir.h
 *
 *  Created on: Feb 27, 2023
 *      Author: g0kla
 */

#ifndef UTILITIES_INC_PACSAT_DIR_H_
#define UTILITIES_INC_PACSAT_DIR_H_

#include "MRAMmap.h"
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
    MRAM_FILE * mram_file; /* This is cached in memory */
    struct dir_node *next;
    struct dir_node *prev;
} DIR_NODE;

int dir_next_file_number();
void dir_free();
int dir_load();
bool dir_mram_get_node(uint32_t file_handle, MRAM_FILE * dir_node);
bool dir_mram_write_file(uint32_t file_handle, uint8_t *data, uint32_t length, uint32_t file_id, uint32_t upload_time,
                         uint16_t body_offset, uint32_t address);
bool dir_mram_append_to_file(uint32_t file_handle, uint8_t *data, uint32_t length );
bool dir_mram_write_file_chunk(MRAM_FILE *mram_file, uint8_t *data, uint32_t chunk_length, uint32_t offset);
bool dir_mram_read_file_chunk(MRAM_FILE *mram_file, uint8_t *data, uint32_t chunk_length, uint32_t offset);
DIR_NODE * dir_get_pfh_by_date(DIR_DATE_PAIR pair, DIR_NODE *p );
DIR_NODE * dir_get_node_by_id(int file_id);

/* Test functions */
int test_pacsat_dir();

#endif /* UTILITIES_INC_PACSAT_DIR_H_ */
