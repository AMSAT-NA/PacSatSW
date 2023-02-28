/*
 * pacsat_dir.h
 *
 *  Created on: Feb 27, 2023
 *      Author: g0kla
 */

#ifndef UTILITIES_INC_PACSAT_DIR_H_
#define UTILITIES_INC_PACSAT_DIR_H_


struct dir_node {
    int mram_address; // this points to the address in MRAM where the PFH is
};
typedef struct dir_node DIR_NODE;


#endif /* UTILITIES_INC_PACSAT_DIR_H_ */
