/*
 * exp_interface.h
 *
 *  Created on: Feb 20, 2026
 *      Author: g0kla
 */

#ifndef TASKS_INC_EXP_INTERFACE_H_
#define TASKS_INC_EXP_INTERFACE_H_

#include "pacsat.h"

enum MSG_TYPE {
    can_msg_type_reserved,
    can_msg_type_status,
    can_msg_type_telem,
    can_msg_type_wod,
    can_msg_type_file1,
    can_msg_type_file2,
    can_msg_type_file3,
    can_msg_type_file4,
    can_msg_type_eof1,
    can_msg_type_eof2,
    can_msg_type_eof3,
    can_msg_type_eof4,
};


void exp_can_handler(int canNum, unsigned int type, unsigned int msgid,
		     unsigned int src, uint8_t *data, unsigned int len);


#endif /* TASKS_INC_EXP_INTERFACE_H_ */
