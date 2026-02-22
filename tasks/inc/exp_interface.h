/*
 * exp_interface.h
 *
 *  Created on: Feb 20, 2026
 *      Author: g0kla
 */

#ifndef TASKS_INC_EXP_INTERFACE_H_
#define TASKS_INC_EXP_INTERFACE_H_

void exp_can_handler(int canNum, int priority, int type, int msgid,
                  int dest, int src, uint8_t *data, unsigned int len);



#endif /* TASKS_INC_EXP_INTERFACE_H_ */
