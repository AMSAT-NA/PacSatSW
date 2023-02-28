/*
 * TncTask.h
 *
 *  Created on: Dec 1, 2022
 *      Author: Chris E. Thompson, G0KLA
 */

#ifndef TASKS_INC_TXTASK_H_
#define TASKS_INC_TXTASK_H_

#include <pacsat.h>

#define BLOCK_IF_QUEUE_FULL true
#define DONT_BLOCK_IF_QUEUE_FULL false

/*
 * Routine prototypes
 */

void TxTask(void *pvParameters);
bool tx_send_packet(char *from_callsign, char *to_callsign, uint8_t pid, uint8_t *bytes, int len, bool block);
bool tx_make_packet(char *from_callsign, char *to_callsign, uint8_t pid, uint8_t *bytes, int len, uint8_t *raw_bytes);
bool tx_test_make_packet();

#endif /* TASKS_INC_TXTASK_H_ */
