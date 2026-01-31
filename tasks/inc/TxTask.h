/*
 * TncTask.h
 *
 *  Created on: Dec 1, 2022
 *      Author: Chris E. Thompson, G0KLA
 */

#ifndef TASKS_INC_TXTASK_H_
#define TASKS_INC_TXTASK_H_

#include <pacsat.h>
#include "ax25_util.h"

#define BLOCK true
#define DONT_BLOCK false

#define EXPEDITED true
#define NOT_EXPEDITED false

/*
 * Routine prototypes
 */

void TxTask(void *pvParameters);
bool tx_send_ui_packet(char *from_callsign, char *to_callsign, uint8_t pid, uint8_t *bytes, int len, bool block);
bool tx_send_packet(AX25_PACKET *packet, bool expedited, bool block);

bool tx_test_make_packet();

/* TODO - This is a hack for inhibiting transmit. */
extern bool inhibitTransmit;

/* Current transmit mode. */
extern enum ax5043_mode tx_mode;

#endif /* TASKS_INC_TXTASK_H_ */
