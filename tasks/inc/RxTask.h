/*
 * RxTask.h
 *
 *  Created on: Feb 8, 2022
 *      Author: Chris E. Thompson, G0KLA
 */

#ifndef TASKS_INC_RXTASK_H_
#define TASKS_INC_RXTASK_H_

#include <pacsat.h>

typedef struct {
    uint8_t channel;
    uint8_t bytes[MAX_DATA_LEN];
    uint16_t len;
} rx_radio_buffer_t;


/*
 * Routine prototypes
 */

void RxTask(void *pvParameters);


#endif /* TASKS_INC_RXTASK_H_ */
