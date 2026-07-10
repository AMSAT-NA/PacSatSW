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

/* Used to set the value of the DAC setting the bias on the PA. */
void set_tx_dac_val(uint8_t val);
uint8_t get_tx_dac_val(void);

/* Used to set the output power for the AX5043. */
void set_tx_pow(uint8_t val);
uint8_t get_tx_pow(void);

typedef struct {
    uint8_t channel;
    uint8_t tx_modulation;
    uint8_t bytes[MAX_DATA_LEN];
    uint16_t len;
} tx_radio_buffer_t;

/*
 * Routine prototypes
 */

void TxTask(void *pvParameters);
bool tx_send_ui_packet(char *from_callsign, char *to_callsign, uint8_t pid,
		       uint8_t *bytes, int len, bool block,
		       enum radio_modulation modulation);
bool tx_send_packet(AX25_PACKET *packet, bool expedited, bool block,
		    enum radio_modulation modulation);

bool tx_test_make_packet(uint32_t len);

/* Setting this will inhibiting transmit. */
extern bool inhibitTransmit;

/* Current transmit mode. */
extern enum radio_modulation tx_modulation;

#endif /* TASKS_INC_TXTASK_H_ */
