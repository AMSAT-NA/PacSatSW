/*
 * radio.h
 *
 *  Created on: Mar 11, 2019
 *      Author: burns
 *
 * This file contains the headers for generic radio routines.
 *
 */

#ifndef DRIVERS_INC_RADIO_H_
#define DRIVERS_INC_RADIO_H_
#include <stdbool.h>
#include "spiDriver.h"

void start_rx(rfchan chan, uint32_t freq, enum radio_modulation mod);
void stop_rx(rfchan chan);
void start_tx(rfchan chan, uint32_t freq, enum radio_modulation mod);
void stop_tx(rfchan chan);

bool rxing(rfchan chan);
bool rx_working(rfchan chan);

void set_modulation(rfchan chan, enum radio_modulation modulation, bool tx);
void set_tx_power(rfchan chan, uint32_t power);

uint8_t get_rssi(rfchan chan);
uint16_t get_tx_power(rfchan chan);
void test_freq(rfchan chan, uint32_t freq,
               enum radio_modulation modulation, unsigned int flags);
void test_pll_2m_range(rfchan chan, enum radio_modulation modulation,
                       unsigned int flags);


#endif /* DRIVERS_INC_AX5043_ACCESS_H_ */
