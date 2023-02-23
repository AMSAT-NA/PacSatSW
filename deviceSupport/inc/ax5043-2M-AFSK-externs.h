#ifdef LEGACY_GOLF

uint8_t mode_tx_2m(void);
uint8_t receive_packet_2m(void);
uint8_t axradio_init_2m(int32_t freq);
uint8_t mode_rx_2m(void);
extern uint8_t axradio_rxbuffer_2m[];
void start_rx();
void test_pll_range();
extern uint8_t ax5043_off_xtal(void);
extern uint8_t ax5043_off(void);
extern uint8_t get_rssi();

#endif
