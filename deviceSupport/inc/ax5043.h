#ifndef DRIVERS_INC_AX5043_H_
#define DRIVERS_INC_AX5043_H_
//#include "axradioinit.h"
#include "ax5043_access.h"
uint16_t fifo_free();
void fifo_commit();
void fifo_queue_buffer(uint8_t *buf, uint8_t len, uint8_t flags);
void fifo_send_sync(int final);
void start_ax25_tx();

void ax5043_set_power(uint32_t powerRegVal);
void ax5043_set_frequency(uint32_t freqRegVal);

uint16_t fifo_free();
void fifo_repeat_byte(uint8_t b, uint8_t count, uint8_t flags);
void fifo_commit();
void fifo_queue_buffer(uint8_t *buf, uint8_t len, uint8_t flags);
void fifo_send_sync(int final);
void fifo_commit();
void fifo_repeat_byte(uint8_t b, uint8_t count, uint8_t flags);
void fifo_queue_buffer(uint8_t *buf, uint8_t len, uint8_t flags);
uint8_t ax5043_off(void);
uint8_t get_rssi(void);

//void ax5043_set_registers(void);
//void ax5043_set_registers_tx(void);
//void ax5043_set_registers_rx(void);

#define AX5043_IRQINVERSION0 0x00B   /* IRQ Inversion 0 */
#define AX5043_IRQINVERSION1 0x00A   /* IRQ Inversion 1 */
#define AX5043_IRQMASK0 0x007   /* IRQ Mask 0 */
#define AX5043_IRQMASK1 0x006   /* IRQ Mask 1 */
#define AX5043_PINFUNCIRQ 0x024   /* Pin Function IRQ */
#define AX5043_XTALCAP 0x184   /* Crystal Oscillator Load Capacitance */
#define AX5043_PWRSTATE_FULL_RX             0x9
#define AX5043_PWRSTATE_FULL_TX             0xd
#define AX5043_FIFOSTAT 0x028   /* FIFO Control */
#define AX5043_IRQREQUEST0 0x00D   /* IRQ Request 0 */
#define AX5043_IRQREQUEST1 0x00C   /* IRQ Request 1 */
#define AX5043_PWRMODE 0x002   /* Power Mode */
#define AX5043_PINFUNCSYSCLK 0x021   /* Pin Function SYSCLK */
#define AX5043_FIFOCOUNT0 0x02B   /* Number of Words currently in FIFO 0 */
#define AX5043_FIFOCOUNT1 0x02A   /* Number of Words currently in FIFO 1 */
#define AX5043_FIFODATA 0x029   /* FIFO Data */
#define AX5043_FIFOFREE0 0x02D   /* Number of Words that can be written to FIFO 0 */
#define AX5043_FIFOFREE1 0x02C   /* Number of Words that can be written to FIFO 1 */
#define AX5043_FIFOSTAT 0x028   /* FIFO Control */
#define AX5043_FIFOTHRESH0 0x02F   /* FIFO Threshold 0 */
#define AX5043_FIFOTHRESH1 0x02E   /* FIFO Threshold 1 */


#endif

