#ifndef SI_4432_H
#define SI_4432_H
#include <stdint.h>


/*
7 ffovfl RX/TX FIFO Overflow Status.
6 ffunfl RX/TX FIFO Underflow Status.
5 rxffem RX FIFO Empty Status.
4 headerr Header Error Status.
Indicates if the received packet has a header check error.
3 freqerr Frequency Error Status.
The programmed frequency is outside of the operating range. The actual frequency is
saturated to the max/min value.
2 Reserved
1:0 cps[1:0] Chip Power State.
00: Idle State
01: RX State
10: TX State
 */

#define SI4432_ERR_FIFO_OVERFLOW (1<<7)
#define SI4432_ERR_FIFO_UNDERFLOW (1<<6)
#define SI4432_ERR_FIFO_EMPTY (1<<5)
#define SI4432_ERR_HDR_ERROR (1<<4)
#define SI4432_ERR_FREQ_ERROR (1<<3)

#define SI4432_STATE_IDLE 0x00
#define SI4432_STATE_RX 0x01
#define SI4432_STATE_TX 0x02

uint8_t si4432_get_device_type();
uint8_t si4432_get_device_version();
uint8_t si4432_get_device_status();
uint8_t si4432_get_errors();
uint8_t si4432_get_device_state();

void si4432_init_transmit();
void si4432_init_receive();
void si4432_send(char *data, uint8_t len);
uint8_t si4432_receive(char *buffer, uint8_t bufferLen);

#endif
