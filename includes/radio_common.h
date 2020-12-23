#ifndef RADIO_COMMON_H
#define RADIO_COMMON_H

// size of message in bytes
#define MSG_LEN 3

#define SHORT_MICROS 3000
#define LONG_MICROS SHORT_MICROS*2

#define SHORT_LOW (uint16_t) (SHORT_MICROS * 0.6f)
#define SHORT_HI (uint16_t) (SHORT_MICROS * 1.4f)

#define LONG_LOW (uint16_t) (LONG_MICROS * 0.6f)
#define LONG_HI (uint16_t) (LONG_MICROS * 1.4f)

void radio_delay_short(void);
void radio_delay_long(void);

#endif
