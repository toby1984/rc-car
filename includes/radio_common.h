#ifndef RADIO_COMMON_H
#define RADIO_COMMON_H

// number of training bits to send before transmitting payload
#define TRAINING_BITS 16

// size of message in bytes
#define MSG_LEN 3

// length of a short pulse in microseconds
#define SHORT_MICROS 3000

// length of a long pulse in microseconds
#define LONG_MICROS (SHORT_MICROS*2)

// lo/hi thresholds for short pulses
#define SHORT_LOW (uint16_t) (SHORT_MICROS * 0.6f)
#define SHORT_HI (uint16_t) (SHORT_MICROS * 1.4f)

// lo/hi thresholds for long pulses
#define LONG_LOW (uint16_t) (LONG_MICROS * 0.6f)
#define LONG_HI (uint16_t) (LONG_MICROS * 1.4f)

void radio_delay_short(void);
void radio_delay_long(void);

#endif
