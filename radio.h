#ifndef RADIO_H
#define RADIO_H    

#include <stdint.h>

#define PREAMBLE_BYTES 2
#define PAYLOAD_BYTES_PER_MESSAGE 3

#define MSG_LEN PREAMBLE_BYTES+ PAYLOAD_BYTES_PER_MESSAGE;

#define SHORT_MS 3
#define LONG_MS SHORT_MS*2

#define SHORT_LOW (uint8_t) (SHORT_MS * 0.6f)
#define SHORT_HI (uint8_t) (SHORT_MS * 1.4f)

#define LONG_LOW (uint8_t) (LONG_MS * 0.6f)
#define LONG_HI (uint8_t) (LONG_MS * 1.4f)

#define RADIO_PIN 7

#define RADIO_DDR_REG DDRD
#define RADIO_IN_REG PIND
#define RADIO_OUT_REG PORTD

void radio_init(void);
void radio_send(char *data,uint8_t dataLen);
uint8_t radio_receive(char *buffer, uint8_t msgSize);

#endif