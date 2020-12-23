#ifndef RADIO_RECEIVER_H
#define RADIO_RECEIVER_H    

#include <stdint.h>
#include "radio_common.h"

#define RADIO_IN_PIN 7
#define RADIO_IN_DDR_REG DDRD
#define RADIO_IN_REG PIND

#define fuzzyEquals(actual,low,high) ( actual >= low && actual <= high )

void radio_receiver_init(void);

uint8_t radio_receive(uint8_t *buffer, uint8_t msgSize);

#endif