#ifndef RADIO_SENDER_H
#define RADIO_SENDER_H    

#include <stdint.h>
#include "radio_common.h"

#define RADIO_OUT_PIN 2
#define RADIO_OUT_DDR_REG DDRD
#define RADIO_OUT_REG PORTD

void radio_sender_init(void);

void radio_send(uint8_t *data,uint8_t dataLen);

#endif