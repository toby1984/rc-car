#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

// callback getting invoked every time the sampling interval is up
typedef void (*encoder_handler)(uint16_t ticksLeftMotor,uint16_t ticksRightMotor);

// result: 0 on success, 1 if sampling interval is too large to be covered by a 16 bit timer
uint8_t enc_init(float samplingIntervalMillis, encoder_handler callback);

void enc_reset();
#endif
